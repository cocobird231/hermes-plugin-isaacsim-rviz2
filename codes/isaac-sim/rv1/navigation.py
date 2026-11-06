"""ROS 2 input and measured feedback for the reviewed RV1 swerve controller.

The actuator is RV1Controller from robots/rv1_tune_control.py; this
module does not implement a second wheel controller. The owning runtime calls
update() on the Kit thread with SimulationManager's physics time. No animation
timeline, global ROS executor, background USD writes, or target-as-measurement
fallback is used. Nav2 is the sole source of driving commands.

start() creates ROS resources but leaves commands disabled. arm() starts a new
volatile subscriber epoch. brake(), pause, time reset, invalid input and watchdog
expiry latch commands disabled; recovery requires an explicit arm(). When
disabled, update() publishes feedback but never writes actuator targets.
"""
import copy
import math
import re
import threading
import time


_DEFAULTS = {
    "cmd_vel_topic": "/cmd_vel", "odom_topic": "/odom", "joint_states_topic": "/joint_states",
    "clock_topic": "/clock", "base_frame": "base_link", "odom_frame": "odom", "map_frame": "map",
    "publish_hz": 20.0, "command_timeout_seconds": 3.0,
    "max_linear_mps": 1.0, "max_yaw_rps": 0.5, "allow_lateral": False,
    "localization_mode": "sim_truth", "map_from_stage_xyz_yaw": [0.0, 0.0, 0.0, 0.0],
    "chassis_from_base_xyz_xyzw": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "joint_name_map": {}, "static_transforms": [],
}


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain {size} numbers")
    return [_finite(x, name) for x in value]


def _quaternion(value):
    value = _vector(value, 4, "quaternion xyzw")
    norm = math.hypot(*value)
    if norm < 1e-8:
        raise ValueError("Quaternion must have nonzero norm")
    return [x / norm for x in value]


def _frame(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", value):
        raise ValueError("Frame IDs must be nonempty relative ROS names")
    if "//" in value or value.endswith("/"):
        raise ValueError("Frame IDs cannot have empty components")
    return value


def validate_config(config=None):
    if config is None:
        config = {}
    if not isinstance(config, dict) or set(config) - set(_DEFAULTS):
        raise ValueError("Unknown RV1 navigation configuration")
    result = {**copy.deepcopy(_DEFAULTS), **copy.deepcopy(config)}
    for key in ("cmd_vel_topic", "odom_topic", "joint_states_topic", "clock_topic"):
        value = result[key]
        if not isinstance(value, str) or not value.startswith("/"):
            raise ValueError(f"{key} must be an absolute ROS topic")
        _frame(value[1:])
    if len({result[k] for k in ("cmd_vel_topic", "odom_topic", "joint_states_topic", "clock_topic")}) != 4:
        raise ValueError("ROS topics of different message types must be distinct")
    for key in ("base_frame", "odom_frame", "map_frame"):
        result[key] = _frame(result[key])
    if len({result[k] for k in ("base_frame", "odom_frame", "map_frame")}) != 3:
        raise ValueError("map, odom and base frames must be distinct")
    for key, low, high in (("publish_hz", 1.0, 120.0), ("command_timeout_seconds", 0.1, 10.0),
                           ("max_linear_mps", 0.01, 2.5), ("max_yaw_rps", 0.01, 0.5)):
        value = _finite(result[key], key)
        if not low <= value <= high:
            raise ValueError(f"{key} must be within [{low}, {high}]")
        result[key] = value
    if not isinstance(result["allow_lateral"], bool):
        raise ValueError("allow_lateral must be boolean")
    if result["localization_mode"] not in {"sim_truth", "external"}:
        raise ValueError("localization_mode must be sim_truth or external")
    result["map_from_stage_xyz_yaw"] = _vector(result["map_from_stage_xyz_yaw"], 4, "map_from_stage_xyz_yaw")
    pose = _vector(result["chassis_from_base_xyz_xyzw"], 7, "chassis_from_base_xyz_xyzw")
    result["chassis_from_base_xyz_xyzw"] = pose[:3] + _quaternion(pose[3:])
    names = result["joint_name_map"]
    if not isinstance(names, dict) or not all(isinstance(k, str) and k and isinstance(v, str) and v for k, v in names.items()):
        raise ValueError("joint_name_map must map source DOF names to URDF joint names")
    if len(set(names.values())) != len(names):
        raise ValueError("joint_name_map cannot merge joints")
    if not isinstance(result["static_transforms"], list):
        raise ValueError("static_transforms must be a list")
    children = {result["map_frame"], result["odom_frame"], result["base_frame"]}
    parents = {}
    for transform in result["static_transforms"]:
        if not isinstance(transform, dict) or set(transform) != {"parent_frame", "child_frame", "translation", "rotation_xyzw"}:
            raise ValueError("Static TF requires parent_frame, child_frame, translation and rotation_xyzw")
        parent, child = _frame(transform["parent_frame"]), _frame(transform["child_frame"])
        if parent == child or child in children:
            raise ValueError("Static TF cannot redefine map/odom/base or another child")
        children.add(child)
        parents[child] = parent
        transform["translation"] = _vector(transform["translation"], 3, "static translation")
        transform["rotation_xyzw"] = _quaternion(transform["rotation_xyzw"])
    for child in parents:
        seen = set()
        while child in parents:
            if child in seen:
                raise ValueError("Static TF contains a cycle")
            seen.add(child)
            child = parents[child]
        if child != result["base_frame"]:
            raise ValueError("Static sensor TF must connect to base_frame")
    return result


def _multiply(a, b):
    x, y, z, w = a
    i, j, k, r = b
    return [w*i + x*r + y*k - z*j, w*j - x*k + y*r + z*i,
            w*k + x*j - y*i + z*r, w*r - x*i - y*j - z*k]


def _rotate(q, vector):
    return _multiply(_multiply(q, list(vector) + [0.0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def measured_base_state(telemetry, chassis_from_base):
    """Rigid offset pose/twist in odom=stage; angular values are radians."""
    q = _quaternion(telemetry["orientation_xyzw"])
    offset = _rotate(q, chassis_from_base[:3])
    orientation = _quaternion(_multiply(q, chassis_from_base[3:]))
    position = [a + b for a, b in zip(telemetry["position_m"], offset)]
    wx, wy, wz = telemetry["angular_velocity_world_rps"]
    x, y, z = offset
    cross = [wy*z - wz*y, wz*x - wx*z, wx*y - wy*x]
    velocity = [a + b for a, b in zip(telemetry["linear_velocity_world_mps"], cross)]
    inverse = [-orientation[0], -orientation[1], -orientation[2], orientation[3]]
    return {"position_m": position, "orientation_xyzw": orientation,
            "linear_velocity_body_mps": _rotate(inverse, velocity),
            "angular_velocity_body_rps": _rotate(inverse, [wx, wy, wz])}


class RV1Navigation:
    def __init__(self, controller, config=None):
        self.controller, self.config = controller, validate_config(config)
        self._thread = threading.get_ident()
        self._context = self._node = self._executor = self._subscription = None
        self._publishers = {}
        self._types = {}
        self._started = self._armed = False
        self._generation = 0
        self._arm_system_ns = 0
        self._sim_time = self._last_publish = self._last_command_wall = self._arm_wall = None
        self._target = [0.0, 0.0, 0.0]
        self._joint_sim = self._articulation = None
        self._joint_names = []
        self._last_measurement = None
        self._last_base = None
        self._rest_since = None
        self._rest_samples = 0
        self._reason = "not_started"
        self._error = None
        self._received = self._rejected = self._published = self._clock_published = 0

    def _main_thread(self):
        if threading.get_ident() != self._thread:
            raise RuntimeError("RV1Navigation must run on its owning Kit thread")

    def start(self):
        """Bridge extension and library environment must be enabled by caller."""
        self._main_thread()
        if self._started:
            return self.status()
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import Twist, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage

        self._types = dict(Twist=Twist, TransformStamped=TransformStamped, Odometry=Odometry,
                           JointState=JointState, Clock=Clock, TFMessage=TFMessage)
        self._command_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                       reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self._context = Context()
        try:
            rclpy.init(context=self._context)
            self._node = Node("rv1_navigation_adapter", context=self._context, use_global_arguments=False)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            for name, msg, topic in (("clock", Clock, self.config["clock_topic"]),
                                      ("odom", Odometry, self.config["odom_topic"]),
                                      ("joints", JointState, self.config["joint_states_topic"]),
                                      ("tf", TFMessage, "/tf")):
                self._publishers[name] = self._node.create_publisher(msg, topic, 10)
            if self.config["static_transforms"]:
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self._publishers["tf_static"] = self._node.create_publisher(TFMessage, "/tf_static", qos)
                message = TFMessage()
                message.transforms = [self._transform(t["parent_frame"], t["child_frame"],
                                                       t["translation"], t["rotation_xyzw"], 0.0)
                                      for t in self.config["static_transforms"]]
                self._publishers["tf_static"].publish(message)
            self._started = True
            self._reason = "commands_disabled"
            return self.status()
        except Exception:
            self.close()
            raise

    def arm(self):
        """Call only after acquiring nav2 ownership; never reuse a queued Twist."""
        self._main_thread()
        if not self._started:
            raise RuntimeError("Start RV1 navigation ROS resources before arming")
        self._armed = False
        self._rest_since = None
        self._rest_samples = 0
        self._generation += 1
        self._target = [0.0, 0.0, 0.0]
        self._last_command_wall = None
        self._reason = "arming"
        self.controller.zero()
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        generation = self._generation
        self._arm_system_ns = time.time_ns()
        self._subscription = self._node.create_subscription(
            self._types["Twist"], self.config["cmd_vel_topic"],
            lambda msg, info: self._on_twist(msg, generation, info), self._command_qos)
        self._arm_wall = time.monotonic()
        self._armed, self._reason, self._error = True, "waiting_for_twist", None
        return self.status()

    def _on_twist(self, msg, generation, info):
        if not self._armed or generation != self._generation:
            return
        try:
            # The first deployment is same-host DDS. A plain Twist has no header;
            # use its DDS source timestamp to reject pre-arm/queued old samples.
            # Remote hosts must synchronize system clocks before using this mode.
            # Jazzy rclpy supplies a dict; newer bindings may expose an object.
            source_ns = int(info["source_timestamp"] if isinstance(info, dict) else info.source_timestamp)
            age_seconds = (time.time_ns() - source_ns) / 1e9
            if source_ns <= self._arm_system_ns or age_seconds > self.config["command_timeout_seconds"]:
                self._rejected += 1
                return
            if age_seconds < -0.5:
                raise ValueError("DDS source clock is ahead of the simulator host")
            all_values = [_finite(x, "Twist") for x in
                          (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)]
            vx, vy, vz, wx, wy, wz = all_values
            if any(abs(x) > 1e-9 for x in (vz, wx, wy)):
                raise ValueError("RV1 accepts planar Twist only")
            if not self.config["allow_lateral"] and abs(vy) > 1e-9:
                raise ValueError("Lateral navigation has not been enabled")
            if abs(vx) > self.config["max_linear_mps"] or abs(vy) > min(self.config["max_linear_mps"], 1.5) or abs(wz) > self.config["max_yaw_rps"]:
                raise ValueError("Twist exceeds configured navigation limits")
            self._target = [vx, vy, wz]
            self._last_command_wall = time.monotonic()
            self._received += 1
            self._reason = "receiving_twist"
        except (ValueError, TypeError, AttributeError, KeyError, OverflowError) as exc:
            self._rejected += 1
            self._error = str(exc)
            self.brake("invalid_twist")

    def brake(self, reason="requested_brake"):
        """Latch input disabled and clear targets; physical rest is measured later."""
        self._main_thread()
        self._armed = False
        self._rest_since = None
        self._rest_samples = 0
        self._generation += 1
        self._target = [0.0, 0.0, 0.0]
        self._last_command_wall = None
        self._reason = str(reason)
        self.controller.zero()
        return self.status()

    @staticmethod
    def _stamp(stamp, seconds):
        total = int(round(seconds * 1_000_000_000))
        stamp.sec, stamp.nanosec = divmod(total, 1_000_000_000)

    @staticmethod
    def _xyz(message, vector):
        message.x, message.y, message.z = vector

    def _transform(self, parent, child, position, quaternion, seconds):
        message = self._types["TransformStamped"]()
        message.header.frame_id, message.child_frame_id = parent, child
        self._stamp(message.header.stamp, seconds)
        self._xyz(message.transform.translation, position)
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = quaternion
        return message

    def _joint_state(self, sim_time):
        sim = self.controller._simulation_manager.get_physics_simulation_view()
        if sim is None:
            raise RuntimeError("RV1 articulation physics view is unavailable")
        if sim is not self._joint_sim or self._articulation is None:
            root = self.controller.chassis_path.rsplit("/", 1)[0]
            view = sim.create_articulation_view(root)
            if view.count != 1:
                raise RuntimeError("RV1 feedback requires exactly one articulation")
            source_names = list(view.shared_metatype.dof_names)
            if not source_names:
                raise RuntimeError("RV1 articulation has no measured joints")
            unknown = set(self.config["joint_name_map"]) - set(source_names)
            if unknown:
                raise ValueError("joint_name_map contains unknown DOFs: " + ", ".join(sorted(unknown)))
            self._joint_names = [self.config["joint_name_map"].get(name, name) for name in source_names]
            if len(set(self._joint_names)) != len(self._joint_names):
                raise ValueError("Mapped joint names must be unique")
            self._joint_sim, self._articulation = sim, view
        n = len(self._joint_names)
        positions = self.controller._row(self._articulation.get_dof_positions(), n)
        velocities = self.controller._row(self._articulation.get_dof_velocities(), n)
        message = self._types["JointState"]()
        self._stamp(message.header.stamp, sim_time)
        message.name, message.position, message.velocity = list(self._joint_names), positions, velocities
        return message

    def _publish_feedback(self, sim_time):
        measured = self.controller.telemetry(True)
        if not measured.get("measurement_valid"):
            raise RuntimeError("RV1 physics feedback is invalid")
        base = measured_base_state(measured, self.config["chassis_from_base_xyz_xyzw"])
        joints = self._joint_state(sim_time)  # Do not publish a partial healthy snapshot.
        odom = self._types["Odometry"]()
        self._stamp(odom.header.stamp, sim_time)
        odom.header.frame_id, odom.child_frame_id = self.config["odom_frame"], self.config["base_frame"]
        self._xyz(odom.pose.pose.position, base["position_m"])
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = base["orientation_xyzw"]
        self._xyz(odom.twist.twist.linear, base["linear_velocity_body_mps"])
        self._xyz(odom.twist.twist.angular, base["angular_velocity_body_rps"])
        # This is simulation truth, not an estimated wheel-encoder covariance.
        tf = self._types["TFMessage"]()
        tf.transforms = [self._transform(self.config["odom_frame"], self.config["base_frame"],
                                         base["position_m"], base["orientation_xyzw"], sim_time)]
        if self.config["localization_mode"] == "sim_truth":
            x, y, z, yaw = self.config["map_from_stage_xyz_yaw"]
            tf.transforms.insert(0, self._transform(self.config["map_frame"], self.config["odom_frame"],
                                                    [x, y, z], [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)], sim_time))
        self._publishers["odom"].publish(odom)
        self._publishers["joints"].publish(joints)
        self._publishers["tf"].publish(tf)
        self._last_measurement, self._last_base = measured, base
        speed = _finite(measured.get("speed_mps"), "measured speed")
        yaw_rate = _finite(measured.get("yaw_rate_rps"), "measured yaw rate")
        if self._last_publish is not None and sim_time - self._last_publish > 2.0 / self.config["publish_hz"] + 1e-9:
            self._rest_since, self._rest_samples = None, 0
        if speed < 0.05 and abs(yaw_rate) < 0.05:
            if self._rest_since is None:
                self._rest_since = sim_time
            self._rest_samples += 1
        else:
            self._rest_since, self._rest_samples = None, 0
        self._last_publish = sim_time
        self._published += 1

    def update(self, sim_time, *, playing=True):
        self._main_thread()
        if not self._started:
            return self.status()
        sim_time = _finite(sim_time, "physics simulation time")
        if sim_time < 0 or not isinstance(playing, bool):
            raise ValueError("Physics time must be nonnegative and playing must be boolean")
        previous = self._sim_time
        dt = 0.0 if previous is None else sim_time - previous
        reset = previous is not None and dt < -1e-9
        self._sim_time = sim_time
        try:
            if reset:
                self._rest_since, self._rest_samples = None, 0
                if self._armed:
                    self.brake("physics_time_reset")
                else:
                    self._reason = "physics_time_reset"
                self._last_publish = None
                self._last_measurement = self._last_base = None
                self._joint_sim = self._articulation = None
            if not playing and self._armed:
                self.brake("simulation_not_playing")
            # Executor callbacks only cache commands on this same owning thread.
            for _ in range(8):
                self._executor.spin_once(timeout_sec=0.0)
            clock = self._types["Clock"]()
            self._stamp(clock.clock, sim_time)
            self._publishers["clock"].publish(clock)
            self._clock_published += 1
            if self._armed:
                reference = self._last_command_wall if self._last_command_wall is not None else self._arm_wall
                if time.monotonic() - reference > self.config["command_timeout_seconds"]:
                    self.brake("command_watchdog_expired")
                elif playing and dt > 0:
                    self.controller.apply(*self._target, dt=min(dt, 0.1))
            if playing and (self._last_publish is None or sim_time - self._last_publish >= 1.0 / self.config["publish_hz"] - 1e-9):
                self._publish_feedback(sim_time)
            elif not playing:
                self._last_measurement = self._last_base = None
                self._rest_since, self._rest_samples = None, 0
            return self.status()
        except Exception as exc:
            self._rest_since, self._rest_samples = None, 0
            self._error = str(exc)
            if self._armed:
                self.brake("navigation_update_failed")
            else:
                self._reason = "navigation_update_failed"
            raise

    def status(self):
        now = time.monotonic()
        age = None if self._last_command_wall is None else max(0.0, now - self._last_command_wall)
        rest_duration = (max(0.0, self._last_publish - self._rest_since)
                         if self._rest_since is not None and self._last_publish is not None else 0.0)
        return {"started": self._started, "armed": self._armed, "reason": self._reason, "error": self._error,
                "simulation_time": self._sim_time, "command_age_wall_seconds": age,
                "command_fresh": self._armed and age is not None and age <= self.config["command_timeout_seconds"],
                "target_twist": list(self._target), "received_twists": self._received, "rejected_twists": self._rejected,
                "feedback_publications": self._published, "clock_publications": self._clock_published,
                "last_feedback_sim_time": self._last_publish, "joint_names": list(self._joint_names),
                "rest_verified": bool(self._last_measurement and self._rest_samples >= 2 and rest_duration >= 0.25 - 1e-9),
                "rest_duration_sim_seconds": rest_duration, "rest_samples": self._rest_samples,
                "localization_mode": self.config["localization_mode"], "odometry_source": "simulated_chassis_physics_truth",
                "command_timestamp_source": "DDS source_timestamp; publishers require synchronized system clocks",
                "odom_frame_definition": "USD stage world in metres and Z-up; no spawn subtraction",
                "base_state": copy.deepcopy(self._last_base), "config": copy.deepcopy(self.config),
                "measured_speed_mps": self._last_measurement.get("speed_mps") if self._last_measurement else None,
                "measured_yaw_rate_rps": self._last_measurement.get("yaw_rate_rps") if self._last_measurement else None}

    def close(self):
        self._main_thread()
        errors = []
        try:
            self.brake("closed")
        except Exception as exc:
            errors.append(str(exc))
        for resource, operation in ((self._executor, "shutdown"), (self._node, "destroy_node"), (self._context, "try_shutdown")):
            if resource is not None:
                try:
                    getattr(resource, operation)()
                except Exception as exc:
                    errors.append(str(exc))
        self._executor = self._node = self._context = self._subscription = None
        self._publishers.clear()
        self._started = False
        self._last_measurement = self._last_base = None
        self._rest_since, self._rest_samples = None, 0
        self._joint_sim = self._articulation = None
        if errors:
            self._error = "; ".join(errors)
        return self.status()
