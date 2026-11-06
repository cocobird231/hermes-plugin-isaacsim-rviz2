"""Control ownership, stale-command rejection and measured ROS frame contracts."""
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rv1 import navigation
from rv1.navigation import RV1Navigation, measured_base_state, validate_config


def xyz():
    return NS(x=0.0, y=0.0, z=0.0)


def header():
    return NS(stamp=NS(sec=0, nanosec=0), frame_id="")


class Twist:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.linear, self.angular = xyz(), xyz()
        self.linear.x, self.linear.y, self.angular.z = x, y, yaw


class TransformStamped:
    def __init__(self):
        self.header, self.child_frame_id = header(), ""
        self.transform = NS(translation=xyz(), rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


class Odometry:
    def __init__(self):
        self.header, self.child_frame_id = header(), ""
        self.pose = NS(pose=NS(position=xyz(), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.twist = NS(twist=Twist())


class JointState:
    def __init__(self):
        self.header = header()
        self.name, self.position, self.velocity = [], [], []


class Clock:
    def __init__(self):
        self.clock = NS(sec=0, nanosec=0)


class TFMessage:
    def __init__(self):
        self.transforms = []


class Context:
    def __init__(self):
        self.stopped = False

    def try_shutdown(self):
        self.stopped = True


class Publisher:
    def __init__(self, topic):
        self.topic, self.messages = topic, []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self, name, **kwargs):
        self.name, self.kwargs = name, kwargs
        self.destroyed_subs = []
        self.destroyed = False

    def create_publisher(self, msg, topic, qos):
        return Publisher(topic)

    def create_subscription(self, msg, topic, callback, qos):
        return NS(callback=callback, topic=topic, qos=qos)

    def destroy_subscription(self, subscription):
        self.destroyed_subs.append(subscription)

    def destroy_node(self):
        self.destroyed = True


class Executor:
    def __init__(self, context):
        self.context, self.stopped = context, False
        self.callbacks = []

    def add_node(self, node):
        self.node = node

    def spin_once(self, timeout_sec):
        if self.callbacks:
            self.callbacks.pop(0)()

    def shutdown(self):
        self.stopped = True


class Controller:
    def __init__(self):
        self.calls = []
        self.chassis_path = "/World/AgentExperiment/RV1/rv1/base_link/chassis_link"
        self.measured = dict(measurement_valid=True, position_m=[160.0, -155.0, 0.87],
                             orientation_xyzw=[0, 0, 0, 1], linear_velocity_world_mps=[0.4, 0.0, 0.0],
                             angular_velocity_world_rps=[0.0, 0.0, 0.2], speed_mps=0.4, yaw_rate_rps=0.2)
        self.articulation = NS(count=1, shared_metatype=NS(dof_names=["shock_0", "steer_0", "drive_0"]),
                               get_dof_positions=lambda: [[0.013, 0.25, 2.4]],
                               get_dof_velocities=lambda: [[0.002, -0.1, 1.2]])
        self.bound_roots = []
        self.sim = NS(create_articulation_view=self.bind)
        self._simulation_manager = NS(get_physics_simulation_view=lambda: self.sim)

    def bind(self, path):
        self.bound_roots.append(path)
        return self.articulation

    def zero(self):
        self.calls.append(("zero",))

    def apply(self, *args, **kwargs):
        self.calls.append(("apply", args, kwargs))

    def telemetry(self, playing):
        assert playing
        return self.measured

    @staticmethod
    def _row(value, n):
        assert len(value[0]) == n
        return value[0]


@pytest.fixture
def fake_ros(monkeypatch):
    def module(name, **members):
        value = ModuleType(name)
        value.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, value)
        return value
    inits = []
    module("rclpy", init=lambda **kwargs: inits.append(kwargs))
    module("rclpy.context", Context=Context)
    module("rclpy.executors", SingleThreadedExecutor=Executor)
    module("rclpy.node", Node=Node)
    module("rclpy.qos", QoSProfile=lambda **kw: NS(**kw),
           ReliabilityPolicy=NS(RELIABLE=1), DurabilityPolicy=NS(VOLATILE=0, TRANSIENT_LOCAL=1),
           HistoryPolicy=NS(KEEP_LAST=1))
    for package, members in (("geometry_msgs", dict(Twist=Twist, TransformStamped=TransformStamped)),
                              ("nav_msgs", dict(Odometry=Odometry)), ("sensor_msgs", dict(JointState=JointState)),
                              ("rosgraph_msgs", dict(Clock=Clock)), ("tf2_msgs", dict(TFMessage=TFMessage))):
        module(package)
        module(package + ".msg", **members)
    return inits


@pytest.fixture
def clock(monkeypatch):
    current = NS(wall=100.0, epoch_ns=1_800_000_000_000_000_000)
    monkeypatch.setattr(navigation.time, "monotonic", lambda: current.wall)
    monkeypatch.setattr(navigation.time, "time_ns", lambda: current.epoch_ns)
    return current


@pytest.fixture
def nav(fake_ros, clock):
    item = RV1Navigation(Controller())
    item.start()
    return item


def receive(nav, clock, twist=None):
    clock.epoch_ns += 1_000_000
    nav._subscription.callback(twist or Twist(0.5), NS(source_timestamp=clock.epoch_ns))


def test_start_is_disarmed_and_has_private_ros_context(nav, fake_ros):
    assert not nav.status()["armed"]
    assert nav.controller.calls == []
    assert fake_ros == [{"context": nav._context}]
    assert nav._node.kwargs["use_global_arguments"] is False
    nav.update(0.0, playing=False)
    nav.update(0.2)
    assert nav.controller.calls == []


def test_safety_brake_latches_and_disabled_updates_do_not_write_targets(nav, clock):
    nav.arm()
    receive(nav, clock)
    nav.update(1.0)
    nav.update(1.1)
    assert nav.controller.calls[-1][0] == "apply"
    nav.brake()
    nav.controller.calls.clear()
    receive(nav, clock, Twist(0.8))
    nav.update(1.2)
    nav.update(1.3)
    assert nav.controller.calls == []
    assert nav.status()["target_twist"] == [0, 0, 0]


def test_rearm_replaces_subscriber_and_rejects_old_callback_or_prearm_sample(nav, clock):
    nav.arm()
    old = nav._subscription
    nav.brake()
    nav.arm()
    assert old in nav._node.destroyed_subs
    old.callback(Twist(0.8), NS(source_timestamp=clock.epoch_ns + 1))
    nav._subscription.callback(Twist(0.8), NS(source_timestamp=clock.epoch_ns - 1))
    assert nav.status()["received_twists"] == 0
    receive(nav, clock, Twist(0.3))
    assert nav.status()["received_twists"] == 1
    assert nav.status()["target_twist"] == [0.3, 0, 0]


def test_watchdog_uses_wall_time_with_frozen_physics_and_requires_rearm(nav, clock):
    nav.arm()
    receive(nav, clock)
    nav.update(1.0)
    clock.wall += 3.01
    nav.update(1.0)
    assert not nav.status()["armed"]
    assert nav.status()["reason"] == "command_watchdog_expired"
    receive(nav, clock)
    assert nav.status()["target_twist"] == [0, 0, 0]
    assert nav.controller.calls[-1] == ("zero",)


@pytest.mark.parametrize("event", ["pause", "reset"])
def test_pause_and_time_reset_latch_commands_and_never_replay(nav, clock, event):
    nav.arm()
    receive(nav, clock)
    nav.update(1.0)
    nav.update(0.0 if event == "reset" else 1.0, playing=event != "pause")
    assert not nav.status()["armed"]
    nav.controller.calls.clear()
    nav.update(2.0)
    assert nav.controller.calls == []


@pytest.mark.parametrize("twist", [Twist(float("nan")), Twist(1.01), Twist(0, 0.1), Twist(0, 0, 0.51)])
def test_invalid_or_out_of_profile_commands_stop_instead_of_clipping(nav, clock, twist):
    nav.arm()
    receive(nav, clock, twist)
    assert nav.status()["reason"] == "invalid_twist"
    assert nav.status()["rejected_twists"] == 1
    assert not nav.status()["armed"]


def test_old_dds_sample_does_not_refresh_watchdog(nav, clock):
    nav.arm()
    source = clock.epoch_ns + 1
    clock.epoch_ns += 4_000_000_000
    nav._subscription.callback(Twist(0.5), NS(source_timestamp=source))
    assert nav.status()["received_twists"] == 0
    assert nav.status()["command_age_wall_seconds"] is None


def test_jazzy_message_info_dictionary_is_supported(nav, clock):
    nav.arm()
    clock.epoch_ns += 1_000_000
    nav._subscription.callback(Twist(0.3), {"source_timestamp": clock.epoch_ns})
    assert nav.status()["received_twists"] == 1
    assert nav.status()["target_twist"] == [0.3, 0, 0]


def test_failed_rearm_leaves_old_commands_disabled(nav, clock):
    nav.arm()
    receive(nav, clock)
    old = nav._subscription
    def fail(*args):
        raise RuntimeError("DDS subscription failed")
    nav._node.create_subscription = fail
    with pytest.raises(RuntimeError, match="DDS subscription failed"):
        nav.arm()
    old.callback(Twist(0.8), NS(source_timestamp=clock.epoch_ns + 1))
    assert not nav.status()["armed"]
    assert nav.status()["target_twist"] == [0, 0, 0]
    assert nav.controller.calls[-1] == ("zero",)


def test_feedback_uses_true_joint_measurements_and_stage_world_pose(nav):
    nav.update(10.25)
    odom = nav._publishers["odom"].messages[-1]
    assert odom.header.frame_id == "odom" and odom.child_frame_id == "base_link"
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == (160, -155)
    assert (odom.header.stamp.sec, odom.header.stamp.nanosec) == (10, 250_000_000)
    joints = nav._publishers["joints"].messages[-1]
    assert joints.name == ["shock_0", "steer_0", "drive_0"]
    assert joints.position == [0.013, 0.25, 2.4]
    assert joints.velocity == [0.002, -0.1, 1.2]
    assert nav.controller.bound_roots == ["/World/AgentExperiment/RV1/rv1/base_link"]
    assert nav.status()["odometry_source"] == "simulated_chassis_physics_truth"


def test_sim_truth_transform_is_explicit_map_alignment(fake_ros, clock):
    nav = RV1Navigation(Controller(), {"map_from_stage_xyz_yaw": [2, 3, 0, math.pi/2]})
    nav.start()
    nav.update(0.1)
    tf = nav._publishers["tf"].messages[-1].transforms
    assert [(t.header.frame_id, t.child_frame_id) for t in tf] == [("map", "odom"), ("odom", "base_link")]
    assert (tf[0].transform.translation.x, tf[0].transform.translation.y) == (2, 3)
    assert tf[0].transform.rotation.z == pytest.approx(math.sqrt(0.5))


def test_external_localization_never_publishes_map_odom(fake_ros, clock):
    nav = RV1Navigation(Controller(), {"localization_mode": "external"})
    nav.start()
    nav.update(0.1)
    assert [t.header.frame_id for t in nav._publishers["tf"].messages[-1].transforms] == ["odom"]


def test_rigid_base_offset_transforms_pose_and_twist_at_the_correct_point():
    t = Controller().measured
    t.update(position_m=[3, 4, 0], orientation_xyzw=[0, 0, math.sqrt(0.5), math.sqrt(0.5)],
             linear_velocity_world_mps=[0, 1, 0], angular_velocity_world_rps=[0, 0, 2])
    result = measured_base_state(t, [1, 0, 0, 0, 0, 0, 1])
    assert result["position_m"] == pytest.approx([3, 5, 0])
    assert result["linear_velocity_body_mps"] == pytest.approx([1, 2, 0])
    assert result["angular_velocity_body_rps"] == pytest.approx([0, 0, 2])


def test_measurement_failure_while_disarmed_does_not_write_actuators(nav):
    nav.controller.measured["measurement_valid"] = False
    with pytest.raises(RuntimeError, match="feedback is invalid"):
        nav.update(0.1)
    assert nav.controller.calls == []


def test_measurement_failure_while_armed_clears_velocity(nav, clock):
    nav.arm()
    receive(nav, clock)
    nav.controller.measured["measurement_valid"] = False
    with pytest.raises(RuntimeError, match="feedback is invalid"):
        nav.update(0.1)
    assert not nav.status()["armed"]
    assert nav.controller.calls[-1] == ("zero",)


def test_close_only_shuts_down_owned_ros_context_and_clears_targets(nav):
    context, node, executor = nav._context, nav._node, nav._executor
    nav.close()
    assert context.stopped and node.destroyed and executor.stopped
    assert not nav.status()["started"]
    assert nav.controller.calls[-1] == ("zero",)


@pytest.mark.parametrize("config", [
    {"max_yaw_rps": 0.8}, {"max_linear_mps": 2.6}, {"command_timeout_seconds": float("inf")},
    {"command_timeout_seconds": 11}, {"base_frame": "odom"}, {"cmd_vel_topic": "/odom"},
    {"allow_lateral": "true"}, {"chassis_from_base_xyz_xyzw": [0] * 7},
    {"static_transforms": [{"parent_frame": "base_link", "child_frame": "base_link", "translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1]}]},
])
def test_invalid_configuration_rejected_before_ros_or_actuator_access(config):
    with pytest.raises(ValueError):
        validate_config(config)


def test_joint_map_cannot_alias_unmapped_joint(nav):
    nav.config["joint_name_map"] = {"shock_0": "steer_0"}
    with pytest.raises(ValueError, match="unique"):
        nav.update(0.1)
    assert not nav._publishers["joints"].messages


def test_feedback_rate_uses_sim_time_but_clock_is_always_published(nav):
    for t in (0.0, 0.01, 0.02, 0.05):
        nav.update(t)
    assert nav.status()["feedback_publications"] == 2
    assert nav.status()["clock_publications"] == 4


def test_rest_requires_measured_low_speed_for_quarter_sim_second(nav):
    nav.controller.measured.update(speed_mps=0.0, yaw_rate_rps=0.0)
    nav.update(0.0)
    assert not nav.status()["rest_verified"]
    for i in range(1,6):
        nav.update(i * .05)
    assert nav.status()["rest_verified"]
    assert nav.status()["rest_samples"] == 6
    nav.controller.measured["yaw_rate_rps"] = .06
    nav.update(.30)
    assert not nav.status()["rest_verified"]
    nav.controller.measured["yaw_rate_rps"] = 0.0
    for i in range(7,13):
        nav.update(i * .05)
    assert nav.status()["rest_verified"]
    nav.brake("cancel")
    assert not nav.status()["rest_verified"]
    for i in range(13,19):
        nav.update(i * .05)
    assert nav.status()["rest_verified"]
    nav.update(.95, playing=False)
    assert not nav.status()["rest_verified"]


def test_sparse_stationary_samples_cannot_prove_continuous_rest(nav):
    nav.controller.measured.update(speed_mps=0.0, yaw_rate_rps=0.0)
    nav.update(0.0)
    nav.update(1.0)
    assert not nav.status()["rest_verified"]
    assert nav.status()["rest_samples"] == 1
