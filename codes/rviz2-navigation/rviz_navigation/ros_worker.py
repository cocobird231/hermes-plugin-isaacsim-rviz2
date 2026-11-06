#!/usr/bin/env python3
"""Owned ROS 2 processes and a local JSON-line RPC for static-map navigation.

Run under ROS 2 Jazzy Python, independently of Isaac. The selected robot
profile provides frames, topics, clock and Nav2 parameters. Initial pose is
verified against existing localization TF; this worker never teleports a robot.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socketserver
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

try:
    from .profiles import load_profile, validate_profile, nav2_parameters, navigation_ids
except ImportError:  # Executed directly by the independent plugin supervisor.
    from profiles import load_profile, validate_profile, nav2_parameters, navigation_ids

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock as WallClock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


FIELDS = {
    "map_load": {"map_yaml_path"}, "model_load": {"urdf_path"},
    "open_rviz": {"display_number"}, "close_rviz": set(), "start": {"params_file"},
    "set_initial_pose": {"x_m", "y_m", "yaw_rad"},
    "set_goal": {"x_m", "y_m", "yaw_rad"}, "plan": {"goal_id"},
    "navigation_ready": {"goal_id"}, "navigate": {"goal_id"},
    "cancel": set(), "status": set(), "stop": set(),
}
NAV_NODES = [
    ("nav2_controller", "controller_server", "controller_server"),
    ("nav2_planner", "planner_server", "planner_server"),
    ("nav2_behaviors", "behavior_server", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
    ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother"),
]


def fail(code, message, **extra):
    return {"success": False, "error": {"code": code, "message": str(message)}, **extra}


def save(path, data):
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ticks(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None
    except (OSError, IndexError):
        return None


def wait_future(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done():
        raise TimeoutError("ROS response exceeded the wall-clock timeout")
    return future.result()


def footprint_from_params(content):
    profiles = []
    for name in ("global_costmap", "local_costmap"):
        params = content[name][name]["ros__parameters"]
        polygon = params["footprint"]
        if isinstance(polygon, str):
            polygon = json.loads(polygon)
        if len(polygon) < 3 or any(len(point) != 2 for point in polygon):
            raise ValueError("A footprint polygon needs at least three XY vertices")
        polygon = [[finite(x, "footprint x"), finite(y, "footprint y")] for x, y in polygon]
        padding = finite(params.get("footprint_padding", 0.0), "footprint_padding")
        if padding < 0:
            raise ValueError("footprint_padding cannot be negative")
        # Match Nav2's padFootprint: pad each coordinate away from base_link.
        profiles.append([[x + math.copysign(padding, x) if x else x,
                          y + math.copysign(padding, y) if y else y] for x, y in polygon])
    if profiles[0] != profiles[1]:
        raise ValueError("This robot profile requires identical global/local footprints and padding")
    return profiles[0]


def footprint_cells_free(grid, pose, footprint):
    """Conservatively test every grid cell touched by a rotated robot polygon."""
    info = grid.info
    map_yaw = yaw(info.origin.orientation)
    c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
    mc, ms = math.cos(map_yaw), math.sin(map_yaw)
    vertices = []
    for x, y in footprint:
        dx = pose["x_m"] + c * x - s * y - info.origin.position.x
        dy = pose["y_m"] + s * x + c * y - info.origin.position.y
        vertices.append(((mc * dx + ms * dy) / info.resolution,
                         (-ms * dx + mc * dy) / info.resolution))
    min_x, max_x = math.floor(min(v[0] for v in vertices) - 1e-9), math.floor(max(v[0] for v in vertices) + 1e-9)
    min_y, max_y = math.floor(min(v[1] for v in vertices) - 1e-9), math.floor(max(v[1] for v in vertices) + 1e-9)
    if (max_x - min_x + 1) * (max_y - min_y + 1) > 100_000:
        raise ValueError("Footprint grid check exceeds 100000 candidate cells")

    def inside(point):
        px, py = point
        result = False
        for a, b in zip(vertices, vertices[1:] + vertices[:1]):
            if (a[1] > py) != (b[1] > py) and px < (b[0] - a[0]) * (py - a[1]) / (b[1] - a[1]) + a[0]:
                result = not result
        return result

    def edge_box(a, b, x, y):
        # Liang-Barsky segment clipping, including contact at cell boundaries.
        dx, dy = b[0] - a[0], b[1] - a[1]
        low, high = 0.0, 1.0
        for p, q in ((-dx, a[0] - x), (dx, x + 1 - a[0]),
                     (-dy, a[1] - y), (dy, y + 1 - a[1])):
            if abs(p) < 1e-12:
                if q < 0:
                    return False
            elif p < 0:
                low = max(low, q / p)
            else:
                high = min(high, q / p)
            if low > high:
                return False
        return True

    checked = 0
    edges = list(zip(vertices, vertices[1:] + vertices[:1]))
    for row in range(min_y, max_y + 1):
        for col in range(min_x, max_x + 1):
            touches = (any(col <= x <= col + 1 and row <= y <= row + 1 for x, y in vertices)
                       or any(inside(point) for point in ((col, row), (col + 1, row), (col, row + 1), (col + 1, row + 1)))
                       or any(edge_box(a, b, col, row) for a, b in edges))
            if not touches:
                continue
            if not 0 <= col < info.width or not 0 <= row < info.height:
                raise ValueError("Robot footprint extends outside the map")
            occupancy = grid.data[row * info.width + col]
            if occupancy != 0:
                raise ValueError(f"Robot footprint touches non-free map cell [{col},{row}] (occupancy={occupancy})")
            checked += 1
    return {"checked_cells": checked, "footprint_clear": True}


class Worker(Node):
    def __init__(self, run_dir, profile):
        self.profile = validate_profile(profile)
        self.robot_id = self.profile["robot_id"]
        self.frames, self.topics = self.profile["frames"], self.profile["topics"]
        self.private_topic = f"/rviz_navigation/{self.robot_id}"
        super().__init__("rviz_navigation_agent", parameter_overrides=[
            Parameter("use_sim_time", value=self.profile["use_sim_time"])],
            cli_args=["--ros-args", "-r", f"/clock:={self.topics['clock']}"])
        self.run_dir = run_dir
        parameters = nav2_parameters(self.profile)
        self.footprint = footprint_from_params(parameters)
        self.nav2_ids = navigation_ids(parameters)
        self.commands = threading.RLock()
        self.state_lock = threading.RLock()
        self.children = {}
        self.records = {}
        self.stopping = threading.Event()
        self.starting = False
        self.startup_cancel = threading.Event()
        self.startup_thread = None
        self.nav_handle = None
        self.nav_attempt_id = None
        self.nav_send_future = None
        self.nav_cancel_requested = False
        self.map_msg = None
        self.description_text = None
        self.static_frames = set()
        self.dynamic_frames = set()
        self.last_clock = self.last_odom = self.last_twist = 0.0
        self.clock_ns = 0
        self._relay_armed = False
        self._relay_subscription = None
        self._relay_generation = 0
        self._relay_last_command = 0.0
        self._relay_armed_at = 0.0
        self._relay_reason = "not_armed"
        self._rest_started_ns = None
        self._rest_samples = 0
        self.state = {"worker_running": True, "robot_id": self.robot_id, "profile": self.profile,
                      "localization_mode": self.profile["localization_mode"],
                      "obstacles_mode": "static_map_only", "navigation_ready": False,
                      "initial_pose_verified": False, "map": None, "model": None,
                      "goals": {}, "plan": None, "navigation": None}
        self._reclaim_owned()
        # Bind TF expiry/reset to this node's simulation clock. A default
        # Buffer watches wall time and retains the previous Isaac time epoch.
        self.tf = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.topics["map"], self._on_map, latched)
        self.create_subscription(String, self.topics["robot_description"], self._on_description, latched)
        self.create_subscription(TFMessage, "/tf_static", self._on_static_tf, latched)
        self.create_subscription(TFMessage, "/tf", self._on_dynamic_tf, 100)
        self.create_subscription(Clock, self.topics["clock"], self._on_clock, 10)
        self.create_subscription(Odometry, self.topics["odom"], self._on_odom, 10)
        self.create_subscription(Twist, self.topics["cmd_vel"], self._on_twist, 10)
        self.zero = self.create_publisher(Twist, self.topics["cmd_vel"], 10)
        self._relay_timer = self.create_timer(0.1, self._relay_watchdog,
                                             clock=WallClock(clock_type=ClockType.STEADY_TIME))
        self.paths = self.create_publisher(RosPath, self.private_topic + "/planned_path", latched)
        self.markers = self.create_publisher(Marker, self.private_topic + "/goal", latched)
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_states = {name: self.create_client(GetState, f"/{name}/get_state")
                                 for name in ["map_server", *[item[2] for item in NAV_NODES]]}

    def _on_map(self, msg):
        if msg.header.frame_id == self.frames["map"]:
            self.map_msg = msg

    def _on_description(self, msg):
        self.description_text = msg.data

    def _on_static_tf(self, msg):
        self.static_frames.update(transform.child_frame_id for transform in msg.transforms)

    def _on_dynamic_tf(self, msg):
        self.dynamic_frames.update(transform.child_frame_id for transform in msg.transforms)

    def _on_clock(self, msg):
        if not self.profile["use_sim_time"]:
            return
        now = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.clock_ns and now < self.clock_ns:
            # tf2's node-clock callback handles large backwards jumps. Also
            # cover sub-second resets and invalidate our own cached readiness.
            self.tf.clear()
            self._disarm_relay("clock_reset")
            self._request_guard_cancel()
            with self.state_lock:
                self.state.update(initial_pose_verified=False, navigation_ready=False,
                                  clock_reset_detected=True, goals={}, plan=None)
                for name in ("initial_pose", "odometry", "cmd_vel"):
                    self.state.pop(name, None)
                self.last_odom = self.last_twist = 0.0
                self._rest_started_ns = None
                self._rest_samples = 0
                self.dynamic_frames.clear()
        self.clock_ns, self.last_clock = now, time.monotonic()

    def _on_odom(self, msg):
        if msg.header.frame_id != self.frames["odom"] or msg.child_frame_id != self.frames["base"]:
            return  # A foreign robot's odometry must not satisfy this profile.
        q = msg.pose.pose.orientation
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                  q.x, q.y, q.z, q.w, msg.twist.twist.linear.x,
                  msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        if not all(math.isfinite(value) for value in values):
            self._disarm_relay("invalid_odometry")
            self._request_guard_cancel()
            self.last_odom = 0.0
            self._rest_started_ns = None
            self._rest_samples = 0
            return
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) < 0.03 and abs(msg.twist.twist.angular.z) < 0.03:
            if self._rest_started_ns is None or stamp < self._rest_started_ns:
                self._rest_started_ns = stamp
                self._rest_samples = 0
            self._rest_samples += 1
        else:
            self._rest_started_ns = None
            self._rest_samples = 0
        self.last_odom = now
        self._rest_stamp_ns = stamp
        with self.state_lock:
            self.state["odometry"] = {"frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id, "x_m": msg.pose.pose.position.x,
                "y_m": msg.pose.pose.position.y, "yaw_rad": yaw(msg.pose.pose.orientation),
                "vx_m_s": msg.twist.twist.linear.x, "wz_rad_s": msg.twist.twist.angular.z}

    def _on_twist(self, msg):
        if not all(math.isfinite(value) for value in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self._disarm_relay("invalid_observed_velocity")
            self._request_guard_cancel()
            return
        self.last_twist = time.monotonic()
        with self.state_lock:
            self.state["cmd_vel"] = {"type": "geometry_msgs/msg/Twist",
                                     "vx_m_s": msg.linear.x, "wz_rad_s": msg.angular.z}

    def _arm_relay(self):
        """Create a fresh volatile subscription epoch; no retained driving input."""
        self._disarm_relay("new_navigation_epoch")
        with self.state_lock:
            self._relay_generation += 1
            generation = self._relay_generation
            self._relay_armed_at = time.monotonic()
            self._relay_last_command = 0.0
            self._relay_subscription = self.create_subscription(
                Twist, self.private_topic + "/cmd_vel_smoothed",
                lambda msg: self._relay_twist(msg, generation), 10)
            self._relay_armed = True
            self._relay_reason = "waiting_for_nav2"

    def _disarm_relay(self, reason):
        with self.state_lock:
            self._relay_armed = False
            self._relay_reason = reason
            self._relay_generation += 1
            subscription, self._relay_subscription = self._relay_subscription, None
            if subscription is not None:
                self.destroy_subscription(subscription)
            # Publication and arm checks share one lock, preventing a queued
            # command from being sent after cancel's final zero publication.
            self.zero.publish(Twist())

    def _relay_twist(self, msg, generation):
        with self.state_lock:
            if not self._relay_armed or generation != self._relay_generation:
                return
            try:
                self.pose()
            except Exception:
                self._disarm_relay("localization_feedback_not_fresh")
                self._request_guard_cancel()
                return
            components = [msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z]
            limits = self.profile["limits"]
            invalid = (not all(math.isfinite(value) for value in components)
                       or any(abs(value) > 1e-8 for value in (msg.linear.z, msg.angular.x, msg.angular.y))
                       or (not self.profile["allow_lateral"] and abs(msg.linear.y) > 1e-8)
                       or math.hypot(msg.linear.x, msg.linear.y) > limits["max_linear_mps"] + 1e-8
                       or abs(msg.angular.z) > limits["max_yaw_rps"] + 1e-8)
            if invalid:
                self._disarm_relay("invalid_nav2_velocity")
                self._request_guard_cancel()
                return
            self._relay_last_command = time.monotonic()
            self._relay_reason = "forwarding_nav2"
            self.zero.publish(msg)

    def _request_guard_cancel(self):
        """Called on the ROS executor: never wait for an action response here."""
        self.nav_cancel_requested = True
        if self.state.get("navigation"):
            self.state["navigation"].update(cancel_requested=True, guard_reason=self._relay_reason)
        if self.nav_handle is not None:
            try:
                self.nav_handle.cancel_goal_async()
            except Exception as exc:
                if self.state.get("navigation"):
                    self.state["navigation"]["cancel_error"] = str(exc)

    def _relay_watchdog(self):
        with self.state_lock:
            if not self._relay_armed:
                return
            now, limits = time.monotonic(), self.profile["limits"]
            reason = None
            if now - (self._relay_last_command or self._relay_armed_at) > limits["command_timeout_seconds"]:
                reason = "nav2_command_watchdog_expired"
            elif not self.last_odom or now - self.last_odom > limits["feedback_timeout_seconds"]:
                reason = "odometry_watchdog_expired"
            elif self.profile["use_sim_time"] and (not self.last_clock or now - self.last_clock > limits["feedback_timeout_seconds"]):
                reason = "clock_watchdog_expired"
            if reason:
                self._disarm_relay(reason)
                self._request_guard_cancel()

    def _reclaim_owned(self):
        path = self.run_dir / "owned-children.json"
        if not path.exists():
            return
        for record in json.loads(path.read_text()).values():
            pid = record.get("pid")
            if pid and record.get("start_ticks") == ticks(pid) and os.getpgid(pid) == pid:
                os.killpg(pid, signal.SIGTERM)
                end = time.monotonic() + 3
                while ticks(pid) and time.monotonic() < end:
                    time.sleep(0.05)
                if record.get("start_ticks") == ticks(pid):
                    os.killpg(pid, signal.SIGKILL)
        save(path, {})

    def spawn(self, package, executable, name, *, params=None, remaps=(), env=None, args=()):
        old = self.children.get(name)
        if old and old.poll() is None:
            return self.records[name]
        if name in self.get_node_names():
            raise RuntimeError(f"An unmanaged ROS node named {name} already exists")
        exe = Path(get_package_prefix(package)) / "lib" / package / executable
        command = [str(exe), *args, "--ros-args", "-r", f"__node:={name}", "-p",
                   "use_sim_time:=" + str(self.profile["use_sim_time"]).lower(),
                   "-r", f"/clock:={self.topics['clock']}"]
        if params:
            command.extend(["--params-file", str(params)])
        for source, destination in remaps:
            command.extend(["-r", f"{source}:={destination}"])
        output = self.run_dir / f"{name}.log"
        with output.open("ab", buffering=0) as stream:
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self.children[name] = child
        self.records[name] = {"pid": child.pid, "start_ticks": ticks(child.pid),
                              "command": command, "log_path": str(output)}
        save(self.run_dir / "owned-children.json", self.records)
        return self.records[name]

    def stop_child(self, name):
        child = self.children.pop(name, None)
        record = self.records.pop(name, None)
        if child and record and child.poll() is None and ticks(child.pid) == record["start_ticks"]:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2)
        save(self.run_dir / "owned-children.json", self.records)

    def _params(self, filename, content):
        path = self.run_dir / filename
        path.write_text(yaml.safe_dump(content, sort_keys=False))
        return path

    def lifecycle(self, name, transition):
        client = self.create_client(ChangeState, f"/{name}/change_state")
        try:
            if not client.wait_for_service(timeout_sec=4):
                raise TimeoutError(f"{name} lifecycle service is not available")
            request = ChangeState.Request()
            request.transition.id = transition
            if not wait_future(client.call_async(request), 6).success:
                raise RuntimeError(f"{name} rejected lifecycle transition {transition}")
        finally:
            self.destroy_client(client)

    def pose(self, *, require_fresh=True):
        timeout = self.profile["limits"]["feedback_timeout_seconds"]
        clock_stale = self.profile["use_sim_time"] and (not self.clock_ns or time.monotonic() - self.last_clock > timeout)
        if require_fresh and (clock_stale or not self.last_odom or time.monotonic() - self.last_odom > timeout):
            raise RuntimeError("Fresh configured clock and odometry are required for this robot")
        transform = self.tf.lookup_transform(self.frames["map"], self.frames["base"], Time())
        if require_fresh:
            stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
            now_ns = self.clock_ns if self.profile["use_sim_time"] else self.get_clock().now().nanoseconds
            if abs(now_ns - stamp) > 1_000_000_000:
                raise RuntimeError("Map-to-base TF is stale relative to the configured clock")
        return {"x_m": transform.transform.translation.x, "y_m": transform.transform.translation.y,
                "yaw_rad": yaw(transform.transform.rotation), "frame_id": self.frames["map"], "robot_id": self.robot_id}

    def pose_msg(self, values):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = self.frames["map"], self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = values["x_m"], values["y_m"]
        msg.pose.orientation.z = math.sin(values["yaw_rad"] / 2)
        msg.pose.orientation.w = math.cos(values["yaw_rad"] / 2)
        return msg

    def map_load(self, map_yaml_path):
        path = Path(map_yaml_path).expanduser().resolve(strict=True)
        content = yaml.safe_load(path.read_text())
        image = Path(content["image"])
        if not image.is_absolute():
            image = path.parent / image
        if not image.is_file() or finite(content["resolution"], "resolution") <= 0:
            raise ValueError("Map needs an existing image and positive resolution")
        if self.starting:
            raise RuntimeError("Wait for navigation startup before replacing the map")
        self._stop_navigation()
        self.stop_child("map_server")
        self.map_msg = None
        self.state.update(map=None, initial_pose_verified=False, goals={}, plan=None)
        params = self._params("map-server.yaml", {"map_server": {"ros__parameters": {
            "use_sim_time": self.profile["use_sim_time"], "yaml_filename": str(path),
            "frame_id": self.frames["map"], "topic_name": self.topics["map"]}}})
        self.spawn("nav2_map_server", "map_server", "map_server", params=params)
        self.lifecycle("map_server", Transition.TRANSITION_CONFIGURE)
        self.lifecycle("map_server", Transition.TRANSITION_ACTIVATE)
        deadline = time.monotonic() + 3
        while self.map_msg is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.map_msg is None:
            raise TimeoutError("Map server activated but no configured map message was received")
        self.state["map"] = {"yaml_path": str(path), "revision": uuid.uuid4().hex,
            "resolution_m": self.map_msg.info.resolution, "width": self.map_msg.info.width,
            "height": self.map_msg.info.height, "frame_id": self.map_msg.header.frame_id}
        return {"success": True, "map": self.state["map"]}

    def model_load(self, urdf_path):
        path = Path(urdf_path).expanduser().resolve(strict=True)
        text = path.read_text()
        root = ET.fromstring(text)
        if root.tag != "robot" or not root.findall("link"):
            raise ValueError("Expected a URDF robot with links")
        if self.frames["base"] not in {n.get("name") for n in root.findall("link")}:
            raise ValueError(f"URDF must contain the configured base frame {self.frames['base']}")
        self.stop_child("robot_state_publisher")
        self.description_text = None
        self.static_frames.clear()
        self.dynamic_frames.clear()
        expected_fixed = {joint.find("child").get("link") for joint in root.findall("joint")
                          if joint.get("type") == "fixed"}
        expected_dynamic = {joint.find("child").get("link") for joint in root.findall("joint")
                            if joint.get("type") != "fixed"}
        params = self._params("robot-state-publisher.yaml", {"robot_state_publisher": {"ros__parameters": {
            "use_sim_time": self.profile["use_sim_time"], "robot_description": text}}})
        self.spawn("robot_state_publisher", "robot_state_publisher", "robot_state_publisher", params=params,
                   remaps=[("joint_states", self.topics["joint_states"]),
                           ("robot_description", self.topics["robot_description"])])
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.children["robot_state_publisher"].poll() is not None:
                raise RuntimeError("robot_state_publisher exited; inspect its owned process log")
            if self.description_text == text and expected_fixed.issubset(self.static_frames):
                break
            time.sleep(0.05)
        else:
            raise TimeoutError("Robot description or required fixed TF frames were not received")
        self.state["model"] = {"urdf_path": str(path), "robot_name": root.get("name"),
                               "link_count": len(root.findall("link")),
                               "description_received": True, "fixed_tf_received": sorted(expected_fixed),
                               "expected_dynamic_tf": sorted(expected_dynamic),
                               "dynamic_tf_complete": expected_dynamic.issubset(self.dynamic_frames),
                               "visual_render_verified": False}
        self.state.update(initial_pose_verified=False, goals={}, plan=None)
        return {"success": True, "model": self.state["model"]}

    def open_rviz(self, display_number=None):
        if display_number is None:
            from deployment import load_config
            display_number = load_config()["display_number"]
        if isinstance(display_number, bool) or not isinstance(display_number, int) or not 0 <= display_number <= 99:
            raise ValueError("display_number must be an integer from 0 to 99")
        if "rviz2" in self.children and self.children["rviz2"].poll() is None:
            return {"success": True, "already_running": True, "process": self.records["rviz2"]}
        displays = [
            {"Class": "rviz_default_plugins/Map", "Name": "Map", "Enabled": True,
             "Topic": {"Value": self.topics["map"], "Durability Policy": "Transient Local"}},
            {"Class": "rviz_default_plugins/RobotModel", "Name": self.robot_id, "Enabled": True,
             "Description Source": "Topic", "Description Topic": {"Value": self.topics["robot_description"], "Durability Policy": "Transient Local"}},
            {"Class": "rviz_default_plugins/Path", "Name": "Plan preview", "Enabled": True,
             "Topic": {"Value": self.private_topic + "/planned_path", "Durability Policy": "Transient Local"}},
            {"Class": "rviz_default_plugins/Path", "Name": "Nav2 active path", "Enabled": True,
             "Color": "0; 170; 255",
             "Topic": {"Value": "/plan", "Durability Policy": "Volatile", "Reliability Policy": "Reliable"}},
            {"Class": "rviz_default_plugins/Marker", "Name": "Goal", "Enabled": True,
             "Topic": {"Value": self.private_topic + "/goal", "Durability Policy": "Transient Local"}},
            {"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": True},
        ]
        view = {"Class": "rviz_default_plugins/TopDownOrtho", "Scale": 20.0,
                "Target Frame": self.frames["base"]}
        if self.map_msg is not None:
            info = self.map_msg.info
            span = max(info.width, info.height) * info.resolution
            # The tested 940 px tiled RViz window has a 560 px canvas;
            # leave margin so the entire map remains visible beside Isaac.
            view.update({"Target Frame": self.frames["map"], "X": info.origin.position.x + info.width * info.resolution / 2,
                         "Y": info.origin.position.y + info.height * info.resolution / 2,
                         "Scale": min(20.0, 480.0 / max(span, 1.0))})
        config = self._params("navigation.rviz", {"Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"}],
            "Visualization Manager": {"Class": "", "Enabled": True, "Displays": displays,
                "Global Options": {"Fixed Frame": self.frames["map"], "Frame Rate": 15},
                "Tools": [{"Class": "rviz_default_plugins/Interact"}, {"Class": "rviz_default_plugins/MoveCamera"}],
                "Views": {"Current": view}}})
        env = os.environ.copy()
        env["DISPLAY"] = f":{display_number}"
        record = self.spawn("rviz2", "rviz2", "rviz2", env=env, args=["-d", str(config)])
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.children["rviz2"].poll() is not None:
                raise RuntimeError(f"RViz2 exited; inspect {record['log_path']}")
            if "rviz2" in self.get_node_names():
                return {"success": True, "display_number": display_number, "process": record,
                        "ros_node_ready": True, "visual_render_verified": False}
            time.sleep(0.05)
        raise TimeoutError("RViz2 process exists but its ROS node did not become visible")

    def _bt_file(self):
        # Avoid the default tree's optional smoothing/waypoint servers. Both
        # supplied actions support cancellation through the BT navigator.
        path = self.run_dir / "navigate.xml"
        content = '''<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
'''
        content = content.replace('planner_id="GridBased"', f'planner_id="{self.nav2_ids["planner"]}"')
        content = content.replace('controller_id="FollowPath"', f'controller_id="{self.nav2_ids["controller"]}"')
        content = content.replace('goal_checker_id="goal_checker"', f'goal_checker_id="{self.nav2_ids["goal_checker"]}"')
        path.write_text(content)
        return path

    def start(self, params_file=None):
        if self.starting:
            return {"success": True, "pending": True, "phase": "starting"}
        if self.state["navigation_ready"]:
            return {"success": True, "already_running": True}
        if not self.state["map"]:
            raise ValueError("Load a map before starting navigation")
        if not self.state["model"]:
            raise ValueError("Load the configured robot's URDF before starting navigation")
        self.pose()
        path = Path(params_file).expanduser().resolve(strict=True) if params_file else Path(self.profile["params_file"])
        if path != Path(self.profile["params_file"]):
            raise ValueError("Configure the robot profile while stopped to change its Nav2 parameters")
        content = nav2_parameters(self.profile)
        if not isinstance(content, dict):
            raise ValueError("Nav2 parameters must be a YAML mapping")
        self.footprint = footprint_from_params(content)
        self.nav2_ids = navigation_ids(content)
        for _, _, name in NAV_NODES:
            block = content.setdefault(name, {}).setdefault("ros__parameters", {})
            block["use_sim_time"] = self.profile["use_sim_time"]
            if name in {"controller_server", "behavior_server", "velocity_smoother"}:
                block["enable_stamped_cmd_vel"] = False
        content["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = str(self._bt_file())
        active = self._params("nav2-active.yaml", content)
        self.state["params_file"] = str(path)
        self.state["navigation_startup"] = {"phase": "starting"}
        # Spawn synchronously, then transition in the background. Thus stop can
        # join startup without a worker thread waiting for the command lock.
        self._stop_navigation()
        for package, executable, name in NAV_NODES:
            remaps = []
            if name in {"controller_server", "behavior_server"}:
                remaps = [("cmd_vel", self.private_topic + "/cmd_vel_nav")]
            elif name == "velocity_smoother":
                remaps = [("cmd_vel", self.private_topic + "/cmd_vel_nav"),
                          ("cmd_vel_smoothed", self.private_topic + "/cmd_vel_smoothed")]
            self.spawn(package, executable, name, params=active, remaps=remaps)
        self.starting = True
        self.startup_cancel.clear()
        self.startup_thread = threading.Thread(target=self._start_navigation, daemon=True)
        self.startup_thread.start()
        return {"success": True, "pending": True, "phase": "starting"}

    def _start_navigation(self):
        try:
            # Manual lifecycle transitions provide a bounded, observable startup
            # without requiring nav2_bringup or its simulation dependencies.
            for _, _, name in NAV_NODES:
                for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                    if self.stopping.is_set() or self.startup_cancel.is_set():
                        return
                    self.lifecycle(name, transition)
            if self.stopping.is_set() or self.startup_cancel.is_set():
                return
            with self.state_lock:
                self.state["navigation_ready"] = True
                self.state["navigation_startup"] = {"phase": "ready"}
        except Exception as exc:
            with self.state_lock:
                self.state["navigation_ready"] = False
                self.state["navigation_startup"] = {"phase": "failed", "error": str(exc)}
        finally:
            self.starting = False

    def set_initial_pose(self, x_m, y_m, yaw_rad):
        if self._navigation_busy():
            raise RuntimeError("Cancel navigation before changing initial-pose verification")
        if not self.state["map"]:
            raise ValueError("Load a map before verifying the initial pose")
        requested = {key: finite(value, key) for key, value in locals().copy().items() if key != "self"}
        actual = self.pose()
        distance = math.hypot(actual["x_m"] - requested["x_m"], actual["y_m"] - requested["y_m"])
        angle = abs(math.atan2(math.sin(actual["yaw_rad"] - requested["yaw_rad"]),
                               math.cos(actual["yaw_rad"] - requested["yaw_rad"])))
        if distance > 0.5 or angle > 0.35:
            return fail("INITIAL_POSE_MISMATCH", "Initial pose must match existing localization TF; this operation does not move the robot",
                        actual_pose=actual, position_error_m=distance, yaw_error_rad=angle)
        self._free_goal(actual)
        self.state["initial_pose_verified"] = True
        self.state["initial_pose"] = {"requested": requested, "actual": actual,
            "position_error_m": distance, "yaw_error_rad": angle, "mode": self.profile["localization_mode"],
            "robot_id": self.robot_id,
            "map_revision": self.state["map"]["revision"] if self.state["map"] else None}
        self.state.pop("clock_reset_detected", None)
        return {"success": True, **self.state["initial_pose"]}

    def _free_goal(self, goal):
        if not self.state["map"] or self.map_msg is None:
            raise ValueError("Load a map before creating a goal")
        return footprint_cells_free(self.map_msg, goal, self.footprint)

    def set_goal(self, x_m, y_m, yaw_rad):
        values = {"x_m": finite(x_m, "x_m"), "y_m": finite(y_m, "y_m"), "yaw_rad": finite(yaw_rad, "yaw_rad")}
        self._free_goal(self.pose())
        self._free_goal(values)
        goal_id = uuid.uuid4().hex
        goal = {**values, "frame_id": self.frames["map"], "robot_id": self.robot_id, "goal_id": goal_id, "map_revision": self.state["map"]["revision"]}
        self.state["goals"][goal_id] = goal
        marker = Marker()
        pose = self.pose_msg(goal)
        marker.header, marker.pose = pose.header, pose.pose
        marker.ns, marker.id, marker.type, marker.action = "navigation_goal", 0, Marker.ARROW, Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 0.8, 0.15, 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 1.0
        self.markers.publish(marker)
        return {"success": True, "goal": goal, "goal_id": goal_id}

    def _goal(self, goal_id):
        if not self.state["navigation_ready"]:
            raise RuntimeError("Navigation lifecycle nodes are not ready")
        if any(name not in self.children or self.children[name].poll() is not None for _, _, name in NAV_NODES):
            raise RuntimeError("An owned navigation process has exited; inspect status/logs")
        if not self.state["initial_pose_verified"]:
            raise RuntimeError("Verify the initial pose before planning or navigating")
        self._free_goal(self.pose())
        goal = self.state["goals"].get(goal_id)
        if not goal or goal.get("robot_id") != self.robot_id or goal["map_revision"] != self.state["map"]["revision"]:
            raise ValueError("Unknown goal or goal belongs to a different map")
        self._free_goal(goal)
        return goal

    def plan(self, goal_id):
        goal = self._goal(goal_id)
        if self._navigation_busy():
            raise RuntimeError("Cancel active navigation before planning without movement")
        if not self.planner.wait_for_server(timeout_sec=2):
            raise TimeoutError("compute_path_to_pose action is not available")
        request = ComputePathToPose.Goal()
        request.goal, request.planner_id, request.use_start = self.pose_msg(goal), self.nav2_ids["planner"], False
        handle = wait_future(self.planner.send_goal_async(request), 3)
        if not handle.accepted:
            return fail("PLAN_REJECTED", "Planner rejected the goal")
        try:
            result = wait_future(handle.get_result_async(), 15)
        except TimeoutError:
            handle.cancel_goal_async()
            raise
        code = getattr(result.result, "error_code", 0)
        if result.status != 4 or code or not result.result.path.poses:
            return fail("PLAN_FAILED", getattr(result.result, "error_msg", "No valid path"), action_status=result.status, error_code=code)
        path = result.result.path
        self.paths.publish(path)
        path_id = uuid.uuid4().hex
        output = self.run_dir / f"path-{path_id}.json"
        save(output, message_to_ordereddict(path))
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
                     for a, b in zip(path.poses, path.poses[1:]))
        self.state["plan"] = {"path_id": path_id, "goal_id": goal_id, "map_revision": goal["map_revision"],
            "robot_id": self.robot_id, "pose_count": len(path.poses), "length_m": length,
            "path_file": str(output), "motion_requested": False}
        return {"success": True, **self.state["plan"]}

    def navigate(self, goal_id):
        ready = self.navigation_ready(goal_id)
        if not ready["success"]:
            return ready
        goal = self._goal(goal_id)
        if self._navigation_busy():
            raise RuntimeError("Cancel the current navigation before issuing another goal")
        if not self.navigator.wait_for_server(timeout_sec=2):
            raise TimeoutError("navigate_to_pose action is not available")
        request = NavigateToPose.Goal()
        request.pose = self.pose_msg(goal)
        request.behavior_tree = str(self.run_dir / "navigate.xml")
        attempt_id = uuid.uuid4().hex
        with self.state_lock:
            self.nav_attempt_id = attempt_id
            self.nav_cancel_requested = False
            self._arm_relay()
            self.state["navigation"] = {"robot_id": self.robot_id, "goal_id": goal_id, "attempt_id": attempt_id,
                "phase": "sending", "requested_wall": time.time()}
        # Keep ownership even when the response exceeds our wall-clock deadline.
        # A late accepted goal must never become invisible to the readiness gate.
        future = None
        try:
            future = self.navigator.send_goal_async(request,
                feedback_callback=lambda event: self._feedback(event, attempt_id))
            self.nav_send_future = future
            wait_future(future, 3)
        except TimeoutError:
            self._disarm_relay("goal_response_timeout")
            with self.state_lock:
                self.nav_cancel_requested = True
                self.state["navigation"].update(phase="awaiting_goal_response",
                    send_timed_out=True, cancel_requested=True)
            if future is not None:
                future.add_done_callback(lambda result: self._navigation_response(result, attempt_id))
            return fail("NAVIGATION_RESPONSE_TIMEOUT", "Goal response timed out; late acceptance will be canceled",
                        goal_id=goal_id, attempt_id=attempt_id, pending=True)
        except Exception as exc:
            # A transport error cannot prove the server never accepted the goal.
            self._disarm_relay("goal_transport_error")
            # Keep the attempt busy until an explicit stop destroys owned Nav2.
            with self.state_lock:
                self.nav_cancel_requested = True
                self.state["navigation"].update(phase="goal_response_unknown", error=str(exc))
            raise
        accepted = self._navigation_response(future, attempt_id)
        if not accepted:
            return fail("NAVIGATION_REJECTED", "Nav2 rejected the goal", attempt_id=attempt_id)
        return {"success": True, "accepted": True, "pending": True,
                "goal_id": goal_id, "attempt_id": attempt_id, "phase": "executing"}

    def _navigation_busy(self):
        with self.state_lock:
            return self.nav_attempt_id is not None or self.nav_handle is not None

    def _navigation_response(self, future, attempt_id):
        try:
            handle = future.result()
        except Exception as exc:
            with self.state_lock:
                if self.nav_attempt_id == attempt_id:
                    self.state["navigation"].update(phase="goal_response_unknown", error=str(exc))
            return False
        with self.state_lock:
            current = self.nav_attempt_id == attempt_id
            if current:
                self.nav_send_future = None
                if not handle.accepted:
                    self._disarm_relay("goal_rejected")
                    self.state["navigation"].update(phase="rejected", complete=True, success=False)
                    self.nav_attempt_id = None
                    self.nav_cancel_requested = False
                    return False
                self.nav_handle = handle
                cancel_requested = self.nav_cancel_requested
                self.state["navigation"].update(phase="canceling" if cancel_requested else "executing")
        if not current:
            # A response from a stopped attempt cannot mutate a later attempt,
            # including when both navigations reuse the same user goal_id.
            if handle.accepted:
                handle.cancel_goal_async()
            return False
        handle.get_result_async().add_done_callback(lambda result: self._navigation_result(result, attempt_id))
        if cancel_requested:
            # This can run on the ROS executor. Never block that executor waiting
            # for a cancellation response; ownership lasts until action result.
            try:
                handle.cancel_goal_async().add_done_callback(
                    lambda result: self._late_cancel_response(result, attempt_id))
            except Exception as exc:
                with self.state_lock:
                    if self.nav_attempt_id == attempt_id:
                        self.state["navigation"]["cancel_error"] = str(exc)
        return True

    def _late_cancel_response(self, future, attempt_id):
        try:
            response = future.result()
            acknowledged = bool(response.goals_canceling)
            error = None if acknowledged else "Nav2 did not acknowledge late-goal cancellation"
        except Exception as exc:
            acknowledged, error = False, str(exc)
        with self.state_lock:
            if self.nav_attempt_id == attempt_id:
                self.state["navigation"]["cancel_acknowledged"] = acknowledged
                if error:
                    self.state["navigation"]["cancel_error"] = error

    def navigation_ready(self, goal_id):
        """Check this robot's lifecycle/TF before any command relay is armed."""
        self._goal(goal_id)
        if self._navigation_busy():
            return fail("NAVIGATION_BUSY", "A goal is active or awaiting a response; cancel and await its result, or stop Nav2")
        if not self.navigator.server_is_ready() or not self.planner.server_is_ready():
            return fail("NAVIGATION_NOT_READY", "Nav2 action servers are not available")
        # Read the actual lifecycle state rather than trusting our startup flag.
        # Clients already exist, so discovery and cold startup happen before arm.
        pending = {}
        for name, client in self.lifecycle_states.items():
            if not client.service_is_ready():
                return fail("NAVIGATION_NOT_READY", f"{name} lifecycle status is unavailable")
            pending[name] = client.call_async(GetState.Request())
        deadline = time.monotonic() + 0.75
        states = {}
        for name, future in pending.items():
            response = wait_future(future, max(0, deadline - time.monotonic()))
            states[name] = response.current_state.label
            if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                return fail("NAVIGATION_NOT_READY", f"{name} is {response.current_state.label}")
        return {"success": True, "ready": True, "goal_id": goal_id,
                "initial_pose_verified": True, "pose": self.pose(),
                "localization_mode": self.profile["localization_mode"], "robot_id": self.robot_id, "active_goal": False, "lifecycle_states": states}

    def _feedback(self, event, attempt_id):
        msg = event.feedback
        with self.state_lock:
            if self.nav_attempt_id == attempt_id and self.state["navigation"]:
                self.state["navigation"]["feedback"] = {
                    "distance_remaining_m": float(msg.distance_remaining),
                    "number_of_recoveries": int(msg.number_of_recoveries),
                    "navigation_time_s": msg.navigation_time.sec + msg.navigation_time.nanosec / 1e9,
                    "x_m": msg.current_pose.pose.position.x, "y_m": msg.current_pose.pose.position.y,
                    "received_wall": time.time()}

    def _navigation_result(self, future, attempt_id):
        try:
            result = future.result()
            with self.state_lock:
                if self.nav_attempt_id != attempt_id or not self.state["navigation"]:
                    return
                self.state["navigation"].update(
                    phase={4: "succeeded", 5: "canceled", 6: "aborted"}.get(result.status, "unknown"),
                    complete=True, success=result.status == 4 and not getattr(result.result, "error_code", 0),
                    action_status=result.status, error_code=getattr(result.result, "error_code", 0),
                    error_message=getattr(result.result, "error_msg", ""), finished_wall=time.time())
                self.nav_handle = None
                self.nav_attempt_id = self.nav_send_future = None
                self.nav_cancel_requested = False
                self._disarm_relay("navigation_result")
        except Exception as exc:
            with self.state_lock:
                if self.nav_attempt_id != attempt_id or not self.state["navigation"]:
                    return
                # Failed result transport does not prove the remote goal ended.
                self._disarm_relay("result_transport_error")
                self.state["navigation"].update(phase="result_unknown", error=str(exc))

    def cancel(self):
        self._disarm_relay("navigation_cancel")
        with self.state_lock:
            handle, attempt_id = self.nav_handle, self.nav_attempt_id
            if attempt_id:
                self.nav_cancel_requested = True
                self.state["navigation"]["cancel_requested"] = True
        if handle is not None:
            response = wait_future(handle.cancel_goal_async(), 3)
            if not response.goals_canceling and self.nav_attempt_id == attempt_id:
                return fail("CANCEL_REJECTED", "Nav2 did not acknowledge cancellation")
            with self.state_lock:
                if self.nav_attempt_id == attempt_id and self.state["navigation"]:
                    self.state["navigation"]["phase"] = "canceling"
        self.zero.publish(Twist())
        return {"success": True, "cancel_requested": True, "pending": self._navigation_busy(),
                "zero_velocity_published": True}

    def _stop_navigation(self):
        self._disarm_relay("navigation_stopped")
        if self._navigation_busy():
            try:
                self.cancel()
            except Exception:
                pass
        for _, _, name in reversed(NAV_NODES):
            self.stop_child(name)
        with self.state_lock:
            self.nav_handle = self.nav_attempt_id = self.nav_send_future = None
            self.nav_cancel_requested = False
            self.state["navigation_ready"] = False
            if self.state["navigation"] and not self.state["navigation"].get("complete"):
                self.state["navigation"].update(phase="canceled", complete=True, success=False, finished_wall=time.time())
        self.zero.publish(Twist())

    def stop(self):
        if self.starting:
            self.startup_cancel.set()
            self.startup_thread.join(timeout=11)
        self._stop_navigation()
        for name in list(self.children):
            self.stop_child(name)
        # The RPC worker remains alive after its children stop. A new Isaac
        # session starts its clock at zero, so no previous TF/freshness claim
        # may survive here and masquerade as feedback from the new session.
        self.tf.clear()
        with self.state_lock:
            self.state.update(map=None, model=None, goals={}, plan=None, navigation=None,
                              navigation_ready=False, initial_pose_verified=False)
            for name in ("initial_pose", "odometry", "cmd_vel", "navigation_startup",
                         "clock_reset_detected", "params_file"):
                self.state.pop(name, None)
            self.map_msg = self.description_text = None
            self.static_frames.clear()
            self.dynamic_frames.clear()
            self.last_clock = self.last_odom = self.last_twist = 0.0
            self._rest_started_ns = None
            self._rest_samples = 0
            self.clock_ns = 0
        return {"success": True, "stopped": True, "owned_processes_remaining": len(self.children)}

    def status(self):
        with self.state_lock:
            state = json.loads(json.dumps(self.state))
        if state["model"]:
            required = set(state["model"].get("expected_dynamic_tf", []))
            state["model"]["dynamic_tf_received"] = sorted(required.intersection(self.dynamic_frames))
            state["model"]["dynamic_tf_complete"] = required.issubset(self.dynamic_frames)
        state.update(worker_pid=os.getpid(), observed_wall=time.time(), clock_ns=self.clock_ns,
            clock_age_s=time.monotonic() - self.last_clock if self.last_clock else None,
            odom_age_s=time.monotonic() - self.last_odom if self.last_odom else None,
            cmd_vel_age_s=time.monotonic() - self.last_twist if self.last_twist else None,
            processes={name: {**self.records.get(name, {}), "running": child.poll() is None,
                              "exit_code": child.poll()} for name, child in list(self.children.items())})
        rest_duration = ((getattr(self, "_rest_stamp_ns", 0) - self._rest_started_ns) / 1e9
                         if self._rest_started_ns is not None else 0.0)
        fresh_odom = bool(self.last_odom and time.monotonic() - self.last_odom
                          <= self.profile["limits"]["feedback_timeout_seconds"])
        state["control"] = {"backend": self.profile["control_backend"],
            "relay_armed": self._relay_armed, "reason": self._relay_reason,
            "output_topic": self.topics["cmd_vel"],
            "nav2_input_topic": self.private_topic + "/cmd_vel_smoothed",
            "rest_verified": fresh_odom and self._rest_samples >= 2 and rest_duration >= 0.25,
            "rest_duration_s": max(0.0, rest_duration), "rest_source": "measured_odometry"}
        try:
            state["pose"] = self.pose()
            state["tf_ready"] = True
        except Exception as exc:
            state["tf_ready"], state["tf_error"] = False, str(exc)
        if any(not info["running"] for name, info in state["processes"].items()
               if name in {n[2] for n in NAV_NODES}):
            state["navigation_ready"] = False
        return {"success": True, "state": state}

    def dispatch(self, request):
        if not isinstance(request, dict):
            return fail("INVALID_ARGUMENT", "ROS worker request must be a JSON object")
        action = request.get("action")
        if not isinstance(action, str) or action not in FIELDS or set(request) - FIELDS.get(action, set()) - {"action", "robot_id"}:
            return fail("INVALID_ARGUMENT", "Unknown ROS worker action or argument")
        if request.get("robot_id", self.robot_id) != self.robot_id:
            return fail("ROBOT_MISMATCH", "This request does not target the configured robot")
        if action == "status":
            return self.status()
        with self.commands:
            try:
                if action == "close_rviz":
                    self.stop_child("rviz2")
                    return {"success": True, "closed": True}
                params = {key: value for key, value in request.items() if key not in {"action", "robot_id"}}
                result = getattr(self, action)(**params)
                return {**result, "robot_id": self.robot_id}
            except (ValueError, TypeError, KeyError) as exc:
                return fail("INVALID_ARGUMENT", exc)
            except Exception as exc:
                return fail("ROS_OPERATION_FAILED", exc)


class RpcHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(25)
        line = self.rfile.readline(1_048_577)
        try:
            if not line.endswith(b"\n") or len(line) > 1_048_576:
                raise ValueError("Expected one JSON line of at most 1 MiB")
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError("Request must be a JSON object")
            response = self.server.worker.dispatch(request)
        except Exception as exc:
            response = fail("INVALID_REQUEST", exc)
        self.wfile.write(json.dumps(response, ensure_ascii=False, allow_nan=False).encode() + b"\n")


class RpcServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    run_dir.chmod(0o700)
    os.umask(0o077)
    lock = (run_dir / "worker.lock").open("a")
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    socket_path = run_dir / "ros.sock"
    if len(os.fsencode(socket_path)) >= 108:
        raise ValueError("run-dir is too long for a Unix socket")
    socket_path.unlink(missing_ok=True)
    rclpy.init()
    worker = Worker(run_dir, load_profile(args.profile))
    server = RpcServer(str(socket_path), RpcHandler)
    server.worker = worker
    socket_path.chmod(0o600)
    executor = SingleThreadedExecutor()
    executor.add_node(worker)
    spinner = threading.Thread(target=executor.spin, daemon=False)
    spinner.start()
    serving = threading.Thread(target=server.serve_forever, daemon=True)
    serving.start()
    quit_event = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: quit_event.set())
    try:
        while not quit_event.wait(0.5):
            save(run_dir / "ros-state.json", worker.status()["state"])
    finally:
        worker.stopping.set()
        worker.startup_cancel.set()
        if worker.startup_thread:
            worker.startup_thread.join(timeout=11)
        server.shutdown()
        with worker.commands:
            worker._stop_navigation()
            for name in list(worker.children):
                worker.stop_child(name)
        worker.state["worker_running"] = False
        save(run_dir / "ros-state.json", worker.status()["state"])
        server.server_close()
        socket_path.unlink(missing_ok=True)
        executor.shutdown(timeout_sec=5)
        spinner.join(timeout=5)
        worker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
