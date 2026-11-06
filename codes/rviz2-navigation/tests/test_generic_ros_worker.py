"""Real ROS/Jazzy regressions in isolated DDS 173; no Isaac or live robot.

Source /opt/ros/jazzy/setup.bash before pytest. Only the fixed-link model test
starts a robot_state_publisher child, which the fixture owns and stops.
"""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs.action")
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformException
from rviz_navigation.ros_worker import Worker

from test_robot_profiles import rover


@pytest.fixture
def worker(tmp_path, rover, monkeypatch, request):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    assert not rclpy.ok(), "Isolated ROS tests require their own context"
    rclpy.init(args=[], domain_id=173)
    rover["use_sim_time"] = getattr(request, "param", False)
    instance = Worker(tmp_path, rover)
    try:
        yield instance
    finally:
        instance.stop()
        instance.tf_listener.unregister()
        instance.destroy_node()
        rclpy.shutdown()


def record_pose(worker, x=2.0, y=3.0, seconds=None, speed=0.0):
    stamp = Time(seconds=seconds) if seconds is not None else worker.get_clock().now()
    if worker.profile["use_sim_time"]:
        worker.get_clock().set_ros_time_override(stamp)
        clock = Clock()
        clock.clock = stamp.to_msg()
        worker._on_clock(clock)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = worker.frames["map"], worker.frames["base"]
    transform.header.stamp = stamp.to_msg()
    transform.transform.translation.x, transform.transform.translation.y = x, y
    transform.transform.rotation.w = 1.0
    worker.tf.set_transform(transform, "isolated_test")
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = worker.frames["odom"], worker.frames["base"]
    msg.header.stamp = stamp.to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = speed
    worker._on_odom(msg)
    return msg


def free_map(worker):
    msg = OccupancyGrid()
    msg.header.frame_id = worker.frames["map"]
    msg.info.resolution, msg.info.width, msg.info.height = 0.1, 200, 200
    msg.info.origin.position.x = msg.info.origin.position.y = -10.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 40_000
    worker._on_map(msg)
    worker.state["map"] = {"revision": "custom_rover_map"}
    return msg


def test_external_robot_uses_custom_topics_frames_without_isaac_or_sim_clock(worker):
    record_pose(worker)
    assert worker.pose()["frame_id"] == "site_map"
    assert worker.pose()["robot_id"] == "inspection_rover"
    assert worker.clock_ns == 0
    topics = dict(worker.get_topic_names_and_types())
    assert "/rover/odom" in topics and "/rover/cmd_vel" in topics
    assert "/odom" not in topics and "/cmd_vel" not in topics
    assert worker.status()["state"]["profile"]["control_backend"] == "ros2"


def test_initial_pose_and_goals_use_custom_robot_footprint_and_identity(worker):
    record_pose(worker)
    grid = free_map(worker)
    assert worker.set_initial_pose(2.0, 3.0, 0.0)["success"]
    assert not worker.set_initial_pose(4.0, 3.0, 0.0)["success"]
    result = worker.dispatch({"action": "set_goal", "robot_id": "inspection_rover", "x_m": 3.0, "y_m": 3.0, "yaw_rad": 0.0})
    assert result["success"] and result["goal"]["robot_id"] == "inspection_rover"
    assert result["goal"]["frame_id"] == "site_map"
    wrong = worker.dispatch({"action": "set_goal", "robot_id": "rv1", "x_m": 3.0, "y_m": 3.0, "yaw_rad": 0.0})
    assert wrong["error"]["code"] == "ROBOT_MISMATCH"
    grid.data[130 * 200 + 130] = 100
    with pytest.raises(ValueError, match="non-free"):
        worker.set_goal(3.0, 3.0, 0.0)


def test_foreign_robot_odometry_cannot_satisfy_readiness(worker):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
    worker._on_odom(msg)
    assert worker.last_odom == 0
    with pytest.raises(RuntimeError, match="Fresh configured"):
        worker.pose()


def test_cancel_invalidates_queued_relay_commands_and_preserves_zero(worker):
    record_pose(worker)
    output = []
    original = worker.zero
    worker.zero = NS(publish=lambda msg: output.append((msg.linear.x, msg.angular.z)))
    try:
        worker._arm_relay()
        generation = worker._relay_generation
        command = Twist()
        command.linear.x = 0.2
        worker._relay_twist(command, generation)
        assert output[-1] == (0.2, 0.0)
        worker.cancel()
        count = len(output)
        worker._relay_twist(command, generation)
        assert output[-1] == (0.0, 0.0) and len(output) == count
        assert not worker.status()["state"]["control"]["relay_armed"]
    finally:
        worker.zero = original


@pytest.mark.parametrize("velocity", [float("nan"), 100.0])
def test_invalid_or_excess_velocity_disarms_instead_of_forwarding(worker, velocity):
    record_pose(worker)
    worker._arm_relay()
    command = Twist()
    command.linear.x = velocity
    worker._relay_twist(command, worker._relay_generation)
    assert not worker._relay_armed and worker._relay_reason == "invalid_nav2_velocity"


def test_wall_time_watchdog_disarms_even_without_simulation_progress(worker):
    record_pose(worker)
    worker._arm_relay()
    worker._relay_armed_at -= 4
    worker._relay_watchdog()
    assert not worker._relay_armed and worker._relay_reason == "nav2_command_watchdog_expired"


def test_measured_rest_requires_duration_and_fresh_odometry(worker):
    now = time.time()
    record_pose(worker, seconds=now)
    assert not worker.status()["state"]["control"]["rest_verified"]
    record_pose(worker, seconds=now + 0.3)
    assert worker.status()["state"]["control"]["rest_verified"]
    worker.last_odom -= 6
    assert not worker.status()["state"]["control"]["rest_verified"]
    record_pose(worker, seconds=now + 0.4, speed=0.2)
    assert not worker.status()["state"]["control"]["rest_verified"]


@pytest.mark.parametrize("worker", [True], indirect=True)
def test_custom_simulation_clock_reset_clears_tf_and_disarms(worker):
    record_pose(worker, seconds=75.0)
    assert worker.pose()["x_m"] == 2.0
    worker._arm_relay()
    worker.state.update(navigation_ready=True, initial_pose_verified=True, goals={"old": {}})
    worker.get_clock().set_ros_time_override(Time(seconds=10))
    with pytest.raises(TransformException):
        worker.tf.lookup_transform("site_map", "chassis", Time())
    record_pose(worker, seconds=10.0, x=3.0)
    assert worker.pose()["x_m"] == 3.0
    assert not worker._relay_armed and worker._relay_reason == "clock_reset"
    assert not worker.state["navigation_ready"] and worker.state["goals"] == {}


@pytest.mark.parametrize("worker", [True], indirect=True)
def test_stop_discards_previous_simulation_epoch_and_accepts_new_feedback(worker):
    record_pose(worker, seconds=75.0)
    free_map(worker)
    worker.state.update(model={"robot_name": "old"}, navigation_ready=True,
                        initial_pose_verified=True, goals={"old": {}}, plan={"path_id": "old"})
    worker.dynamic_frames = {"old_sensor"}
    result = worker.stop()
    assert result["success"] and result["owned_processes_remaining"] == 0
    assert worker.state["map"] is worker.state["model"] is worker.state["plan"] is None
    assert not worker.state["goals"] and not worker.dynamic_frames
    assert not worker.clock_ns and not worker.last_odom
    with pytest.raises(TransformException):
        worker.tf.lookup_transform("site_map", "chassis", Time())
    with pytest.raises(RuntimeError, match="Fresh configured"):
        worker.pose()
    record_pose(worker, seconds=10.0, x=4.0)
    assert worker.pose()["x_m"] == 4.0
    assert not worker.state["initial_pose_verified"]


@pytest.mark.parametrize("worker", [True], indirect=True)
def test_subsecond_reset_invalidates_tf_readiness_and_feedback(worker):
    record_pose(worker, seconds=2.25)
    worker.state.update(navigation_ready=True, initial_pose_verified=True,
                        initial_pose={"actual": "old"}, goals={"old": {}}, plan={"path_id": "old"})
    worker.dynamic_frames = {"old_wheel"}
    clock = Clock()
    clock.clock = Time(seconds=2.0).to_msg()
    worker._on_clock(clock)
    with pytest.raises(TransformException):
        worker.tf.lookup_transform("site_map", "chassis", Time())
    assert not worker.state["initial_pose_verified"] and not worker.state["navigation_ready"]
    assert worker.state["goals"] == {} and worker.state["plan"] is None
    assert "initial_pose" not in worker.state and "odometry" not in worker.state
    assert not worker.dynamic_frames and not worker.last_odom


def test_load_non_rv1_urdf_and_custom_robot_description_topic(worker, tmp_path):
    path = tmp_path / "inspection-rover.urdf"
    path.write_text('''<robot name="inspection_rover"><link name="chassis"/>
      <link name="sensor_mount"/><joint name="mount" type="fixed">
      <parent link="chassis"/><child link="sensor_mount"/>
      <origin xyz="0 0 0.2"/></joint></robot>''')
    executor = SingleThreadedExecutor()
    executor.add_node(worker)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        result = worker.model_load(str(path))
        assert result["success"]
        assert result["model"]["robot_name"] == "inspection_rover"
        assert result["model"]["fixed_tf_received"] == ["sensor_mount"]
        command = worker.records["robot_state_publisher"]["command"]
        assert "joint_states:=/rover/joint_states" in command
        assert "robot_description:=/rover/robot_description" in command
    finally:
        worker.stop()
        executor.shutdown(timeout_sec=5)
        spinner.join(timeout=5)


def test_nonmatching_urdf_base_link_is_rejected(worker, tmp_path):
    path = tmp_path / "wrong.urdf"
    path.write_text('<robot name="wrong"><link name="base_link"/></robot>')
    with pytest.raises(ValueError, match="chassis"):
        worker.model_load(str(path))


@pytest.mark.parametrize("payload", [None, [], {"action": []}, {"action": {}}, {"action": "unknown"}])
def test_malformed_requests_are_structured_errors(worker, payload):
    assert worker.dispatch(payload)["error"]["code"] == "INVALID_ARGUMENT"
