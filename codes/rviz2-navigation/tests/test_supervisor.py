"""Cross-process ownership, optional Isaac boundary and cancellation guarantees."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rviz_navigation.supervisor import NavigationSupervisor
from rviz_navigation.profiles import default_rv1_profile
from rviz_navigation.isaac_adapter import IsaacAdapter

PROJECT = Path(__file__).resolve().parents[3]

class NoIsaac:
    def arm(self, *args):
        raise AssertionError("ROS-only profiles must not contact Isaac")
    def disarm(self):
        raise AssertionError("ROS-only profiles must not contact Isaac")

@pytest.fixture
def manager(tmp_path):
    return NavigationSupervisor(tmp_path, isaac=NoIsaac())

def external():
    p = default_rv1_profile(PROJECT)
    p.update(robot_id="warehouse_cart", control_backend="ros2", localization_mode="tf_verified", use_sim_time=False)
    return p

def test_ros_only_operations_work_without_isaac_endpoint(manager, monkeypatch):
    assert manager.dispatch({"action": "nav_configure_robot", "profile": external()})["success"]
    seen = []
    def rpc(args, **kwargs):
        seen.append(args["action"])
        return {"success": True}
    monkeypatch.setattr(manager, "_rpc", rpc)
    assert manager.dispatch({"action": "nav_map_load", "map_yaml_path": "/map.yaml"})["success"]
    assert manager.dispatch({"action": "nav_model_load", "urdf_path": "/cart.urdf"})["success"]
    assert manager.dispatch({"action": "nav_navigate", "goal_id": "checked"})["success"]
    assert manager.dispatch({"action": "nav_cancel"})["success"]
    assert seen == ["map_load", "model_load", "navigation_ready", "navigate"]

def test_robot_assertion_never_starts_worker(manager, monkeypatch):
    manager.dispatch({"action": "nav_configure_robot", "profile": external()})
    monkeypatch.setattr(manager, "_rpc", lambda *a, **k: pytest.fail("must reject before RPC"))
    result = manager.dispatch({"action": "nav_set_goal", "robot_id": "rv1", "x_m": 0, "y_m": 0, "yaw_rad": 0})
    assert result["error"]["code"] == "ROBOT_MISMATCH"
    assert manager.profile["robot_id"] == "warehouse_cart"

@pytest.mark.parametrize("fault", [None, "isaac", "ros"])
def test_cancel_closes_relay_before_optional_http_and_attempts_both(manager, monkeypatch, fault):
    order = []
    monkeypatch.setattr(manager, "_record", lambda: {"pid": 123})
    monkeypatch.setattr(manager, "_alive", lambda record: True)
    def rpc(args):
        order.append("relay_zero_and_cancel")
        if fault == "ros":
            raise OSError("socket unavailable")
        return {"success": True}
    def disarm():
        order.append("simulator_gate")
        if fault == "isaac":
            raise TimeoutError("HTTP unavailable")
        return {"success": True}
    monkeypatch.setattr(manager, "_rpc", rpc)
    monkeypatch.setattr(manager, "disarm", disarm)
    result = manager.cancel_control()
    assert order == ["relay_zero_and_cancel", "simulator_gate"]
    assert result["success"] == (fault is None)
    if fault:
        assert result["error"]["code"] == "CANCEL_NOT_CONFIRMED"
        assert "ros_cancel" in result and "disarm" in result

def test_pending_simulator_disarm_is_not_completion(monkeypatch):
    a = IsaacAdapter()
    monkeypatch.setattr(a, "command", lambda action, *args: {"success": True, "state": {"process_running": True, "ros2": {"started": True}}} if action == "status" else {"success": True, "pending": True})
    assert a.disarm()["error"]["code"] == "DISARM_NOT_CONFIRMED"

def test_profile_change_rejects_live_owned_children(manager, monkeypatch):
    manager.dispatch({"action": "nav_configure_robot", "profile": external()})
    monkeypatch.setattr(manager, "_record", lambda: {"pid": 123})
    monkeypatch.setattr(manager, "_alive", lambda record: True)
    monkeypatch.setattr(manager, "_rpc", lambda args: {"success": True, "state": {"processes": {"rviz2": {"running": True}}}})
    result = manager.dispatch({"action": "nav_configure_robot", "profile_name": "rv1"})
    assert result["error"]["code"] == "PROFILE_BUSY"
    assert manager.profile["robot_id"] == "warehouse_cart"

@pytest.mark.parametrize("payload", [[], {"action": []}, {"action": "nav_teleop"}, {"action": "nav_status", "unknown": True}])
def test_invalid_actions_are_structured_errors(manager, payload):
    assert manager.dispatch(payload)["error"]["code"] == "INVALID_ARGUMENT"
