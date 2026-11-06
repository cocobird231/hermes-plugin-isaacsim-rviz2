"""Isaac asset export ownership and runtime control-gate regression tests.

The independent RViz plugin owns ROS worker / goal orchestration tests.
No subprocess, ROS participant, GPU worker or simulator is started here.
"""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from manager import Manager
from navigation.supervisor import AssetSupervisor


class Owner:
    def __init__(self, directory):
        self.base = directory
        self.python = Path(sys.executable)
        self.state = {"process_running": False}

    def status(self):
        return {"success": True, "state": self.state}


@pytest.fixture
def supervisor(tmp_path, monkeypatch):
    instance = AssetSupervisor(Owner(tmp_path))
    instance.project = tmp_path / "project"
    instance.project.mkdir()
    monkeypatch.setattr(instance, "_spawn", lambda *a, **kw: pytest.fail("must not launch an export worker"))
    return instance


@pytest.mark.parametrize("action", ["nav_rviz_open", "nav_map_load", "nav_model_load", "nav_set_initial_pose",
                                    "nav_set_goal", "nav_plan_path", "nav_navigate", "nav_cancel", "nav_stop"])
def test_ros_actions_are_rejected_by_isaac_asset_supervisor(supervisor, action):
    result = supervisor.dispatch({"action": action})
    assert not result["success"] and result["error"]["code"] == "INVALID_ARGUMENT"


def test_export_cannot_overlap_managed_isaac(supervisor):
    supervisor.manager.state["process_running"] = True
    result = supervisor.dispatch({"action": "nav_export_urdf", "source_path": "/cart.usda"})
    assert result["error"]["code"] == "ISAAC_RUNNING"
    assert not list(supervisor.base.glob("export-*"))


def test_only_one_owned_export_may_run(supervisor, monkeypatch):
    job = supervisor.base / ("export-" + "a" * 32)
    job.mkdir()
    (job / "job.json").write_text(json.dumps({"pid": 123, "start_ticks": "456"}))
    monkeypatch.setattr(supervisor, "_alive", lambda record: True)
    result = supervisor.dispatch({"action": "nav_export_urdf", "source_path": "/cart.usda"})
    assert result["error"]["code"] == "EXPORT_BUSY"
    assert list(supervisor.base.glob("export-*")) == [job]


def test_export_output_outside_project_rejected(supervisor):
    result = supervisor.dispatch({"action": "nav_export_urdf", "source_path": "/cart.usda",
                                  "output_dir": str(supervisor.project.parent / "outside")})
    assert result["error"]["code"] == "ASSET_EXPORT_ERROR"
    assert "Output directory" in result["error"]["message"]


def test_export_accepts_robot_neutral_parameters_without_touching_source(supervisor, monkeypatch):
    source = supervisor.project / "cart.usda"
    source.write_text("source unchanged")
    requests = []
    def spawn(command, directory, environment):
        request = json.loads((directory / "request.json").read_text())
        requests.append((command, request, environment))
        return {"pid": 123, "start_ticks": "456", "log_path": str(directory / "process.log")}
    monkeypatch.setattr(supervisor, "_spawn", spawn)
    monkeypatch.setenv("PYTHONHOME", "/unrelated/python")
    monkeypatch.setenv("LD_PRELOAD", "/unrelated/library")
    result = supervisor.dispatch({"action": "nav_export_urdf", "source_path": str(source),
                                  "root_prim_path": "/Cart", "robot_name": "inspection_cart", "base_frame": "cart_base"})
    assert result["success"] and result["pending"]
    command, request, environment = requests[0]
    assert request["operation"] == "urdf" and request["robot_name"] == "inspection_cart"
    assert request["root_prim_path"] == "/Cart" and request["base_frame"] == "cart_base"
    assert Path(request["output_dir"]).is_relative_to(supervisor.project)
    assert command[0] == sys.executable and "asset_worker.py" in command[1]
    assert "PYTHONHOME" not in environment and "LD_PRELOAD" not in environment
    assert source.read_text() == "source unchanged"


@pytest.mark.parametrize("job_id", [None, "../outside", "a" * 31, "A" * 32])
def test_invalid_job_ids_never_read_arbitrary_paths(supervisor, job_id):
    result = supervisor.dispatch({"action": "nav_job_status", "job_id": job_id})
    assert result["error"]["code"] == "ASSET_EXPORT_ERROR"


@pytest.mark.parametrize("exit_code,expected", [(0, True), (1, False), (None, False)])
def test_written_artifacts_require_verified_clean_worker_exit(supervisor, monkeypatch, exit_code, expected):
    job_id = "b" * 32
    directory = supervisor.base / ("export-" + job_id)
    directory.mkdir()
    result_path = directory / "result.json"
    result_path.write_text(json.dumps({"ok": True, "source_unchanged": True}))
    (directory / "job.json").write_text(json.dumps({
        "pid": 123, "start_ticks": "456", "result_path": str(result_path),
        "log_path": str(directory / "process.log")}))
    if exit_code is not None:
        (directory / "exit.json").write_text(json.dumps({"exit_code": exit_code}))
    monkeypatch.setattr(supervisor, "_alive", lambda record: False)
    result = supervisor.dispatch({"action": "nav_job_status", "job_id": job_id})
    assert result["artifact_ok"] is True and result["success"] is expected
    assert result["pending"] is False
    if not expected:
        assert result["error"]["code"] == "EXPORT_SHUTDOWN_FAILED"


def test_isaac_open_waits_for_asset_worker_exit(tmp_path, monkeypatch):
    manager = Manager(sys.executable, tmp_path)
    (tmp_path / "nav").mkdir()
    manager.assets = NS(active_exports=lambda: ["owned-export"])
    monkeypatch.setattr(manager, "_session", lambda: pytest.fail("must reject before opening Isaac"))
    result = manager.dispatch({"action": "open", "wait_seconds": 0})
    assert result["error"]["code"] == "EXPORT_BUSY"


@pytest.mark.parametrize("action", ["stop", "close"])
@pytest.mark.parametrize("failure", [OSError("runtime gone"), ValueError("broken runtime JSON"), RuntimeError("disarm RPC failed")])
def test_disarm_exception_cannot_prevent_runtime_stop_or_close(tmp_path, monkeypatch, action, failure):
    manager = Manager(sys.executable, tmp_path)
    events = []
    monkeypatch.setattr(manager, "status", lambda: {"state": {"process_running": True, "ros2": {"armed": True}}})
    monkeypatch.setattr(manager, "_session", lambda: {"session_id": "owned"})
    monkeypatch.setattr(manager, "_alive", lambda session: True)
    def rpc(request_action, request_id, **params):
        events.append(request_action)
        if request_action == "rv1_ros2_disarm":
            raise failure
        return {"success": True, "action": request_action}
    monkeypatch.setattr(manager, "_rpc", rpc)
    monkeypatch.setattr(manager, "_wait", lambda action, request_id, seconds, initial: initial)
    result = manager.dispatch({"action": action, "wait_seconds": 0})
    assert events == ["rv1_ros2_disarm", action]
    assert result["success"] is True


def _runtime():
    spec = importlib.util.spec_from_file_location("navigation_runtime_gate_tests", ROOT / "runtime.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_disarmed_feedback_error_does_not_write_actuators(tmp_path, monkeypatch):
    worker = _runtime().Worker(tmp_path, "runtime-feedback-test")
    worker.usd = NS(get_stage=lambda: None)
    worker.timeline = NS(is_playing=lambda: True, is_stopped=lambda: False, get_current_time=lambda: 500.0)
    worker.simulation_manager = NS(get_simulation_time=lambda: 2.0, get_num_physics_steps=lambda: 1920)
    monkeypatch.setattr(worker, "sample_rv1", lambda stage: None)
    calls = []
    def failed_feedback(sim_time, *, playing):
        assert sim_time == 2.0  # Must not pass the independent animation time.
        raise RuntimeError("joint physics tensor unavailable")
    worker.rv1_navigation = NS(update=failed_feedback, status=lambda: {"started": True, "armed": False},
                                brake=lambda reason: calls.append(reason))
    worker.sample()
    assert calls == []
    assert worker.state["control_owner"] == "stopped"
    assert "joint physics tensor unavailable" in worker.state["ros2"]["error"]


def test_watchdog_disarm_updates_reported_control_owner(tmp_path, monkeypatch):
    worker = _runtime().Worker(tmp_path, "runtime-watchdog-test")
    worker.usd = NS(get_stage=lambda: None)
    worker.timeline = NS(is_playing=lambda: True, is_stopped=lambda: False, get_current_time=lambda: 500.0)
    worker.simulation_manager = NS(get_simulation_time=lambda: 2.0, get_num_physics_steps=lambda: 1920)
    worker.change(control_owner="nav2")
    monkeypatch.setattr(worker, "sample_rv1", lambda stage: None)
    worker.rv1_navigation = NS(update=lambda *args, **kwargs: None,
                                status=lambda: {"started": True, "armed": False, "reason": "command_watchdog_expired"})
    worker.sample()
    assert worker.state["control_owner"] == "stopped"
    assert worker.state["ros2"]["reason"] == "command_watchdog_expired"
