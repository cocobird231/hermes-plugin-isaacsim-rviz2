import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from manager import Manager, atomic_json, process_identity, process_matches
import manager as manager_module


@pytest.fixture
def manager():
    # AF_UNIX names have a 108-byte limit; pytest's descriptive paths can exceed it.
    with tempfile.TemporaryDirectory(prefix="hisaac-") as directory:
        yield Manager(sys.executable, Path(directory), display=":1")


@pytest.fixture
def launched_processes(monkeypatch):
    """Exercise launch environment and ownership without starting Isaac."""
    real_popen = subprocess.Popen
    launches = []

    def spawn(command, **kwargs):
        child = real_popen([sys.executable, "-c", "import time; time.sleep(60)"], **kwargs)
        launches.append({"child": child, "command": command, "environment": kwargs["env"]})
        return child

    monkeypatch.setattr(manager_module.subprocess, "Popen", spawn)
    yield launches
    for launch in launches:
        child = launch["child"]
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=5)


@pytest.mark.parametrize("args", [
    {"action": "open", "headless": "false"},
    {"action": "close", "force": 1},
    {"action": "play", "wait_seconds": float("nan")},
    {"action": "play", "wait_seconds": True},
    {"action": "play", "wait_seconds": 61},
    {"action": "shell", "command": "ls"},
    {"action": "open", "code": "print(1)"},
    {"action": "open", "display_number": True},
    {"action": "open", "display_number": False},
    {"action": "open", "display_number": 1.0},
    {"action": "open", "display_number": "1"},
    {"action": "open", "display_number": ":1"},
    {"action": "open", "display_number": None},
    {"action": "open", "display_number": -1},
    {"action": "open", "display_number": 100},
])
def test_reject_invalid_inputs_without_launch(manager, args):
    assert manager.dispatch(args)["success"] is False
    assert not manager.pointer.exists()


def test_closed_and_absolute_path_guard(manager):
    assert manager.dispatch({"action": "status"})["state"]["phase"] == "closed"
    assert manager.dispatch({"action": "close"})["already_closed"]
    assert manager.dispatch({"action": "load", "path": "map.usd"})["error"]["code"] == "INVALID_PATH"
    assert manager.dispatch({"action": "play"})["error"]["code"] == "NOT_RUNNING"


def test_capture_routes_without_request_operation_or_wait(manager, monkeypatch):
    expected = {"success": True, "action": "capture", "path": "/fixed/capture.png"}
    calls = []
    monkeypatch.setattr(manager_module, "capture_managed_window", lambda owner, label: calls.append((owner, label)) or expected)
    monkeypatch.setattr(manager, "_rpc", lambda *a, **kw: pytest.fail("capture must not call runtime"))
    monkeypatch.setattr(manager, "_wait", lambda *a, **kw: pytest.fail("capture must not wait on operations"))
    monkeypatch.setattr(manager_module.uuid, "uuid4", lambda: pytest.fail("capture is not a request operation"))
    assert manager.dispatch({"action": "capture", "label": "ready"}) == expected
    assert calls == [(manager, "ready")]


@pytest.mark.parametrize("args", [{"action": "capture"}, {"action": "capture", "label": "../bad"},
                                 {"action": "capture", "label": "test", "wait_seconds": 0},
                                 {"action": "capture", "label": "test", "path": "/tmp/image.png"}])
def test_capture_rejects_missing_or_unsafe_label_and_extra_arguments(manager, args):
    assert manager.dispatch(args)["error"]["code"] == "INVALID_ARGUMENT"


def test_rv1_insert_uses_existing_rpc_and_wait_only(manager, monkeypatch):
    calls = []
    initial = {"success": True, "accepted": True}
    monkeypatch.setattr(manager, "_rpc", lambda *a, **kw: calls.append((a, kw)) or initial)
    monkeypatch.setattr(manager, "_wait", lambda *a: {"wait": a})
    result = manager.dispatch({"action": "rv1_insert", "wait_seconds": 12})
    action, request_id, seconds, accepted = result["wait"]
    assert action == "rv1_insert" and seconds == 12 and accepted == initial
    assert calls == [(("rv1_insert", request_id), {})]
    assert manager.dispatch({"action": "rv1_insert", "path": "/arbitrary.usd"})["error"]["code"] == "INVALID_ARGUMENT"


@pytest.mark.parametrize("configured_display", ["omitted", None])
def test_default_display_one_ignores_inherited_display(manager, monkeypatch, launched_processes, configured_display):
    monkeypatch.setenv("DISPLAY", ":3")
    options = {} if configured_display == "omitted" else {"display": configured_display}
    default_manager = Manager(sys.executable, manager.base, **options)
    result = default_manager.dispatch({"action": "open", "wait_seconds": 0})
    assert result["success"] and result["pending"]
    assert launched_processes[0]["environment"]["DISPLAY"] == ":1"
    assert result["state"]["display"] == ":1"
    assert result["state"]["display_number"] == 1
    assert result["state"]["headless"] is False
    saved = json.loads(manager.pointer.read_text())
    assert saved["display"] == ":1" and saved["headless"] is False


@pytest.mark.parametrize("number", [0, 3, 99])
def test_explicit_display_overrides_bridge_default(manager, launched_processes, number):
    manager.display = ":7"
    result = manager.dispatch({"action": "open", "display_number": number, "wait_seconds": 0})
    assert result["success"]
    assert launched_processes[0]["environment"]["DISPLAY"] == f":{number}"
    assert result["state"]["display_number"] == number
    assert json.loads(manager.pointer.read_text())["display"] == f":{number}"


def test_configured_bridge_display_remains_fallback(manager, launched_processes):
    manager.display = ":7.0"
    result = manager.dispatch({"action": "open", "wait_seconds": 0})
    assert launched_processes[0]["environment"]["DISPLAY"] == ":7.0"
    assert result["state"]["display"] == ":7.0"
    assert result["state"]["display_number"] == 7


def test_open_reuses_actual_display_and_rejects_display_change(manager, launched_processes):
    opened = manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    for args in ({}, {"display_number": 3}):
        reused = manager.dispatch({"action": "open", "wait_seconds": 0, **args})
        assert reused["success"] and reused["already_running"]
        assert reused["state"]["session_id"] == opened["state"]["session_id"]
        assert reused["state"]["display"] == ":3"
    rejected = manager.dispatch({"action": "open", "display_number": 1, "wait_seconds": 0})
    assert rejected["error"]["code"] == "DISPLAY_CONFLICT"
    assert rejected["state"]["display_number"] == 3
    assert len(launched_processes) == 1
    assert launched_processes[0]["child"].poll() is None


@pytest.mark.parametrize("configured_display, matches_local", [
    (":3.0", True), (":3.1", False), ("other-host:3", False),
])
def test_explicit_display_requires_local_server_and_default_screen(manager, launched_processes,
                                                                 configured_display, matches_local):
    manager.display = configured_display
    manager.dispatch({"action": "open", "wait_seconds": 0})
    reused = manager.dispatch({"action": "open", "wait_seconds": 0})
    assert reused["success"] and reused["already_running"]
    assert reused["state"]["display"] == configured_display
    explicit = manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    if matches_local:
        assert explicit["success"] and explicit["already_running"]
    else:
        assert explicit["error"]["code"] == "DISPLAY_CONFLICT"
        assert explicit["state"]["display"] == configured_display
    assert len(launched_processes) == 1
    assert launched_processes[0]["child"].poll() is None


def test_headless_does_not_report_gui_or_conflict_on_display(manager, launched_processes):
    first = manager.dispatch({"action": "open", "headless": True, "display_number": 3, "wait_seconds": 0})
    second = manager.dispatch({"action": "open", "headless": True, "display_number": 1, "wait_seconds": 0})
    assert first["success"] and second["success"] and second["already_running"]
    assert first["state"]["headless"] is True
    assert first["state"]["display"] is None and first["state"]["display_number"] is None
    assert json.loads(manager.pointer.read_text())["display"] is None
    assert "--headless" in launched_processes[0]["command"]
    assert len(launched_processes) == 1
    conflict = manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    assert conflict["error"]["code"] == "MODE_CONFLICT"


def test_legacy_live_display_recovered_from_owned_process(manager, launched_processes):
    manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    session = json.loads(manager.pointer.read_text())
    session.pop("display")
    atomic_json(manager.pointer, session)
    # A restarted bridge uses :1, while the legacy owned GUI still uses :3.
    restarted = Manager(sys.executable, manager.base)
    state = restarted.status()["state"]
    assert state["display"] == ":3" and state["display_number"] == 3
    assert restarted.dispatch({"action": "open", "wait_seconds": 0})["already_running"]
    assert restarted.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})["already_running"]
    assert restarted.dispatch({"action": "open", "display_number": 1, "wait_seconds": 0})["error"]["code"] == "DISPLAY_CONFLICT"
    assert len(launched_processes) == 1


@pytest.mark.parametrize("legacy", [False, True])
def test_closed_session_retains_only_recorded_display(manager, launched_processes, legacy):
    manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    session = json.loads(manager.pointer.read_text())
    if legacy:
        session.pop("display")
        atomic_json(manager.pointer, session)
    child = launched_processes[0]["child"]
    child.terminate()
    child.wait(timeout=5)
    state = manager.status()["state"]
    assert state["process_running"] is False
    assert state["display"] == (None if legacy else ":3")
    assert state["display_number"] == (None if legacy else 3)


def test_unknown_legacy_display_does_not_inherit_bridge_display(manager, launched_processes, monkeypatch):
    manager.dispatch({"action": "open", "display_number": 3, "wait_seconds": 0})
    session = json.loads(manager.pointer.read_text())
    session.pop("display")
    atomic_json(manager.pointer, session)
    real_read = Path.read_bytes

    def deny_environ(path):
        if path.name == "environ":
            raise PermissionError("process environment inaccessible")
        return real_read(path)

    monkeypatch.setattr(Path, "read_bytes", deny_environ)
    assert manager.status()["state"]["display"] is None
    assert manager.dispatch({"action": "open", "wait_seconds": 0})["already_running"]
    assert manager.dispatch({"action": "open", "display_number": 1, "wait_seconds": 0})["error"]["code"] == "DISPLAY_CONFLICT"


def test_reused_pid_is_not_owned():
    ticks, _ = process_identity(os.getpid())
    assert process_matches(os.getpid(), ticks)
    assert not process_matches(os.getpid(), str(int(ticks) + 1))


def test_stale_ready_cannot_complete_open(manager, monkeypatch):
    monkeypatch.setattr(manager, "status", lambda: {"success": True, "state": {
        "phase": "ready", "process_running": True, "state_fresh": False}})
    result = manager._wait("open", "id", 0, {"success": True})
    assert result["pending"]


def test_crash_during_close_is_not_success(manager, monkeypatch):
    monkeypatch.setattr(manager, "status", lambda: {"success": True, "state": {
        "phase": "error", "process_running": False}})
    assert not manager._wait("close", "id", 0, {"success": True})["success"]


def test_interrupted_play_reports_unknown_outcome_instead_of_pending(manager, monkeypatch):
    initial = {"success": True, "accepted": True, "state": {
        "updated_monotonic": 10.0,
        "operation": {"id": "playing", "action": "play", "status": "running"}}}
    state = {"phase": "ready", "state_fresh": True, "process_running": True,
             "updated_monotonic": 11.0,
             "operation": {"id": "stop", "action": "stop", "status": "running"}}
    monkeypatch.setattr(manager, "status", lambda: {"success": True, "state": state})
    result = manager._wait("play", "playing", 60, initial)
    assert result["success"] is False and result["pending"] is False
    assert result["error"]["code"] == "OPERATION_SUPERSEDED"
    assert "outcome is unknown" in result["error"]["message"]
    assert result["request_id"] == "playing" and result["action"] == "play"
    assert result["state"] == state


@pytest.mark.parametrize("updated_at,fresh", [(9.0, True), (10.0, True), (11.0, False)])
def test_old_or_stale_status_cannot_prove_operation_replaced(manager, monkeypatch, updated_at, fresh):
    initial = {"success": True, "accepted": True, "state": {
        "updated_monotonic": 10.0, "operation": {"id": "playing", "status": "running"}}}
    state = {"phase": "ready", "state_fresh": fresh, "process_running": True,
             "updated_monotonic": updated_at,
             "operation": {"id": "other", "status": "succeeded"}}
    monkeypatch.setattr(manager, "status", lambda: {"success": True, "state": state})
    result = manager._wait("play", "playing", 0, initial)
    assert result["success"] is True and result["pending"] is True


def test_wait_can_establish_operation_before_detecting_replacement(manager, monkeypatch):
    states = iter([
        {"phase": "ready", "state_fresh": True, "process_running": True,
         "updated_monotonic": 10.0, "operation": {"id": "playing", "status": "running"}},
        {"phase": "ready", "state_fresh": True, "process_running": True,
         "updated_monotonic": 11.0, "operation": {"id": "stop", "status": "succeeded"}},
    ])
    monkeypatch.setattr(manager, "status", lambda: {"success": True, "state": next(states)})
    monkeypatch.setattr(manager_module.time, "sleep", lambda _: None)
    result = manager._wait("play", "playing", 60, {"success": True, "accepted": True})
    assert result["error"]["code"] == "OPERATION_SUPERSEDED"
    assert result["state"]["operation"]["id"] == "stop"


def test_invalid_action_lists_only_supported_rv1_commands(manager):
    result = manager.dispatch({"action": "unknown"})
    assert result["error"]["code"] == "INVALID_ACTION"
    for action in ("rv1_load", "rv1_insert", "rv1_ros2_prepare"):
        assert action in result["error"]["message"]


@pytest.mark.parametrize("action", ["rv1_drive", "rv1_brake"])
def test_manager_rejects_removed_direct_motion_actions(manager, monkeypatch, action):
    monkeypatch.setattr(manager, "_rpc", lambda *args, **kwargs: pytest.fail("must not reach Isaac"))
    result = manager.dispatch({"action": action})
    assert result["error"]["code"] == "INVALID_ACTION"
    assert action not in result["error"]["message"]


@pytest.mark.parametrize("action", ["rv1_ros2_arm", "rv1_ros2_disarm"])
def test_internal_adapter_routes_require_a_running_owned_isaac(manager, action):
    # These authenticated bridge actions support the independent Nav2 adapter;
    # neither action is exposed as a Hermes motion tool (test_plugin.py).
    result = manager.dispatch({"action": action})
    assert result["error"]["code"] == "NOT_RUNNING"


@pytest.mark.parametrize("action", ["nav_rviz_open", "nav_navigate", "nav_cancel", "nav_status"])
def test_ros_commands_have_moved_out_of_isaac_manager(manager, monkeypatch, action):
    monkeypatch.setattr(manager, "_rpc", lambda *args, **kwargs: pytest.fail("must not reach Isaac"))
    result = manager.dispatch({"action": action})
    assert result["error"]["code"] == "TOOL_MOVED"


def test_failed_session_registration_cleans_up_new_process(manager, monkeypatch):
    real_popen = subprocess.Popen
    children = []

    def spawn(command, **kwargs):
        import uuid
        uuid.UUID(command[command.index("--session-id") + 1])
        child = real_popen([sys.executable, "-c", "import time; time.sleep(60)"], **kwargs)
        children.append(child)
        return child

    def fail_write(path, value):
        raise OSError("Simulated full disk")

    monkeypatch.setattr(manager_module.subprocess, "Popen", spawn)
    monkeypatch.setattr(manager_module, "atomic_json", fail_write)
    result = manager.dispatch({"action": "open", "headless": True})
    assert result["error"]["code"] == "HOST_ERROR"
    assert children and children[0].poll() is not None
    assert not manager.children


def test_force_close_reaps_worker_after_launcher_exits(manager):
    """Regression: python.sh exits on TERM while its Kit child can ignore TERM."""
    run = manager.base / "test-session"
    run.mkdir()
    helper = run / "worker.py"
    helper.write_text('''import json, os, pathlib, signal, time
signal.signal(signal.SIGTERM, signal.SIG_IGN)
fields = pathlib.Path("/proc/self/stat").read_text().rsplit(")", 1)[1].split()
pathlib.Path("state.json").write_text(json.dumps({"session_id": "test-session", "pid": os.getpid(), "start_ticks": fields[19], "phase": "ready", "updated_monotonic": time.monotonic()}))
while True: time.sleep(0.1)
''')
    child = subprocess.Popen(["bash", "-c", '"$1" worker.py & wait', "test", sys.executable],
                             cwd=run, start_new_session=True)
    ticks, _ = process_identity(child.pid)
    session = {"session_id": "test-session", "run_dir": str(run), "launcher_pid": child.pid,
               "start_ticks": ticks, "headless": True}
    atomic_json(manager.pointer, session)
    manager.children["test-session"] = child
    try:
        deadline = time.monotonic() + 5
        while not (run / "state.json").exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert (run / "state.json").exists()
        state = json.loads((run / "state.json").read_text())
        result = manager.dispatch({"action": "close", "force": True})
        assert result["success"] and result["state"]["phase"] == "closed"
        assert not process_matches(state["pid"], state["start_ticks"])
    finally:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5)
