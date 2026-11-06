"""Owned-window capture contracts without a GUI or an Isaac installation."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture


@pytest.fixture
def managed(tmp_path):
    session = {"session_id": "session-a", "run_dir": str(tmp_path / "session-a"),
               "launcher_pid": 111, "start_ticks": "100", "headless": False, "display": ":3"}
    Path(session["run_dir"]).mkdir()
    state = {"session_id": "session-a", "pid": 222, "start_ticks": "101", "phase": "ready",
             "process_running": True, "state_fresh": True, "display": ":3", "headless": False,
             "operation": {"id": "drive-123", "status": "running"}, "timeline": "playing"}
    manager = SimpleNamespace(screenshot_python="/usr/bin/python3", base=tmp_path, xauthority="/host/.Xauthority", session=session,
                              state=state, owned=[111, 222])
    manager._session = lambda: copy.deepcopy(manager.session)
    manager.status = lambda: {"success": True, "state": copy.deepcopy(manager.state)}
    manager._owned_pids = lambda session: list(manager.owned)
    return manager


@pytest.fixture
def worker(monkeypatch):
    calls = []
    # Minimal PNG header used only to exercise host-side metadata validation.
    png = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR" + struct.pack(">II", 640, 480) + b"test fixture"
    metadata = {"success": True, "pid": 222, "window_id": "0x123", "width": 640, "height": 480,
                "captured_at_utc": "2026-10-30T08:00:00+00:00"}

    def run(command, environment):
        calls.append((command, environment))
        Path(command[command.index("--output") + 1]).write_bytes(png)
        return metadata.copy()

    monkeypatch.setattr(capture, "_worker_result", run)
    return SimpleNamespace(calls=calls, png=png, metadata=metadata, run=run)


@pytest.mark.parametrize("label", [None, "", "UPPER", "../image", "a/b", "a.png", "-bad", " space", "a\n", "a" * 49, 1, True])
def test_label_rejected_before_host_inspection(label):
    with pytest.raises(capture.CaptureError) as error:
        capture.capture_managed_window(None, label)
    assert error.value.code == "INVALID_ARGUMENT"


def test_capture_fixed_paths_sidecar_hash_and_unchanged_operation(managed, worker, monkeypatch):
    monkeypatch.setenv("DISPLAY", ":99")
    monkeypatch.setenv("PYTHONPATH", "/isaac/python")
    monkeypatch.setenv("LD_LIBRARY_PATH", "/isaac/lib")
    before = copy.deepcopy(managed.state)
    result = capture.capture_managed_window(managed, "rv1-ready")
    assert result["success"] and result["action"] == "capture"
    assert not {"request_id", "pending", "accepted"}.intersection(result)
    png = Path(result["path"])
    assert png.parent == Path(managed.session["run_dir"]) / "screenshots"
    assert "rv1-ready" in png.name
    assert result["sha256"] == hashlib.sha256(worker.png).hexdigest()
    assert (result["width"], result["height"]) == (640, 480)
    assert result["pid"] == 222 and result["session_id"] == "session-a"
    assert json.loads(Path(result["metadata_path"]).read_text()) == result
    assert png.stat().st_mode & 0o777 == 0o600
    assert Path(result["metadata_path"]).stat().st_mode & 0o777 == 0o600
    assert managed.state == before
    command, environment = worker.calls[0]
    assert command[:3] == ["/usr/bin/python3", "-I", str(Path(capture.__file__).resolve())]
    assert command[-3:] == ["--owned-pids", "111", "222"]
    assert environment["DISPLAY"] == ":3" and environment["XAUTHORITY"] == "/host/.Xauthority"
    assert "PYTHONPATH" not in environment and "LD_LIBRARY_PATH" not in environment
    second = capture.capture_managed_window(managed, "rv1-ready")
    assert second["path"] != result["path"] and png.read_bytes() == worker.png


@pytest.mark.parametrize("changes, code", [
    ({"process_running": False}, "NOT_RUNNING"),
    ({"phase": "closed"}, "NOT_RUNNING"),
    ({"headless": True}, "HEADLESS_CAPTURE_UNAVAILABLE"),
    ({"state_fresh": False}, "STALE_STATE"),
    ({"phase": "closing"}, "CAPTURE_NOT_READY"),
    ({"display": None}, "DISPLAY_UNAVAILABLE"),
    ({"display": "other-host:1"}, "DISPLAY_UNAVAILABLE"),
    ({"session_id": "different"}, "CAPTURE_STATE_CHANGED"),
])
def test_capture_requires_live_fresh_owned_gui(managed, worker, changes, code):
    managed.state.update(changes)
    with pytest.raises(capture.CaptureError) as error:
        capture.capture_managed_window(managed, "test")
    assert error.value.code == code
    assert worker.calls == []


@pytest.mark.parametrize("changed", ["pid", "session", "stale", "ownership"])
def test_capture_discards_image_if_identity_or_freshness_changes(managed, worker, monkeypatch, changed):
    def run(command, environment):
        result = worker.run(command, environment)
        if changed == "pid":
            managed.state["pid"] = 333
        elif changed == "session":
            managed.session["start_ticks"] = "new"
        elif changed == "stale":
            managed.state["state_fresh"] = False
        else:
            managed.owned = [111]
        return result
    monkeypatch.setattr(capture, "_worker_result", run)
    with pytest.raises(capture.CaptureError) as error:
        capture.capture_managed_window(managed, "test")
    assert error.value.code in {"CAPTURE_STATE_CHANGED", "STALE_STATE"}
    assert list((Path(managed.session["run_dir"]) / "screenshots").iterdir()) == []


def test_rejects_redirected_screenshot_directory(managed, worker, tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    (Path(managed.session["run_dir"]) / "screenshots").symlink_to(outside, target_is_directory=True)
    with pytest.raises(capture.CaptureError, match="symbolic link"):
        capture.capture_managed_window(managed, "test")
    assert worker.calls == [] and list(outside.iterdir()) == []


def test_invalid_png_is_removed(managed, worker):
    worker.metadata["width"] = 1
    with pytest.raises(capture.CaptureError, match="dimensions"):
        capture.capture_managed_window(managed, "test")
    assert list((Path(managed.session["run_dir"]) / "screenshots").iterdir()) == []


def test_worker_invocation_is_bounded_without_shell(monkeypatch):
    def run(command, **kwargs):
        assert command == ["/usr/bin/python3", "-I", "capture.py"]
        assert kwargs["timeout"] == 20 and "shell" not in kwargs
        raise subprocess.TimeoutExpired(command, 20)
    monkeypatch.setattr(capture.subprocess, "run", run)
    with pytest.raises(capture.CaptureError) as error:
        capture._worker_result(["/usr/bin/python3", "-I", "capture.py"], {"DISPLAY": ":3"})
    assert error.value.code == "CAPTURE_FAILED"


def test_worker_error_is_preserved_without_stderr(monkeypatch):
    monkeypatch.setattr(capture.subprocess, "run", lambda *a, **kw: SimpleNamespace(
        returncode=1, stdout=json.dumps({"success": False, "error": {
            "code": "WINDOW_NOT_FOUND", "message": "No owned window."}}), stderr="do not expose"))
    with pytest.raises(capture.CaptureError) as error:
        capture._worker_result([], {})
    assert error.value.code == "WINDOW_NOT_FOUND" and "stderr" not in str(error.value)


def test_window_must_have_pid_and_be_viewable(monkeypatch):
    answers = iter(["_NET_WM_PID(CARDINAL) = 222\n", "  Width: 640\n  Height: 480\n  Map State: IsUnMapped\n"])
    monkeypatch.setattr(capture, "_x_query", lambda *a, **kw: next(answers))
    assert capture._window_info("0x123", {}) is None
    monkeypatch.setattr(capture, "_x_query", lambda *a, **kw: "_NET_WM_PID: not found.")
    assert capture._window_info("0x123", {}) is None


def test_finds_largest_viewable_window_only_for_owned_pid(monkeypatch):
    monkeypatch.setattr(capture, "_x_query", lambda *a, **kw: "  0x1 other\n  0x2 tiny\n  0x3 Isaac\n  0x4 hidden\n")
    windows = {"0x1": {"pid": 999, "width": 5000, "height": 5000},
               "0x2": {"pid": 222, "width": 50, "height": 50},
               "0x3": {"pid": 222, "width": 640, "height": 480}, "0x4": None}
    monkeypatch.setattr(capture, "_window_info", lambda wid, env: {"window_id": wid, **windows[wid]} if windows[wid] else None)
    assert capture._find_window({111, 222}, {})["window_id"] == "0x3"
    with pytest.raises(capture.CaptureError) as error:
        capture._find_window({333}, {})
    assert error.value.code == "WINDOW_NOT_FOUND"
