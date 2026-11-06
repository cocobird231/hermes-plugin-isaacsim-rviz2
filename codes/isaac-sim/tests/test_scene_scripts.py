import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import scene_scripts

SCRIPT = "load_gongguan_xinhai"
SCENE = "/configured/scenes/gongguan.usdz"
ROBOT = "/configured/robots/rv1.usd"


@pytest.fixture(autouse=True)
def registered_assets(monkeypatch):
    values = {"gongguan_usd": SCENE, "rv1_usd": ROBOT}
    monkeypatch.setattr(scene_scripts, "asset_path", values.__getitem__)
    monkeypatch.setattr(scene_scripts, "load_config", lambda: values.copy())


@pytest.fixture
def bridge_calls(monkeypatch):
    calls = []

    def invoke(action, params):
        calls.append((action, params))
        return {"success": True, "action": action, "pending": True, "request_id": "operation-123",
                "state": {"session_id": "existing-session", "pid": 123,
                          "operation": {"id": "operation-123", "status": "running"}}}

    monkeypatch.setattr(scene_scripts, "command", invoke)
    return calls


def test_load_only_preserves_pending_and_owned_instance(bridge_calls, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: Path("/different/hermes/home"))
    result = scene_scripts.run_script({"script_name": SCRIPT})
    assert bridge_calls == [("load", {"path": SCENE, "wait_seconds": 10})]
    assert result["script_name"] == SCRIPT
    assert result["action"] == "load" and result["pending"] is True
    assert result["request_id"] == result["state"]["operation"]["id"] == "operation-123"
    assert result["state"]["session_id"] == "existing-session"
    assert result["state"]["pid"] == 123


def test_rv1_uses_host_selected_asset_without_forwarding_a_path(bridge_calls):
    result = scene_scripts.run_script({"script_name": "load_rv1", "wait_seconds": 0})
    assert bridge_calls == [("rv1_load", {"wait_seconds": 0})]
    assert result["script_name"] == "load_rv1" and result["action"] == "rv1_load"
    assert result["pending"] and result["request_id"] == "operation-123"
    assert result["state"]["session_id"] == "existing-session"
    listed = {entry["script_name"]: entry for entry in scene_scripts.list_scripts()["scripts"]}
    assert listed["load_rv1"]["scene_path"] == ROBOT
    assert listed["load_rv1"]["scene_path_base"] == "generated_session_wrapper"
    assert listed["load_rv1"]["preconditions"] == ["running", "stopped"]
    assert len(bridge_calls) == 1


def test_insert_uses_only_registered_action_and_preserves_stage_contract(bridge_calls):
    result = scene_scripts.run_script({"script_name": "add_rv1_to_gongguan", "wait_seconds": 1})
    assert bridge_calls == [("rv1_insert", {"wait_seconds": 1})]
    assert result["pending"] and result["request_id"] == "operation-123"
    entry = next(e for e in scene_scripts.list_scripts()["scripts"] if e["script_name"] == "add_rv1_to_gongguan")
    assert entry["effect"] == "augment_current_stage_session_layer"


@pytest.mark.parametrize("seconds", [0, 2.5, 60])
def test_wait_boundaries_are_forwarded(bridge_calls, seconds):
    assert scene_scripts.run_script({"script_name": SCRIPT, "wait_seconds": seconds})["success"]
    assert bridge_calls == [("load", {"path": SCENE, "wait_seconds": seconds})]


@pytest.mark.parametrize("args", [
    None, [], "load_gongguan_xinhai", {}, {"script_name": None},
    {"script_name": []}, {"script_name": {}}, {"script_name": True}, {"script_name": ""},
    {"script_name": SCRIPT, "path": "/tmp/unreviewed.usd"},
    {"script_name": SCRIPT, "code": "print('unexpected')"},
    {"script_name": SCRIPT, "action": "open"},
    *({"script_name": SCRIPT, "wait_seconds": seconds}
      for seconds in [None, True, False, "10", -1, 61, float("nan"), float("inf"), -float("inf"), 10**400]),
])
def test_invalid_arguments_never_contact_bridge(bridge_calls, args):
    result = scene_scripts.run_script(args)
    assert result["success"] is False
    assert result["error"]["code"] == "INVALID_ARGUMENT"
    assert bridge_calls == []


@pytest.mark.parametrize("name", ["other_script", "../load_gongguan_xinhai.py", "/tmp/code.py"])
def test_unknown_script_never_contacts_bridge(bridge_calls, name):
    result = scene_scripts.run_script({"script_name": name})
    assert result["error"]["code"] == "UNKNOWN_SCRIPT"
    assert bridge_calls == []


@pytest.mark.parametrize("code", ["NOT_RUNNING", "STOP_REQUIRED", "BUSY", "BRIDGE_CONNECTION_ERROR"])
@pytest.mark.parametrize("script,action", [(SCRIPT, "load"), ("load_rv1", "rv1_load")])
def test_errors_propagate_without_implicit_recovery(monkeypatch, code, script, action):
    calls = []
    original = {"success": False, "error": {"code": code, "message": "original host error"},
                "state": {"pid": 123}, "request_id": "failed-load"}

    def invoke(action, params):
        calls.append(action)
        return original

    monkeypatch.setattr(scene_scripts, "command", invoke)
    result = scene_scripts.run_script({"script_name": script})
    assert result == {**original, "script_name": script}
    assert "script_name" not in original
    assert calls == [action]


def test_list_is_offline_and_cannot_mutate_registry(bridge_calls):
    entry = scene_scripts.list_scripts()["scripts"][0]
    assert entry["script_name"] == SCRIPT and "公館辛亥路" in entry["aliases"]
    assert entry["action"] == "load" and entry["effect"] == "replace_current_stage"
    assert entry["preconditions"] == ["running", "stopped"]
    entry["scene_path"] = "/tmp/other.usd"
    entry["aliases"].clear()
    entry["preconditions"].append("automatic-launch")
    current = scene_scripts.list_scripts({})["scripts"][0]
    assert current["scene_path"] == SCENE and "公館辛亥路" in current["aliases"]
    assert current["preconditions"] == ["running", "stopped"]
    assert bridge_calls == []


@pytest.mark.parametrize("args", [[], "", {"action": "load"}])
def test_list_rejects_unexpected_arguments(bridge_calls, args):
    assert scene_scripts.list_scripts(args)["error"]["code"] == "INVALID_ARGUMENT"
    assert bridge_calls == []


@pytest.mark.parametrize("success,pending", [(True, False), (True, True), (False, False)])
@pytest.mark.parametrize("script", [SCRIPT, "load_rv1"])
def test_standalone_script_uses_same_runner_and_preserves_result(monkeypatch, capsys, success, pending, script):
    calls = []
    result = {"success": success, "pending": pending, "request_id": "cli-load"}

    def invoke(args):
        calls.append(args)
        return result

    monkeypatch.setattr(scene_scripts, "run_script", invoke)
    spec = importlib.util.spec_from_file_location(script + "_cli", ROOT / "scripts" / (script + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.main(["--wait-seconds", "0"]) == (0 if success else 1)
    assert calls == [{"script_name": script, "wait_seconds": 0}]
    assert json.loads(capsys.readouterr().out) == result
