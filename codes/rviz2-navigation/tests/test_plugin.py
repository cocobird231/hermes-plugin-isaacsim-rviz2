"""Public split-plugin contracts; no bridge, ROS, Isaac or live actuation."""
import builtins
import importlib.util
import itertools
import json
from pathlib import Path
import shutil
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
NAMES = {
    "rviz2_configure_robot", "rviz2_open", "rviz2_close", "rviz2_load_map",
    "rviz2_load_robot", "rviz2_navigation_start", "rviz2_navigation_status",
    "rviz2_set_initial_pose", "rviz2_set_goal", "rviz2_plan_path", "rviz2_navigate",
    "rviz2_cancel_navigation", "rviz2_navigation_stop",
}
_COUNTER = itertools.count()


class Context:
    def __init__(self):
        self.tools, self.skills = {}, {}

    def register_tool(self, **tool):
        assert tool["name"] not in self.tools, "Two plugins attempted to register one tool name"
        self.tools[tool["name"]] = tool

    def register_skill(self, name, path):
        assert name not in self.skills
        self.skills[name] = path


def load(directory, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        f"split_plugin_contract_{next(_COUNTER)}", directory / "__init__.py",
        submodule_search_locations=[str(directory)])
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_two_plugins_have_disjoint_toolsets_and_no_legacy_navigation(monkeypatch):
    context = Context()
    isaac = load(ROOT.parent / "isaac-sim", monkeypatch)
    rviz = load(ROOT, monkeypatch)
    isaac.register(context)
    rviz.register(context)
    assert len(context.tools) == 26
    assert {name for name, tool in context.tools.items() if tool["toolset"] == "rviz2-navigation"} == NAMES
    assert len([tool for tool in context.tools.values() if tool["toolset"] == "isaac-sim"]) == 13
    assert set(context.skills) == {"isaac-sim", "rviz2-navigation"}
    assert all(path.is_file() for path in context.skills.values())
    forbidden = {"isaac_sim_export_rv1_urdf", "isaac_sim_rv1_set_initial_pose",
                 "isaac_sim_rv1_set_goal", "isaac_sim_rv1_plan_path", "isaac_sim_rv1_navigate",
                 "isaac_sim_rv1_cancel_navigation", "isaac_sim_rviz2_load_rv1",
                 "isaac_sim_rv1_drive", "isaac_sim_rv1_brake"}
    assert not forbidden.intersection(context.tools)
    assert not any(name.startswith(("isaac_sim_rviz2_", "isaac_sim_navigation_")) for name in context.tools)


def test_rviz_manifest_schema_and_handler_route_match(monkeypatch):
    plugin = load(ROOT, monkeypatch)
    context, calls = Context(), []
    monkeypatch.setattr(plugin, "command", lambda action, params: calls.append((action, params))
                        or {"success": True, "action": action})
    plugin.register(context)
    declared = {line.strip()[2:] for line in (ROOT / "plugin.yaml").read_text().splitlines()
                if line.strip().startswith("- rviz2_")}
    assert set(context.tools) == declared == NAMES
    for name, action, _, _, _ in plugin.TOOLS:
        tool = context.tools[name]
        params = {"profile_name": "rv1"} if name == "rviz2_configure_robot" else {"robot_id": "inspection_cart"}
        result = json.loads(tool["handler"](params))
        assert calls[-1] == (action, params)
        assert result == {"success": True, "action": action}
        assert tool["schema"]["name"] == name
        assert tool["schema"]["parameters"]["additionalProperties"] is False
        if name != "rviz2_configure_robot":
            assert "robot_id" in tool["schema"]["parameters"]["properties"]
    for name in ("rviz2_set_initial_pose", "rviz2_set_goal"):
        assert set(context.tools[name]["schema"]["parameters"]["required"]) == {"x_m", "y_m", "yaw_rad"}
    for name in ("rviz2_plan_path", "rviz2_navigate"):
        assert context.tools[name]["schema"]["parameters"]["required"] == ["goal_id"]


def test_profile_schema_requires_exactly_one_configuration_source(monkeypatch):
    jsonschema = pytest.importorskip("jsonschema")
    context = Context()
    load(ROOT, monkeypatch).register(context)
    schema = context.tools["rviz2_configure_robot"]["schema"]["parameters"]
    jsonschema.Draft202012Validator.check_schema(schema)
    validator = jsonschema.Draft202012Validator(schema)
    assert validator.is_valid({"profile_name": "rv1"})
    assert validator.is_valid({"profile": {"robot_id": "inspection_cart"}})  # Full profile validated by backend.
    assert not validator.is_valid({})
    assert not validator.is_valid({"profile_name": "rv1", "profile": {}})
    assert not validator.is_valid({"profile_name": "other"})
    assert not validator.is_valid({"profile_name": "rv1", "unregistered": True})


def test_rviz_frontend_loads_without_sibling_or_simulation_packages(tmp_path, monkeypatch):
    isolated = tmp_path / "package/codes/rviz2-navigation"
    isolated.mkdir(parents=True)
    shutil.copy2(ROOT.parents[1] / "hermes_sim_config.py", isolated.parents[1] / "hermes_sim_config.py")
    for name in ("__init__.py", "transport.py", "tool_specs.py", "plugin.yaml", "deployment.py"):
        shutil.copy2(ROOT / name, isolated / name)
    skill = isolated / "skills/rviz2-navigation/SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("# Isolated registration fixture\n")
    original_import = builtins.__import__

    def reject_runtime_imports(name, *args, **kwargs):
        if name.split(".")[0] in {"isaacsim", "omni", "pxr", "rclpy", "rviz_navigation", "isaac_sim"}:
            raise AssertionError("Frontend attempted to import runtime dependency: " + name)
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_runtime_imports)
    context = Context()
    load(isolated, monkeypatch).register(context)
    assert set(context.tools) == NAMES
    assert context.skills["rviz2-navigation"] == skill


def test_transport_settings_stay_separate_in_same_process(tmp_path, monkeypatch):
    isaac, rviz = load(ROOT.parent / "isaac-sim", monkeypatch), load(ROOT, monkeypatch)
    for plugin, prefix, port in ((isaac, "ISAAC", 18768), (rviz, "RVIZ2", 18769)):
        monkeypatch.setattr(plugin.transport, "CLIENT_CONFIG", tmp_path / "nonexistent.json")
        monkeypatch.setenv(f"HERMES_{prefix}_URL", f"http://127.0.0.1:{port}")
        monkeypatch.setenv(f"HERMES_{prefix}_TOKEN_FILE", str(tmp_path / f"{prefix}.token"))
    assert isaac.transport._client_settings() == ("http://127.0.0.1:18768", tmp_path / "ISAAC.token")
    assert rviz.transport._client_settings() == ("http://127.0.0.1:18769", tmp_path / "RVIZ2.token")
