"""Check Hermes entrypoint routing without importing Isaac or Hermes packages."""
import importlib.util
import json
from pathlib import Path
import sys


def test_registered_tools_route_only_isaac_lifecycle_assets_and_adapter(monkeypatch):
    directory = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "isaac_plugin_registration_test", directory / "__init__.py",
        submodule_search_locations=[str(directory)],
    )
    plugin = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, plugin)
    spec.loader.exec_module(plugin)
    calls = []
    monkeypatch.setattr(plugin, "command", lambda *args: calls.append(("bridge", args)) or {"success": True})
    monkeypatch.setattr(plugin, "run_script", lambda args: calls.append(("script", args)) or {"success": True, "pending": True, "request_id": "load-123"})
    monkeypatch.setattr(plugin, "list_scripts", lambda args: calls.append(("list", args)) or {"success": True, "scripts": []})

    class Context:
        def __init__(self):
            self.tools = {}
            self.skills = {}

        def register_tool(self, **tool):
            self.tools[tool["name"]] = tool

        def register_skill(self, name, path):
            self.skills[name] = path

    ctx = Context()
    plugin.register(ctx)
    declared = {line.strip()[2:] for line in (directory / "plugin.yaml").read_text().splitlines()
                if line.strip().startswith("- isaac_sim_")}
    assert set(ctx.tools) == declared
    assert len(plugin.SPECS) == 9  # Lifecycle, scene loading and capture remain.
    assert len(ctx.tools) == 13 == 9 + len(plugin.ASSET_TOOLS)
    assert set(ctx.tools) == {
        "isaac_sim_open", "isaac_sim_load_usd", "isaac_sim_play", "isaac_sim_stop",
        "isaac_sim_close", "isaac_sim_status", "isaac_sim_capture", "isaac_sim_list_scripts",
        "isaac_sim_run_script", "isaac_sim_export_occupancy_map", "isaac_sim_export_robot_urdf",
        "isaac_sim_export_status", "isaac_sim_prepare_rv1_ros2"}
    assert ctx.skills["isaac-sim"].is_file()
    params = {"script_name": "load_gongguan_xinhai", "wait_seconds": 0}
    result = json.loads(ctx.tools["isaac_sim_run_script"]["handler"](params))
    assert result["pending"] and result["request_id"] == "load-123"
    assert calls == [("script", params)]
    schema = ctx.tools["isaac_sim_run_script"]["schema"]["parameters"]
    assert schema["required"] == ["script_name"]
    assert schema["additionalProperties"] is False
    ctx.tools["isaac_sim_list_scripts"]["handler"]({})
    assert calls[-1] == ("list", {})
    ctx.tools["isaac_sim_load_usd"]["handler"]({"path": "/fixture.usd"})
    assert calls[-1] == ("bridge", ("load", {"path": "/fixture.usd"}))
    assert "isaac_sim_rv1_drive" not in ctx.tools
    assert "isaac_sim_rv1_brake" not in ctx.tools
    ctx.tools["isaac_sim_capture"]["handler"]({"label": "rv1-ready"})
    assert calls[-1] == ("bridge", ("capture", {"label": "rv1-ready"}))
    capture_schema = ctx.tools["isaac_sim_capture"]["schema"]["parameters"]
    assert capture_schema["required"] == ["label"]
    assert capture_schema["additionalProperties"] is False
    assert set(capture_schema["properties"]) == {"label"}
    assert capture_schema["properties"]["label"]["maxLength"] == 48
    urdf_schema = ctx.tools["isaac_sim_export_robot_urdf"]["schema"]["parameters"]
    assert urdf_schema["required"] == ["source_path"]
    assert set(urdf_schema["properties"]) == {"source_path", "output_dir", "root_prim_path", "base_frame", "robot_name"}
    assert urdf_schema["properties"]["robot_name"]["default"] == "robot"
    params = {"source_path": "/cart.usda", "root_prim_path": "/Cart", "robot_name": "inspection_cart", "base_frame": "cart_base"}
    ctx.tools["isaac_sim_export_robot_urdf"]["handler"](params)
    assert calls[-1] == ("bridge", ("nav_export_urdf", params))
    assert "isaac_sim_export_rv1_urdf" not in ctx.tools
    for name, action, _, _, _ in plugin.ASSET_TOOLS:
        ctx.tools[name]["handler"]({})
        assert calls[-1] == ("bridge", (action, {}))
