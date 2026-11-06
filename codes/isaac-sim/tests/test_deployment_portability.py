"""Portable configuration contracts; these tests never launch Isaac or drive robots."""
import json
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import scene_scripts
import transport
from manager import Manager
from navigation.supervisor import AssetSupervisor
from navigation.launch_environment import configure_ros_libraries


@pytest.fixture
def configured(tmp_path, monkeypatch):
    home = tmp_path / "new user home"
    home.mkdir()
    scene, robot = home / "custom road.usdz", home / "custom robot.usd"
    scene.write_text("fixture scene")
    robot.write_text("fixture robot")
    profile = home / "host settings.json"
    data = {"gongguan_usd": str(scene), "rv1_usd": str(robot),
            "state_root": str(home / "state"), "export_root": str(home / "outputs"),
            "isaac_port": 18718, "rviz2_port": 18719, "display_number": 4,
            "ros_domain_id": 28, "ros_localhost_only": False,
            "rmw_implementation": "rmw_cyclonedds_cpp", "screenshot_python": "/custom/python"}
    profile.write_text(json.dumps(data))
    monkeypatch.setenv("HERMES_SIM_CONFIG", str(profile))
    monkeypatch.setattr(transport, "CLIENT_CONFIG", home / "missing-client.json")
    for key in ("HERMES_ISAAC_URL", "HERMES_ISAAC_TOKEN_FILE"):
        monkeypatch.delenv(key, raising=False)
    return profile, data


def test_named_scenes_use_persisted_paths_and_reload_edits(configured, monkeypatch):
    profile, data = configured
    seen = []
    monkeypatch.setattr(scene_scripts, "command", lambda action, args: seen.append((action, args)) or {"success": True})
    assert scene_scripts.run_script({"script_name": "load_gongguan_xinhai"})["success"]
    assert seen[-1] == ("load", {"path": data["gongguan_usd"], "wait_seconds": 10})
    replacement = profile.parent / "replacement road.usdz"
    replacement.write_text("second scene")
    data["gongguan_usd"] = str(replacement)
    profile.write_text(json.dumps(data))
    assert scene_scripts.run_script({"script_name": "load_gongguan_xinhai"})["success"]
    assert seen[-1][1]["path"] == str(replacement)
    listed = scene_scripts.list_scripts()["scripts"]
    assert listed[0]["configured"] is True and listed[0]["scene_path"] == str(replacement)
    assert listed[1]["scene_path"] == data["rv1_usd"]


@pytest.mark.parametrize("value", ["", "/missing/custom.usdz"])
def test_missing_scene_requests_configuration_before_bridge(configured, monkeypatch, value):
    profile, data = configured
    data["gongguan_usd"] = value
    profile.write_text(json.dumps(data))
    monkeypatch.setattr(scene_scripts, "command", lambda *args: pytest.fail("must not contact bridge"))
    result = scene_scripts.run_script({"script_name": "load_gongguan_xinhai"})
    assert result["error"]["code"] == "CONFIGURATION_ERROR"
    assert "gongguan_usd" in result["error"]["message"]


def test_insertion_requires_configured_robot_before_bridge(configured, monkeypatch):
    profile, data = configured
    data["rv1_usd"] = ""
    profile.write_text(json.dumps(data))
    monkeypatch.setattr(scene_scripts, "command", lambda *args: pytest.fail("must not contact bridge"))
    result = scene_scripts.run_script({"script_name": "add_rv1_to_gongguan"})
    assert result["error"]["code"] == "CONFIGURATION_ERROR"
    assert "rv1_usd" in result["error"]["message"]


def test_transport_uses_external_state_and_port(configured):
    _, data = configured
    assert transport._client_settings() == ("http://127.0.0.1:18718", Path(data["state_root"]) / "isaac/token")


def test_malformed_profile_fails_before_network(configured, monkeypatch):
    profile, _ = configured
    profile.write_text('{"isaac_port": "bad"}')
    monkeypatch.setattr(transport.urllib.request, "build_opener", lambda *args: pytest.fail("must not open network"))
    assert transport.command("status")["error"]["code"] == "BRIDGE_CONNECTION_ERROR"


def test_export_root_is_outside_package_and_checks_escape(configured):
    _, data = configured
    manager = Manager(sys.executable, Path(data["state_root"]) / "isaac", display=":4")
    assert manager.screenshot_python == "/custom/python"
    supervisor = AssetSupervisor(manager)
    assert supervisor.project == Path(data["export_root"])
    assert supervisor._output(str(supervisor.project / "new run"), None) == supervisor.project / "new run"
    with pytest.raises(ValueError, match="export_root"):
        supervisor._output(str(supervisor.project / "../escape"), None)
    external = supervisor.project.parent / "external"
    external.mkdir()
    (supervisor.project / "linked").symlink_to(external, target_is_directory=True)
    with pytest.raises(ValueError, match="export_root"):
        supervisor._output(str(supervisor.project / "linked/output"), None)


def test_ros_library_discovery_preserves_explicit_rmw(tmp_path):
    library = tmp_path / "exts/isaacsim.ros2.core/jazzy/lib"
    library.mkdir(parents=True)
    (library / "librcl.so").touch()
    environment = {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    configure_ros_libraries(tmp_path / "python.sh", environment)
    assert environment["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"


def test_rv1_stage_template_does_not_reference_any_host_asset():
    template = (ROOT / "rv1/rv1_test.usda").read_text()
    assert "prepend references" not in template
    assert "/home/" not in template


def test_explicit_config_path_supports_bridge_help_without_starting(configured):
    profile, _ = configured
    result = subprocess.run([sys.executable, str(ROOT / "bridge.py"), "--config", str(profile), "--help"],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0
    assert "--screenshot-python" in result.stdout
    assert not (profile.parent / "state").exists()
