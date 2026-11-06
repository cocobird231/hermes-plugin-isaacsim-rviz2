"""Machine-independent ROS launch/configuration; no simulator or navigation goals."""
import io
import json
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rviz_navigation.supervisor import NavigationSupervisor
from rviz_navigation.isaac_adapter import IsaacAdapter
import rviz_navigation.isaac_adapter as adapter_module
import transport


@pytest.fixture
def configured(tmp_path, monkeypatch):
    profile = tmp_path / "deployment.json"
    data = {"state_root": str(tmp_path / "state"), "export_root": str(tmp_path / "exports"),
            "ros_python": str(tmp_path / "ros python"), "ros_setup": str(tmp_path / "ros setup.bash"),
            "ros_domain_id": 52, "ros_localhost_only": False, "rmw_implementation": "rmw_cyclonedds_cpp",
            "display_number": 8, "xauthority": str(tmp_path / "display auth"),
            "isaac_port": 18718, "rviz2_port": 18719}
    profile.write_text(json.dumps(data))
    monkeypatch.setenv("HERMES_SIM_CONFIG", str(profile))
    for key in ("HERMES_RVIZ2_URL", "HERMES_RVIZ2_TOKEN_FILE", "HERMES_ISAAC_URL", "HERMES_ISAAC_TOKEN_FILE", "HERMES_ISAAC_TOKEN"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(transport, "CLIENT_CONFIG", tmp_path / "no-client.json")
    return profile, data


def test_rviz_connection_and_supervisor_use_same_deployment(configured):
    _, config = configured
    assert transport._client_settings() == ("http://127.0.0.1:18719", Path(config["state_root"]) / "rviz2/token")
    supervisor = NavigationSupervisor(Path(config["state_root"]) / "rviz2")
    assert supervisor.ros_python == config["ros_python"]
    assert supervisor.ros_setup == config["ros_setup"]
    assert supervisor.ros_domain_id == 52
    assert supervisor.display_number == 8
    assert supervisor.rmw_implementation == "rmw_cyclonedds_cpp"
    assert supervisor.ros_localhost_only is False
    assert supervisor.xauthority == config["xauthority"]
    assert supervisor.project == ROOT.parents[1]


def test_open_rviz_uses_profile_display_unless_explicit(configured, monkeypatch):
    _, config = configured
    supervisor = NavigationSupervisor(Path(config["state_root"]) / "rviz2")
    assert supervisor.configure({"profile_name": "rv1"})["success"]
    seen = []
    monkeypatch.setattr(supervisor, "_rpc", lambda args, **kwargs: seen.append(args) or {"success": True})
    assert supervisor.dispatch({"action": "nav_open_rviz"})["success"]
    assert seen[-1]["display_number"] == 8
    assert supervisor.dispatch({"action": "nav_open_rviz", "display_number": 2})["success"]
    assert seen[-1]["display_number"] == 2


def test_isaac_adapter_without_bundled_client_json(configured, monkeypatch):
    _, config = configured
    token_path = Path(config["state_root"]) / "isaac/token"
    token_path.parent.mkdir(parents=True)
    token_path.write_text("fixture-token")
    # Move only the adapter's reference location, never the installed plugin file.
    monkeypatch.setattr(adapter_module, "__file__", str(token_path.parent / "no-plugin/isaac_adapter.py"))
    seen = []
    class Opener:
        def open(self, request, timeout):
            seen.append(request)
            return io.BytesIO(b'{"success": true}')
    monkeypatch.setattr(adapter_module.urllib.request, "build_opener", lambda *args: Opener())
    assert IsaacAdapter().command("status")["success"]
    assert seen[0].full_url == "http://127.0.0.1:18718/v1/command"
    assert seen[0].get_header("Authorization") == "Bearer fixture-token"


def test_ros_setup_and_python_with_spaces_are_shell_arguments(tmp_path):
    setup = tmp_path / "setup with spaces.bash"
    setup.write_text('export PORTABLE_TEST_VALUE="configured setup"\n')
    launcher = tmp_path / "python with spaces"
    launcher.symlink_to(sys.executable)
    command = ["/bin/bash", "-c", 'source "$1" && shift && exec "$@"', "rviz2-navigation", str(setup), str(launcher),
               "-c", 'import os; print(os.environ["PORTABLE_TEST_VALUE"])']
    result = subprocess.run(command, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0 and result.stdout.strip() == "configured setup"


def test_explicit_profile_selects_bridge_defaults_without_starting(configured):
    profile, _ = configured
    result = subprocess.run([sys.executable, str(ROOT / "bridge.py"), "--config", str(profile), "--help"],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0
    assert "--ros-python" in result.stdout and "--display-number" in result.stdout
    assert not (profile.parent / "state").exists()


def test_relative_preset_path_does_not_depend_on_process_cwd(configured, tmp_path, monkeypatch):
    from rviz_navigation.profiles import default_rv1_profile, validate_profile, load_profile
    profile = default_rv1_profile(ROOT.parents[1])
    profile["params_file"] = "configs/rv1_nav2_max_speed.yaml"
    monkeypatch.chdir(tmp_path)
    validated = validate_profile(profile)
    expected = str(ROOT.parents[1] / "configs/rv1_nav2_max_speed.yaml")
    assert validated["params_file"] == expected
    file = tmp_path / "robot.json"
    file.write_text(json.dumps(profile))
    assert load_profile(file)["params_file"] == expected
