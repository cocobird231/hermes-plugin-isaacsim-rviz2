"""Portable helper behavior; does not start Isaac, ROS, Hermes or systemd units."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import launch_bridges
import portable_common
from hermes_sim_config import load_config


@pytest.fixture
def configured(tmp_path, monkeypatch):
    directory = tmp_path / "different user (space)"
    directory.mkdir()
    interpreter = directory / "fake python"
    interpreter.write_text("#!/bin/sh\nexit 0\n")
    interpreter.chmod(0o755)
    setup = directory / "ROS setup.bash"
    setup.write_text("# fixture\n")
    config = directory / "my machine.json"
    config.write_text(json.dumps({"isaac_python": str(interpreter),
        "ros_python": sys.executable, "ros_setup": str(setup), "display_number": 7,
        "isaac_port": 18768, "rviz2_port": 18769, "ros_domain_id": 62,
        "state_root": str(directory / "private state"), "xauthority": "",
        "ros_localhost_only": True}))
    monkeypatch.setenv("HERMES_SIM_CONFIG", str(config))
    monkeypatch.setenv("HERMES_HOME", str(directory / "different hermes"))
    return config, load_config()


@pytest.mark.parametrize("kind,port", [("isaac", "18768"), ("rviz2", "18769")])
def test_plan_uses_config_and_keeps_spaces_as_single_arguments(configured, kind, port):
    path, config = configured
    plan = launch_bridges.launch_plan(kind, config)
    assert plan["argv"][plan["argv"].index("--port") + 1] == port
    assert plan["argv"][plan["argv"].index("--state-dir") + 1] == str(Path(config["state_root"]) / kind)
    assert "--setenv=HERMES_SIM_CONFIG=" + str(path) in plan["command"]
    assert "--setenv=ROS_LOCALHOST_ONLY=1" in plan["command"]
    assert "--setenv=ROS_DOMAIN_ID=62" in plan["command"]
    assert "--setenv=DISPLAY=:7" in plan["command"]
    assert plan["command"][-len(plan["argv"]):] == plan["argv"]
    assert "start" not in plan["command"] and "restart" not in plan["command"]


def test_unit_stable_for_same_file_but_fingerprint_detects_changed_config(configured):
    _, config = configured
    old = launch_bridges.launch_plan("isaac", config)
    new = launch_bridges.launch_plan("isaac", {**config, "display_number": 8})
    assert old["unit"] == new["unit"]
    assert old["description"] != new["description"]
    assert old["unit"] != launch_bridges.launch_plan("rviz2", config)["unit"]


def test_read_only_print_command(configured, monkeypatch, capsys):
    path, _ = configured
    monkeypatch.setattr(sys, "argv", ["launch_bridges.py", "isaac", "--config", str(path), "--print-command"])
    monkeypatch.setattr(launch_bridges.subprocess, "run", lambda *a, **k: pytest.fail("dry run launched subprocess"))
    assert launch_bridges.main() == 0
    assert json.loads(capsys.readouterr().out)["config"] == str(path)


def test_mismatched_active_service_refused_without_restart(configured, monkeypatch):
    path, _ = configured
    calls = []
    def fake_run(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0, "ActiveState=active\nDescription=other-service\nMainPID=1\n", "")
    monkeypatch.setattr(sys, "argv", ["launch_bridges.py", "isaac", "--config", str(path)])
    monkeypatch.setattr(launch_bridges.subprocess, "run", fake_run)
    with pytest.raises(SystemExit) as error:
        launch_bridges.main()
    assert error.value.code == 2
    assert len(calls) == 1 and calls[0][0] == "systemctl"


def test_owned_active_checks_process_arguments_and_config(tmp_path):
    config = str(tmp_path / "config with spaces.json")
    argv = [sys.executable, "-c", "import time; time.sleep(20)"]
    proc = subprocess.Popen(argv, env={**os.environ, "HERMES_SIM_CONFIG": config})
    try:
        plan = {"description": "expected", "argv": argv, "config": config}
        props = {"Description": "expected", "MainPID": str(proc.pid)}
        assert launch_bridges.owned_active(plan, props)
        assert not launch_bridges.owned_active({**plan, "config": config + "-different"}, props)
        assert not launch_bridges.owned_active({**plan, "argv": argv + ["different"]}, props)
    finally:
        proc.terminate()
        proc.wait(timeout=5)


def test_generated_evidence_cannot_enter_public_checkout(tmp_path):
    assert portable_common.private_output(tmp_path / "result.json") == tmp_path / "result.json"
    with pytest.raises(ValueError):
        portable_common.private_output(ROOT / "result.json")
    with pytest.raises(ValueError):
        portable_common.private_output(ROOT)


@pytest.mark.parametrize("script", ["plugin_call.py", "capture_desktop.py", "monitor_navigation.py",
    "setup_navigation.py", "run_hermes.py", "gateway_evidence.py", "export_hermes_tool_evidence.py"])
def test_help_works_without_host_paths_or_hermes_bootstrap(script):
    result = subprocess.run([sys.executable, str(ROOT / "scripts" / script), "--help"], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert "--config" in result.stdout


def test_run_hermes_uses_selected_executable_and_workdir(configured, tmp_path, monkeypatch):
    path, _ = configured
    source = ROOT / "scripts/run_hermes.py"
    spec = importlib.util.spec_from_file_location("portable_run_hermes_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    prompt = tmp_path / "prompt.txt"
    prompt.write_text("fixture")
    output = tmp_path / "private run"
    calls = []
    def fake_run(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0)
    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.setattr(sys, "argv", [str(source), str(prompt), str(output), "--config", str(path),
                                     "--hermes", "/fake hermes/cli", "--cwd", str(tmp_path), "--max-turns", "2"])
    prior = os.umask(0o077)
    try:
        assert module.main() == 0
    finally:
        os.umask(prior)
    assert calls[0][0] == "/fake hermes/cli"
    assert calls[0][calls[0].index("--in") + 1] == str(tmp_path)
    assert calls[0][calls[0].index("--max-turns") + 1] == "2"


def test_listener_check_distinguishes_another_process():
    import socket
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]
        assert launch_bridges.owns_listener(os.getpid(), port)
        assert not launch_bridges.owns_listener(99999999, port)


def test_exited_service_never_reports_ready(monkeypatch):
    monkeypatch.setattr(launch_bridges, "service_properties", lambda plan: {"ActiveState": "failed"})
    with pytest.raises(ValueError, match="exited before readiness"):
        launch_bridges.wait_ready({"unit": "fixture.service"}, 1)


def test_readiness_requires_owned_listener_and_authentication(monkeypatch):
    plan = {"unit": "fixture.service", "port": 12345}
    properties = {"ActiveState": "active", "MainPID": "123"}
    calls = []
    monkeypatch.setattr(launch_bridges, "service_properties", lambda plan: properties)
    monkeypatch.setattr(launch_bridges, "owned_active", lambda *args: True)
    monkeypatch.setattr(launch_bridges, "owns_listener", lambda pid, port: calls.append((pid, port)) or True)
    monkeypatch.setattr(launch_bridges, "authenticated_status", lambda *args: calls.append("authenticated") or True)
    assert launch_bridges.wait_ready(plan, 1) == 123
    assert calls == [(123, 12345), "authenticated"]


def test_authenticated_readiness_never_accepts_wrong_token(tmp_path):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    import threading
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            accepted = self.headers.get("Authorization") == "Bearer test-only-value"
            self.send_response(200 if accepted else 401)
            self.end_headers()
            self.wfile.write(json.dumps({"success": accepted}).encode())
        def log_message(self, *args):
            pass
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    token_file = tmp_path / "private-token"
    plan = {"token_file": str(token_file), "kind": "isaac", "port": server.server_port}
    try:
        token_file.write_text("wrong-test-only-value")
        assert not launch_bridges.authenticated_status(plan)
        token_file.write_text("test-only-value")
        assert launch_bridges.authenticated_status(plan)
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
