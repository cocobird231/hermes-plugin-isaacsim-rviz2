import json
from pathlib import Path
import sys
import threading
from http.server import ThreadingHTTPServer
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import make_handler
from manager import Manager
from transport import command


@pytest.fixture
def endpoint(tmp_path, monkeypatch):
    manager = Manager(sys.executable, tmp_path / "state")
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(manager, "test-token"))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    monkeypatch.setenv("HERMES_ISAAC_URL", url)
    monkeypatch.setenv("HERMES_ISAAC_TOKEN", "test-token")
    try:
        yield url
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_real_http_client_status_and_action_cannot_be_overridden(endpoint):
    assert command("status")["state"]["phase"] == "closed"
    assert command("status", {"action": "open"})["state"]["phase"] == "closed"


def test_wrong_token_is_rejected(endpoint, monkeypatch):
    monkeypatch.setenv("HERMES_ISAAC_TOKEN", "wrong-token")
    result = command("status")
    assert not result["success"]
    assert result["error"]["code"] == "UNAUTHORIZED"


def test_non_object_body_is_rejected(endpoint):
    request = urllib.request.Request(endpoint + "/v1/command", data=b"[]",
                                     headers={"Authorization": "Bearer test-token"})
    with urllib.request.urlopen(request) as response:
        result = json.load(response)
    assert result["error"]["code"] == "INVALID_ARGUMENT"


def test_no_arbitrary_code_endpoint(endpoint):
    request = urllib.request.Request(endpoint + "/execute", data=b"{}",
                                     headers={"Authorization": "Bearer test-token"})
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request)
    assert error.value.code == 404


@pytest.mark.parametrize("action", ["rv1_drive", "rv1_brake"])
def test_authenticated_raw_http_cannot_reach_removed_direct_controls(endpoint, action):
    request = urllib.request.Request(endpoint + "/v1/command",
                                     data=json.dumps({"action": action}).encode(),
                                     headers={"Authorization": "Bearer test-token"})
    with urllib.request.urlopen(request) as response:
        result = json.load(response)
    assert result["success"] is False
    assert result["error"]["code"] == "INVALID_ACTION"


@pytest.mark.parametrize("action", ["rv1_ros2_arm", "rv1_ros2_disarm"])
def test_nav2_adapter_endpoint_requires_owned_running_simulator(endpoint, action):
    result = command(action)
    assert result["success"] is False
    assert result["error"]["code"] == "NOT_RUNNING"


@pytest.mark.parametrize("action", ["rv1_ros2_arm", "rv1_ros2_disarm"])
def test_nav2_adapter_endpoint_requires_authentication(endpoint, monkeypatch, action):
    monkeypatch.setenv("HERMES_ISAAC_TOKEN", "wrong-token")
    result = command(action)
    assert result["error"]["code"] == "UNAUTHORIZED"
