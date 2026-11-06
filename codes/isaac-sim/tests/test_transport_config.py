"""Select the plugin copy's bridge without changing Hermes service environment."""
import io
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import transport


@pytest.fixture
def client(tmp_path, monkeypatch):
    for name in ("HERMES_ISAAC_URL", "HERMES_ISAAC_TOKEN_FILE", "HERMES_ISAAC_TOKEN"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("HERMES_SIM_CONFIG", str(tmp_path / "missing-deployment.json"))
    monkeypatch.setenv("HERMES_HOME", str(tmp_path / "hermes-home"))
    config = tmp_path / "bridge-client.json"
    monkeypatch.setattr(transport, "CLIENT_CONFIG", config)
    captured = []

    class Opener:
        def open(self, request, timeout):
            captured.append(request)
            return io.BytesIO(b'{"success":true,"state":{"phase":"closed"}}')

    monkeypatch.setattr(transport.urllib.request, "build_opener", lambda *args: Opener())
    return config, captured


def token_file(directory, name, value):
    path = directory / name
    path.write_text(value + "\n")
    return str(path)


def test_missing_config_uses_portable_defaults(client):
    config, captured = client
    url, path = transport._client_settings()
    assert url == "http://127.0.0.1:8768"
    assert path == config.parent / "hermes-home/sim-plugins-state/isaac/token"
    assert not config.exists()
    assert captured == []


def test_plugin_local_config_routes_request_and_keeps_token_out_of_result(client):
    config, captured = client
    secret = "secret-local-bridge-token"
    path = token_file(config.parent, "local-token", secret)
    config.write_text(json.dumps({"url": "http://127.0.0.1:8768/", "token_file": path}))
    result = transport.command("status", {"action": "close"})
    assert result["success"]
    assert captured[0].full_url == "http://127.0.0.1:8768/v1/command"
    assert captured[0].get_header("Authorization") == "Bearer " + secret
    assert json.loads(captured[0].data) == {"action": "status"}
    assert secret not in json.dumps(result)
    assert secret not in config.read_text()


@pytest.mark.parametrize("url_override,token_override", [(True, False), (False, True), (True, True)])
def test_environment_overrides_each_field_independently(client, monkeypatch, url_override, token_override):
    config, captured = client
    local_path = token_file(config.parent, "local-token", "local-value")
    env_path = token_file(config.parent, "env-token", "environment-value")
    config.write_text(json.dumps({"url": "http://127.0.0.1:8768", "token_file": local_path}))
    if url_override:
        monkeypatch.setenv("HERMES_ISAAC_URL", "http://127.0.0.1:18768")
    if token_override:
        monkeypatch.setenv("HERMES_ISAAC_TOKEN_FILE", env_path)
    assert transport.command("status")["success"]
    assert captured[0].full_url == f"http://127.0.0.1:{18768 if url_override else 8768}/v1/command"
    assert captured[0].get_header("Authorization") == "Bearer " + (
        "environment-value" if token_override else "local-value")


def test_direct_token_environment_remains_compatible(client, monkeypatch):
    config, captured = client
    config.write_text(json.dumps({"url": "http://127.0.0.1:8768", "token_file": "/missing-token"}))
    monkeypatch.setenv("HERMES_ISAAC_TOKEN", "direct-legacy-value")
    assert transport.command("status")["success"]
    assert captured[0].get_header("Authorization") == "Bearer direct-legacy-value"


def test_partial_config_retains_unspecified_deployment_default(client):
    config, _ = client
    config.write_text(json.dumps({"url": "http://127.0.0.1:8768"}))
    assert transport._client_settings() == (
        "http://127.0.0.1:8768", config.parent / "hermes-home/sim-plugins-state/isaac/token")
    config.write_text(json.dumps({"token_file": str(config.parent / "token")}))
    assert transport._client_settings() == ("http://127.0.0.1:8768", config.parent / "token")


def test_config_is_reread_between_commands(client):
    config, captured = client
    path = token_file(config.parent, "token", "selected-token-value")
    for port in (8768, 18768):
        config.write_text(json.dumps({"url": f"http://127.0.0.1:{port}", "token_file": path}))
        assert transport.command("status")["success"]
    assert [request.full_url for request in captured] == [
        "http://127.0.0.1:8768/v1/command", "http://127.0.0.1:18768/v1/command"]


@pytest.mark.parametrize("value", [[], None, {"token": "do-not-disclose-this"},
    {"url": 8768}, {"token_file": ""}, {"unexpected": "do-not-disclose-this"}])
def test_invalid_config_fails_without_network_or_echoing_contents(client, value):
    config, captured = client
    config.write_text(json.dumps(value))
    result = transport.command("status")
    assert not result["success"]
    assert result["error"]["code"] == "BRIDGE_CONNECTION_ERROR"
    assert "do-not-disclose-this" not in json.dumps(result)
    assert captured == []


def test_malformed_json_does_not_fall_back_to_legacy_bridge(client):
    config, captured = client
    config.write_text('{"token":"never-echo-malformed-secret",')
    result = transport.command("status")
    assert not result["success"]
    assert "never-echo-malformed-secret" not in json.dumps(result)
    assert captured == []


@pytest.mark.parametrize("url", ["http://user:do-not-disclose-this@127.0.0.1:8768",
    "http://127.0.0.1:8768?token=do-not-disclose-this", "file:///tmp/bridge"])
def test_url_does_not_embed_credentials_or_use_non_http_transport(client, url):
    config, captured = client
    config.write_text(json.dumps({"url": url}))
    result = transport.command("status")
    assert not result["success"]
    assert "do-not-disclose-this" not in json.dumps(result)
    assert captured == []


def test_connection_exception_redacts_bearer_token(client, monkeypatch):
    config, _ = client
    secret = "never-show-this-auth-value"
    path = token_file(config.parent, "token", secret)
    config.write_text(json.dumps({"url": "http://127.0.0.1:8768", "token_file": path}))

    class BrokenOpener:
        def open(self, request, timeout):
            raise ValueError("Invalid header: " + request.get_header("Authorization"))

    monkeypatch.setattr(transport.urllib.request, "build_opener", lambda *args: BrokenOpener())
    result = transport.command("status")
    assert not result["success"]
    assert secret not in json.dumps(result)
    assert "[redacted]" in result["error"]["message"]
