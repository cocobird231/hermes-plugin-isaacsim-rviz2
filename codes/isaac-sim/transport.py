"""Hermes-side client; works without Isaac or ROS packages."""
import json
import os
from pathlib import Path
import urllib.error
import urllib.parse
import urllib.request


if __package__:
    from .deployment import load_config
else:
    from deployment import load_config


CLIENT_CONFIG = Path(__file__).resolve().with_name("bridge-client.json")


def _client_settings():
    """Read this plugin copy's non-secret settings, then apply per-field env overrides."""
    try:
        config = json.loads(CLIENT_CONFIG.read_text())
    except FileNotFoundError:
        config = {}
    if not isinstance(config, dict) or set(config) - {"url", "token_file"}:
        raise ValueError("bridge-client.json accepts only url and token_file; never store a token in it.")
    if any(not isinstance(value, str) or not value.strip() for value in config.values()):
        raise ValueError("bridge-client.json settings must be non-empty strings.")
    deployment = load_config()
    default_url = "http://127.0.0.1:" + str(deployment["isaac_port"])
    default_token = str(Path(deployment["state_root"]).expanduser() / "isaac" / "token")
    url = os.environ.get("HERMES_ISAAC_URL", config.get("url", default_url))
    token_file = os.environ.get("HERMES_ISAAC_TOKEN_FILE", config.get("token_file", default_token))
    parsed = urllib.parse.urlsplit(url)
    if (parsed.scheme not in {"http", "https"} or not parsed.hostname
            or parsed.username is not None or parsed.password is not None
            or parsed.query or parsed.fragment):
        raise ValueError("Bridge URL must be HTTP(S) with a host and no credentials, query or fragment.")
    if not token_file.strip():
        raise ValueError("Bridge token_file must be a non-empty path.")
    return url.rstrip("/"), Path(token_file).expanduser()


def command(action: str, params=None) -> dict:
    args = {**(params or {}), "action": action}
    token = ""
    try:
        url, token_file = _client_settings()
        token = os.environ.get("HERMES_ISAAC_TOKEN", "").strip()
        if not token:
            token = token_file.read_text().strip()
        if not token:
            raise ValueError("Bridge token is empty.")
        request = urllib.request.Request(url + "/v1/command", data=json.dumps(args).encode(),
                                         headers={"Content-Type": "application/json", "Authorization": "Bearer " + token})
        # Local bridge should not route via the agent container's HTTP proxy.
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(request, timeout=105) as response:
                result = json.load(response)
        except urllib.error.HTTPError as exc:
            result = json.loads(exc.read())
        if not isinstance(result, dict):
            raise ValueError("Bridge returned a non-object response.")
        return result
    except (OSError, ValueError, urllib.error.URLError) as exc:
        message = str(exc).replace(token, "[redacted]") if token else str(exc)
        return {"success": False, "error": {"code": "BRIDGE_CONNECTION_ERROR", "message": message},
                "next_action": "Check host bridge and token configuration. If a request timed out, query isaac_sim_status before retrying."}
