"""Optional authenticated Isaac gate. ROS-only profiles never call this adapter."""
import json
import os
from pathlib import Path
import urllib.request
import urllib.error
import urllib.parse
from deployment import load_config

class IsaacAdapter:
    def command(self, action, params=None):
        token = ""
        try:
            path = Path(__file__).parents[1].joinpath("isaac-client.json")
            config = json.loads(path.read_text()) if path.exists() else {}
            if not isinstance(config, dict) or set(config) - {"url", "token_file"}:
                raise ValueError("isaac-client.json accepts only url and token_file")
            if any(not isinstance(v, str) or not v.strip() for v in config.values()):
                raise ValueError("Isaac client settings must be nonempty strings")
            deployment = load_config()
            url = os.environ.get("HERMES_ISAAC_URL", config.get("url", "http://127.0.0.1:" + str(deployment["isaac_port"])))
            parts = urllib.parse.urlsplit(url)
            if parts.scheme not in {"http", "https"} or not parts.hostname or parts.username or parts.password or parts.query or parts.fragment:
                raise ValueError("Invalid Isaac bridge URL")
            token_file = Path(os.environ.get("HERMES_ISAAC_TOKEN_FILE", config.get("token_file", str(Path(deployment["state_root"]).expanduser() / "isaac/token")))).expanduser()
            token = os.environ.get("HERMES_ISAAC_TOKEN", "").strip() or token_file.read_text().strip()
            if not token:
                raise ValueError("Isaac bridge token is empty")
            req = urllib.request.Request(url.rstrip("/") + "/v1/command",
                    data=json.dumps({"action": action, **(params or {})}).encode(),
                    headers={"Authorization": "Bearer " + token, "Content-Type": "application/json"})
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(req, timeout=20) as response:
                result = json.load(response)
            if not isinstance(result, dict):
                raise ValueError("Invalid Isaac response")
            return result
        except (OSError, ValueError, urllib.error.URLError) as exc:
            message = str(exc).replace(token, "[redacted]") if token else str(exc)
            return {"success": False, "error": {"code": "ISAAC_ADAPTER_UNAVAILABLE", "message": message}}

    def disarm(self):
        status = self.command("status")
        if not status.get("success"):
            return status
        state = status.get("state", {})
        if not state.get("process_running") or not state.get("ros2"):
            return {"success": True, "already_disarmed": True}
        result = self.command("rv1_ros2_disarm", {"wait_seconds": 5})
        if result.get("pending"):
            return {"success": False, "error": {"code": "DISARM_NOT_CONFIRMED", "message": "Simulator disarm is still pending; inspect fresh state before reconfiguration"}}
        return result

    def arm(self, profile):
        status = self.command("status")
        if not status.get("success"):
            return status
        state = status.get("state", {})
        ros = state.get("ros2") or {}
        config = ros.get("config") or {}
        if not state.get("state_fresh") or state.get("timeline") != "playing" or not ros.get("started"):
            return {"success": False, "error": {"code": "ISAAC_NOT_READY", "message": "Fresh playing Isaac with a prepared ROS adapter required"}}
        expected = {"map_frame": profile["frames"]["map"], "odom_frame": profile["frames"]["odom"], "base_frame": profile["frames"]["base"],
                    **{k + "_topic": profile["topics"][k] for k in ("cmd_vel", "odom", "joint_states", "clock")}}
        if any(config.get(k) != v for k, v in expected.items()):
            return {"success": False, "error": {"code": "ROBOT_PROFILE_MISMATCH", "message": "Isaac adapter topics/frames differ from the navigation profile"}}
        result = self.command("rv1_ros2_arm", {"wait_seconds": 5})
        if result.get("pending"):
            return {"success": False, "error": {"code": "ARM_NOT_CONFIRMED", "message": "Simulator arm is still pending; navigation was not submitted"}}
        return result
