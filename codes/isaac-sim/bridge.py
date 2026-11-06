"""Authenticated, restricted host endpoint for Hermes in another environment."""
from __future__ import annotations

import argparse
import fcntl
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets

from manager import Manager, fail
from deployment import load_config, config_path


def make_handler(manager, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # Never log Authorization headers or scene paths.

        def respond(self, code, data):
            raw = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            try:
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Accepted commands can outlive a disconnected caller.

        def do_POST(self):
            self.connection.settimeout(10)
            if not hmac.compare_digest(self.headers.get("Authorization", ""), "Bearer " + token):
                return self.respond(401, fail("UNAUTHORIZED", "Configure the bridge token."))
            if self.path != "/v1/command":
                return self.respond(404, fail("NOT_FOUND", "Use POST /v1/command."))
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 65536:
                    return self.respond(413, fail("INVALID_REQUEST", "Body must be 1..65536 bytes."))
                args = json.loads(self.rfile.read(length))
            except (ValueError, OSError) as exc:
                return self.respond(400, fail("INVALID_REQUEST", str(exc)))
            self.respond(200, manager.dispatch(args))

    return Handler


def main():
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--config")
    early, _ = bootstrap.parse_known_args()
    if early.config:
        os.environ["HERMES_SIM_CONFIG"] = str(Path(early.config).expanduser().resolve())
    defaults = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Host deployment JSON; defaults to HERMES_SIM_CONFIG or HERMES_HOME/sim-plugins.json")
    parser.add_argument("--isaac-python", default=defaults.get("isaac_python"), help="Absolute Isaac python.sh or Python executable")
    parser.add_argument("--state-dir", type=Path, default=Path(defaults["state_root"]) / "isaac")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=defaults["isaac_port"])
    parser.add_argument("--display", default=":" + str(defaults["display_number"]),
                        help="Default X display for new GUI sessions (default :1); open display_number overrides it")
    parser.add_argument("--xauthority", default=defaults.get("xauthority"))
    parser.add_argument("--screenshot-python", default=defaults["screenshot_python"])
    parser.add_argument("--ros-domain-id", type=int, default=defaults["ros_domain_id"])
    parser.add_argument("--rmw-implementation", default=defaults["rmw_implementation"])
    parser.add_argument("--ros-localhost-only", choices=("0", "1"), default="1" if defaults["ros_localhost_only"] else "0")
    options = parser.parse_args()
    if not options.isaac_python:
        parser.error("Configure isaac_python with bash install.sh or pass --isaac-python")
    os.environ["HERMES_SIM_CONFIG"] = str(config_path())
    os.environ.update(ROS_DOMAIN_ID=str(options.ros_domain_id), RMW_IMPLEMENTATION=options.rmw_implementation,
                      ROS_LOCALHOST_ONLY=options.ros_localhost_only)
    manager = Manager(options.isaac_python, options.state_dir, options.display, options.xauthority,
                      screenshot_python=options.screenshot_python)
    with (manager.base / "bridge.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another bridge owns this state directory.")
        token_path = manager.base / "token"
        if not token_path.exists():
            fd = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w") as stream:
                stream.write(secrets.token_urlsafe(32))
        token_path.chmod(0o600)
        token = token_path.read_text().strip()
        if not token:
            parser.error("Token file is empty.")
        with ThreadingHTTPServer((options.host, options.port), make_handler(manager, token)) as server:
            print(f"Isaac bridge ready at {options.host}:{options.port}; token file: {token_path}", flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
