#!/usr/bin/env python3
"""Start one configured bridge as a user service; never restart an active service."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request
import urllib.error
from portable_common import PUBLIC_ROOT, configure
from hermes_sim_config import config_path


def launch_plan(kind, config):
    selected = str(config_path().expanduser().resolve())
    state = Path(config["state_root"]) / kind
    plugin = "isaac-sim" if kind == "isaac" else "rviz2-navigation"
    argv = [config["ros_python"], str(PUBLIC_ROOT / "codes" / plugin / "bridge.py"),
            "--state-dir", str(state), "--host", "127.0.0.1",
            "--port", str(config["isaac_port" if kind == "isaac" else "rviz2_port"])]
    if kind == "isaac":
        argv.extend(["--isaac-python", config["isaac_python"],
                     "--display", ":" + str(config["display_number"])])
        if config.get("xauthority"):
            argv.extend(["--xauthority", config["xauthority"]])
    else:
        argv.extend(["--ros-domain-id", str(config["ros_domain_id"])])
    environment = {"HERMES_SIM_CONFIG": selected,
                   "DISPLAY": ":" + str(config["display_number"]),
                   "ROS_DOMAIN_ID": str(config["ros_domain_id"]),
                   "ROS_LOCALHOST_ONLY": "1" if config["ros_localhost_only"] else "0",
                   "ROS_DISTRO": "jazzy", "RMW_IMPLEMENTATION": config["rmw_implementation"]}
    if os.environ.get("HERMES_HOME"):
        environment["HERMES_HOME"] = str(Path(os.environ["HERMES_HOME"]).expanduser().resolve())
    if config.get("xauthority"):
        environment["XAUTHORITY"] = config["xauthority"]
    if config.get("isaac_ros_lib_dir"):
        environment["HERMES_ISAAC_ROS_LIB_DIR"] = config["isaac_ros_lib_dir"]
    identity = hashlib.sha256(selected.encode()).hexdigest()[:12]
    fingerprint = hashlib.sha256(json.dumps({"argv": argv, "environment": environment,
        "config": config}, sort_keys=True).encode()).hexdigest()
    unit = f"hermes-sim-{kind}-{identity}.service"
    description = f"Hermes sim plugins {kind} {fingerprint}"
    command = ["systemd-run", "--user", "--collect", "--unit=" + unit,
               "--description=" + description, "--property=Type=exec", "--property=UMask=0077"]
    command += ["--setenv=" + key + "=" + value for key, value in environment.items()]
    command += ["--", *argv]
    return {"kind": kind, "unit": unit, "description": description,
            "config": selected, "argv": argv, "command": command,
            "port": config["isaac_port" if kind == "isaac" else "rviz2_port"],
            "token_file": str(state / "token")}


def owned_active(plan, properties):
    if properties.get("Description") != plan["description"]:
        return False
    try:
        pid = int(properties["MainPID"])
        proc = Path("/proc") / str(pid)
        argv = [os.fsdecode(part) for part in (proc / "cmdline").read_bytes().split(b"\0") if part]
        environment = dict(part.split(b"=", 1) for part in (proc / "environ").read_bytes().split(b"\0") if b"=" in part)
        return (pid > 0 and argv == plan["argv"] and
                os.fsdecode(environment.get(b"HERMES_SIM_CONFIG", b"")) == plan["config"])
    except (OSError, ValueError, KeyError):
        return False



def service_properties(plan):
    query = subprocess.run(["systemctl", "--user", "show", plan["unit"],
        "--property=ActiveState,Description,MainPID"], text=True, capture_output=True, timeout=5)
    return dict(line.split("=", 1) for line in query.stdout.splitlines() if "=" in line)


def owns_listener(pid, port):
    """Confirm the selected PID owns this loopback TCP listener, not another bridge."""
    try:
        sockets = set()
        for fd in (Path("/proc") / str(pid) / "fd").iterdir():
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target.startswith("socket:[") and target.endswith("]"):
                sockets.add(target[8:-1])
        for line in Path("/proc/net/tcp").read_text().splitlines()[1:]:
            fields = line.split()
            if (fields[3] == "0A" and fields[1] == f"0100007F:{port:04X}"
                    and fields[9] in sockets):
                return True
    except (OSError, ValueError, IndexError):
        pass
    return False


def authenticated_status(plan, timeout=2):
    """Read status without printing tokens, responses or untrusted error text."""
    try:
        token = Path(plan["token_file"]).read_text().strip()
        if not token:
            return False
        action = "status" if plan["kind"] == "isaac" else "nav_status"
        request = urllib.request.Request(f"http://127.0.0.1:{plan['port']}/v1/command",
            data=json.dumps({"action": action}).encode(),
            headers={"Authorization": "Bearer " + token, "Content-Type": "application/json"})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request, timeout=timeout) as response:
            result = json.load(response)
        return isinstance(result, dict) and result.get("success") is True
    except (OSError, ValueError, urllib.error.URLError):
        return False


def wait_ready(plan, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        properties = service_properties(plan)
        state = properties.get("ActiveState")
        if state in {"failed", "inactive", "deactivating"} or not state:
            raise ValueError("Bridge service exited before readiness; inspect journalctl --user -u " + plan["unit"])
        if state == "active":
            if not owned_active(plan, properties):
                raise ValueError("Bridge service ownership/configuration changed: " + plan["unit"])
            pid = int(properties["MainPID"])
            if owns_listener(pid, plan["port"]) and authenticated_status(plan, min(2, max(.1, deadline - time.monotonic()))):
                after = service_properties(plan)
                if after.get("ActiveState") == "active" and after.get("MainPID") == str(pid) and owned_active(plan, after):
                    return pid
        time.sleep(.1)
    raise ValueError("Bridge did not pass authenticated readiness within timeout; inspect " + plan["unit"])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("isaac", "rviz2"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--print-command", action="store_true", help="Print argv JSON without starting services")
    parser.add_argument("--wait-seconds", type=float, default=15, help="Authenticated bridge readiness deadline, 1-60 seconds")
    args = parser.parse_args()
    if not 1 <= args.wait_seconds <= 60:
        parser.error("wait-seconds must be in [1,60]")
    try:
        config = configure(args.config, required=("ros_python", "isaac_python") if args.kind == "isaac" else ("ros_python", "ros_setup"))
        plan = launch_plan(args.kind, config)
        if args.print_command:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return 0
        properties = service_properties(plan)
        if properties.get("ActiveState") in {"active", "activating", "reloading", "deactivating"}:
            if properties["ActiveState"] == "active" and owned_active(plan, properties):
                pid = wait_ready(plan, args.wait_seconds)
                print(json.dumps({"already_active": True, "ready": True, "bridge_pid": pid, "unit": plan["unit"], "config": plan["config"]}))
                return 0
            raise ValueError("Existing service does not match this configuration or is transitioning; inspect it without automatic restart: " + plan["unit"])
        completed = subprocess.run(plan["command"], check=False)
        if completed.returncode:
            return completed.returncode
        pid = wait_ready(plan, args.wait_seconds)
        print(json.dumps({"already_active": False, "ready": True, "bridge_pid": pid, "unit": plan["unit"], "config": plan["config"]}))
        return 0
    except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
