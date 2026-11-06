"""Standalone ROS process supervisor; Isaac is an optional HTTP control adapter."""
from __future__ import annotations
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

from rviz_navigation.profiles import validate_profile, default_rv1_profile, maximum_speed_rv1_profile
from rviz_navigation.isaac_adapter import IsaacAdapter
from deployment import load_config, config_path, PACKAGE_ROOT

FIELDS = {
    "nav_configure_robot": {"profile_name", "profile"},
    "nav_status": set(), "nav_map_load": {"map_yaml_path"},
    "nav_model_load": {"urdf_path"}, "nav_open_rviz": {"display_number"},
    "nav_close_rviz": set(), "nav_start": {"params_file"},
    "nav_set_initial_pose": {"x_m", "y_m", "yaw_rad"},
    "nav_set_goal": {"x_m", "y_m", "yaw_rad"},
    "nav_plan": {"goal_id"}, "nav_navigate": {"goal_id"},
    "nav_cancel": set(), "nav_stop": set(),
}


def fail(code, message, **extra):
    return {"success": False, "error": {"code": code, "message": str(message)}, **extra}


def identity(pid):
    try:
        p = Path(f"/proc/{pid}")
        if p.stat().st_uid != os.getuid():
            return None
        fields = (p / "stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None
    except (OSError, IndexError):
        return None


def save(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    temporary.chmod(0o600)
    temporary.replace(path)


class NavigationSupervisor:
    def __init__(self, base, *, ros_domain_id=None, isaac=None, ros_python=None, ros_setup=None,
                 display_number=None, xauthority=None, rmw_implementation=None, ros_localhost_only=None):
        config = load_config()
        self.project = PACKAGE_ROOT
        self.ros_python = ros_python or config["ros_python"]
        self.ros_setup = ros_setup or config["ros_setup"]
        self.display_number = config["display_number"] if display_number is None else display_number
        self.xauthority = config.get("xauthority") if xauthority is None else xauthority
        self.rmw_implementation = rmw_implementation or config["rmw_implementation"]
        self.ros_localhost_only = config["ros_localhost_only"] if ros_localhost_only is None else ros_localhost_only
        if ros_domain_id is None:
            ros_domain_id = config["ros_domain_id"]
        self.base = Path(base).expanduser().resolve()
        self.base.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.base.chmod(0o700)
        self.lock = threading.RLock()
        self.children = {}
        self.worker_file = self.base / "worker.json"
        self.profile_file = self.base / "robot-profile.json"
        self.profile = validate_profile(json.loads(self.profile_file.read_text())) if self.profile_file.exists() else None
        if isinstance(ros_domain_id, bool) or not isinstance(ros_domain_id, int) or not 0 <= ros_domain_id <= 232:
            raise ValueError("ros_domain_id must be an integer from 0 through 232")
        self.ros_domain_id = ros_domain_id
        self.isaac = isaac if isaac is not None else IsaacAdapter()

    def _alive(self, record):
        child = self.children.get(record.get("pid"))
        if child:
            child.poll()
        return bool(record.get("start_ticks") and identity(record["pid"]) == record["start_ticks"])

    def _record(self):
        return json.loads(self.worker_file.read_text()) if self.worker_file.exists() else {}

    def _ensure_worker(self):
        if not self.profile:
            raise ValueError("Call rviz2_configure_robot before starting RViz or navigation")
        record = self._record()
        if record and self._alive(record):
            return
        run_dir = self.base / "ros"
        run_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        if len(os.fsencode(run_dir / "ros.sock")) >= 108:
            raise ValueError("Use a shorter navigation state directory")
        (run_dir / "ros.sock").unlink(missing_ok=True)
        worker = Path(__file__).with_name("ros_worker.py")
        if not Path(self.ros_setup).is_file():
            raise ValueError("Configured ros_setup file does not exist")
        command = ["/bin/bash", "-c", 'source "$1" && shift && exec "$@"',
                   "rviz2-navigation", self.ros_setup, self.ros_python, str(worker), "--run-dir", str(run_dir), "--profile", str(self.profile_file)]
        env = os.environ.copy()
        for key in ("PYTHONHOME", "PYTHONPATH", "LD_LIBRARY_PATH", "LD_PRELOAD", "CONDA_PREFIX"):
            env.pop(key, None)
        env.update(ROS_DOMAIN_ID=str(self.ros_domain_id), ROS_LOCALHOST_ONLY="1" if self.ros_localhost_only else "0",
                   RMW_IMPLEMENTATION=self.rmw_implementation, DISPLAY=f":{self.display_number}",
                   HERMES_SIM_CONFIG=str(config_path()))
        if self.xauthority:
            env["XAUTHORITY"] = self.xauthority
        with (run_dir / "process.log").open("ab", buffering=0) as log:
            child = subprocess.Popen(command, cwd=self.project, env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.children[child.pid] = child
        record = {"pid": child.pid, "start_ticks": identity(child.pid), "command": command,
                  "robot_id": self.profile["robot_id"], "log_path": str(run_dir / "process.log"), "created_wall": time.time()}
        save(self.worker_file, record)
        def reap():
            code = child.wait()
            save(run_dir / "exit.json", {"pid": child.pid, "exit_code": code, "finished_wall": time.time()})
        threading.Thread(target=reap, daemon=True).start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if not self._alive(record):
                raise RuntimeError(f"ROS worker exited; inspect {record['log_path']}")
            if (run_dir / "ros.sock").exists():
                return
            time.sleep(.1)
        raise TimeoutError(f"ROS worker not ready; inspect {record['log_path']}")

    def _rpc(self, args, *, ensure=False):
        if ensure:
            self._ensure_worker()
        path = self.base / "ros/ros.sock"
        if not path.exists():
            return fail("NAV_WORKER_NOT_RUNNING", "No managed ROS worker is running")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(25)
            sock.connect(str(path))
            sock.sendall(json.dumps(args).encode() + b"\n")
            data = bytearray()
            while b"\n" not in data:
                block = sock.recv(65536)
                if not block:
                    raise RuntimeError("ROS worker disconnected")
                data.extend(block)
                if len(data) > 4 * 1024 * 1024:
                    raise ValueError("ROS response exceeds limit")
        return json.loads(data.split(b"\n", 1)[0])

    def disarm(self):
        if self.profile and self.profile["control_backend"] == "isaac_rv1":
            return self.isaac.disarm()
        return {"success": True, "backend": "ros2", "isaac_required": False}

    def cancel_control(self):
        # Close the local relay first: an unavailable optional simulator HTTP
        # endpoint must not delay zero velocity or leave Nav2 commands flowing.
        try:
            record = self._record()
            ros = self._rpc({"action": "cancel"}) if record and self._alive(record) else {"success": True, "already_stopped": True}
        except Exception as exc:
            ros = fail("ROS_CANCEL_FAILED", str(exc))
        try:
            gate = self.disarm()
        except Exception as exc:
            gate = fail("DISARM_FAILED", str(exc))
        if not ros.get("success") or not gate.get("success"):
            return fail("CANCEL_NOT_CONFIRMED", "One or more control gates could not be confirmed stopped",
                        disarm=gate, ros_cancel=ros)
        return {**ros, "disarm": gate}

    def configure(self, params):
        if set(params) not in ({"profile_name"}, {"profile"}):
            raise ValueError("Supply exactly profile_name or profile")
        if "profile_name" in params:
            presets = {"rv1": default_rv1_profile, "rv1_max_speed": maximum_speed_rv1_profile}
            if params["profile_name"] not in presets:
                raise ValueError("Built-ins are rv1 and rv1_max_speed; supply a full profile for another robot")
            profile = presets[params["profile_name"]](self.project)
        else:
            profile = validate_profile(params["profile"])
        if self.profile == profile:
            return {"success": True, "already_configured": True, "profile": profile}
        record = self._record()
        if record and self._alive(record):
            state = self._rpc({"action": "status"})
            if not state.get("success"):
                return state
            if any(p.get("running") for p in state.get("state", {}).get("processes", {}).values()):
                return fail("PROFILE_BUSY", "Call rviz2_navigation_stop and verify stopped before changing profiles")
            cancelled = self.cancel_control()
            if not cancelled.get("success"):
                return cancelled
            # PID + UID + birth tick guard; only the child recorded by this service.
            if self._alive(record):
                os.kill(record["pid"], signal.SIGTERM)
            deadline = time.monotonic() + 15
            while self._alive(record) and time.monotonic() < deadline:
                time.sleep(.1)
            if self._alive(record):
                return fail("WORKER_STOP_PENDING", "The previous profile worker has not exited; retry after inspection")
        save(self.profile_file, profile)
        self.profile = profile
        return {"success": True, "profile": profile, "robot_id": profile["robot_id"], "worker_started": False}

    def dispatch(self, args):
        if not isinstance(args, dict):
            return fail("INVALID_ARGUMENT", "Request must be a JSON object")
        action = args.get("action")
        if not isinstance(action, str) or action not in FIELDS or set(args) - (FIELDS[action] | {"action", "robot_id"}):
            return fail("INVALID_ARGUMENT", "Unknown RViz/navigation action or argument")
        try:
            with self.lock:
                params = {k: v for k, v in args.items() if k != "action"}
                if action == "nav_configure_robot":
                    return self.configure(params)
                if args.get("robot_id") is not None and (not self.profile or args["robot_id"] != self.profile["robot_id"]):
                    return fail("ROBOT_MISMATCH", "Request robot_id differs from configured robot")
                if action == "nav_status":
                    record = self._record()
                    if not record or not self._alive(record):
                        return {"success": True, "state": {"worker_running": False, "profile": self.profile,
                                                           "robot_id": self.profile["robot_id"] if self.profile else None}}
                    result = self._rpc({"action": "status", **params})
                    result.setdefault("state", {})["profile"] = self.profile
                    return result
                if not self.profile:
                    return fail("PROFILE_REQUIRED", "Use rviz2_configure_robot first")
                if action in {"nav_cancel", "nav_stop", "nav_map_load", "nav_model_load"}:
                    result = self.cancel_control()
                    if not result.get("success") or action == "nav_cancel":
                        return result
                    if action == "nav_stop" and not self._alive(self._record()):
                        return {"success": True, "already_stopped": True, "owned_processes_remaining": 0}
                if action == "nav_navigate":
                    ready = self._rpc({"action": "navigation_ready", **params}, ensure=True)
                    if not ready.get("success"):
                        return ready
                    if self.profile["control_backend"] == "isaac_rv1":
                        armed = self.isaac.arm(self.profile)
                        if not armed.get("success"):
                            self.disarm()
                            return armed
                if action == "nav_open_rviz":
                    params.setdefault("display_number", self.display_number)
                result = self._rpc({"action": action.removeprefix("nav_"), **params}, ensure=True)
                if action == "nav_navigate" and not result.get("success"):
                    self.disarm()
                return result
        except (OSError, ValueError, KeyError, TypeError, RuntimeError, ImportError) as exc:
            if action == "nav_navigate":
                try:
                    self.cancel_control()
                except Exception:
                    pass
            return fail("NAVIGATION_ERROR", str(exc))
