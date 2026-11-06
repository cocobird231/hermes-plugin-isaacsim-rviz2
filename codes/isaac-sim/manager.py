"""Host-side owner of one Isaac process. No Isaac imports in Hermes Python."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
import uuid

from capture import CaptureError, capture_managed_window
from deployment import load_config, config_path


def read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Not a JSON object: {path}")
    return value


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
    temporary.chmod(0o600)
    temporary.replace(path)


def process_identity(pid: int) -> tuple[str, str] | None:
    """Linux start ticks guard against PID reuse; zombies are not alive."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19], fields[0]
    except (OSError, IndexError):
        return None


def process_matches(pid: int, start_ticks: str) -> bool:
    identity = process_identity(pid)
    return bool(identity and identity[0] == start_ticks and identity[1] != "Z")


def fail(code: str, message: str, **extra) -> dict:
    return {"success": False, "error": {"code": code, "message": message}, **extra}


class Manager:
    def __init__(self, isaac_python: str, state_dir: Path, display=":1", xauthority=None, *, screenshot_python=None):
        self.deployment = load_config()
        self.screenshot_python = screenshot_python or self.deployment["screenshot_python"]
        self.python = Path(isaac_python).expanduser().absolute()
        self.base = state_dir.expanduser().absolute()
        self.base.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.base.chmod(0o700)
        self.pointer = self.base / "session.json"
        self.display = ":1" if display is None else display
        self.xauthority = xauthority
        self.lock = threading.Lock()
        self.children: dict[str, subprocess.Popen] = {}
        self.assets = None

    def _assets(self):
        if self.assets is None:
            from navigation.supervisor import AssetSupervisor
            self.assets = AssetSupervisor(self)
        return self.assets

    def _session(self):
        return read_json(self.pointer) if self.pointer.exists() else None

    def _owned_pids(self, session):
        child = self.children.get(session["session_id"])
        if child is not None:
            child.poll()  # Reap only our own children.
        candidates = [(session["launcher_pid"], session["start_ticks"])]
        state_path = Path(session["run_dir"]) / "state.json"
        if state_path.exists():
            state = read_json(state_path)
            if state.get("session_id") == session["session_id"] and state.get("start_ticks"):
                candidates.append((state["pid"], state["start_ticks"]))
        owned = []
        for pid, ticks in candidates:
            if process_matches(pid, ticks):
                try:
                    if os.getpgid(pid) == session["launcher_pid"]:
                        owned.append(pid)
                except ProcessLookupError:
                    pass
        return owned

    def _alive(self, session):
        return bool(self._owned_pids(session))

    def _session_display(self, session, owned_pids):
        if session.get("headless"):
            return None
        if "display" in session:
            return session["display"]
        # Legacy session pointers did not record DISPLAY. Recover it only from
        # an owned live process, never from the bridge's current configuration.
        for pid in reversed(owned_pids):
            identity = process_identity(pid)
            try:
                environment = Path(f"/proc/{pid}/environ").read_bytes()
            except OSError:
                continue
            if identity is None or not process_matches(pid, identity[0]):
                continue
            for entry in environment.split(b"\0"):
                if entry.startswith(b"DISPLAY="):
                    return os.fsdecode(entry[len(b"DISPLAY="):]) or None
        return None

    @staticmethod
    def _display_number(display):
        match = re.search(r":(\d+)(?:\.\d+)?$", display or "")
        return int(match.group(1)) if match else None

    def status(self) -> dict:
        session = self._session()
        if session is None:
            return {"success": True, "state": {"phase": "closed", "process_running": False,
                                                "headless": None, "display": None, "display_number": None}}
        owned_pids = self._owned_pids(session)
        alive = bool(owned_pids)
        state_path = Path(session["run_dir"]) / "state.json"
        state = read_json(state_path) if state_path.exists() else {
            "session_id": session["session_id"], "phase": "starting",
            "stage_path": None, "timeline": "stopped", "operation": None,
        }
        if state.get("session_id") != session["session_id"]:
            return fail("SESSION_MISMATCH", "Runtime state belongs to a different session.")
        state.update(process_running=alive, launcher_pid=session["launcher_pid"],
                     log_path=str(Path(session["run_dir"]) / "isaac.log"))
        display = self._session_display(session, owned_pids)
        state.update(headless=session.get("headless"), display=display,
                     display_number=self._display_number(display))
        age = max(0.0, time.monotonic() - state.get("updated_monotonic", 0))
        state.update(state_age_seconds=round(age, 3), state_fresh=age < 10.0)
        if not alive and state.get("phase") != "closed":
            state["phase"] = "error"
            state.setdefault("error", {"code": "PROCESS_EXITED", "message": "Isaac exited; inspect log_path."})
        return {"success": True, "state": state}

    def _rpc(self, action: str, request_id: str, **params) -> dict:
        session = self._session()
        if session is None or not self._alive(session):
            return fail("NOT_RUNNING", "Call isaac_sim_open first.", **self._state_only())
        request = {"action": action, "session_id": session["session_id"],
                   "request_id": request_id, **params}
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(35)
                client.connect(str(Path(session["run_dir"]) / "bridge.sock"))
                client.sendall(json.dumps(request).encode() + b"\n")
                received = bytearray()
                while b"\n" not in received:
                    block = client.recv(8192)
                    if not block:
                        raise ConnectionError("Runtime closed connection before response.")
                    received.extend(block)
                    if len(received) > 262144:
                        raise ValueError("Runtime response too large.")
            result = json.loads(received.split(b"\n", 1)[0])
            if not isinstance(result, dict):
                raise ValueError("Invalid runtime response.")
            return result
        except (OSError, ValueError) as exc:
            return fail("RUNTIME_UNAVAILABLE", str(exc), **self._state_only())

    def _state_only(self):
        return {"state": self.status().get("state", {})}

    def _open(self, headless: bool, request_id: str, display_number=None) -> dict:
        if (self.base / "nav").exists() and self._assets().active_exports():
            return fail("EXPORT_BUSY", "Wait for the owned asset export worker to exit before opening Isaac")
        existing = self._session()
        if existing and self._alive(existing):
            state = self.status()["state"]
            if bool(existing["headless"]) != headless:
                return fail("MODE_CONFLICT", "Close the managed instance before changing headless mode.", state=state)
            if (not headless and display_number is not None
                    and state.get("display") not in {f":{display_number}", f":{display_number}.0"}):
                return fail("DISPLAY_CONFLICT", "Close the managed instance before changing its display. "
                            "The current display may be unknown for an older session.", state=state)
            return {"success": True, "accepted": True, "already_running": True, "state": state}
        if not self.python.is_file() or not os.access(self.python, os.X_OK):
            return fail("INVALID_LAUNCHER", "--isaac-python must be an executable Isaac Python or python.sh.")
        environment = os.environ.copy()
        for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONEXE", "CONDA_PREFIX", "CONDA_DEFAULT_ENV",
                     "LD_LIBRARY_PATH", "LD_PRELOAD", "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION",
                     "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_CURRENT_PREFIX"):
            environment.pop(name, None)
        from navigation.launch_environment import configure_ros_libraries
        environment["HERMES_SIM_CONFIG"] = str(config_path())
        environment.setdefault("ROS_DOMAIN_ID", str(self.deployment["ros_domain_id"]))
        environment.setdefault("ROS_LOCALHOST_ONLY", "1" if self.deployment["ros_localhost_only"] else "0")
        environment.setdefault("RMW_IMPLEMENTATION", self.deployment["rmw_implementation"])
        if self.deployment.get("isaac_ros_lib_dir"):
            environment.setdefault("HERMES_ISAAC_ROS_LIB_DIR", self.deployment["isaac_ros_lib_dir"])
        configure_ros_libraries(self.python, environment)
        display = f":{display_number}" if display_number is not None else self.display
        environment["DISPLAY"] = display
        if self.xauthority is not None:
            environment["XAUTHORITY"] = self.xauthority
        if not headless and not environment.get("DISPLAY"):
            return fail("DISPLAY_REQUIRED", "GUI launch needs a host DISPLAY; configure the bridge or use headless=true.")
        session_id = uuid.uuid4().hex
        run_dir = self.base / session_id
        if len(os.fsencode(run_dir / "bridge.sock")) >= 108:
            return fail("STATE_PATH_TOO_LONG", "Use a shorter --state-dir for the Unix socket.")
        run_dir.mkdir(mode=0o700)
        runtime = Path(__file__).with_name("runtime.py")
        command = [str(self.python), str(runtime), "--run-dir", str(run_dir), "--session-id", session_id]
        if headless:
            command.append("--headless")
        with (run_dir / "isaac.log").open("ab", buffering=0) as output:
            child = subprocess.Popen(command, cwd=self.python.parent, env=environment,
                                     stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        identity = process_identity(child.pid)
        if identity is None:
            child.poll()
            return fail("START_FAILED", "Launcher exited immediately.", log_path=str(run_dir / "isaac.log"))
        self.children[session_id] = child
        try:
            atomic_json(self.pointer, {"session_id": session_id, "run_dir": str(run_dir),
                                      "launcher_pid": child.pid, "start_ticks": identity[0],
                                      "headless": headless, "display": None if headless else display,
                                      "request_id": request_id})
        except OSError:
            # Failed registration must not leave an unreachable GPU application.
            if process_matches(child.pid, identity[0]) and os.getpgid(child.pid) == child.pid:
                os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=5)
            self.children.pop(session_id, None)
            raise
        return {"success": True, "accepted": True, **self._state_only()}

    def _force_close(self, session) -> dict:
        if not self._alive(session):
            return {"success": True, "forced": False, **self._state_only()}
        pid = session["launcher_pid"]
        if not self._owned_pids(session):
            return fail("OWNERSHIP_MISMATCH", "Refusing to signal an unowned process group.")
        for sig, timeout in ((signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            if not self._alive(session):
                break
            os.killpg(pid, sig)
            deadline = time.monotonic() + timeout
            while self._alive(session) and time.monotonic() < deadline:
                time.sleep(0.1)
        if self._alive(session):
            return fail("CLOSE_TIMEOUT", "Owned process did not exit.", **self._state_only())
        state = self.status()["state"]
        state.update(phase="closed", process_running=False, forced=True)
        state.pop("error", None)
        atomic_json(Path(session["run_dir"]) / "state.json", state)
        return {"success": True, "accepted": True, "forced": True, "state": state}

    def _wait(self, action: str, request_id: str, seconds: float, initial: dict) -> dict:
        if not initial.get("success"):
            return initial
        initial_state = initial.get("state") or {}
        observed_at = None
        if initial.get("accepted") and (initial_state.get("operation") or {}).get("id") == request_id:
            observed_at = initial_state.get("updated_monotonic")
        deadline = time.monotonic() + seconds
        while True:
            status = self.status()
            if not status.get("success"):
                return status
            state = status["state"]
            operation = state.get("operation") or {}
            complete = False
            if action == "open":
                complete = state.get("phase") == "ready" and state.get("process_running", False) and state.get("state_fresh")
            elif action == "close":
                complete = not state.get("process_running", False) and state.get("phase") == "closed"
            elif operation.get("id") == request_id:
                if state.get("state_fresh"):
                    updated_at = state.get("updated_monotonic")
                    if updated_at is not None:
                        observed_at = max(observed_at, updated_at) if observed_at is not None else updated_at
                if operation.get("status") == "failed":
                    return {"success": False, "action": action, "request_id": request_id,
                            "error": operation.get("error"), "state": state}
                complete = operation.get("status") == "succeeded" and state.get("state_fresh") and state.get("process_running", False)
            elif (operation.get("id") and observed_at is not None and state.get("state_fresh")
                  and state.get("updated_monotonic", 0) > observed_at):
                # A lifecycle stop can replace the active operation while another
                # client waits. A newer snapshot proves replacement, but does
                # not tell us whether the earlier operation failed or finished.
                return fail("OPERATION_SUPERSEDED",
                            "Another operation replaced this request's status; its final outcome is unknown. "
                            "Inspect current state before retrying.",
                            action=action, request_id=request_id, pending=False, state=state)
            if complete:
                return {**initial, "success": True, "action": action, "request_id": request_id,
                        "pending": False, "state": state}
            if state.get("phase") == "error":
                return {"success": False, "action": action, "request_id": request_id,
                        "error": state.get("error", {"code": "RUNTIME_ERROR", "message": "Inspect log_path."}),
                        "state": state}
            if time.monotonic() >= deadline:
                return {**initial, "action": action, "request_id": request_id, "pending": True,
                        "state": state, "next_action": {"tool": "isaac_sim_status", "args": {}}}
            time.sleep(0.1)

    def dispatch(self, args: dict) -> dict:
        if not isinstance(args, dict):
            return fail("INVALID_ARGUMENT", "Request must be a JSON object.")
        action = args.get("action")
        if action in {"nav_export_map", "nav_export_urdf", "nav_job_status"}:
            return self._assets().dispatch(args)
        if isinstance(action, str) and action.startswith("nav_"):
            return fail("TOOL_MOVED", "RViz/Nav2 are served by the independent rviz2-navigation plugin")
        fields = {"open": {"headless", "display_number", "wait_seconds"}, "load": {"path", "wait_seconds"},
                  "play": {"wait_seconds"}, "stop": {"wait_seconds"},
                  "close": {"force", "wait_seconds"}, "status": set(), "capture": {"label"},
                  "rv1_load": {"wait_seconds"}, "rv1_insert": {"wait_seconds"},
                  "rv1_ros2_prepare": {"config", "wait_seconds"},
                  "rv1_ros2_arm": {"wait_seconds"}, "rv1_ros2_disarm": {"wait_seconds"}}
        if not isinstance(action, str) or action not in fields:
            return fail("INVALID_ACTION", "Allowed: " + ", ".join(fields) + ".")
        if set(args) - (fields[action] | {"action"}):
            return fail("INVALID_ARGUMENT", "Unknown argument for this action.")
        seconds = args.get("wait_seconds", 5 if action in ("play", "stop") else 10)
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not 0 <= seconds <= 60:
            return fail("INVALID_ARGUMENT", "wait_seconds must be a finite number from 0 to 60.")
        for key in ("headless", "force"):
            if key in args and not isinstance(args[key], bool):
                return fail("INVALID_ARGUMENT", f"{key} must be a boolean.")
        if "display_number" in args:
            display_number = args["display_number"]
            if isinstance(display_number, bool) or not isinstance(display_number, int) or not 0 <= display_number <= 99:
                return fail("INVALID_ARGUMENT", "display_number must be an integer from 0 to 99.")
        try:
            if action == "status":
                return {"action": action, **self.status()}
            if action == "capture":
                # Capture checks ownership again afterward. Do not delay a
                # concurrent navigation cancel/stop while waiting on the desktop worker.
                return capture_managed_window(self, args.get("label"))
            request_id = uuid.uuid4().hex
            if action in {"load", "rv1_load", "stop", "close"}:
                # Runtime independently revokes its subscriber on stage/timeline
                # lifecycle changes. An unavailable external Nav2 bridge cannot
                # block closing the simulator.
                try:
                    state = self.status().get("state", {})
                    if state.get("process_running") and state.get("ros2"):
                        self._rpc("rv1_ros2_disarm", uuid.uuid4().hex)
                except Exception:
                    pass
            with self.lock:
                if action == "open":
                    initial = self._open(args.get("headless", False), request_id, args.get("display_number"))
                elif action == "close" and (not self._session() or not self._alive(self._session())):
                    return {"success": True, "action": action, "pending": False, "already_closed": True,
                            **self._state_only()}
                elif action == "close" and args.get("force", False):
                    return {"action": action, "pending": False, **self._force_close(self._session())}
                else:
                    params = {}
                    if action == "load":
                        raw = args.get("path")
                        if not isinstance(raw, str) or not raw or not Path(raw).is_absolute():
                            return fail("INVALID_PATH", "path must be an absolute USD file path on the Isaac host.")
                        path = Path(raw).resolve(strict=True)
                        if not path.is_file() or path.suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
                            return fail("INVALID_PATH", "Expected a local USD/usda/usdc/usdz file.")
                        params["path"] = str(path)
                    elif action == "rv1_ros2_prepare":
                        if not isinstance(args.get("config", {}), dict):
                            return fail("INVALID_ARGUMENT", "config must be an object")
                        params = {"config": args.get("config", {})}
                    initial = self._rpc(action, request_id, **params)
            return self._wait(action, request_id, seconds, initial)
        except CaptureError as exc:
            return fail(exc.code, str(exc), action=action)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return fail("HOST_ERROR", str(exc))
