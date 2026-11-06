"""Private Isaac Sim 6 worker; launched with its dedicated Python runtime.

Socket threads only move JSON messages. Every USD/timeline operation runs on
the Kit main thread. Importing this module does not import or start Isaac Sim.
"""

import argparse
from collections import OrderedDict
import copy
import json
import math
import os
from pathlib import Path
import queue
import signal
import socketserver
import tempfile
import threading
import time
import traceback
import uuid


MAX_REQUEST = 65536
REQUEST_TIMEOUT = 30.0
OPERATION_TIMEOUT = 300.0


def validate_usd_path(value):
    """Resolve an existing local USD file; never interpret shell text or URLs."""
    if not isinstance(value, str) or not value or "\x00" in value or "://" in value:
        raise ValueError("path must be an absolute local USD file path")
    path = Path(value)
    if not path.is_absolute():
        raise ValueError("path must be absolute")
    path = path.resolve(strict=True)
    if not path.is_file() or path.suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
        raise ValueError("path must identify an existing USD, USDA, USDC, or USDZ file")
    return str(path)


class Bridge(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, worker):
        self.worker = worker
        self.slots = threading.BoundedSemaphore(16)
        super().__init__(str(worker.run_dir / "bridge.sock"), RequestHandler)
        os.chmod(self.server_address, 0o600)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        worker = self.server.worker
        sent = threading.Event()
        self.request.settimeout(5.0)
        try:
            raw = self.rfile.readline(MAX_REQUEST + 1)
            if len(raw) > MAX_REQUEST or not raw.endswith(b"\n"):
                raise ValueError("request must be one newline-terminated JSON object, at most 64 KiB")
            request = json.loads(raw)
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            response_queue = queue.Queue(maxsize=1)
            worker.requests.put_nowait((request, response_queue, sent, time.monotonic() + REQUEST_TIMEOUT))
            try:
                response = response_queue.get(timeout=REQUEST_TIMEOUT)
            except queue.Empty:
                response = worker.reply(False, "REQUEST_TIMEOUT", "No reply yet; poll status before retrying")
        except (ValueError, UnicodeError, OSError, queue.Full) as exc:
            response = worker.reply(False, "INVALID_REQUEST", str(exc))
        try:
            self.wfile.write(json.dumps(response, ensure_ascii=False).encode("utf-8") + b"\n")
            self.wfile.flush()
        except OSError:
            pass
        finally:
            sent.set()


class Worker:
    def __init__(self, run_dir, session_id):
        self.run_dir = Path(run_dir)
        self.requests = queue.Queue(maxsize=64)
        self.lock = threading.Lock()
        self.cache = OrderedDict()
        self.pending = None
        self.close_sent = None
        self.close_deadline = 0
        self.shutdown_requested = False
        self.last_write = 0
        self.app = self.usd = self.timeline = self.simulation_manager = None
        self.rv1 = None
        self.rv1_navigation = None
        self.state = {"session_id": session_id, "pid": os.getpid(), "phase": "starting",
                      "start_ticks": Path("/proc/self/stat").read_text().rsplit(")", 1)[1].split()[19],
                      "stage_path": None, "timeline": "stopped", "simulation_time": 0.0,
                      "physics_steps": 0, "animation_time": 0.0,
                      "frame_count": 0, "operation": None}

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def reply(self, success=True, code=None, message=None, accepted=False):
        result = {"success": success, "state": self.snapshot()}
        if code:
            result["error"] = {"code": code, "message": message}
        if accepted:
            result["accepted"] = True
        return result

    def change(self, **values):
        with self.lock:
            self.state.update(values)

    def persist(self):
        self.change(updated_monotonic=time.monotonic())
        with tempfile.NamedTemporaryFile(mode="w", dir=self.run_dir, prefix=".state-", delete=False) as stream:
            temp_path = stream.name
            json.dump(self.snapshot(), stream, ensure_ascii=False)
            stream.flush()
        os.replace(temp_path, self.run_dir / "state.json")
        self.last_write = time.monotonic()

    def sample(self):
        stage = self.usd.get_stage()
        root = stage.GetRootLayer() if stage else None
        # USDZ identifiers can include the packaged layer after an opening '['.
        actual_path = str(Path(root.realPath.split("[", 1)[0]).resolve()) if root and root.realPath else None
        mode = "playing" if self.timeline.is_playing() else "stopped" if self.timeline.is_stopped() else "paused"
        self.change(stage_path=actual_path, timeline=mode,
                    simulation_time=float(self.simulation_manager.get_simulation_time()),
                    physics_steps=int(self.simulation_manager.get_num_physics_steps()),
                    animation_time=float(self.timeline.get_current_time()),
                    frame_count=self.state["frame_count"] + 1)
        self.sample_rv1(stage)
        if self.rv1_navigation is not None:
            try:
                self.rv1_navigation.update(self.state["simulation_time"], playing=self.timeline.is_playing())
                status = self.rv1_navigation.status()
                self.change(ros2=status, control_owner=("nav2" if status.get("armed") else "stopped"))
            except Exception as exc:
                status = self.rv1_navigation.status()
                # The adapter latches command failures and revokes ownership.
                if status.get("armed"):
                    self.rv1_navigation.brake(f"ROS 2 update failed: {exc}")
                    status = self.rv1_navigation.status()
                self.change(ros2={**status, "ready": False, "error": str(exc)},
                            control_owner=("nav2" if status.get("armed") else "stopped"))

    def cancel_rv1(self, reason, *, detach=False):
        if self.rv1_navigation is not None:
            try:
                self.rv1_navigation.close()
            except Exception as exc:
                # An invalid stage must never prevent lifecycle stop/close.
                self.change(rv1={"measurement_valid": False, "error": str(exc)})
            finally:
                self.rv1_navigation = None
                self.change(ros2=None, control_owner="stopped")
        if self.rv1:
            try:
                self.rv1.reset_physics()
            except Exception as exc:
                # An externally replaced USD stage invalidates cached attrs;
                # this must never prevent timeline.stop or application close.
                self.change(rv1={"measurement_valid": False, "error": str(exc)})
        if detach:
            self.rv1 = None
            self.change(rv1=None)

    def sample_rv1(self, stage):
        controller = self.rv1
        if controller is None:
            return
        changed_stage = stage != controller.stage
        try:
            if changed_stage:
                raise RuntimeError("RV1 stage was replaced outside the plugin; load RV1 again")
            telemetry = controller.telemetry(self.timeline.is_playing())
            self.change(rv1=telemetry)
        except Exception as exc:
            # Navigation cannot retain actuator ownership after feedback or
            # stage invalidation. Lifecycle reset also clears surviving targets.
            navigation_active = (self.rv1_navigation is not None
                                 and self.rv1_navigation.status().get("armed"))
            if navigation_active or changed_stage:
                self.cancel_rv1(str(exc), detach=changed_stage)
                if self.timeline.is_playing():
                    self.timeline.stop()
            self.change(rv1={"measurement_valid": False, "error": str(exc)})
        else:
            # Camera failures are isolated from the navigation feedback guard.
            self.sample_rv1_camera(stage, telemetry)

    def sample_rv1_camera(self, stage, telemetry):
        """Translate the owned view using PhysX pose; never write robot targets."""
        setup = self.state.get("scene_setup") or {}
        follow = setup.get("camera_follow") or {}
        if follow.get("mode") != "physics_translation":
            return
        try:
            if not self.timeline.is_playing() or not telemetry.get("measurement_valid"):
                status = "frozen" if not self.timeline.is_playing() else "waiting_for_physics"
                self.change(scene_setup={**setup, "camera_follow": {**follow, "status": status}})
                return
            from pxr import Gf, Usd, UsdGeom
            path = setup.get("camera_path")
            if path != "/World/AgentExperiment/OverviewCamera":
                raise RuntimeError("Camera follow requires the owned Gongguan overview camera")
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsA(UsdGeom.Camera):
                raise RuntimeError("Owned camera is unavailable")
            transform = prim.GetAttribute("xformOp:transform")
            if not transform:
                raise RuntimeError("Owned camera transform is unavailable")
            position = telemetry["position_m"]
            offsets = [follow["eye_offset_m"], follow["target_offset_m"]]
            if any(len(values) != 3 or not all(math.isfinite(float(x)) for x in values)
                   for values in [position, *offsets]):
                raise ValueError("Camera follow requires finite XYZ coordinates")
            eye, target = [Gf.Vec3d(*(float(p) + float(delta) for p, delta in zip(position, offset)))
                           for offset in offsets]
            matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                if not transform.Set(matrix):
                    raise RuntimeError("Camera transform could not be written")
            updated = {**follow, "status": "tracking", "update_count": follow.get("update_count", 0) + 1,
                       "last_update_sim_seconds": self.state["simulation_time"],
                       "measured_position_m": list(position)}
            updated.pop("error", None)
            self.change(scene_setup={**setup, "camera_eye_m": list(eye), "camera_target_m": list(target),
                                     "camera_follow": updated})
        except Exception as exc:
            self.change(scene_setup={**setup, "camera_follow": {**follow, "status": "error", "error": str(exc)}})

    def remember(self, request, response):
        request_id = request["request_id"]
        self.cache[request_id] = (request, response)
        self.cache.move_to_end(request_id)
        while len(self.cache) > 256:
            self.cache.popitem(last=False)

    def finish(self, error=None, phase="ready"):
        pending = self.pending
        operation = {"id": pending["request"]["request_id"], "action": pending["request"]["action"],
                     "status": "failed" if error else "succeeded"}
        if error:
            operation["error"] = {"code": "OPERATION_FAILED", "message": str(error)}
        self.change(phase=phase, operation=operation)
        self.pending = None
        self.persist()
        response = self.reply(not error, "OPERATION_FAILED" if error else None, str(error) if error else None)
        self.remember(pending["request"], response)

    def dispatch(self, item):
        request, response_queue, sent, deadline = item
        action, request_id = request.get("action"), request.get("request_id")
        if request.get("session_id") != self.state["session_id"]:
            response_queue.put_nowait(self.reply(False, "SESSION_MISMATCH", "Wrong session_id"))
            return
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            response_queue.put_nowait(self.reply(False, "INVALID_REQUEST", "request_id must be a 1–128 character string"))
            return
        if not isinstance(action, str):
            response_queue.put_nowait(self.reply(False, "INVALID_REQUEST", "action must be a string"))
            return
        if request_id in self.cache:
            previous, result = self.cache[request_id]
            response_queue.put_nowait(result if request == previous else self.reply(False, "REQUEST_ID_CONFLICT", "request_id reused for different input"))
            return
        if time.monotonic() > deadline:
            result = self.reply(False, "REQUEST_EXPIRED", "Request expired before execution")
        elif action == "status":
            result = self.reply()
        elif action in {"rv1_ros2_prepare", "rv1_ros2_arm", "rv1_ros2_disarm"}:
            result = self.configure_rv1_ros2(request)
        elif action == "close":
            self.cancel_rv1("Cancelled by close request")
            if self.pending:
                task = self.pending.get("task")
                if task and not task.done():
                    task.cancel()
                self.finish("Cancelled by close request")
            self.timeline.stop()
            self.change(phase="closing", operation={"id": request_id, "action": "close", "status": "running"})
            self.close_sent, self.close_deadline = sent, time.monotonic() + 1.0
            self.persist()
            result = self.reply(accepted=True)
        elif action not in {"load", "play", "stop", "rv1_load", "rv1_insert"}:
            result = self.reply(False, "UNKNOWN_ACTION", "Supported actions: status, load, play, stop, close, rv1_load, rv1_insert")
        elif (self.state["phase"] != "ready"
              or (self.pending and not (
                  action == "stop" and self.pending["request"]["action"] == "play"))):
            result = self.reply(False, "BUSY", "A lifecycle operation is already running")
        elif action in {"load", "rv1_load", "rv1_insert"} and not self.timeline.is_stopped():
            result = self.reply(False, "STOP_REQUIRED", "Stop simulation before loading another stage")
        elif action == "play" and not self.state["stage_path"]:
            result = self.reply(False, "NO_STAGE", "Load a USD scene before playing")
        else:
            try:
                from rv1.scene import prepare_test_scene
                path = (validate_usd_path(prepare_test_scene(self.run_dir))
                        if action == "rv1_load" else validate_usd_path(request.get("path")) if action == "load" else None)
                if action == "rv1_insert":
                    from rv1.gongguan import validate_target
                    validate_target(self.usd.get_stage())
                if action in {"load", "rv1_load", "stop"}:
                    self.cancel_rv1(f"Cancelled by {action} request", detach=action in {"load", "rv1_load"})
                if action == "stop" and self.pending:
                    # Stopping must remain possible while a play command is
                    # still waiting for observable physics advancement.
                    self.finish("Cancelled by stop request")
                self.pending = {"request": request, "started": time.monotonic(), "frame": self.state["frame_count"],
                                "initial_simulation_time": self.state["simulation_time"],
                                "initial_physics_steps": self.state["physics_steps"]}
                self.change(operation={"id": request_id, "action": action, "status": "running"})
                if action in {"load", "rv1_load"}:
                    self.change(scene_setup=None)
                    self.change(phase="loading")
                    self.pending.update(path=path, task=self.app.run_coroutine(self.usd.open_stage_async(path), run_until_complete=False))
                elif action == "rv1_insert":
                    self.change(phase="loading")
                else:
                    (self.timeline.play if action == "play" else self.timeline.stop)()
                self.persist()
                # A later app.update can spend minutes compiling RTX shaders.
                # Acknowledge every operation now; completion is persisted by
                # advance() and observed through status / the manager's wait.
                result = self.reply(accepted=True)
            except Exception as exc:
                if self.pending:
                    self.finish(exc)
                result = self.reply(False, "INVALID_PATH" if action == "load" else "OPERATION_FAILED", str(exc))
        self.remember(request, result)
        response_queue.put_nowait(result)

    def configure_rv1_ros2(self, request):
        """Prepare only while stopped; acquiring command ownership is explicit."""
        action = request["action"]
        if self.pending or self.state["phase"] != "ready":
            return self.reply(False, "BUSY", "Wait for the current operation before changing ROS 2 control")
        if self.rv1 is None:
            return self.reply(False, "RV1_NOT_LOADED", "Load RV1 through the plugin first")
        try:
            if action == "rv1_ros2_prepare":
                if not self.timeline.is_stopped():
                    return self.reply(False, "STOP_REQUIRED", "Prepare ROS 2 with simulation stopped")
                config = request.get("config", {})
                if not isinstance(config, dict):
                    raise ValueError("config must be a JSON object")
                if self.rv1_navigation is not None:
                    self.rv1_navigation.close()
                    self.rv1_navigation = None
                from isaacsim.core.utils.extensions import enable_extension
                import carb.settings
                # The host is Ubuntu 24.04/Jazzy. Manager removes sourced ROS
                # environment so Isaac can load its own compatible libraries.
                carb.settings.get_settings().set_string("/exts/isaacsim.ros2.bridge/ros_distro", "jazzy")
                enable_extension("isaacsim.ros2.bridge")
                from rv1.navigation import RV1Navigation
                navigation = RV1Navigation(self.rv1, config)
                try:
                    navigation.start()
                except Exception:
                    navigation.close()
                    raise
                self.rv1_navigation = navigation
                self.change(control_owner="stopped")
            elif self.rv1_navigation is None:
                return self.reply(False, "ROS2_NOT_PREPARED", "Prepare the RV1 ROS 2 adapter first")
            elif action == "rv1_ros2_arm":
                if not self.timeline.is_playing():
                    return self.reply(False, "PLAY_REQUIRED", "Start simulation before arming ROS 2 control")
                measured = self.rv1.telemetry(True)
                if not measured.get("measurement_valid"):
                    raise RuntimeError("Current RV1 physics measurements are unavailable")
                self.rv1_navigation.arm()
                self.change(control_owner="nav2")
            else:
                self.rv1_navigation.brake("ROS 2 control disarmed")
                self.change(control_owner="stopped")
            self.change(ros2=self.rv1_navigation.status(),
                        operation={"id": request["request_id"], "action": action, "status": "succeeded"})
            self.persist()
            return self.reply()
        except Exception as exc:
            return self.reply(False, "ROS2_CONTROL_ERROR", str(exc))

    def advance(self):
        pending = self.pending
        if not pending:
            return
        try:
            action = pending["request"]["action"]
            if time.monotonic() - pending["started"] > OPERATION_TIMEOUT:
                raise TimeoutError(f"{action} did not complete within its runtime timeout")
            if self.state["frame_count"] - pending["frame"] < 2:
                return
            if action == "rv1_insert":
                if not pending.get("inserted"):
                    from rv1.gongguan import insert_rv1
                    setup = insert_rv1(self.usd.get_stage())
                    self.change(scene_setup=setup)
                    pending.update(inserted=True, frame=self.state["frame_count"])
                    return
                from rv1.controller import RV1Controller
                self.rv1 = RV1Controller(self.usd.get_stage(), self.simulation_manager)
                from omni.kit.viewport.utility import get_active_viewport
                viewport = get_active_viewport()
                if viewport:
                    viewport.camera_path = self.state["scene_setup"]["camera_path"]
                    viewport.fill_frame = False
                    viewport.resolution = (640, 360)
                    self.change(scene_setup={**self.state["scene_setup"],
                                             "viewport_resolution": list(viewport.resolution)})
                self.sample_rv1(self.usd.get_stage())
                if not self.rv1:
                    raise RuntimeError("Inserted RV1 controller did not bind")
            elif action in {"load", "rv1_load"}:
                task = pending["task"]
                if not task.done():
                    return
                success, error = task.result()
                if not success:
                    raise RuntimeError(error or "USD stage could not be opened")
                if self.usd.get_stage_loading_status()[2] > 0:
                    return
                if self.state["stage_path"] != pending["path"]:
                    raise RuntimeError("Opened root layer does not match the requested USD path")
                if action == "rv1_load":
                    from rv1.controller import RV1Controller
                    self.rv1 = RV1Controller(self.usd.get_stage(), self.simulation_manager)
                    from omni.kit.viewport.utility import get_active_viewport
                    viewport = get_active_viewport()
                    if viewport:
                        viewport.camera_path = "/World/OverviewCamera"
                    self.sample_rv1(self.usd.get_stage())
                    if not self.rv1:
                        raise RuntimeError("RV1 controller did not bind")
            elif action == "play":
                if (not self.timeline.is_playing()
                        or self.state["simulation_time"] <= pending["initial_simulation_time"]
                        or self.state["physics_steps"] <= pending["initial_physics_steps"]):
                    return
            else:
                current_time = self.state["simulation_time"]
                current_steps = self.state["physics_steps"]
                previous_time = pending.get("stopped_time")
                previous_steps = pending.get("stopped_steps")
                pending["stopped_time"] = current_time
                pending["stopped_steps"] = current_steps
                if (not self.timeline.is_stopped() or previous_time is None
                        or abs(current_time - previous_time) > 1e-9 or current_steps != previous_steps):
                    return
            self.finish()
        except Exception as exc:
            task = pending.get("task")
            if task and not task.done():
                task.cancel()
            self.finish(exc)
            if isinstance(exc, TimeoutError) and pending["request"]["action"] in {"load", "rv1_load"}:
                # Cancelling the coroutine may not cancel native USD loading.
                # Require a new session instead of allowing overlapping loads.
                self.change(phase="error", error={"code": "LOAD_TIMEOUT", "message": str(exc)})
                self.persist()

    def run(self, headless=False):
        bridge = None
        failed = False
        self.persist()
        try:
            from isaacsim import SimulationApp

            self.app = SimulationApp({"headless": headless, "sync_loads": False,
                                      "fast_shutdown": False, "renderer": "RaytracedLighting"})
            import omni.timeline
            import omni.usd
            from isaacsim.core.simulation_manager import SimulationManager

            self.usd, self.timeline = omni.usd.get_context(), omni.timeline.get_timeline_interface()
            self.simulation_manager = SimulationManager
            bridge = Bridge(self)
            threading.Thread(target=bridge.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True).start()
            self.change(phase="ready")
            self.persist()
            # SimulationApp.is_running() is false temporarily while a stage is replaced.
            while self.app.app.is_running() and not self.app.is_exiting():
                if self.shutdown_requested and not self.close_sent:
                    self.dispatch(({"action": "close", "session_id": self.state["session_id"],
                                    "request_id": "signal-close"}, queue.Queue(1), threading.Event(),
                                   time.monotonic() + REQUEST_TIMEOUT))
                for _ in range(32):
                    try:
                        item = self.requests.get_nowait()
                    except queue.Empty:
                        break
                    self.dispatch(item)
                if self.close_sent and (self.close_sent.is_set() or time.monotonic() >= self.close_deadline):
                    break
                self.app.update()
                self.sample()
                self.advance()
                if time.monotonic() - self.last_write >= 0.25:
                    self.persist()
        except BaseException as exc:
            failed = True
            self.change(phase="error", error={"code": "RUNTIME_ERROR", "message": str(exc)})
            self.persist()
            traceback.print_exc()
        finally:
            if not failed:
                self.change(phase="closing")
                self.persist()
            if self.pending:
                task = self.pending.get("task")
                if task and not task.done():
                    task.cancel()
                self.finish("Application closed before operation completed", phase=self.state["phase"])
            try:
                if self.app:
                    try:
                        self.cancel_rv1("Application closing")
                        if self.timeline:
                            self.timeline.stop()
                            if self.app.app.is_running() and not self.app.is_exiting():
                                self.app.update()
                                self.sample()
                    finally:
                        self.app.close()
            except BaseException as exc:
                failed = True
                self.change(phase="error", error={"code": "CLOSE_FAILED", "message": str(exc)})
                traceback.print_exc()
            if bridge:
                bridge.shutdown()
                bridge.server_close()
                Path(bridge.server_address).unlink(missing_ok=True)
            if not failed:
                operation = self.state["operation"]
                if operation and operation["action"] == "close":
                    operation = {**operation, "status": "succeeded"}
                self.change(phase="closed", timeline="stopped", operation=operation)
            self.persist()
        return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    uuid.UUID(args.session_id)
    run_dir = Path(args.run_dir)
    if not run_dir.is_absolute() or run_dir.is_symlink():
        parser.error("--run-dir must be an absolute, non-symlink directory")
    run_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    if run_dir.stat().st_uid != os.getuid():
        parser.error("--run-dir must be owned by the current user")
    os.chmod(run_dir, 0o700)
    worker = Worker(run_dir, args.session_id)
    # Signals request orderly shutdown; they never invoke Kit from a signal handler.
    def request_close(signum, frame):
        worker.shutdown_requested = True
    signal.signal(signal.SIGTERM, request_close)
    signal.signal(signal.SIGINT, request_close)
    return worker.run(args.headless)


if __name__ == "__main__":
    raise SystemExit(main())
