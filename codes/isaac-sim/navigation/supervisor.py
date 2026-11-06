"""Host-only isolated USD asset export process ownership.

No ROS or Isaac imports here: Hermes and the authenticated bridge stay small.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid
from deployment import load_config, config_path, PACKAGE_ROOT


FIELDS = {
    "nav_export_map": {"source_path", "output_dir", "resolution_m", "seed_stage_xyz_m",
                       "bounds_stage_xy_m", "z_min_m", "z_max_m", "max_cells", "max_voxels",
                       "geometry_mode", "exclude_prim_paths", "free_thresh", "occupied_thresh",
                       "bounds_mode", "scene_padding_m"},
    "nav_export_urdf": {"source_path", "output_dir", "root_prim_path", "base_frame", "robot_name"},
    "nav_job_status": {"job_id"},

}


def fail(code, message, **extra):
    return {"success": False, "error": {"code": code, "message": str(message)}, **extra}


def identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None
    except (OSError, IndexError):
        return None


def save(path, data):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    temp.replace(path)


class AssetSupervisor:
    def __init__(self, manager):
        self.manager = manager
        self.project = Path(load_config()["export_root"]).expanduser().resolve()
        self.project.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.base = manager.base / "nav"
        self.base.mkdir(exist_ok=True, mode=0o700)
        self.lock = threading.RLock()
        self.children = {}

    def _spawn(self, command, run_dir, env=None):
        run_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        exit_path = run_dir / "exit.json"
        exit_path.unlink(missing_ok=True)
        with (run_dir / "process.log").open("ab", buffering=0) as output:
            child = subprocess.Popen(command, cwd=PACKAGE_ROOT, env=env,
                                     stdin=subprocess.DEVNULL, stdout=output,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        ticks = identity(child.pid)
        self.children[child.pid] = child
        def reap():
            code = child.wait()
            save(exit_path, {"pid": child.pid, "exit_code": code, "finished_wall": time.time()})
        threading.Thread(target=reap, daemon=True).start()
        return {"pid": child.pid, "start_ticks": ticks, "log_path": str(run_dir / "process.log"),
                "command": command, "created_wall": time.time()}

    def _alive(self, record):
        child = self.children.get(record.get("pid"))
        if child:
            child.poll()
        return bool(record.get("start_ticks") and identity(record["pid"]) == record["start_ticks"])

    def _output(self, value, default):
        path = Path(value).expanduser().resolve() if value else default
        if not path.is_relative_to(self.project) or path == self.project:
            raise ValueError("Output directory must be below configured export_root")
        return path

    def _export(self, operation, params):
        if self.manager.status().get("state", {}).get("process_running"):
            return fail("ISAAC_RUNNING", "The isolated export worker needs Isaac closed first; it does not replace an open stage")
        # Only one isolated GPU export at a time, including after bridge restart.
        for path in self.base.glob("export-*/job.json"):
            if self._alive(json.loads(path.read_text())):
                return fail("EXPORT_BUSY", "Another owned export worker is running")
        from navigation.assets import validate_asset_request
        job_id = uuid.uuid4().hex
        run_dir = self.base / ("export-" + job_id)
        run_dir.mkdir(mode=0o700)
        output = self._output(params.get("output_dir"), self.project / job_id)
        request = {**params, "operation": operation, "output_dir": str(output)}
        validated = validate_asset_request(request)
        if isinstance(validated, dict):
            request = validated
        request_path, result_path = run_dir / "request.json", run_dir / "result.json"
        save(request_path, request)
        environment = os.environ.copy()
        for name in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
            environment.pop(name, None)
        environment["HERMES_SIM_CONFIG"] = str(config_path())
        environment["DISPLAY"] = getattr(self.manager, "display", ":1")
        if getattr(self.manager, "xauthority", None):
            environment["XAUTHORITY"] = self.manager.xauthority
        command = [str(self.manager.python), str(Path(__file__).with_name("asset_worker.py")),
                   "--request", str(request_path), "--result", str(result_path)]
        record = self._spawn(command, run_dir, environment)
        record.update(job_id=job_id, operation=operation, request=request,
                      result_path=str(result_path), output_dir=str(output))
        save(run_dir / "job.json", record)
        return {"success": True, "accepted": True, "pending": True, "job_id": job_id,
                "log_path": record["log_path"], "output_dir": str(output)}

    def active_exports(self):
        return [json.loads(path.read_text())["job_id"] for path in self.base.glob("export-*/job.json")
                if self._alive(json.loads(path.read_text()))]

    def job_status(self, job_id):
        if not isinstance(job_id, str) or len(job_id) != 32 or any(c not in "0123456789abcdef" for c in job_id):
            raise ValueError("job_id must be the 32-character ID returned by an export")
        path = self.base / ("export-" + job_id) / "job.json"
        if not path.is_file():
            return fail("UNKNOWN_JOB", "No export with this ID")
        record = json.loads(path.read_text())
        running = self._alive(record)
        result = Path(record["result_path"])
        if result.is_file():
            data = json.loads(result.read_text())
            exit_path = path.parent / "exit.json"
            exit_data = json.loads(exit_path.read_text()) if exit_path.exists() else {}
            artifact_ok = data.get("success", data.get("ok", False))
            success = artifact_ok and (running or exit_data.get("exit_code") == 0)
            if not running and exit_data.get("exit_code") != 0:
                data["error"] = {"code": "EXPORT_SHUTDOWN_FAILED",
                                 "message": "Artifacts may exist, but clean worker exit was not verified"}
            return {**data, "success": success, "artifact_ok": artifact_ok,
                    "worker_exit_code": exit_data.get("exit_code"),
                    "job_id": job_id, "process_running": running,
                    "pending": running, "log_path": record["log_path"]}
        if not running:
            return fail("EXPORT_EXITED", "Export ended without a result", job_id=job_id, log_path=record["log_path"])
        return {"success": True, "pending": True, "job_id": job_id, "log_path": record["log_path"]}

    def dispatch(self, args):
        fields = {"nav_export_map": FIELDS["nav_export_map"],
                  "nav_export_urdf": FIELDS["nav_export_urdf"], "nav_job_status": {"job_id"}}
        action = args.get("action")
        if action not in fields or set(args) - (fields[action] | {"action"}):
            return fail("INVALID_ARGUMENT", "Only asset exports belong to the Isaac bridge; use rviz2-navigation for ROS operations")
        params = {k: v for k, v in args.items() if k != "action"}
        try:
            with self.lock:
                if action == "nav_job_status":
                    return self.job_status(params.get("job_id"))
                return self._export("map" if action.endswith("map") else "urdf", params)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError, ImportError) as exc:
            return fail("ASSET_EXPORT_ERROR", str(exc))
