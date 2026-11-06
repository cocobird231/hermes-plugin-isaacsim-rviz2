"""Capture only the live X11 window owned by the managed Isaac session.

The bridge remains stdlib-only. Its private worker uses the host system Python
and GI, and records visible window pixels, not an offscreen simulation render.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import time
import uuid


LABEL_PATTERN = r"^[a-z0-9][a-z0-9_-]{0,47}$"


class CaptureError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def validate_label(label):
    if not isinstance(label, str) or re.fullmatch(LABEL_PATTERN, label) is None:
        raise CaptureError("INVALID_ARGUMENT", "label must be 1-48 lowercase letters, digits, underscores or hyphens, beginning with a letter or digit.")


def _snapshot(manager):
    session = manager._session()
    status = manager.status()
    state = status.get("state", {})
    if not session or not state.get("process_running") or state.get("phase") == "closed":
        raise CaptureError("NOT_RUNNING", "Capture requires the plugin's running Isaac application.")
    if not status.get("success") or state.get("session_id") != session.get("session_id"):
        raise CaptureError("CAPTURE_STATE_CHANGED", "Managed session identity changed; read status before retrying.")
    if session.get("headless") or state.get("headless"):
        raise CaptureError("HEADLESS_CAPTURE_UNAVAILABLE", "A headless instance has no GUI window to capture.")
    if not state.get("state_fresh"):
        raise CaptureError("STALE_STATE", "Runtime state is stale; capture cannot establish current ownership and state.")
    if state.get("phase") not in {"ready", "loading"}:
        raise CaptureError("CAPTURE_NOT_READY", "Wait until the managed application is ready before capturing its window.")
    display = state.get("display")
    if not isinstance(display, str) or re.fullmatch(r":\d+(?:\.\d+)?", display) is None:
        raise CaptureError("DISPLAY_UNAVAILABLE", "Capture requires the managed instance's local X11 display.")
    owned = set(manager._owned_pids(session))
    if not owned:
        raise CaptureError("NOT_RUNNING", "The managed Isaac process is no longer running.")
    return session, state, owned


def _environment(manager, display):
    environment = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        environment.pop(name, None)
    environment["DISPLAY"] = display
    if manager.xauthority is not None:
        environment["XAUTHORITY"] = manager.xauthority
    return environment


def _screenshots_dir(manager, session):
    run_dir = Path(session["run_dir"])
    expected = manager.base.resolve() / session["session_id"]
    if (not run_dir.is_absolute() or run_dir.is_symlink() or not run_dir.is_dir()
            or run_dir.resolve() != expected):
        raise CaptureError("CAPTURE_FAILED", "Managed session directory is invalid.")
    directory = run_dir / "screenshots"
    if directory.is_symlink():
        raise CaptureError("CAPTURE_FAILED", "Screenshot directory must not be a symbolic link.")
    directory.mkdir(mode=0o700, exist_ok=True)
    if not directory.is_dir() or directory.resolve() != expected / "screenshots":
        raise CaptureError("CAPTURE_FAILED", "Screenshot directory is invalid.")
    directory.chmod(0o700)
    return directory


def _worker_result(command, environment):
    try:
        completed = subprocess.run(command, env=environment, capture_output=True,
                                   text=True, timeout=20, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CaptureError("CAPTURE_FAILED", "Host screenshot worker failed or exceeded its 20-second limit.") from exc
    try:
        result = json.loads(completed.stdout)
        if not isinstance(result, dict):
            raise ValueError("Invalid worker response")
    except (ValueError, TypeError) as exc:
        raise CaptureError("CAPTURE_FAILED", "Host screenshot worker did not return valid capture metadata.") from exc
    if completed.returncode or not result.get("success"):
        error = result.get("error", {})
        allowed = {"DISPLAY_UNAVAILABLE", "WINDOW_NOT_FOUND", "CAPTURE_STATE_CHANGED", "CAPTURE_FAILED"}
        code = error.get("code", "CAPTURE_FAILED")
        raise CaptureError(code if code in allowed else "CAPTURE_FAILED",
                           error.get("message", "Host screenshot worker failed."))
    return result


def capture_managed_window(manager, label):
    """Read-only with respect to runtime: never submits or replaces an operation."""
    validate_label(label)
    session, before, owned = _snapshot(manager)
    directory = _screenshots_dir(manager, session)
    capture_id = uuid.uuid4().hex
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    png = directory / f"{stamp}-{label}-{capture_id}.png"
    sidecar = png.with_suffix(".json")
    # Reserve our unique destination; labels never select or overwrite a file.
    with png.open("xb"):
        pass
    png.chmod(0o600)
    try:
        command = [manager.screenshot_python, "-I", str(Path(__file__).resolve()), "--worker",
                   "--display", before["display"], "--output", str(png),
                   "--owned-pids", *[str(pid) for pid in sorted(owned)]]
        captured = _worker_result(command, _environment(manager, before["display"]))
        current, state, current_owned = _snapshot(manager)
        identity_keys = ("session_id", "run_dir", "launcher_pid", "start_ticks")
        if (any(current.get(key) != session.get(key) for key in identity_keys)
                or state.get("pid") != before.get("pid")
                or state.get("start_ticks") != before.get("start_ticks")
                or state.get("display") != before["display"]
                or captured.get("pid") not in owned.intersection(current_owned)):
            raise CaptureError("CAPTURE_STATE_CHANGED", "Session or window ownership changed during capture; no screenshot was retained.")
        if png.is_symlink() or not png.is_file():
            raise CaptureError("CAPTURE_FAILED", "Screenshot worker did not write a regular PNG file.")
        data = png.read_bytes()
        if len(data) < 24 or data[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR":
            raise CaptureError("CAPTURE_FAILED", "Screenshot worker did not produce a PNG image.")
        width, height = struct.unpack(">II", data[16:24])
        if not width or not height or (width, height) != (captured.get("width"), captured.get("height")):
            raise CaptureError("CAPTURE_FAILED", "Screenshot dimensions do not match worker metadata.")
        png.chmod(0o600)
        result = {"success": True, "action": "capture", "capture_id": capture_id, "label": label,
                  "path": str(png), "metadata_path": str(sidecar),
                  "sha256": hashlib.sha256(data).hexdigest(), "width": width, "height": height,
                  "window_id": captured["window_id"], "pid": captured["pid"],
                  "session_id": session["session_id"], "display": before["display"],
                  "captured_at_utc": captured["captured_at_utc"], "state": state}
        with sidecar.open("x", encoding="utf-8") as output:
            os.fchmod(output.fileno(), 0o600)
            json.dump(result, output, ensure_ascii=False, indent=2)
            output.write("\n")
        return result
    except Exception:
        png.unlink(missing_ok=True)
        sidecar.unlink(missing_ok=True)
        raise


def _x_query(command, environment, timeout=2):
    try:
        return subprocess.run(command, env=environment, capture_output=True, text=True,
                              timeout=timeout, check=True).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        raise CaptureError("DISPLAY_UNAVAILABLE", "Cannot query the managed X11 display; verify X access and xwininfo/xprop installation.") from exc


def _window_info(window_id, environment):
    properties = _x_query(["/usr/bin/xprop", "-id", window_id, "_NET_WM_PID"], environment)
    match = re.search(r"^_NET_WM_PID\([^\n]*\)\s*=\s*(\d+)\s*$", properties, re.MULTILINE)
    if not match:
        return None
    details = _x_query(["/usr/bin/xwininfo", "-id", window_id], environment)
    width = re.search(r"^\s*Width:\s*(\d+)\s*$", details, re.MULTILINE)
    height = re.search(r"^\s*Height:\s*(\d+)\s*$", details, re.MULTILINE)
    if "Map State: IsViewable" not in details or not width or not height:
        return None
    return {"window_id": window_id, "pid": int(match[1]),
            "width": int(width[1]), "height": int(height[1])}


def _find_window(owned_pids, environment):
    tree = _x_query(["/usr/bin/xwininfo", "-root", "-tree"], environment, timeout=3)
    candidates = list(dict.fromkeys(re.findall(r"^\s*(0x[0-9a-fA-F]+)\s", tree, re.MULTILINE)))
    deadline = time.monotonic() + 8
    windows = []
    for window_id in candidates[:512]:
        if time.monotonic() >= deadline:
            break
        try:
            info = _window_info(window_id, environment)
        except CaptureError:
            continue  # A transient window may disappear while enumerating.
        if info and info["pid"] in owned_pids and info["width"] > 0 and info["height"] > 0:
            windows.append(info)
    if not windows:
        raise CaptureError("WINDOW_NOT_FOUND", "No IsViewable X11 window has _NET_WM_PID matching the managed process.")
    return max(windows, key=lambda item: item["width"] * item["height"])


def _capture_x11(display_name, output, owned_pids):
    environment = os.environ.copy()
    environment["DISPLAY"] = display_name
    found = _find_window(set(owned_pids), environment)
    try:
        import gi
        gi.require_version("Gdk", "3.0")
        gi.require_version("GdkX11", "3.0")
        from gi.repository import Gdk, GdkX11
    except (ImportError, ValueError) as exc:
        raise CaptureError("CAPTURE_FAILED", "Host /usr/bin/python3 requires GI Gdk 3 and GdkX11 to capture the actual window.") from exc
    display = Gdk.Display.open(display_name)
    if display is None:
        raise CaptureError("DISPLAY_UNAVAILABLE", "Gdk cannot open the managed X11 display.")
    try:
        window = GdkX11.X11Window.foreign_new_for_display(display, int(found["window_id"], 16))
        if window is None:
            raise CaptureError("WINDOW_NOT_FOUND", "Managed window disappeared before capture.")
        # Recheck immediately around pixel acquisition; never fall back to root.
        checked = _window_info(found["window_id"], environment)
        if not checked or checked["pid"] != found["pid"]:
            raise CaptureError("CAPTURE_STATE_CHANGED", "Window ownership or visibility changed before capture.")
        pixels = Gdk.pixbuf_get_from_window(window, 0, 0, window.get_width(), window.get_height())
        captured_at = datetime.now(timezone.utc).isoformat()
        if pixels is None:
            raise CaptureError("CAPTURE_FAILED", "Gdk could not read the managed window's pixels.")
        checked = _window_info(found["window_id"], environment)
        if not checked or checked["pid"] != found["pid"]:
            raise CaptureError("CAPTURE_STATE_CHANGED", "Window ownership or visibility changed during capture.")
        pixels.savev(str(output), "png", [], [])
        return {"success": True, "window_id": found["window_id"], "pid": found["pid"],
                "width": pixels.get_width(), "height": pixels.get_height(), "captured_at_utc": captured_at}
    finally:
        display.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", action="store_true", required=True)
    parser.add_argument("--display", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--owned-pids", type=int, nargs="+", required=True)
    args = parser.parse_args()
    try:
        result = _capture_x11(args.display, args.output, args.owned_pids)
    except CaptureError as exc:
        result = {"success": False, "error": {"code": exc.code, "message": str(exc)}}
    except Exception:
        result = {"success": False, "error": {"code": "CAPTURE_FAILED", "message": "Host Gdk window capture failed."}}
    print(json.dumps(result))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
