#!/usr/bin/env python3
"""Capture actual display pixels plus plugin state; optionally tile owned windows."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from portable_common import configure, private_output

project = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project / "codes/isaac-sim"))
from transport import command as isaac_command
from capture import _find_window
rviz_spec = importlib.util.spec_from_file_location("desktop_rviz_transport", project / "codes/rviz2-navigation/transport.py")
rviz_transport = importlib.util.module_from_spec(rviz_spec)
rviz_spec.loader.exec_module(rviz_transport)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("label")
parser.add_argument("--tile", action="store_true")
parser.add_argument("--output-dir", type=Path, required=True)
parser.add_argument("--config", type=Path)
parser.add_argument("--display-number", type=int)
args = parser.parse_args()
config = configure(args.config)
display_number = args.display_number if args.display_number is not None else config["display_number"]
if not 0 <= display_number <= 99:
    parser.error("display-number must be between 0 and 99")
display_name = ":" + str(display_number)
if not args.label or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in args.label):
    parser.error("Use a lowercase filename label")
directory = private_output(args.output_dir)
output = directory / (args.label + ".png")
output.parent.mkdir(parents=True, exist_ok=True)
if output.exists():
    parser.error("Screenshot already exists")
isaac, nav = isaac_command("status"), rviz_transport.command("nav_status")
assert isaac["success"] and nav["success"]
state = isaac["state"]
assert state["process_running"] and state["state_fresh"] and state["display"] == display_name
env = {**os.environ, "DISPLAY": display_name}
if config.get("xauthority"):
    env["XAUTHORITY"] = config["xauthority"]
    os.environ["XAUTHORITY"] = config["xauthority"]
import gi
gi.require_version("Gdk", "3.0")
gi.require_version("GdkX11", "3.0")
from gi.repository import Gdk, GdkX11
display = Gdk.Display.open(display_name)
assert display
windows = []
try:
    if args.tile:
        screen = display.get_default_screen()
        width, height = screen.get_width(), screen.get_height()
        half, top = width // 2, min(55, height // 10)
        for pid, rect in [(state["pid"], (0, top, half, height - top)),
                          (nav["state"]["processes"]["rviz2"]["pid"], (half, top, width - half, height - top))]:
            found = _find_window({pid}, env)
            window = GdkX11.X11Window.foreign_new_for_display(display, int(found["window_id"], 16))
            window.move_resize(*rect)
            window.show()
            windows.append(found)
        display.sync()
        time.sleep(2)
    root = display.get_default_screen().get_root_window()
    pixels = Gdk.pixbuf_get_from_window(root, 0, 0, root.get_width(), root.get_height())
    assert pixels
    pixels.savev(str(output), "png", [], [])
    meta = {"captured_at_utc": datetime.now(timezone.utc).isoformat(), "display": display_name,
            "width": pixels.get_width(), "height": pixels.get_height(),
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            "tiled_windows": windows, "isaac": isaac, "navigation": nav,
            "note": "Actual desktop pixels; states sampled before capture. No offscreen rendering."}
    output.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(output)
finally:
    # GDK owns screen/visual objects until process teardown; closing their X
    # connection here causes spurious GI finalizer errors after a valid image.
    display.flush()
