#!/usr/bin/env python3
"""Read-only sampler of plugin navigation result and Isaac physics measurements."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import time
from portable_common import configure, private_output

project = Path(__file__).resolve().parents[1]
def bridge_client(plugin):
    """Load only this plugin's client under a unique name; never a bare transport."""
    spec = importlib.util.spec_from_file_location(
        "monitor_" + plugin.replace("-", "_"), project / "codes" / plugin / "transport.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.command


p = argparse.ArgumentParser(description=__doc__)
p.add_argument("label")
p.add_argument("--seconds", type=float, default=400)
p.add_argument("--output-dir", type=Path, required=True)
p.add_argument("--config", type=Path)
p.add_argument("--wait-new-goal", action="store_true", help="Ignore terminal goals older than this sampler")
a = p.parse_args()
configure(a.config)
isaac_command = bridge_client("isaac-sim")
rviz_command = bridge_client("rviz2-navigation")
if not a.label or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in a.label):
    p.error("Use a lowercase evidence label")
directory = private_output(a.output_dir)
output = directory / (a.label + "-samples.jsonl")
output.parent.mkdir(parents=True, exist_ok=True)
start = time.monotonic()
started_wall = time.time()
last_print = -10
rest_count = 0
with output.open("x") as stream:
    while time.monotonic() - start < a.seconds:
        isaac, nav = isaac_command("status"), rviz_command("nav_status")
        sample = {"wall": time.time(), "elapsed": time.monotonic() - start,
                  "isaac": isaac, "navigation": nav}
        stream.write(json.dumps(sample) + "\n")
        stream.flush()
        state, ns = isaac.get("state", {}), nav.get("state", {})
        rv1, ros2 = state.get("rv1") or {}, state.get("ros2") or {}
        result = ns.get("navigation") or {}
        new_goal = not a.wait_new_goal or result.get("requested_wall", 0) >= started_wall
        if sample["elapsed"] - last_print >= 10 or (new_goal and result.get("complete")):
            print(json.dumps({"elapsed": round(sample["elapsed"], 1), "simtime": state.get("simulation_time"),
                "position": rv1.get("position_m"), "yaw": rv1.get("yaw_rad"),
                "speed": rv1.get("speed_mps"), "armed": ros2.get("armed"),
                "reason": ros2.get("reason"), "twists": ros2.get("received_twists"),
                "navigation": result}), flush=True)
            last_print = sample["elapsed"]
        if new_goal and result.get("complete") and state.get("state_fresh") and rv1.get("measurement_valid"):
            rest_count = rest_count + 1 if (rv1.get("speed_mps", math.inf) < .05 and
                                          abs(rv1.get("yaw_rate_rps", math.inf)) < .05) else 0
            if rest_count >= 3 and ros2.get("rest_verified") and not ros2.get("armed"):
                break
        time.sleep(1)
