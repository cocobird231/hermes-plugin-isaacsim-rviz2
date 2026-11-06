#!/usr/bin/env python3
"""Reproduce scene/RViz setup through registered plugin handlers; never drive."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from portable_common import configure, private_output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--record-dir", required=True, type=Path)
    parser.add_argument("--resume", action="store_true", help="Reuse a fresh stopped Gongguan stage after inspecting a prior setup failure")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--display-number", type=int)
    args = parser.parse_args()
    config = configure(args.config, required=("gongguan_usd", "rv1_usd"))
    display = args.display_number if args.display_number is not None else config["display_number"]
    if not 0 <= display <= 99:
        parser.error("display-number must be between 0 and 99")
    project = Path(__file__).resolve().parents[1]
    output = private_output(args.record_dir)
    if output.exists():
        parser.error("record-dir must be a new directory outside the public checkout")
    output.mkdir(parents=True)
    sequence = 0

    def call(tool, params=None):
        nonlocal sequence
        sequence += 1
        proc = subprocess.run(
            [sys.executable, str(project / "scripts/plugin_call.py"), tool,
             json.dumps(params or {}), "--record", str(output / f"{sequence:03d}-{tool}.json")],
            text=True, capture_output=True, timeout=125)
        result = json.loads(proc.stdout)
        if not result.get("success"):
            raise RuntimeError(json.dumps(result))
        if tool not in {"isaac_sim_status", "rviz2_navigation_status"}:
            print(json.dumps({"tool": tool, "success": True, "pending": result.get("pending")}), flush=True)
        return result

    def wait_isaac(result, predicate):
        request_id = result.get("request_id")
        deadline = time.monotonic() + 600
        while True:
            state = call("isaac_sim_status")["state"]
            operation = state.get("operation") or {}
            if request_id and operation.get("id") == request_id and operation.get("status") == "failed":
                raise RuntimeError(json.dumps(state))
            if state.get("state_fresh") and predicate(state):
                return state
            if time.monotonic() > deadline:
                raise TimeoutError("Isaac operation did not reach its requested state")
            time.sleep(2)

    result = call("isaac_sim_open", {"display_number": display, "wait_seconds": 60})
    wait_isaac(result, lambda s: s.get("phase") == "ready")
    state = call("isaac_sim_status")["state"]
    if state.get("timeline") != "stopped":
        raise RuntimeError("Setup requires a stopped Isaac stage; cancel/stop existing navigation first")
    scripts = ["load_gongguan_xinhai", "add_rv1_to_gongguan"]
    if args.resume:
        if not state.get("state_fresh") or Path(str(state.get("stage_path", ""))).expanduser().resolve() != Path(config["gongguan_usd"]).resolve():
            raise RuntimeError("Resume requires the fresh, already loaded Gongguan stage")
        scripts = [] if state.get("scene_setup") and state.get("rv1") else ["add_rv1_to_gongguan"]
    for script in scripts:
        result = call("isaac_sim_run_script", {"script_name": script, "wait_seconds": 60})
        request_id = result.get("request_id")
        wait_isaac(result, lambda s: s.get("operation", {}).get("id") == request_id
                   and s.get("operation", {}).get("status") == "succeeded")
    call("isaac_sim_prepare_rv1_ros2", {"wait_seconds": 60, "config": {
        "localization_mode": "sim_truth", "max_linear_mps": 1.0,
        "max_yaw_rps": 0.5, "command_timeout_seconds": 3.0}})
    result = call("isaac_sim_play", {"wait_seconds": 60})
    wait_isaac(result, lambda s: s.get("timeline") == "playing" and s.get("simulation_time", 0) > 0)
    call("rviz2_configure_robot", {"profile_name": "rv1"})
    call("rviz2_load_map", {"map_yaml_path": args.map_yaml})
    call("rviz2_load_robot", {"urdf_path": args.urdf})
    call("rviz2_open", {"display_number": display})
    call("rviz2_navigation_start")
    deadline = time.monotonic() + 180
    while True:
        state = call("rviz2_navigation_status")["state"]
        if state.get("navigation_ready"):
            break
        if time.monotonic() > deadline:
            raise TimeoutError("Nav2 did not become ready")
        time.sleep(2)
    pose = state["pose"]
    call("rviz2_set_initial_pose", {k: pose[k] for k in ("x_m", "y_m", "yaw_rad")})
    call("isaac_sim_capture", {"label": "nav2-full-map-ready"})
    print(json.dumps({"setup_complete": True, "pose": pose, "map": state.get("map"),
                      "robot": state.get("model"), "driving_requested": False}), flush=True)


if __name__ == "__main__":
    main()
