#!/usr/bin/env python3
"""Read-only RV1 footprint checks on an exported map; does not plan/control ROS."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from portable_common import private_output

import numpy as np
from PIL import Image


def analyze(metadata_path, poses, footprint_bounds=(-2.06, 1.98, -1.521, 1.521)):
    metadata = json.loads(metadata_path.read_text())
    image_path = metadata_path.parent / "map.png"
    image_hash = hashlib.sha256(image_path.read_bytes()).hexdigest()
    if image_hash != metadata["output_sha256"]["map.png"]:
        raise ValueError("Image does not match export metadata hash")
    pixels = np.array(Image.open(image_path))
    resolution = metadata["request"]["resolution_m"]
    ox, oy, yaw = metadata["metadata"]["map_origin_m"]
    if yaw != 0:
        raise ValueError("This diagnostic assumes map yaw=0")
    height, width = pixels.shape
    blocked = np.argwhere(pixels != 255)
    centers = np.stack((ox + (blocked[:, 1] + .5) * resolution,
                        oy + (height - 1 - blocked[:, 0] + .5) * resolution), axis=1)
    # USD measured chassis X bounds and full-steering wheel envelope + 0.10 m.
    xlo, xhi, ylo, yhi = footprint_bounds

    def check(x, y, angle):
        c, s = math.cos(angle), math.sin(angle)
        u, v = np.array([c, s]), np.array([-s, c])
        center = np.array([x, y]) + u * ((xlo + xhi) / 2)
        half_x, half_y = (xhi - xlo) / 2, (yhi - ylo) / 2
        delta = centers - center
        # Four separating axes give exact intersection of oriented rectangle
        # footprint and axis-aligned blocked square; touching counts as blocked.
        hit = ((np.abs(delta @ u) <= half_x + resolution / 2 * (abs(c) + abs(s)))
               & (np.abs(delta @ v) <= half_y + resolution / 2 * (abs(c) + abs(s)))
               & (np.abs(delta[:, 0]) <= half_x * abs(c) + half_y * abs(s) + resolution / 2)
               & (np.abs(delta[:, 1]) <= half_x * abs(s) + half_y * abs(c) + resolution / 2))
        corners = np.array([[xlo, ylo], [xhi, ylo], [xhi, yhi], [xlo, yhi]]) @ np.array([[c, s], [-s, c]]) + [x, y]
        inside = (corners[:, 0].min() >= ox and corners[:, 0].max() <= ox + width * resolution
                  and corners[:, 1].min() >= oy and corners[:, 1].max() <= oy + height * resolution)
        return {"inside_map": bool(inside), "intersected_blocked_cells": int(hit.sum()), "footprint_corners": corners.tolist()}

    segments = []
    for start, goal in zip(poses, poses[1:]):
        failed = []
        for fraction in np.linspace(0, 1, 201):
            pose = [start[i] + float(fraction) * (goal[i] - start[i]) for i in range(3)]
            result = check(*pose)
            if not result["inside_map"] or result["intersected_blocked_cells"]:
                failed.append({"fraction": float(fraction), "pose": pose})
        segments.append({"start": start, "goal": goal, "samples": 201, "failed_samples": len(failed), "first_failure": failed[:1]})
    return {"source_map_job": Path(metadata["request"]["output_dir"]).name, "map_sha256": image_hash,
            "footprint_bounds_m": [xlo, xhi, ylo, yhi],
            "method": "Rectangle-square SAT; unknown occupied; full footprint inside map; 201 samples per segment.",
            "poses": [{"pose": p, **check(*p)} for p in poses], "segments": segments,
            "limits": "Geometric feasibility of sampled linear pose interpolation, not a kinematically valid Nav2 path or dynamic collision guarantee."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--poses-json", type=Path, required=True, help="JSON list of [x_m,y_m,yaw_rad] poses")
    parser.add_argument("--footprint-bounds", nargs=4, type=float, default=(-2.06, 1.98, -1.521, 1.521), metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"))
    args = parser.parse_args()
    poses = json.loads(args.poses_json.read_text())
    if not isinstance(poses, list) or not poses or any(not isinstance(p, list) or len(p) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in p) for p in poses):
        parser.error("poses-json must contain finite [x,y,yaw] lists")
    if any(not math.isfinite(x) for x in args.footprint_bounds) or args.footprint_bounds[0] >= args.footprint_bounds[1] or args.footprint_bounds[2] >= args.footprint_bounds[3]:
        parser.error("Invalid footprint bounds")
    result = analyze(args.metadata, poses, args.footprint_bounds)
    args.output = private_output(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"source_map_job": result["source_map_job"],
                      "poses_clear": all(p["inside_map"] and not p["intersected_blocked_cells"] for p in result["poses"]),
                      "sampled_segments_clear": all(not p["failed_samples"] for p in result["segments"])}))
