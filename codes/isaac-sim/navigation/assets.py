"""Validated asset-export requests shared by the supervisor and Isaac worker.

All public distances are meters in the Z-up USD world's coordinate system.
Only the dedicated worker imports Isaac; this module uses the standard library.
"""
from __future__ import annotations

import math
import re
from pathlib import Path


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must have {size} elements")
    return [_number(x, name) for x in value]


def validate_asset_request(request):
    """Return a canonical copy, rejecting unknown or unsupported parameters.

The supervisor owns output-root policy. The worker separately requires a fresh
output directory, so old export products and source files cannot be overwritten.
"""
    if not isinstance(request, dict):
        raise ValueError("asset request must be an object")
    data = dict(request)
    aliases = {"source": "source_path", "resolution": "resolution_m", "seed": "seed_stage_xyz_m",
               "bounds_xy": "bounds_stage_xy_m"}
    for old, new in aliases.items():
        if old in data:
            if new in data:
                raise ValueError(f"Use only {new}, not both {old} and {new}")
            data[new] = data.pop(old)
    if "z_range" in data:
        low, high = _vector(data.pop("z_range"), 2, "z_range")
        if "z_min_m" in data or "z_max_m" in data:
            raise ValueError("Use z_range or z_min_m/z_max_m, not both")
        data.update(z_min_m=low, z_max_m=high)
    operation = data.get("operation")
    if operation not in ("map", "urdf"):
        raise ValueError("operation must be map or urdf")
    common = {"operation", "source_path", "output_dir"}
    allowed = (common | {"resolution_m", "seed_stage_xyz_m", "bounds_stage_xy_m", "z_min_m", "z_max_m",
                        "geometry_mode", "exclude_prim_paths", "free_thresh", "occupied_thresh", "max_cells", "max_voxels",
                        "bounds_mode", "scene_padding_m"}
               if operation == "map" else common | {"root_prim_path", "base_frame", "robot_name"})
    unknown = set(data) - allowed
    if unknown:
        raise ValueError("Unsupported export parameters: " + ", ".join(sorted(unknown)))
    for key in ("source_path", "output_dir"):
        value = data.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{key} is required")
        data[key] = str(Path(value).expanduser().resolve())
    if Path(data["source_path"]).suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        raise ValueError("source_path must be USD, USDA, USDC or USDZ")
    if not Path(data["source_path"]).is_file():
        raise ValueError("source_path must be an existing local file")
    if operation == "urdf":
        path = data.get("root_prim_path")
        if path is not None and (not isinstance(path, str) or not path.startswith("/")):
            raise ValueError("root_prim_path must be an absolute USD prim path")
        data.setdefault("robot_name", "robot")
        if not isinstance(data["robot_name"], str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,63}", data["robot_name"]):
            raise ValueError("robot_name must be 1..64 ASCII letters/digits/underscore, beginning with a letter")
        data.setdefault("base_frame", "base_link")
        if not isinstance(data["base_frame"], str) or not data["base_frame"].isidentifier():
            raise ValueError("base_frame must be a simple frame name")
        return data
    data["resolution_m"] = _number(data.get("resolution_m", .1), "resolution_m")
    if not .005 <= data["resolution_m"] <= 2:
        raise ValueError("resolution_m must be between 0.005 and 2 meters")
    data["seed_stage_xyz_m"] = _vector(data.get("seed_stage_xyz_m"), 3, "seed_stage_xyz_m")
    data.setdefault("bounds_mode", "explicit" if "bounds_stage_xy_m" in data else "full_scene")
    if data["bounds_mode"] not in ("explicit", "full_scene"):
        raise ValueError("bounds_mode must be explicit or full_scene")
    if data["bounds_mode"] == "full_scene" and "bounds_stage_xy_m" in data:
        raise ValueError("full_scene computes bounds; omit bounds_stage_xy_m")
    if data["bounds_mode"] == "explicit":
        data["bounds_stage_xy_m"] = _vector(data.get("bounds_stage_xy_m"), 4, "bounds_stage_xy_m")
    data["scene_padding_m"] = _number(data.get("scene_padding_m", 0), "scene_padding_m")
    if not 0 <= data["scene_padding_m"] <= 100:
        raise ValueError("scene_padding_m must be between 0 and 100 meters")
    if data["bounds_mode"] == "explicit" and data["scene_padding_m"]:
        raise ValueError("scene_padding_m applies only to full_scene bounds")
    for key in ("z_min_m", "z_max_m"):
        data[key] = _number(data.get(key), key)
    if data["z_min_m"] >= data["z_max_m"]:
        raise ValueError("z_min_m must be below z_max_m for volume projection")
    if not data["z_min_m"] <= data["seed_stage_xyz_m"][2] <= data["z_max_m"]:
        raise ValueError("seed Z must be within z_min_m and z_max_m")
    data.setdefault("geometry_mode", "physx")
    if data["geometry_mode"] not in ("physx", "visual_meshes"):
        raise ValueError("geometry_mode must be physx or visual_meshes")
    paths = data.setdefault("exclude_prim_paths", [])
    if not isinstance(paths, list) or any(not isinstance(x, str) or not x.startswith("/") or x == "/" for x in paths):
        raise ValueError("exclude_prim_paths must contain absolute non-root USD prim paths")
    for key, default in (("free_thresh", .196), ("occupied_thresh", .65)):
        data[key] = _number(data.get(key, default), key)
    if not 0 <= data["free_thresh"] < 128 / 255 < data["occupied_thresh"] <= 1:
        raise ValueError("Thresholds must classify gray=127 as unknown: free_thresh < 128/255 < occupied_thresh")
    data.setdefault("max_cells", 4_000_000)
    if isinstance(data["max_cells"], bool) or not isinstance(data["max_cells"], int) or not 1 <= data["max_cells"] <= 16_000_000:
        raise ValueError("max_cells must be an integer from 1 to 16000000")
    data.setdefault("max_voxels", 16_000_000)
    if isinstance(data["max_voxels"], bool) or not isinstance(data["max_voxels"], int) or not 1 <= data["max_voxels"] <= 64_000_000:
        raise ValueError("max_voxels must be an integer from 1 to 64000000")
    if data["bounds_mode"] == "explicit":
        validate_map_budget(data, data["bounds_stage_xy_m"])
    return data


def validate_map_budget(request, bounds):
    """Recheck after auto bounds are known, before native voxel allocation."""
    xmin, ymin, xmax, ymax = bounds
    if xmin >= xmax or ymin >= ymax:
        raise ValueError("bounds_stage_xy_m must be [xmin, ymin, xmax, ymax] with increasing bounds")
    if not xmin <= request["seed_stage_xyz_m"][0] < xmax or not ymin <= request["seed_stage_xyz_m"][1] < ymax:
        raise ValueError("seed XY must be inside bounds_stage_xy_m")
    resolution = request["resolution_m"]
    width, height = [math.ceil((high - low) / resolution - 1e-8)
                     for low, high in ((xmin, xmax), (ymin, ymax))]
    cells = width * height
    if cells > request["max_cells"]:
        raise ValueError(f"Requested map has {cells} cells, exceeding max_cells; increase resolution_m or the explicit budget")
    layers = max(1, math.ceil((request["z_max_m"] - request["z_min_m"]) / resolution - 1e-8))
    # Native grid quantization may add a cell at each side. Budget for that
    # overhead before calling generate3d, not after expensive allocation.
    padded_voxels = (width + 2) * (height + 2) * (layers + 2)
    if padded_voxels > request["max_voxels"]:
        raise ValueError(f"Requested volume including native grid padding has {padded_voxels} voxels, exceeding max_voxels; increase resolution_m or the explicit budget")
    return {"requested_cells": cells, "requested_volume_cells": cells * layers,
            "native_padding_budget_voxels": padded_voxels}


def resolve_map_bounds(request, geometry_min_m=None, geometry_max_m=None):
    """Resolve explicit or full-scene bounds without changing the request."""
    if request["bounds_mode"] == "explicit":
        bounds = list(request["bounds_stage_xy_m"])
    else:
        lower = _vector(geometry_min_m, 3, "geometry_min_m")
        upper = _vector(geometry_max_m, 3, "geometry_max_m")
        if any(lower[i] >= upper[i] for i in (0, 1)):
            raise ValueError("Scene geometry has no finite nonempty XY bounds")
        resolution, padding = request["resolution_m"], request["scene_padding_m"]
        bounds = [math.floor((lower[i] - padding) / resolution) * resolution for i in (0, 1)]
        bounds += [math.ceil((upper[i] + padding) / resolution) * resolution for i in (0, 1)]
    return bounds, validate_map_budget(request, bounds)


def build_worker_command(isaac_python, request_path, result_path):
    """Return argv; never shell-interpolate paths or parameters."""
    return [str(isaac_python), str(Path(__file__).with_name("asset_worker.py")),
            "--request", str(request_path), "--result", str(result_path)]
