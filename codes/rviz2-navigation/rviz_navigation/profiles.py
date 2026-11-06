"""Validated, immutable single-robot configuration; no ROS or Isaac imports."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re
from deployment import PACKAGE_ROOT


FIELDS = {"robot_id", "frames", "topics", "use_sim_time", "params_file", "footprint",
          "footprint_padding_m", "localization_mode", "control_backend", "limits", "allow_lateral"}
TOPICS = {"map", "odom", "cmd_vel", "joint_states", "robot_description", "clock"}
LIMITS = {"max_linear_mps": 1.0, "max_yaw_rps": 1.0,
          "command_timeout_seconds": 3.0, "feedback_timeout_seconds": 5.0}


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _name(value, name, *, absolute=False):
    if not isinstance(value, str):
        raise ValueError(f"{name} must be a ROS name")
    raw = value[1:] if absolute and value.startswith("/") else value
    if absolute and not value.startswith("/"):
        raise ValueError(f"{name} must be an absolute topic")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", raw):
        raise ValueError(f"{name} is not a valid ROS name")
    return value


def polygon(value):
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError("footprint must contain at least three XY vertices")
    result = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("footprint vertices must contain two numbers")
        result.append([finite(point[0], "footprint x"), finite(point[1], "footprint y")])
    area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(result, result[1:] + result[:1])))
    if area < 1e-8:
        raise ValueError("footprint must have nonzero area")
    return result


def validate_profile(profile):
    if not isinstance(profile, dict) or set(profile) - FIELDS:
        raise ValueError("Unknown robot profile fields")
    required = FIELDS - {"footprint", "footprint_padding_m", "limits", "allow_lateral"}
    if required - set(profile):
        raise ValueError(f"Missing robot profile fields: {sorted(required - set(profile))}")
    result = copy.deepcopy(profile)
    if not isinstance(result["robot_id"], str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,63}", result["robot_id"]):
        raise ValueError("robot_id must be a letter followed by up to 63 letters, digits or underscores")
    for block, keys in (("frames", {"map", "odom", "base"}), ("topics", TOPICS)):
        if not isinstance(result[block], dict) or set(result[block]) != keys:
            raise ValueError(f"{block} must specify exactly {sorted(keys)}")
        for name, value in result[block].items():
            _name(value, f"{block}.{name}", absolute=block == "topics")
        if len(set(result[block].values())) != len(keys):
            raise ValueError(f"All {block} must be distinct")
    if not isinstance(result["use_sim_time"], bool):
        raise ValueError("use_sim_time must be boolean")
    if result["localization_mode"] not in {"sim_truth", "tf_verified"}:
        raise ValueError("localization_mode must be sim_truth or tf_verified; an existing map-to-base TF is required")
    if result["localization_mode"] == "sim_truth" and not result["use_sim_time"]:
        raise ValueError("sim_truth requires use_sim_time")
    if result["control_backend"] not in {"isaac_rv1", "ros2"}:
        raise ValueError("control_backend must be isaac_rv1 or ros2")
    if not isinstance(result["params_file"], str) or not result["params_file"]:
        raise ValueError("params_file must name an existing Nav2 YAML file")
    path = Path(result["params_file"]).expanduser()
    if not path.is_absolute():
        path = PACKAGE_ROOT / path
    path = path.resolve(strict=True)
    if not path.is_file():
        raise ValueError("params_file must be a regular file")
    result["params_file"] = str(path)
    if "footprint" in result:
        result["footprint"] = polygon(result["footprint"])
    if "footprint_padding_m" in result:
        result["footprint_padding_m"] = finite(result["footprint_padding_m"], "footprint_padding_m")
        if result["footprint_padding_m"] < 0:
            raise ValueError("footprint_padding_m cannot be negative")
    limits = result.get("limits", {})
    if not isinstance(limits, dict) or set(limits) - set(LIMITS):
        raise ValueError("Unknown velocity/feedback limits")
    result["limits"] = {**LIMITS, **limits}
    for name, value in result["limits"].items():
        value = finite(value, name)
        if not 0 < value <= (60 if "seconds" in name else 10):
            raise ValueError(f"{name} exceeds the supported positive range")
        result["limits"][name] = value
    result.setdefault("allow_lateral", False)
    if not isinstance(result["allow_lateral"], bool):
        raise ValueError("allow_lateral must be boolean")
    reserved = f"/rviz_navigation/{result['robot_id']}/"
    if any(topic.startswith(reserved) for topic in result["topics"].values()):
        raise ValueError("Configured topics cannot alias the private navigation relay topics")
    # Validate the robot's own Nav2 footprint here, before starting processes.
    nav2_parameters(result)
    return result


def load_profile(path):
    return validate_profile(json.loads(Path(path).expanduser().read_text()))


def default_rv1_profile(project):
    return validate_profile({
        "robot_id": "rv1", "frames": {"map": "map", "odom": "odom", "base": "base_link"},
        "topics": {name: "/" + name for name in TOPICS}, "use_sim_time": True,
        "params_file": str(Path(project) / "configs/rv1_nav2.yaml"),
        "localization_mode": "sim_truth", "control_backend": "isaac_rv1",
        "limits": {"max_linear_mps": 1.0, "max_yaw_rps": 0.5}})


def maximum_speed_rv1_profile(project):
    """Explicit opt-in; preserve the baseline profile and controller envelope."""
    profile = default_rv1_profile(project)
    profile["params_file"] = str(Path(project) / "configs/rv1_nav2_max_speed.yaml")
    profile["limits"]["max_linear_mps"] = 2.5
    return validate_profile(profile)


def nav2_parameters(profile):
    import yaml
    content = yaml.safe_load(Path(profile["params_file"]).read_text())
    if not isinstance(content, dict):
        raise ValueError("Nav2 parameters must be a YAML mapping")
    content = copy.deepcopy(content)
    frames, topics = profile["frames"], profile["topics"]
    sim = profile["use_sim_time"]
    footprints = []
    for name, frame in (("global_costmap", frames["map"]), ("local_costmap", frames["odom"])):
        try:
            params = content[name][name]["ros__parameters"]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"Missing {name} parameters for this robot") from exc
        if "footprint" in profile:
            params["footprint"] = json.dumps(profile["footprint"])
        params["footprint"] = json.dumps(polygon(params.get("footprint")))
        if "footprint_padding_m" in profile:
            params["footprint_padding"] = profile["footprint_padding_m"]
        padding = finite(params.get("footprint_padding", 0.0), "footprint_padding")
        if padding < 0:
            raise ValueError("footprint_padding cannot be negative")
        footprints.append((params["footprint"], padding))
        params.update(use_sim_time=sim, global_frame=frame, robot_base_frame=frames["base"])
        static_layers = [layer for layer in params.get("plugins", [])
                         if isinstance(params.get(layer), dict)
                         and params[layer].get("plugin") in {"nav2_costmap_2d::StaticLayer", "nav2_costmap_2d/StaticLayer"}]
        if not static_layers:
            raise ValueError(f"{name} requires a configured static-map layer")
        for layer in static_layers:
            params[layer]["map_topic"] = topics["map"]
    if footprints[0] != footprints[1]:
        raise ValueError("Global/local footprints and padding must match")
    for name in ("controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"):
        params = content.setdefault(name, {}).setdefault("ros__parameters", {})
        params["use_sim_time"] = sim
        if name in {"controller_server", "velocity_smoother", "bt_navigator"}:
            params["odom_topic"] = topics["odom"]
        if name in {"behavior_server", "bt_navigator"}:
            params.update(global_frame=frames["map"], robot_base_frame=frames["base"])
        if name == "behavior_server":
            params["local_frame"] = frames["odom"]
        if name in {"controller_server", "behavior_server", "velocity_smoother"}:
            params["enable_stamped_cmd_vel"] = False
    navigation_ids(content)
    return content


def navigation_ids(content):
    """Use the robot's first configured plugins, not RV1-specific plugin IDs."""
    result = {}
    for purpose, node, field in (("planner", "planner_server", "planner_plugins"),
                                 ("controller", "controller_server", "controller_plugins"),
                                 ("goal_checker", "controller_server", "goal_checker_plugins")):
        plugins = content[node]["ros__parameters"].get(field)
        if not isinstance(plugins, list) or not plugins:
            raise ValueError(f"{node}.{field} must list at least one configured plugin")
        selected = plugins[0]
        if not isinstance(selected, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", selected):
            raise ValueError(f"Unsupported {purpose} plugin ID")
        declaration = content[node]["ros__parameters"].get(selected)
        if not isinstance(declaration, dict) or not isinstance(declaration.get("plugin"), str):
            raise ValueError(f"Missing plugin declaration for {node}.{selected}")
        result[purpose] = selected
    return result
