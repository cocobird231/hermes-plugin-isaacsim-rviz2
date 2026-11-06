"""Portable host configuration shared by the two Hermes simulation plugins.

Configuration is private machine data, never a file shipped with this repository.
Importing this module does not start software or read Hermes model credentials.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil


class ConfigError(ValueError):
    """Invalid or incomplete host configuration."""


PATH_FIELDS = frozenset({
    "isaac_python", "ros_python", "ros_setup", "screenshot_python",
    "gongguan_usd", "rv1_usd", "xauthority", "state_root", "export_root",
    "hermes_cli", "hermes_source", "isaac_ros_lib_dir",
})
EXECUTABLE_FIELDS = frozenset({"isaac_python", "ros_python", "screenshot_python", "hermes_cli"})
DIRECTORY_FIELDS = frozenset({"state_root", "export_root", "hermes_source", "isaac_ros_lib_dir"})
REPOSITORY_ROOT = Path(__file__).resolve().parent


def _hermes_home():
    return Path(os.environ.get("HERMES_HOME") or Path.home() / ".hermes").expanduser().resolve()


def config_path():
    """Return the selected external JSON file without creating it."""
    return Path(os.environ.get("HERMES_SIM_CONFIG") or _hermes_home() / "sim-plugins.json").expanduser().resolve()


def defaults():
    root = _hermes_home() / "sim-plugins-state"
    local_cli = Path.home() / ".local/bin/hermes"
    return {
        "schema_version": 1,
        "isaac_python": "",
        "ros_python": "/usr/bin/python3",
        "ros_setup": "/opt/ros/jazzy/setup.bash",
        "screenshot_python": "/usr/bin/python3",
        "gongguan_usd": "",
        "rv1_usd": "",
        "display_number": 1,
        "xauthority": str(Path(os.environ.get("XAUTHORITY") or Path.home() / ".Xauthority").expanduser()),
        "ros_domain_id": 37,
        "rmw_implementation": "rmw_fastrtps_cpp",
        "ros_localhost_only": True,
        "isaac_port": 8768,
        "rviz2_port": 8769,
        "state_root": str(root),
        "export_root": str(root / "exports"),
        "hermes_cli": shutil.which("hermes") or (str(local_cli) if local_cli.is_file() else ""),
        "hermes_source": "",
        "isaac_ros_lib_dir": "",
    }


def normalize_config(data, required=(), base_dir=None):
    """Validate JSON values, expand paths, and check explicitly required resources.

    Relative paths, when supplied, are resolved against the JSON file directory.
    Optional resources may be absent so RViz/Nav2 can operate without Isaac.
    """
    if not isinstance(data, dict):
        raise ConfigError("Configuration must be a JSON object")
    result = defaults()
    unknown = set(data) - set(result)
    if unknown:
        raise ConfigError("Unknown configuration fields: " + ", ".join(sorted(unknown)))
    result.update(data)
    if type(result["schema_version"]) is not int or result["schema_version"] != 1:
        raise ConfigError("schema_version must be integer 1")
    for name, low, high in (("display_number", 0, 99), ("ros_domain_id", 0, 232),
                             ("isaac_port", 1024, 65535), ("rviz2_port", 1024, 65535)):
        value = result[name]
        if type(value) is not int or not low <= value <= high:
            raise ConfigError(f"{name} must be an integer in [{low}, {high}]")
    if result["isaac_port"] == result["rviz2_port"]:
        raise ConfigError("isaac_port and rviz2_port must differ")
    if type(result["ros_localhost_only"]) is not bool:
        raise ConfigError("ros_localhost_only must be boolean")
    rmw = result["rmw_implementation"]
    if not isinstance(rmw, str) or not rmw or not all(c.isalnum() or c == "_" for c in rmw):
        raise ConfigError("rmw_implementation must be a nonempty ROS middleware identifier")
    base = Path(base_dir or Path.cwd()).expanduser().resolve()
    for name in PATH_FIELDS:
        value = result[name]
        if not isinstance(value, str) or "\0" in value or "\n" in value or "\r" in value:
            raise ConfigError(f"{name} must be a path string without control characters")
        if not value:
            if name in {"state_root", "export_root"}:
                raise ConfigError(f"{name} cannot be empty")
            continue
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = base / path
        result[name] = os.path.abspath(path)
    # Choosing a custom state root also places default exports under that root.
    if "state_root" in data and "export_root" not in data:
        result["export_root"] = str(Path(result["state_root"]) / "exports")
    for name in ("state_root", "export_root"):
        path = Path(result[name]).resolve()
        if path == REPOSITORY_ROOT or REPOSITORY_ROOT in path.parents:
            raise ConfigError(f"{name} must be outside the public plugin repository")
        if path.exists() and not path.is_dir():
            raise ConfigError(f"{name} is not a directory")
    for name in ("gongguan_usd", "rv1_usd"):
        if result[name] and Path(result[name]).suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
            raise ConfigError(f"{name} must reference a USD, USDA, USDC or USDZ file")
    for name in required:
        if name not in result:
            raise ConfigError(f"Unknown required field: {name}")
        value = result[name]
        if value is None or value == "":
            raise ConfigError(f"Missing {name}; ask the user for it and run bash install.sh --{name.replace(chr(95), chr(45))} PATH")
        if name in PATH_FIELDS:
            path = Path(value)
            if name in {"state_root", "export_root"}:
                continue
            if name in DIRECTORY_FIELDS:
                if not path.is_dir():
                    raise ConfigError(f"{name} directory does not exist: {path}")
            elif not path.is_file():
                raise ConfigError(f"{name} file does not exist: {path}")
            if name in EXECUTABLE_FIELDS and not os.access(path, os.X_OK):
                raise ConfigError(f"{name} is not executable: {path}")
    return result


def load_config(required=()):
    """Read configuration on each call, allowing tools to use updated host settings."""
    path = config_path()
    if not path.exists():
        return normalize_config({}, required=required, base_dir=path.parent)
    try:
        with path.open(encoding="utf-8") as source:
            data = json.load(source, object_pairs_hook=_unique_object)
    except (OSError, ValueError) as exc:
        raise ConfigError(f"Cannot read valid configuration from {path}: {exc}") from exc
    return normalize_config(data, required=required, base_dir=path.parent)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result
