"""Named, reviewed scene operations using the existing authenticated bridge."""
import math
from types import MappingProxyType

if __package__:
    from .transport import command
    from .deployment import asset_path, load_config
else:
    from transport import command
    from deployment import asset_path, load_config


_SCRIPTS = MappingProxyType({
    "load_gongguan_xinhai": MappingProxyType({
        "title": "公館辛亥路場景",
        "aliases": ("公館辛亥路", "公館辛亥路場景"),
        "asset_key": "gongguan_usd",
        "action": "load",
        "effect": "replace_current_stage",
        "preconditions": ("running", "stopped"),
    }),
    "load_rv1": MappingProxyType({
        "title": "RV1 載具測試場景",
        "aliases": ("RV1", "RV1 載具", "RV1 測試場景"),
        "asset_key": "rv1_usd",
        "scene_path_base": "generated_session_wrapper",
        "action": "rv1_load",
        "effect": "replace_current_stage",
        "preconditions": ("running", "stopped"),
    }),
    "add_rv1_to_gongguan": MappingProxyType({
        "title": "在公館辛亥路加入 RV1 與物理碰撞",
        "aliases": ("公館辛亥路加入RV1",),
        "asset_key": "gongguan_usd",
        "action": "rv1_insert",
        "effect": "augment_current_stage_session_layer",
        "preconditions": ("running", "stopped", "gongguan_loaded", "rv1_not_already_inserted"),
    }),
})


def _error(code, message):
    return {"success": False, "error": {"code": code, "message": message}}


def list_scripts(args=None):
    """List scripts without contacting the host or exposing mutable registry data."""
    if args is not None and (not isinstance(args, dict) or args):
        return _error("INVALID_ARGUMENT", "list_scripts accepts no arguments.")
    try:
        config = load_config()
    except (OSError, ValueError) as exc:
        return _error("CONFIGURATION_ERROR", str(exc))
    return {"success": True, "scripts": [
        {"script_name": name, **entry, "aliases": list(entry["aliases"]),
         "preconditions": list(entry["preconditions"]),
         "scene_path": config.get(entry["asset_key"]),
         "configured": bool(config.get(entry["asset_key"]))}
        for name, entry in _SCRIPTS.items()
    ]}


def run_script(args):
    """Submit one allowlisted scene action; never open, stop, play, or execute user code."""
    if not isinstance(args, dict) or set(args) - {"script_name", "wait_seconds"}:
        return _error("INVALID_ARGUMENT", "Expected script_name and optional wait_seconds only.")
    name = args.get("script_name")
    if not isinstance(name, str) or not name:
        return _error("INVALID_ARGUMENT", "script_name must be a nonempty string.")
    if name not in _SCRIPTS:
        return _error("UNKNOWN_SCRIPT", "Unknown script_name; use isaac_sim_list_scripts to list available scripts.")
    seconds = args.get("wait_seconds", 10)
    if (isinstance(seconds, bool) or not isinstance(seconds, (int, float))
            or not 0 <= seconds <= 60 or not math.isfinite(seconds)):
        return _error("INVALID_ARGUMENT", "wait_seconds must be a finite number from 0 to 60.")
    entry = _SCRIPTS[name]
    params = {"wait_seconds": seconds}
    try:
        path = asset_path(entry["asset_key"])
        if entry["action"] == "rv1_insert":
            asset_path("rv1_usd")
    except (OSError, ValueError) as exc:
        return _error("CONFIGURATION_ERROR", str(exc))
    if entry["action"] == "load":
        params["path"] = path
    result = command(entry["action"], params)
    return {**result, "script_name": name}
