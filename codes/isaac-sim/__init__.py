"""Hermes plugin: Isaac lifecycle, robot asset exports and simulator-side ROS adapters."""
import json
from pathlib import Path

from .transport import command
from .scene_scripts import list_scripts, run_script
from .navigation.tool_specs import TOOLS as ASSET_TOOLS

WAIT = {"type": "number", "minimum": 0, "maximum": 60,
        "description": "Wait up to this many seconds; pending=true requires isaac_sim_status polling."}
SPECS = [
    ("open", "Open Isaac Sim (GUI by default, new windows use the configured display_number; generic default :1). display_number selects an existing host X display. Omit it to reuse an already running instance; an explicit different display requires closing first. Does not start simulation.",
     {"headless": {"type": "boolean", "default": False},
      "display_number": {"type": "integer", "minimum": 0, "maximum": 99,
                         "description": "Host X display number, e.g. 1 for :1 or 3 for :3. Overrides the bridge default for a new GUI. The desktop must already exist. Omit to reuse an existing instance. Headless mode has no GUI window."},
      "wait_seconds": WAIT}, []),
    ("load_usd", "Load a local USD scene using an absolute path on the Isaac HOST. Stop simulation first. Poll isaac_sim_status while pending.",
     {"path": {"type": "string", "description": "Absolute .usd/.usda/.usdc/.usdz path on the Isaac host, not the Hermes container."}, "wait_seconds": WAIT}, ["path"]),
    ("play", "Start the simulation of the loaded USD scene. Verifies Playing plus advancing physics time and steps; animation time may loop independently. Does not launch a new application.",
     {"wait_seconds": WAIT}, []),
    ("stop", "Stop simulation via timeline.stop (not pause). The Isaac application remains open; simulation time resets to the start.",
     {"wait_seconds": WAIT}, []),
    ("close", "Close only the Isaac application launched by this plugin. Wait for process exit. force=true terminates only its verified owned process group if graceful close is unavailable.",
     {"wait_seconds": WAIT, "force": {"type": "boolean", "default": False}}, []),
    ("status", "Read managed process, display, display_number, headless mode, scene, timeline, physics simulation_time and physics_steps, animation_time, operation and errors. Includes measured RV1 pose, velocity and Nav2 ownership when loaded. accepted/pending is not completion. Verify fresh state before retrying.", {}, []),
    ("capture", "Capture the visible X11 window of the managed GUI instance to a PNG and state JSON sidecar in its session screenshots directory. Requires a live owned window and fresh runtime state; rejects headless mode. Returns host paths, SHA-256, dimensions and capture timestamp. Synchronous, does not change or wait for the current simulation operation. Visible desktop pixels can include occlusion; this is not an offscreen sensor render.",
     {"label": {"type": "string", "minLength": 1, "maxLength": 48,
                "pattern": "^[a-z0-9][a-z0-9_-]{0,47}$",
                "description": "Required safe filename label, e.g. gongguan-rv1-ready. Lowercase letters, digits, underscores and hyphens; begin with a letter or digit. The output directory is fixed by the managed session."}}, ["label"]),
    ("list_scripts", "List the plugin's registered scene scripts, their host paths and preconditions. Includes 公館辛亥路 (Gongguan Xinhai Road), the generated RV1 vehicle test scene, and RV1 insertion into the current Gongguan scene. Does not change Isaac Sim.", {}, []),
    ("run_script", "Run a registered scene script in the existing managed instance, with simulation stopped. load_gongguan_xinhai and load_rv1 replace the current USD stage with Gongguan or a generated RV1 test scene using configured rv1_usd. add_rv1_to_gongguan only augments the existing Gongguan stage in its session layer; it requires Gongguan loaded and no existing insertion. Does not launch, stop or play. Returns the host operation request_id; poll status and verify fresh state after rendering warmup.",
     {"script_name": {"type": "string", "description": "Registered identifier from isaac_sim_list_scripts: load_gongguan_xinhai, load_rv1 or add_rv1_to_gongguan. Not a filename or Python source."},
      "wait_seconds": WAIT}, ["script_name"]),

]


def register(ctx):
    for suffix, description, properties, required in SPECS:
        name = "isaac_sim_" + suffix
        action = "load" if suffix == "load_usd" else suffix

        def handler(args, _action=action, **kwargs):
            if _action == "list_scripts":
                result = list_scripts(args)
            elif _action == "run_script":
                result = run_script(args)
            else:
                result = command(_action, args)
            return json.dumps(result, ensure_ascii=False)

        schema = {"name": name, "description": description,
                  "parameters": {"type": "object", "properties": properties,
                                 "required": required, "additionalProperties": False}}
        ctx.register_tool(name=name, toolset="isaac-sim", schema=schema,
                          handler=handler, description=description)
    for name, action, description, properties, required in ASSET_TOOLS:
        def navigation_handler(args, _action=action, **kwargs):
            return json.dumps(command(_action, args), ensure_ascii=False)
        schema = {"name": name, "description": description,
                  "parameters": {"type": "object", "properties": properties,
                                 "required": required, "additionalProperties": False}}
        ctx.register_tool(name=name, toolset="isaac-sim", schema=schema,
                          handler=navigation_handler, description=description)
    ctx.register_skill("isaac-sim", Path(__file__).parent / "skills/isaac-sim/SKILL.md")
