#!/usr/bin/env python3
"""Call the plugin handler registration API without Hermes runtime or an LLM session."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
from portable_common import configure, private_output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tool", help="Full isaac_sim_* or rviz2_* tool name")
    parser.add_argument("arguments", nargs="?", default="{}", help="JSON object")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--record", help="New JSON evidence path outside the public checkout")
    args = parser.parse_args()
    configure(args.config)
    project = Path(__file__).resolve().parents[1]
    if args.tool.startswith("isaac_sim_"):
        plugin_name = "isaac-sim"
    elif args.tool.startswith("rviz2_"):
        plugin_name = "rviz2-navigation"
    else:
        parser.error("Tool must belong to isaac-sim or rviz2-navigation")
    source = project / "codes" / plugin_name
    spec = importlib.util.spec_from_file_location(
        "plugin_call_" + plugin_name.replace("-", "_"), source / "__init__.py",
        submodule_search_locations=[str(source)])
    plugin = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = plugin
    spec.loader.exec_module(plugin)

    class Context:
        def __init__(self):
            self.handlers = {}

        def register_tool(self, **kwargs):
            self.handlers[kwargs["name"]] = kwargs["handler"]

        def register_skill(self, *args, **kwargs):
            pass

    context = Context()
    plugin.register(context)
    if args.tool not in context.handlers:
        parser.error("Unknown tool: " + args.tool)
    arguments = json.loads(args.arguments)
    if not isinstance(arguments, dict):
        parser.error("Tool arguments must be a JSON object")
    # Resolve and reserve evidence before performing a mutating tool call.
    evidence = None
    if args.record:
        path = private_output(args.record)
        path.parent.mkdir(parents=True, exist_ok=True)
        evidence = path.open("x")
    result = json.loads(context.handlers[args.tool](arguments))
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    print(text, end="")
    if evidence:
        with evidence:
            evidence.write(json.dumps({"tool": args.tool, "arguments": arguments,
                                       "result": result}, ensure_ascii=False, indent=2) + "\n")
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
