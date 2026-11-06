"""Hermes frontend for independent RViz2/Nav2 management; no ROS imports."""
import json
from pathlib import Path
from .transport import command
from .tool_specs import TOOLS


def register(ctx):
    for name, action, description, properties, required in TOOLS:
        def handler(args, _action=action, **kwargs):
            return json.dumps(command(_action, args), ensure_ascii=False)
        parameters = {"type": "object", "properties": properties, "required": required, "additionalProperties": False}
        if action == "nav_configure_robot":
            parameters["oneOf"] = [{"required": ["profile_name"], "not": {"required": ["profile"]}},
                                   {"required": ["profile"], "not": {"required": ["profile_name"]}}]
        schema = {"name": name, "description": description, "parameters": parameters}
        ctx.register_tool(name=name, toolset="rviz2-navigation", schema=schema,
                          handler=handler, description=description)
    ctx.register_skill("rviz2-navigation", Path(__file__).parent / "skills/rviz2-navigation/SKILL.md")
