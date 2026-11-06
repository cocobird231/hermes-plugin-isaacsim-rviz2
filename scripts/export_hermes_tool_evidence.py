"""Read a completed Hermes session and export only plugin tool calls/results.

Never serialize the raw snapshot or assistant prose/reasoning, session config,
provider credentials, or any private CLI stream. This does not start a turn.
"""
import argparse
import asyncio
import json
from pathlib import Path
from portable_common import configure, load_hermes_source, private_output


async def snapshot_for(session_id):
    from hermes_cli.gateway_client import connect_gateway
    async with connect_gateway() as client:
        return await client.rpc("session.resume", session_id=session_id)


def inspect_structure(snapshot):
    """Show field names and collection sizes, never message/config values."""
    result = {"snapshot_keys": sorted(snapshot)}
    for key, value in snapshot.items():
        item = {"type": type(value).__name__}
        if isinstance(value, (list, dict)):
            item["length"] = len(value)
        if isinstance(value, list):
            item["dictionary_entry_keys"] = sorted({k for entry in value if isinstance(entry, dict) for k in entry})
        result[key] = item
    result["tool_structure"] = [
        {"role": message.get("role"), "tool_name": message.get("tool_name"),
         "tool_call_id": message.get("tool_call_id"),
         "tool_content_lengths": {key: len(message[key]) if isinstance(message.get(key), (str, list, dict)) else None
                                  for key in ("content", "api_content")} if message.get("role") == "tool" else None,
         "calls": [{"keys": sorted(call), "name": (call.get("function") or {}).get("name"),
                    "id": call.get("id")} for call in message.get("tool_calls") or []]}
        for message in snapshot.get("messages", []) if isinstance(message, dict)
        and (message.get("role") == "tool" or message.get("tool_calls"))]
    return result


def tool_events(snapshot):
    messages = snapshot.get("messages")
    if not isinstance(messages, list):
        raise ValueError("Snapshot does not expose a messages list; use --inspect to inspect only field names.")
    def allowed(name):
        return isinstance(name, str) and (name.startswith(("isaac_sim_", "rviz2_")) or name == "tool_describe")

    # Hermes can persist a gateway-level tool_call wrapper while the result names
    # the concrete plugin tool. Join by call ID; never export unrelated tools.
    events = []
    calls = {message.get("tool_call_id"): message.get("tool_name")
             for message in messages if isinstance(message, dict)
             and message.get("role") == "tool" and allowed(message.get("tool_name"))}
    for index, message in enumerate(messages):
        if not isinstance(message, dict):
            continue
        for call in message.get("tool_calls") or []:
            function = call.get("function") or {}
            identifier = call.get("id")
            name = function.get("name", "")
            if name == "tool_call":
                name = calls.get(identifier)
            if not allowed(name):
                continue
            calls[identifier] = name
            events.append({"type": "tool_use", "message_index": index,
                           "name": name, "tool_call_id": identifier,
                           "gateway_function": function.get("name"),
                           "arguments": function.get("arguments")})
        if message.get("role") == "tool":
            identifier = message.get("tool_call_id")
            name = calls.get(identifier)
            if name is None:
                continue
            content = message.get("content")
            if isinstance(content, str):
                try:
                    output = json.loads(content)
                    complete_json = True
                except json.JSONDecodeError:
                    output, complete_json = content, False
            else:
                output, complete_json = content, isinstance(content, (dict, list))
            events.append({"type": "tool_result", "message_index": index,
                           "name": name, "tool_call_id": identifier,
                           "output": output, "complete_json": complete_json,
                           "content_characters": len(content) if isinstance(content, str) else None})
    return {"source": "gateway session.resume messages; tool-only whitelist",
            "running": snapshot.get("running"),
            "execution_generation": snapshot.get("execution_generation"),
            "events": events}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    config = configure(args.config)
    load_hermes_source(config)
    if args.output:
        args.output = private_output(args.output)
    snapshot = asyncio.run(snapshot_for(args.session))
    if args.inspect:
        print(json.dumps(inspect_structure(snapshot), ensure_ascii=False))
        return
    if not args.output:
        parser.error("--output is required for a tool-only export")
    result = {"session_id": args.session, **tool_events(snapshot)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "events": len(result["events"]),
                      "running": result["running"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
