"""Record only whitelisted gateway reload/session fields, never its full snapshot."""
import argparse
import asyncio
import json
from pathlib import Path
from portable_common import configure, load_hermes_source, private_output


async def session_status(session_id):
    from hermes_cli.gateway_client import connect_gateway
    async with connect_gateway() as client:
        snapshot = await client.rpc("session.resume", session_id=session_id)
    result = {"session_id": session_id,
              "running": snapshot.get("running"),
              "execution_generation": snapshot.get("execution_generation")}
    model = (snapshot.get("info") or {}).get("model")
    if isinstance(model, str):
        result["model"] = model
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--reload", action="store_true")
    group.add_argument("--session")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    config = configure(args.config)
    load_hermes_source(config)
    if args.output:
        args.output = private_output(args.output)
    if args.reload:
        from hermes_cli.plugins_activation import reload_running_gateway
        response = reload_running_gateway() or {}
        result = {"reloaded": bool(response.get("reloaded")),
                  "gateway_answered": bool(response), "gateway_error": response.get("error")}
        for name in ("isaac-sim", "rviz2-navigation"):
            activation = next(
                (item for item in response.get("activations", []) if item.get("name") == name), None)
            result[name.replace("-", "_")] = None if activation is None else {
                "name": activation.get("name"), "key": activation.get("key"),
                "tools_for_new_sessions": (activation.get("deferred") or {}).get("tools", []),
            }
        result["both_plugins_available"] = bool(result["reloaded"] and result["isaac_sim"]
                                               and result["rviz2_navigation"])
    else:
        result = asyncio.run(session_status(args.session))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False))
    # Persist sanitized failure evidence before returning a failing status.
    return 1 if args.reload and not result["both_plugins_available"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
