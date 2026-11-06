"""Call the same authenticated API used by the Hermes tools."""
import argparse
import json

from transport import command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("open", "load", "play", "stop", "close", "status"):
        item = sub.add_parser(name)
        if name != "status":
            item.add_argument("--wait-seconds", type=float, default=5 if name in ("play", "stop") else 10)
        if name == "open":
            item.add_argument("--headless", action="store_true")
            item.add_argument("--display-number", type=int, default=argparse.SUPPRESS,
                              help="Host X display number (0..99); new GUI defaults to :1")
        if name == "close":
            item.add_argument("--force", action="store_true")
        if name == "load":
            item.add_argument("path")
    args = vars(parser.parse_args())
    action = args.pop("action")
    result = command(action, args)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
