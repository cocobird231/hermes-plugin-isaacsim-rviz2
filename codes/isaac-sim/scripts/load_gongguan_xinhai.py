"""Load 公館辛亥路 in the existing, stopped Isaac Sim instance.

Does not open Isaac Sim or start/stop simulation. A successful response with
pending=true means accepted, not completed; poll isaac_sim_status afterward.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene_scripts import run_script


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wait-seconds", type=float, default=10,
                        help="Wait 0–60 seconds; pending=true requires status polling.")
    options = parser.parse_args(argv)
    result = run_script({"script_name": "load_gongguan_xinhai", "wait_seconds": options.wait_seconds})
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
