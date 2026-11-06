"""Run one real configured-model session; private raw stream stays on the host."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import shutil
from portable_common import configure, private_output


def now():
    return datetime.now(timezone.utc).isoformat()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("prompt", type=Path)
    p.add_argument("directory", type=Path)
    p.add_argument("--config", type=Path)
    p.add_argument("--hermes", help="Hermes CLI executable; defaults to saved hermes_cli or PATH")
    p.add_argument("--cwd", type=Path, help="Agent working directory, defaults to output directory")
    p.add_argument("--max-turns", type=int, default=60)
    p.add_argument("--reasoning", default="low")
    a = p.parse_args()
    config = configure(a.config)
    a.directory = private_output(a.directory)
    a.prompt = a.prompt.expanduser().resolve(strict=True)
    executable = a.hermes or config.get("hermes_cli") or shutil.which("hermes")
    if not executable:
        p.error("Configure hermes_cli, provide --hermes, or add hermes to PATH")
    cwd = a.cwd.expanduser().resolve(strict=True) if a.cwd else a.directory
    if a.max_turns < 1:
        p.error("--max-turns must be positive")
    os.umask(0o077)
    a.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    meta = {"started_utc": now(), "prompt": str(a.prompt), "model": "host-configured",
            "reasoning_override": a.reasoning, "cli_returncode": None}
    command = [executable, "chat", "--oneshot", "--format", "stream-json",
               "-t", "isaac-sim,rviz2-navigation",
               "-s", "isaac-sim:isaac-sim", "-s", "rviz2-navigation:rviz2-navigation",
               "--max-turns", str(a.max_turns),
               "--reasoning", a.reasoning, "--source", "tool", "--in",
               str(cwd), "--query-file", str(a.prompt)]
    meta["argv"] = command
    with (a.directory / "private-stream.jsonl").open("x") as stdout, (a.directory / "private-stderr.log").open("x") as stderr:
        result = subprocess.run(command, stdout=stdout, stderr=stderr)
    meta.update(cli_returncode=result.returncode, finished_utc=now())
    match = re.search(r"Session:\s*(local-[a-z0-9]+)", (a.directory / "private-stderr.log").read_text())
    meta["hermes_session_id"] = match.group(1) if match else None
    events = []
    for line in (a.directory / "private-stream.jsonl").read_text().splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get("type") not in {"tool_use", "tool_result"}:
            continue
        if event.get("type") == "tool_result" and isinstance(event.get("output"), str):
            try:
                event["output"] = json.loads(event["output"])
            except ValueError:
                pass
        events.append(event)
    (a.directory / "tool-events.json").write_text(json.dumps(events, ensure_ascii=False, indent=2))
    (a.directory / "execution.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(json.dumps(meta, ensure_ascii=False))
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
