#!/usr/bin/env python3
"""Check that the C++ host and the Python runner write the same event stream.

Usage: check_events.py <path to agimus_taskplan_bt>

Runs the fake conformance session both ways -- one transaction and a plan
using every composite, each successful and with a failing capability -- and
compares the transitions (IR id, role, name, status,
previous). Exit code 0 when they match. Run by CTest (taskplan_bt_events).
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from long_tamp.tasks.task_planning.events import SCHEMA, read_events
from long_tamp.tasks.task_planning.host import create_fake_session
from long_tamp.tasks.task_planning.runner import run_plan

KEYS = ("ir_id", "role", "name", "status", "previous")


def transitions(events):
    return [tuple(event[k] for k in KEYS) for event in events]


def check(binary: str, options: dict) -> bool:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.jsonl"
        subprocess.run(
            [binary, "--factory", "create_fake_session", "--options",
             json.dumps(options), "--events", str(path)],
            check=False,
            capture_output=True,
        )
        bt = read_events(path)
    python: list[dict] = []
    run_plan(create_fake_session(json.dumps(options)), on_event=python.append)
    ok = transitions(bt) == transitions(python) and bool(bt)
    ok = ok and all(e["schema"] == SCHEMA and e["source"] == "bt" for e in bt)
    print(f"options={options}: {'match' if ok else 'MISMATCH'} ({len(bt)} events)")
    if not ok:
        for side, events in (("bt", bt), ("python", python)):
            print(f"  {side}:")
            for t in transitions(events):
                print("   ", t)
    return ok


def main() -> int:
    binary = sys.argv[1]
    results = [
        check(binary, options)
        for options in (
            {},
            {"fault": "capability_raises"},
            {"shape": "composite"},
            {"shape": "composite", "fault": "capability_raises"},
        )
    ]
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
