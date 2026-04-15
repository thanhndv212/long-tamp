"""
Utilities for loading and inspecting agimus_spacelab run logs.

Supports both:
- ``*.jsonl`` streaming event logs (from :class:`~RunLogger`)
- ``*.json`` snapshot files (written at :meth:`~RunLogger.close`)

Quick-start::

    from agimus_spacelab.logging import load_run_log, print_run_summary

    # Human-readable overview
    print_run_summary("/tmp/runs/run_20260415_abc12345.jsonl")

    # Structured access
    run = load_run_log("/tmp/runs/run_20260415_abc12345.jsonl")
    print(run["run_start"]["backend"])
    print(run["config_snapshot"]["task_config"]["GRIPPERS"])
    for phase in run["phase_results"]:
        print(phase["phase"], phase["success"], phase["phase_time"])

    # Streaming iteration (memory-efficient for large files)
    for event in iter_events("/tmp/runs/run_20260415_abc12345.jsonl"):
        if event["event"] == "edge_end" and not event["success"]:
            print("FAILED EDGE:", event["edge_name"], event["error"])

    # Extract replay parameters
    cfg = get_replay_config("/tmp/runs/run_20260415_abc12345.jsonl")
    # cfg["sequence"]["q_init"] → initial config
    # cfg["sequence"]["grasp_sequence"] → [(gripper, handle), ...]
"""

import json
import os
from typing import Any, Dict, Generator, List, Optional


def iter_events(
    jsonl_path: str,
) -> Generator[Dict[str, Any], None, None]:
    """Iterate over events in a JSONL run log file.

    Args:
        jsonl_path: Path to the ``.jsonl`` log file.

    Yields:
        Parsed event dicts in the order they were logged.  Malformed lines
        are silently skipped.
    """
    with open(jsonl_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def load_run_log(path: str) -> Dict[str, Any]:
    """Load a run log file into a structured dict.

    Accepts either a ``*.jsonl`` streaming log (parsed event by event) or a
    ``*.json`` snapshot file (loaded directly).

    Returns a dict with:

    - ``run_id``: Run identifier string.
    - ``events``: Ordered list of all event records.
    - One key per unique event type (first occurrence), e.g.
      ``"run_start"``, ``"config_snapshot"``, ``"sequence_start"``,
      ``"run_end"``.
    - ``"phase_results"``: List of ``phase_end`` records in order.

    Args:
        path: Absolute path to a ``.jsonl`` or ``.json`` log file.

    Returns:
        Structured log dict as described above.
    """
    if path.endswith(".json"):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    result: Dict[str, Any] = {"events": [], "phase_results": []}
    for rec in iter_events(path):
        result["events"].append(rec)
        et = rec.get("event")
        if et and et not in result:
            result[et] = rec
        if et == "phase_end":
            result["phase_results"].append(rec)
        if "run_id" not in result and "run_id" in rec:
            result["run_id"] = rec["run_id"]
    return result


def get_replay_config(path: str) -> Dict[str, Any]:
    """Extract the parameters needed to replay a run from a log file.

    Args:
        path: Path to the ``.jsonl`` or ``.json`` log file.

    Returns:
        Dict with keys:

        - ``"backend"``, ``"task_name"``
        - ``"task_config"``: serialised ``BaseTaskConfig`` fields.
        - ``"setup_params"``: ``setup()`` call parameters.
        - ``"sequence"``: dict with ``grasp_sequence``, ``q_init``, and
          all ``plan_sequence()`` call parameters.
    """
    run_log = load_run_log(path)
    replay: Dict[str, Any] = {}

    run_start = run_log.get("run_start", {})
    if run_start:
        replay["backend"] = run_start.get("backend")
        replay["task_name"] = run_start.get("task_name")

    config_snap = run_log.get("config_snapshot", {})
    if config_snap:
        replay["task_config"] = config_snap.get("task_config", {})
        replay["setup_params"] = config_snap.get("setup_params", {})

    seq_start = run_log.get("sequence_start", {})
    if seq_start:
        replay["sequence"] = {
            k: seq_start.get(k)
            for k in [
                "grasp_sequence",
                "q_init",
                "validate",
                "max_iterations_per_edge",
                "timeout_per_edge",
                "frozen_arms_mode",
                "time_parameterize",
                "reset_roadmap",
            ]
        }

    return replay


def list_runs(log_dir: str) -> List[str]:
    """List all run log files (``*.jsonl``) in *log_dir*.

    Args:
        log_dir: Directory to search.

    Returns:
        Sorted list of absolute paths to ``.jsonl`` files, most recent last.
    """
    if not os.path.isdir(log_dir):
        return []
    files = [
        os.path.join(log_dir, f)
        for f in os.listdir(log_dir)
        if f.startswith("run_") and f.endswith(".jsonl")
    ]
    return sorted(files)


def print_run_summary(path: str) -> None:
    """Print a human-readable summary of a run log to stdout.

    Args:
        path: Path to a ``.jsonl`` or ``.json`` log file.
    """
    run = load_run_log(path)
    run_start = run.get("run_start", {})
    run_end = run.get("run_end", {})

    sep = "=" * 60
    print(sep)
    print(f"Run ID   : {run.get('run_id', 'unknown')}")
    print(f"Task     : {run_start.get('task_name', '?')}")
    print(f"Backend  : {run_start.get('backend', '?')}")
    print(f"Host     : {run_start.get('hostname', '?')}")
    print(f"Started  : {run_start.get('timestamp', '?')}")
    success = run_end.get("success")
    print(f"Success  : {success}")
    total_t = run_end.get("total_time")
    if isinstance(total_t, (int, float)):
        print(f"Total t  : {total_t:.2f}s")
    total_plan = run_end.get("total_planning_time")
    if isinstance(total_plan, (int, float)):
        print(f"Plan t   : {total_plan:.2f}s")
    phases = run.get("phase_results", [])
    print(f"Phases   : {len(phases)}")
    for p in phases:
        status = "✓" if p.get("success") else "✗"
        t = p.get("phase_time", 0)
        t_str = f"{t:.2f}s" if isinstance(t, (int, float)) else "?"
        handle = p.get("handle") or "(release)"
        print(
            f"  Phase {p.get('phase', '?'):>2} [{status}]  "
            f"{p.get('gripper', '?')} → {handle}  {t_str}"
        )
        err = p.get("error")
        if err:
            print(f"           error: {err}")
    if run_end.get("error"):
        print(f"Error    : {run_end['error']}")
    print(sep)
