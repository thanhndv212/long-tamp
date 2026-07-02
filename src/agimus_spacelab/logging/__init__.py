"""
Run logging for agimus_spacelab planning sessions.

Provides structured, crash-safe logging of all task configurations and
planning results for debugging and reproduction.

Usage::

    from agimus_spacelab.logging import RunLogger, configure_logging

    # Attach a logger to a ManipulationTask via log_dir=
    task = MyTask(backend="pyhpp", log_dir="/tmp/runs")

    # Or use RunLogger standalone
    with RunLogger("/tmp/runs") as logger:
        logger.log("run_start", task_name="demo", backend="pyhpp")
        logger.log("run_end", success=True, total_time=12.3)

    # Load and inspect a previous run
    from agimus_spacelab.logging import load_run_log
    run = load_run_log("/tmp/runs/run_20260415_103045_abc12345.jsonl")
"""

from .log_loader import (
    get_replay_config,
    iter_events,
    load_run_log,
    print_run_summary,
)
from .run_logger import RunLogger
from .setup import configure_logging, get_logger

__all__ = [
    "RunLogger",
    "configure_logging",
    "get_logger",
    "get_replay_config",
    "iter_events",
    "load_run_log",
    "print_run_summary",
]
