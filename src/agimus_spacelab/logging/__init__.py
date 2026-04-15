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

from .run_logger import RunLogger
from .setup import configure_logging, get_logger
from .log_loader import (
    load_run_log,
    iter_events,
    get_replay_config,
    print_run_summary,
)

__all__ = [
    "RunLogger",
    "configure_logging",
    "get_logger",
    "load_run_log",
    "iter_events",
    "get_replay_config",
    "print_run_summary",
]
