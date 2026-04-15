"""
Crash-safe structured run logger.

Writes one JSON object per line (JSONL) to a file, flushing immediately after
each event so no data is lost if the planner crashes or is interrupted.

A compact snapshot ``.json`` and a ``_replay.yaml`` (if PyYAML is available)
are written when the logger is closed.
"""

import json
import os
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, Optional


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------


def _make_serializable(obj: Any) -> Any:
    """Recursively convert *obj* to a JSON-serialisable value.

    Handles: None, bool, int, float, str, list/tuple, dict, numpy scalars,
    numpy arrays, dataclass instances, and arbitrary objects with ``__dict__``.
    Falls back to ``str(obj)`` for anything else.
    """
    # Primitives — pass through unchanged
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj

    # Numpy types (optional import — gracefully skipped if not present)
    try:
        import numpy as np  # type: ignore[import]
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
    except ImportError:
        pass

    # Sequences
    if isinstance(obj, (list, tuple)):
        return [_make_serializable(v) for v in obj]

    # Mappings
    if isinstance(obj, dict):
        return {str(k): _make_serializable(v) for k, v in obj.items()}

    # Dataclasses
    try:
        import dataclasses
        if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
            return _make_serializable(dataclasses.asdict(obj))
    except ImportError:
        pass

    # Objects with __dict__
    try:
        if hasattr(obj, "__dict__"):
            return _make_serializable(
                {k: v for k, v in vars(obj).items() if not k.startswith("_")}
            )
    except Exception:
        pass

    return str(obj)


def _serialize_task_config(config: Any) -> Dict[str, Any]:
    """Serialise a task config class or instance to a plain dict.

    Handles both the SpaceLab pattern (class-level attributes) and regular
    instance attributes.  Private attributes and callables are skipped.
    """
    if config is None:
        return {}

    result: Dict[str, Any] = {}

    # Class-level attributes (typical SpaceLab task config pattern)
    src = config if isinstance(config, type) else type(config)
    for k, v in vars(src).items():
        if k.startswith("_"):
            continue
        if callable(v) and not isinstance(v, (list, dict, tuple)):
            continue
        try:
            result[k] = _make_serializable(v)
        except Exception:
            result[k] = str(v)

    # Instance-level attributes (override / supplement class attrs)
    if not isinstance(config, type):
        for k, v in vars(config).items():
            if k.startswith("_"):
                continue
            if callable(v):
                continue
            try:
                result[k] = _make_serializable(v)
            except Exception:
                result[k] = str(v)

    return result


# ---------------------------------------------------------------------------
# RunLogger
# ---------------------------------------------------------------------------


class RunLogger:
    """Structured, crash-safe run logger for agimus_spacelab planning runs.

    Writes events as newline-delimited JSON (JSONL) to a file.  Each call to
    :meth:`log` flushes immediately so no data is lost on crash or interrupt.

    A compact snapshot ``*.json`` and an optional ``*_replay.yaml`` (requires
    PyYAML) are written when the logger is :meth:`close`-d.

    Usage::

        logger = RunLogger("/tmp/runs")
        logger.log("run_start", task_name="MyTask", backend="pyhpp")
        # ... planning ...
        logger.log("run_end", success=True, total_time=42.0)
        logger.close()

        # Or as a context manager:
        with RunLogger("/tmp/runs") as logger:
            logger.log("run_start", ...)

    The log files are named:
    ``run_<YYYYMMDD_HHMMSS>_<run_id>.{jsonl,json,_replay.yaml}``
    """

    def __init__(self, log_dir: str, run_id: Optional[str] = None) -> None:
        """Create a RunLogger.

        Args:
            log_dir: Directory where log files are written (created if needed).
            run_id: Optional 8-character hex ID.  A random one is generated if
                not provided.
        """
        self.log_dir = log_dir
        self.run_id: str = run_id or uuid.uuid4().hex[:8]
        self._start_time = datetime.now(timezone.utc)
        timestamp_str = self._start_time.strftime("%Y%m%d_%H%M%S")

        os.makedirs(log_dir, exist_ok=True)

        base = f"run_{timestamp_str}_{self.run_id}"
        self.jsonl_path = os.path.join(log_dir, base + ".jsonl")
        self.snapshot_path = os.path.join(log_dir, base + ".json")
        self.replay_path = os.path.join(log_dir, base + "_replay.yaml")

        self._file = open(self.jsonl_path, "a", encoding="utf-8")
        self._events: list = []
        self._closed = False

        # Ensure close() is called even if the caller never does so explicitly
        # (e.g. on an unhandled exception that exits the process).
        import atexit
        atexit.register(self.close)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def log(self, event: str, **kwargs: Any) -> None:
        """Append a structured event record to the JSONL log.

        This method NEVER raises — any serialisation or I/O error is silently
        swallowed so that logging can never crash the planner.

        Args:
            event: Event type string (e.g. ``"run_start"``, ``"edge_end"``).
            **kwargs: Arbitrary payload fields included in the record.
        """
        if self._closed:
            return
        try:
            record: Dict[str, Any] = {
                "event": event,
                "run_id": self.run_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
            record.update(
                {k: _make_serializable(v) for k, v in kwargs.items()}
            )
            line = json.dumps(record, ensure_ascii=False)
            self._file.write(line + "\n")
            self._file.flush()
            self._events.append(record)
        except Exception:
            pass  # Never crash the planner

    def log_task_config(
        self,
        task_config: Any,
        setup_params: Optional[Dict[str, Any]] = None,
        backend: str = "",
        task_name: str = "",
    ) -> None:
        """Helper: log a ``config_snapshot`` event from a task-config object.

        Args:
            task_config: ``BaseTaskConfig`` subclass or instance.
            setup_params: Dict of ``setup()`` call parameters.
            backend: Backend name string.
            task_name: Task name string.
        """
        try:
            config_data = _serialize_task_config(task_config)
        except Exception:
            config_data = {}
        self.log(
            "config_snapshot",
            task_config=config_data,
            setup_params=setup_params or {},
            backend=backend,
            task_name=task_name,
        )

    def snapshot(self) -> Dict[str, Any]:
        """Return a dict summarising all events logged so far.

        Top-level keys:
        - ``run_id``: Run identifier string.
        - ``events``: Complete ordered list of event records.
        - One key per unique event type (first occurrence), e.g.
          ``run_start``, ``config_snapshot``, ``run_end``.
        - ``phase_results``: List of ``phase_end`` records in order.
        """
        result: Dict[str, Any] = {
            "run_id": self.run_id,
            "events": list(self._events),
            "phase_results": [],
        }
        for rec in self._events:
            et = rec.get("event")
            if et and et not in result:
                result[et] = rec
            if et == "phase_end":
                result["phase_results"].append(rec)
        return result

    def save_snapshot(self) -> None:
        """Write the full snapshot JSON (``*.json``) alongside the JSONL file."""
        try:
            snap = self.snapshot()
            with open(self.snapshot_path, "w", encoding="utf-8") as f:
                json.dump(snap, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def save_replay_config(self) -> None:
        """Write a ``*_replay.yaml`` file with parameters needed to rerun.

        Requires PyYAML.  Silently skipped if PyYAML is not installed.
        """
        try:
            import yaml  # type: ignore[import]
        except ImportError:
            return

        replay: Dict[str, Any] = {"run_id": self.run_id}
        for rec in self._events:
            et = rec.get("event")
            if et == "run_start":
                replay["task_name"] = rec.get("task_name")
                replay["backend"] = rec.get("backend")
            elif et == "config_snapshot":
                replay["task_config"] = rec.get("task_config", {})
                replay["setup_params"] = rec.get("setup_params", {})
            elif et == "sequence_start":
                replay["sequence"] = {
                    k: rec.get(k)
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
        try:
            with open(self.replay_path, "w", encoding="utf-8") as f:
                yaml.dump(
                    replay, f, default_flow_style=False, allow_unicode=True
                )
        except Exception:
            pass

    def close(self) -> None:
        """Flush, write snapshot + replay config, and close the log file."""
        if self._closed:
            return
        self._closed = True
        try:
            self._file.close()
        except Exception:
            pass
        self.save_snapshot()
        self.save_replay_config()

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "RunLogger":
        return self

    def __exit__(
        self,
        exc_type: Any,
        exc_val: Any,
        exc_tb: Any,
    ) -> bool:
        self.close()
        return False  # Do not suppress exceptions

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def path(self) -> str:
        """Absolute path to the JSONL log file."""
        return self.jsonl_path
