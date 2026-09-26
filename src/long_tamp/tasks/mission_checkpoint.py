"""Per-run folder that is both a mission's resume point and its run log.

A long mission (a chain of blocks, see ``block_recovery``) can run for tens
of minutes. ``MissionCheckpoint`` keeps one folder per run:

- ``mission.json`` -- the run log: metadata (seed, commit, start time, ...)
  and one record per completed block (label, success, seconds, resumes,
  replans, message), plus the final outcome once the mission ends;
- ``checkpoint.json`` -- the resume point: the next block's index, the
  configuration and the held grasps after the last completed block.

Both are rewritten atomically after every block, so a run that is killed
or crashes leaves an accurate log of what it finished. Point the task's
``log_dir`` at the same folder to keep ``run.log`` alongside, and set
``phase_dump_dir(...)`` to keep ``GraspSequencePlanner``'s per-phase dumps
there too.

Resuming: ``load()`` returns the checkpoint; the caller restores the
configuration, re-seeds the planner's grasp tracker with ``held`` (see
``restore_grasps``), and skips blocks before ``next_block``. The block
*label* is stored too, and ``expect_label()`` refuses to resume into a
mission whose definition changed -- a positional index alone would silently
skip or repeat work.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

MISSION_FILE = "mission.json"
CHECKPOINT_FILE = "checkpoint.json"
# Read by GraspSequencePlanner._dump_phase_checkpoint.
PHASE_DUMP_ENV = "LONG_TAMP_CHECKPOINT_DIR"


def _write_json(path: Path, data: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2))
    os.replace(tmp, path)


class MissionCheckpoint:
    """One run's folder: resume point plus run log."""

    def __init__(self, directory: str | Path, meta: dict[str, Any] | None = None):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        existing = self.dir / MISSION_FILE
        if existing.exists():
            self._mission = json.loads(existing.read_text())
            self._mission.setdefault("resumes", []).append(
                {"at": time.strftime("%Y-%m-%dT%H:%M:%S"), **(meta or {})}
            )
        else:
            self._mission = {
                "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "meta": dict(meta or {}),
                "blocks": [],
            }
        self._save_mission()

    # -- run log -----------------------------------------------------------

    @property
    def blocks(self) -> list[dict[str, Any]]:
        return self._mission["blocks"]

    def record(
        self,
        index: int,
        label: str,
        result: dict[str, Any],
        seconds: float,
        q: list[float],
        held: dict[str, str | None],
    ) -> None:
        """Log a finished block; on success also advance the resume point."""
        self.blocks.append(
            {
                "index": index,
                "label": label,
                "success": bool(result.get("success")),
                "seconds": round(seconds, 2),
                "resumes": result.get("resumes", 0),
                "replans": result.get("replans", 0),
                "message": result.get("message", ""),
            }
        )
        self._save_mission()
        if result.get("success"):
            _write_json(
                self.dir / CHECKPOINT_FILE,
                {
                    "next_block": index + 1,
                    "last_label": label,
                    "q": [float(v) for v in q],
                    "held": {g: h for g, h in held.items() if h is not None},
                },
            )

    def finish(self, success: bool, seconds: float) -> None:
        self._mission["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._mission["success"] = bool(success)
        self._mission["seconds"] = round(seconds, 2)
        self._save_mission()

    def _save_mission(self) -> None:
        _write_json(self.dir / MISSION_FILE, self._mission)

    # -- resume ------------------------------------------------------------

    def load(self) -> dict[str, Any] | None:
        """The latest resume point, or None if no block has completed."""
        path = self.dir / CHECKPOINT_FILE
        return json.loads(path.read_text()) if path.exists() else None

    @staticmethod
    def expect_label(checkpoint: dict[str, Any], labels: list[str]) -> None:
        """Raise if the mission's block list no longer matches the checkpoint."""
        idx = checkpoint["next_block"] - 1
        if idx >= len(labels) or labels[idx] != checkpoint["last_label"]:
            raise ValueError(
                f"checkpoint's last block {checkpoint['last_label']!r} is not block "
                f"{idx} of this mission -- its definition changed; refusing to resume"
            )

    @staticmethod
    def restore_grasps(tracker: Any, held: dict[str, str]) -> None:
        """Re-seed a GraspStateTracker with the checkpoint's held grasps."""
        for gripper, handle in list(tracker.current_grasps.items()):
            if handle is not None:
                tracker.update_grasp(gripper, None)
        for gripper, handle in held.items():
            tracker.update_grasp(gripper, handle)

    # -- per-phase dumps ---------------------------------------------------

    def phase_dump_dir(self, index: int, label: str) -> Path:
        """Point GraspSequencePlanner's per-phase dumps at this block's
        subfolder (each plan_sequence call numbers its phases from 0)."""
        safe = "".join(c if c.isalnum() else "_" for c in label)
        path = self.dir / "phases" / f"{index:03d}_{safe}"
        os.environ[PHASE_DUMP_ENV] = str(path)
        return path


__all__ = ["PHASE_DUMP_ENV", "MissionCheckpoint"]
