"""Run a parallel group's lanes at the same time, as merged motion (#21).

A ``parallel`` node's lanes are independent (``task_planning.partial_order``)
and are planned one after another, so each lane's paths hold the other lanes'
arms still. Each lane moves its own configuration entries (its arm, the
objects it carries) and never another lane's. The lanes' motions can
therefore be combined entry by entry: ``merge_lanes`` pairs the lanes' k-th
commands into one command whose path takes each lane's entries from that
lane's path, and its time span from the longest. A lane with fewer commands
holds its arm where its last command ended.

The merge is refused (``None``: run the lanes one after another instead)
when:

- a lane has a skill command: skills run their own controllers;
- two lanes move the same entry: they were not independent after all;
- ``validate(q)`` rejects a configuration sampled along the merged motion:
  each lane's path was checked for collisions with the other arms still,
  not moving; ``validate`` (the planner's configuration check) closes that
  gap.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from itertools import zip_longest

import numpy as np

from .contract import ExecutionCommand
from .sampled import SampledPath, sampled

#: Configuration change below which an entry counts as not moving.
MOVING_TOLERANCE = 1e-6
#: Merged motion is validated every this many seconds of path time.
VALIDATE_STEP = 0.02


def moving_entries(path: SampledPath) -> np.ndarray:
    """Indices of the configuration entries ``path`` changes."""
    q = path.configurations
    return np.flatnonzero(np.any(np.abs(q - q[0]) > MOVING_TOLERANCE, axis=0))


class MergedPath:
    """Lanes' paths played together: entry ``i`` comes from the lane that
    moves it, the rest from the first lane (unmoved, so the same in all)."""

    def __init__(
        self,
        paths: Sequence[SampledPath],
        holds: Sequence[np.ndarray],
        owners: Sequence[np.ndarray],
    ) -> None:
        #: Per lane: its path this round, or ``None`` (hold ``holds[k]``).
        self.paths = list(paths)
        self.holds = [np.asarray(h, dtype=float) for h in holds]
        self.owners = list(owners)
        self._length = max(
            (p.length() for p in self.paths if p is not None), default=0.0
        )

    def length(self) -> float:
        return self._length

    def lane_config(self, k: int, t: float) -> np.ndarray:
        path = self.paths[k]
        if path is None:
            return self.holds[k]
        q, _ = path.eval(min(t, path.length()))
        return np.asarray(q, dtype=float)

    def eval(self, t: float) -> tuple[np.ndarray, bool]:
        t = min(max(float(t), 0.0), self._length)
        q = self.lane_config(0, t).copy()
        for k in range(1, len(self.paths)):
            q[self.owners[k]] = self.lane_config(k, t)[self.owners[k]]
        return q, True


def merge_lanes(
    lanes: Sequence[Sequence[ExecutionCommand]],
    validate: Callable[[np.ndarray], bool] | None = None,
    step_id: str = "parallel",
) -> list[ExecutionCommand] | None:
    """The lanes' commands merged into concurrent ones, or ``None`` when
    they must run one after another (see the module docstring)."""
    from long_tamp.tasks.task_planning.skills import SkillCommand

    lanes = [list(lane) for lane in lanes if lane]
    if len(lanes) < 2:
        return None
    if any(isinstance(c.payload, SkillCommand) for lane in lanes for c in lane):
        return None
    paths = [[sampled(c.payload) for c in lane] for lane in lanes]
    if not all(isinstance(p, SampledPath) for lane in paths for p in lane):
        return None
    owners = []
    for lane in paths:
        moved = set()
        for path in lane:
            moved.update(moving_entries(path).tolist())
        owners.append(np.array(sorted(moved), dtype=int))
    for k, mine in enumerate(owners):
        for other in owners[k + 1 :]:
            if np.intersect1d(mine, other).size:
                return None

    # Where each lane's arm waits once its commands are done (every lane
    # has a command in the first round).
    ends = [np.asarray(lane[-1].eval(lane[-1].length())[0]) for lane in paths]
    merged: list[ExecutionCommand] = []
    for k_round, round_ in enumerate(zip_longest(*paths)):
        merged_path = MergedPath(round_, ends, owners)
        if validate is not None and not _valid(merged_path, validate):
            return None
        durations = [
            c.duration
            for lane in lanes
            for c in lane[k_round : k_round + 1]
            if c.duration is not None
        ]
        merged.append(
            ExecutionCommand(
                f"{step_id}#{k_round + 1}",
                max(durations) if durations else merged_path.length(),
                merged_path,
            )
        )
    return merged


def _valid(path: MergedPath, validate: Callable[[np.ndarray], bool]) -> bool:
    n = max(1, int(np.ceil(path.length() / VALIDATE_STEP)))
    return all(validate(path.eval(path.length() * i / n)[0]) for i in range(n + 1))
