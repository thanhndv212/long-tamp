"""A path sampled into arrays: safe to evaluate from another thread.

HPP paths may re-project onto their constraints when evaluated, through state
shared with the planner, so they must not be evaluated while the planner is
planning in another thread. ``SampledPath(path)`` evaluates a path once, in
the calling (planning) thread, and then interpolates between the samples with
numpy only. The executor's plan-ahead mode hands backends sampled paths.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .playback import _t0


class SampledPath:
    """``path`` (``length()``, ``eval(t) -> (q, ok)``) sampled every ``dt``
    seconds, evaluated by linear interpolation (``timeRange`` starts at 0).

    Linear interpolation of quaternion components between samples ``dt``
    apart is accurate to second order in the rotation per sample (1e-6 at
    typical speeds and the default 5 ms).
    """

    def __init__(self, path: Any, dt: float = 0.005) -> None:
        length = float(path.length())
        n = max(1, math.ceil(length / dt))
        self._times = np.linspace(0.0, length, n + 1)
        t0 = _t0(path)
        samples = []
        for t in self._times:
            q, ok = path.eval(t0 + float(t))
            if not ok:
                raise ValueError(f"path evaluation failed at t={t:.3f}")
            samples.append(np.asarray(q, dtype=float))
        self._q = np.array(samples)
        self._length = length

    def length(self) -> float:
        return self._length

    @property
    def configurations(self) -> np.ndarray:
        """The samples, one configuration per row (read-only)."""
        view = self._q.view()
        view.flags.writeable = False
        return view

    def eval(self, t: float) -> tuple[np.ndarray, bool]:
        t = min(max(float(t), 0.0), self._length)
        if self._length <= 0.0:
            return self._q[0].copy(), True
        x = t / self._length * (len(self._times) - 1)
        i = min(int(x), len(self._times) - 2)
        u = x - i
        return self._q[i] * (1.0 - u) + self._q[i + 1] * u, True


def sampled(payload: Any, dt: float = 0.005) -> Any:
    """``payload`` with its path(s) sampled: a path becomes a ``SampledPath``, a
    skill command gets a sampled approach; anything else is returned as is."""
    from long_tamp.tasks.task_planning.skills import SkillCommand

    if isinstance(payload, SkillCommand):
        return SkillCommand(
            payload.spec,
            dict(payload.parameters),
            sampled(payload.approach, dt),
            dict(payload.options),
        )
    if isinstance(payload, SampledPath):
        return payload
    if hasattr(payload, "eval") and hasattr(payload, "length"):
        return SampledPath(payload, dt)
    return payload
