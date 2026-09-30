"""Skill controllers for the MuJoCo backend.

``ScrewDriving`` is a screwing stub. Grasps are welds and the screw isn't
modelled as geometry, so the screw is *virtual*. The arm and the controller
are real:

1. **Approach.** The tool follows the planned approach path (from the skill's
   start pose to its end pose) at ``feed`` m/s under the backend's tracking
   control, with its stiffness lowered to ``bandwidth`` Hz (compliant).
   Once engaged, the arm feeds forward the thrust and holding torque it
   expects (``J^T w``): hybrid force/position control, as screwdriving
   controllers do.
2. **Touch.** ``thread_length`` before the end pose, the screw meets the hole.
   If the tool is further than ``align_tolerance`` off the hole's axis, the
   skill fails with ``screw_misaligned``.
3. **Drive.** The screw goes in as the tool advances. The screw pushes back
   along the axis (``contact_force``) and twists the driver's housing
   (reaction torque growing with depth up to ``torque_threshold``). The arm
   must hold both.
4. **Seated.** At ``torque_threshold`` the screw is driven, and the skill
   reports its postconditions (``screwed(part, hole)``). If the approach ends
   and the tool never reaches the touch point, it fails with
   ``screw_no_contact``.

Metrics: ``screw_depth`` [m], ``screw_torque`` [N m], ``lateral_error`` [m],
``contact_force`` [N], ``tracking_error``, ``sim_seconds``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from long_tamp.execution.contract import ExecutionStatus, Feedback
from long_tamp.tasks.task_planning.skills import SkillCommand, SkillPose, SkillSpec

#: The screw-assembly screwing skill: planned motion brings the driver's tip
#: onto the hole's axis (HPP's pregrasp); the skill drives the screw.
SCREW = SkillSpec(
    name="screw",
    parameters=("tool", "part", "hole"),
    postconditions=("screwed(?part, ?hole)",),
    failures=("screw_misaligned(?tool, ?hole)", "screw_no_contact(?tool, ?hole)"),
    start=SkillPose("?tool", "?hole", offset=-0.02),
    end=SkillPose("?tool", "?hole"),
    description="drive a screw: compliant approach, then torque threshold",
)


@dataclass
class ScrewDriving:
    """Factory for ``MuJoCoBackend(skills={"screw": ScrewDriving()})``."""

    feed: float = 0.02
    thread_length: float = 0.008
    align_tolerance: float = 0.002
    contact_force: float = 15.0
    torque_threshold: float = 2.0
    bandwidth: float = 4.0
    settle: float = 0.2
    #: Where the real hole is relative to where the planner put it (world
    #: frame, m): a perception error. The tool follows the plan.
    hole_error: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __call__(self, backend: Any, command: SkillCommand) -> _ScrewRun:
        return _ScrewRun(self, backend, command)


class _ScrewRun:
    def __init__(self, params: ScrewDriving, backend: Any, command: SkillCommand):
        self.p, self.backend, self.command = params, backend, command
        self.saved_omega = backend._omega
        self.depth = self.torque = self.lateral = self.force = self.short = 0.0
        carried = list(backend.carried_by_robot().values())
        self.tool = None
        self.failure = ""
        if not carried:
            self.failure = "screw: no tool is carried along the approach"
            return
        # The planned positions of what the robots carry along the approach;
        # the tool is the one the approach moves (the other hand may hold
        # the part still).
        length = backend._length
        n = int(min(400, max(20, length / 0.005)))
        self.grid = np.linspace(0.0, length, n + 1)
        points = []
        for s in self.grid:
            qpos = backend.planned_qpos(float(s))
            if qpos is None:
                self.failure = "screw: approach path evaluation failed"
                return
            points.append([backend.body_position(qpos, body) for body in carried])
        points = np.array(points)  # (samples, carried, 3)
        moved = np.linalg.norm(points[-1] - points[0], axis=1)
        k = int(np.argmax(moved))
        self.tool = carried[k]
        self.points = points[:, k]
        steps = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.arc = np.concatenate([[0.0], np.cumsum(steps)])
        total = float(self.arc[-1])
        if total <= 1e-6:
            self.failure = "screw: the approach doesn't move the tool"
            return
        self.axis = (self.points[-1] - self.points[0]) / np.linalg.norm(
            self.points[-1] - self.points[0]
        )
        self.hole = self._actual_hole(self.points[-1]) + np.asarray(
            params.hole_error, dtype=float
        )
        self.touch_arc = max(0.0, total - params.thread_length)
        self.total = total
        self.done_at: float | None = None
        self.engaged = False
        backend._omega = 2.0 * math.pi * params.bandwidth

    # -- helpers ---------------------------------------------------------

    def _actual_hole(self, planned: np.ndarray) -> np.ndarray:
        """The planned hole where its part actually is: a part held by
        friction (contact grasps) sits a few mm from where the planner put
        it, and its hole with it."""
        backend = self.backend
        params = self.command.parameters
        part = params.get("part") or params.get("hole", "").split("/")[0]
        body = next(
            (b for n, b in backend._objects.items() if n.split("/")[0] == part), None
        )
        if body is None or body == self.tool:
            return planned
        qpos = backend.planned_qpos(backend._length)
        if qpos is None:
            return planned
        pos, rot = backend.body_pose(qpos, body)
        local = rot.T @ (planned - pos)
        actual = backend.data
        return actual.xpos[body] + actual.xmat[body].reshape(3, 3) @ local

    def _s_at(self, arc: float) -> float:
        return float(np.interp(min(arc, self.total), self.arc, self.grid))

    def _finish(self, status: ExecutionStatus, message: str, facts=()) -> tuple:
        backend = self.backend
        backend._omega = self.saved_omega
        backend.feedforward[:] = 0.0
        if self.tool is not None:
            backend.data.xfrc_applied[self.tool] = 0.0
        metrics = backend._metrics()
        metrics.update(
            {
                "screw_depth": round(self.depth, 5),
                "screw_torque": round(self.torque, 4),
                "lateral_error": round(self.lateral, 5),
                "contact_force": round(self.force, 3),
            }
        )
        return status, Feedback(
            progress=1.0, message=message, metrics=metrics, facts=tuple(facts)
        )

    def _fail(self, predicate: str, why: str) -> tuple:
        fact = self.command.failure(predicate)
        return self._finish(ExecutionStatus.FAILURE, f"{fact}: {why}", (fact,))

    # -- the controller --------------------------------------------------

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        backend, p = self.backend, self.p
        if self.failure:
            return self._finish(ExecutionStatus.FAILURE, self.failure)
        dt = backend.model.opt.timestep
        horizon = self.total / p.feed + p.settle + 1.0
        target = backend.sim_target(horizon)
        while backend._t < target:
            arc = min(self.total, backend._t * p.feed)
            ref = backend.planned_qpos(self._s_at(arc))
            ahead = backend.planned_qpos(self._s_at(arc + p.feed * dt))
            if ref is None or ahead is None:
                return self._finish(
                    ExecutionStatus.FAILURE, "screw: approach path evaluation failed"
                )
            qvel_ref = backend.velocity(ref, ahead)
            if arc >= self.total:
                qvel_ref[:] = 0.0

            # The virtual screw acts on the tool (the driver's housing).
            offset = backend.data.xpos[self.tool] - self.hole
            along = float(offset @ self.axis)  # < 0 before seated
            self.lateral = float(np.linalg.norm(offset - along * self.axis))
            self.short = -along - p.thread_length  # > 0 before the touch point
            self.depth = max(0.0, -self.short)
            wrench = np.zeros(6)
            if self.depth > 0.0:
                if not self.engaged:
                    self.engaged = True
                    if self.lateral > p.align_tolerance:
                        return self._fail(
                            "screw_misaligned",
                            f"{self.lateral * 1000:.1f} mm off the hole's axis "
                            f"(tolerance {p.align_tolerance * 1000:.1f} mm)",
                        )
                self.force = p.contact_force
                self.torque = p.torque_threshold * min(
                    1.0, self.depth / p.thread_length
                )
                wrench[:3] = -self.axis * self.force
                wrench[3:] = -self.axis * self.torque
            backend.data.xfrc_applied[self.tool] = wrench
            # Force control along the axis: the arm pushes and holds against
            # what the screw exerts, instead of letting it deflect the
            # (compliant) position loop.
            backend.feedforward[:] = backend.wrench_torques(self.tool, -wrench)

            error = backend.control_step(ref, qvel_ref, ahead)
            if error:
                return self._finish(ExecutionStatus.FAILURE, f"screw: {error}")

            if self.torque >= p.torque_threshold * 0.999 and self.done_at is None:
                self.done_at = backend._t
            if self.done_at is not None and backend._t - self.done_at >= p.settle:
                return self._finish(
                    ExecutionStatus.SUCCESS,
                    "screw driven",
                    self.command.postconditions(),
                )
        if backend._t >= horizon:
            if not self.engaged:
                return self._fail(
                    "screw_no_contact",
                    f"the tool stopped {self.short * 1000:.1f} mm short of the hole",
                )
            return self._fail(
                "screw_no_contact",
                f"the screw went {self.depth * 1000:.1f} of "
                f"{p.thread_length * 1000:.1f} mm in",
            )
        progress = min(1.0, backend._t / horizon)
        return ExecutionStatus.RUNNING, Feedback(
            progress=progress,
            metrics={"screw_depth": round(self.depth, 5), "sim_seconds": backend._t},
        )
