"""Grasp-planner capabilities for task plans and the behaviour-tree host.

Grasp planning is its own step, separate from motion planning: a task plan
asks the grasp planner which handle to use and how far the fingers close,
asks the motion planner (HPP) to bring the gripper frame onto the handle,
then commands the gripper. This module exposes the first and last of those
as capabilities on a :class:`CapabilityRegistry`:

``plan_grasp`` (operation; ``gripper``, ``object``)
    Ranked grasps of ``object`` for ``gripper``'s hand; metrics carry the
    best candidates (handle pose, width, joint value). Fails (so the BT
    retries or falls back) when none is feasible.
``grasp_feasible`` (condition; ``gripper``, ``handle``)
    Whether the hand can close on an existing handle.
``close_gripper`` (operation; ``gripper``, ``handle``)
    Close on ``handle``: computes the finger joint values and hands them to
    the ``actuate`` callback (a viewer overlay, a gripper driver, a
    trajectory writer). Metrics carry the closure.
``open_gripper`` (operation; ``gripper``)
    Open the fingers through the same callback.

Every capability is planning-only unless the ``actuate`` callback drives
hardware; restartable, since planning and commanding a width are both
idempotent.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping

from long_tamp.grasping import FingerClosureTable

from .capabilities import CapabilityDescriptor, CapabilityRegistry

Actuate = Callable[[str, Mapping[str, float]], None]

PLAN_GRASP = CapabilityDescriptor(
    "plan_grasp",
    "1.0",
    {"gripper": str, "object": str},
    writes=("grasp_candidates",),
    restartable=True,
)
GRASP_FEASIBLE = CapabilityDescriptor(
    "grasp_feasible",
    "1.0",
    {"gripper": str, "handle": str},
)
CLOSE_GRIPPER = CapabilityDescriptor(
    "close_gripper",
    "1.0",
    {"gripper": str, "handle": str},
    resources=("gripper",),
    writes=("finger_state",),
    restartable=True,
)
OPEN_GRIPPER = CapabilityDescriptor(
    "open_gripper",
    "1.0",
    {"gripper": str},
    resources=("gripper",),
    writes=("finger_state",),
    restartable=True,
)


def register_grasp_capabilities(
    registry: CapabilityRegistry,
    closures: FingerClosureTable,
    actuate: Actuate | None = None,
    max_candidates: int = 5,
) -> None:
    """Register the four grasp capabilities, backed by ``closures``.

    ``actuate(gripper, joint_values)`` is called by ``close_gripper`` and
    ``open_gripper``; omit it to only compute the commands.
    """

    def plan_grasp(parameters: dict) -> dict:
        gripper, obj = parameters["gripper"], parameters["object"]
        candidates = closures.plan(gripper, obj, max_candidates=max_candidates)
        if not candidates:
            raise RuntimeError(f"no feasible grasp of {obj} for {gripper}")
        return {
            "gripper": gripper,
            "object": obj,
            "count": len(candidates),
            "candidates": [c.to_dict() for c in candidates],
        }

    def grasp_feasible(parameters: dict) -> bool:
        ev = closures.evaluation(parameters["gripper"], parameters["handle"])
        return bool(ev.feasible)

    def close_gripper(parameters: dict) -> dict:
        gripper, handle = parameters["gripper"], parameters["handle"]
        ev = closures.evaluation(gripper, handle)
        if not ev.feasible:
            raise RuntimeError(
                f"cannot close {gripper} on {handle}: {'; '.join(ev.reasons)}"
            )
        values = closures.closed_values(gripper, handle)
        if actuate is not None:
            actuate(gripper, values)
        return {
            "gripper": gripper,
            "handle": handle,
            "joint_values": values,
            **ev.to_dict(),
        }

    def open_gripper(parameters: dict) -> dict:
        gripper = parameters["gripper"]
        values = closures.open_values(gripper)
        if actuate is not None:
            actuate(gripper, values)
        return {"gripper": gripper, "joint_values": values}

    registry.register(PLAN_GRASP, plan_grasp)
    registry.register(GRASP_FEASIBLE, grasp_feasible)
    registry.register(CLOSE_GRIPPER, close_gripper)
    registry.register(OPEN_GRIPPER, open_gripper)
