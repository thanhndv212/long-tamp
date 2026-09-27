"""The screw-assembly mission as blocks and as a TaskPlan (pure Python, no HPP).

``build_mission(n)`` is the geometric definition: a chain of short blocks, each
a grasp sequence planned with ``run_block_with_recovery()`` (or a home move).
``build_plan_document(n)`` expresses the same mission as a TaskPlan: one
transaction per block, in the same order, each naming its block. The
capabilities in ``DESCRIPTORS`` declare what each block needs and achieves
(ADR-0002), over two predicates:

- ``holds(gripper, handle)``, *observed* from the grasp tracker, which covers
  the arms' grippers, the driver tip, the jig clamps and the dock;
- ``screwed(part, hole)``, *recorded*: nothing shows it once the driver lets go.

Home moves declare no effects (an arm pose isn't a predicate here), so they
always run; they are cheap and safe to repeat.
"""

from __future__ import annotations

from typing import Any

from long_tamp.tasks.task_planning import CapabilityDescriptor

LEFT, RIGHT = "ur10_left", "ur10_right"
DRIVER_TIP = "driver/tip"
RECORDED_PREDICATES = frozenset({"screwed"})

# Home retreat for the driver arm (the same idea as agimus_spacelab's
# home-retreat policy): parked where it stopped, ur10_right boxed in
# ur10_left twice per part -- live, the lookahead's clamp targets kept
# failing to path-plan (hint chain broken, 15 times) and the driver's path
# to hole 1 failed 45 resumes on "ur10_left/wrist_1 vs ur10_right's
# gripper". So it retreats, still holding the driver, after the pickup
# and after each part's screws.
HOME_MOVE = {"move": (RIGHT, "driver")}


def build_mission(n_parts: int) -> list[dict[str, Any]]:
    """The mission as a list of blocks, each a dict the runner consumes."""
    blocks: list[dict[str, Any]] = [
        {
            "label": "bootstrap: pick driver",
            "seq": [(f"{RIGHT}/gripper", "driver/h_grip")],
            "frozen": {0: [LEFT]},
        },
        {"label": "ur10_right home (bootstrap)", **HOME_MOVE},
    ]
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        blocks += [
            {
                "label": f"{p} A0: grasp",
                "seq": [(f"{LEFT}/gripper", f"{p}/h_grasp")],
                "frozen": {0: [RIGHT]},
            },
            {
                "label": f"{p} A: clamp + screw",
                "seq": [
                    (f"fixtures/clamp{i}", f"{p}/h_seat"),
                    (DRIVER_TIP, f"{p}/h_hole1"),
                    (DRIVER_TIP, None),
                    (DRIVER_TIP, f"{p}/h_hole2"),
                    (DRIVER_TIP, None),
                ],
                # Phase 0 moves ur10_left (it carries the part into the
                # clamp); the screw phases move ur10_right (driver).
                "frozen": {0: [RIGHT], 1: [LEFT], 2: [LEFT], 3: [LEFT], 4: [LEFT]},
                # Clamp candidate must leave hole 1 (phase 1) AND hole 2
                # (phase 3) reachable: the clamp pose fixes ur10_left
                # around the part for both.
                "lookahead": {"pair": (0, 1), "also": (3,)},
            },
            {"label": f"ur10_right home ({p})", **HOME_MOVE},
            {
                "label": f"{p} B: release",
                "seq": [(f"{LEFT}/gripper", None)],
                "frozen": {0: [RIGHT]},
            },
        ]
    blocks.append(
        {
            "label": "return: rack driver",
            "seq": [
                ("fixtures/rack_hold", "driver/h_rack"),
                (f"{RIGHT}/gripper", None),
            ],
            "frozen": {0: [LEFT], 1: [LEFT]},
        }
    )
    return blocks


# ------------------------------------------------------------------ contracts

_GRASP_PRE = ("not holds(?gripper, _)", "not holds(_, ?handle)")

DESCRIPTORS: dict[str, CapabilityDescriptor] = {
    # Pick the driver off its dock, or a part off the staging row.
    "grasp": CapabilityDescriptor(
        "grasp",
        "1.0",
        {"block": str, "gripper": str, "handle": str},
        preconditions=_GRASP_PRE,
        effects=("holds(?gripper, ?handle)",),
        writes=("grasp_state",),
        restartable=True,
    ),
    # The jig clamp takes the part (held by ``holder``), then the driver
    # (held by ``tool_gripper``) screws both holes.
    "clamp_and_screw": CapabilityDescriptor(
        "clamp_and_screw",
        "1.0",
        {
            "block": str,
            "part": str,
            "clamp": str,
            "seat": str,
            "hole1": str,
            "hole2": str,
            "holder": str,
            "held": str,
            "tool_gripper": str,
            "tool": str,
        },
        preconditions=(
            "holds(?holder, ?held)",
            "holds(?tool_gripper, ?tool)",
            "not holds(?clamp, _)",
        ),
        effects=(
            "holds(?clamp, ?seat)",
            "screwed(?part, ?hole1)",
            "screwed(?part, ?hole2)",
        ),
        writes=("grasp_state", "screws"),
        restartable=True,
    ),
    "release": CapabilityDescriptor(
        "release",
        "1.0",
        {"block": str, "gripper": str},
        preconditions=("holds(?gripper, _)",),
        effects=("not holds(?gripper, _)",),
        writes=("grasp_state",),
        restartable=True,
    ),
    # The dock takes the driver back, then the arm lets go.
    "rack": CapabilityDescriptor(
        "rack",
        "1.0",
        {"block": str, "dock": str, "dock_handle": str, "gripper": str},
        preconditions=("holds(?gripper, _)", "not holds(?dock, _)"),
        effects=("holds(?dock, ?dock_handle)", "not holds(?gripper, _)"),
        writes=("grasp_state",),
        restartable=True,
    ),
    "home": CapabilityDescriptor(
        "home",
        "1.0",
        {"block": str, "arm": str},
        writes=("arm_pose",),
        restartable=True,
    ),
}


def _step(index: int, block: dict[str, Any]) -> tuple[str, dict[str, str]]:
    """The capability and parameters that stand for ``block``."""
    label = block["label"]
    if "move" in block:
        return "home", {"block": label, "arm": block["move"][0]}
    seq = block["seq"]
    if label.startswith("return"):
        (dock, dock_handle), (gripper, _) = seq
        return "rack", {
            "block": label,
            "dock": dock,
            "dock_handle": dock_handle,
            "gripper": gripper,
        }
    if len(seq) == 1 and seq[0][1] is None:
        return "release", {"block": label, "gripper": seq[0][0]}
    if len(seq) == 1:
        return "grasp", {"block": label, "gripper": seq[0][0], "handle": seq[0][1]}
    (clamp, seat), (tip, hole1), _, (_, hole2), _ = seq
    part = seat.split("/")[0]
    return "clamp_and_screw", {
        "block": label,
        "part": part,
        "clamp": clamp,
        "seat": seat,
        "hole1": hole1,
        "hole2": hole2,
        "holder": f"{LEFT}/gripper",
        "held": f"{part}/h_grasp",
        "tool_gripper": f"{RIGHT}/gripper",
        "tool": "driver/h_grip",
    }


def build_plan_document(
    n_parts: int, initial_state: list[str] | None = None
) -> dict[str, Any]:
    """The mission as a TaskPlan: one transaction per ``build_mission`` block.

    ``initial_state`` (ground atoms) is what the plan is checked against at
    load time; the default is the nominal start, where nothing is held.
    """
    children = []
    for index, block in enumerate(build_mission(n_parts)):
        capability, parameters = _step(index, block)
        step_id = f"b{index:02d}-{capability}"
        children.append(
            {
                "type": "transaction",
                "id": step_id,
                "label": block["label"],
                "restart_state": ["q_current", "grasp_state"],
                "children": [
                    {
                        "type": "operation",
                        "id": f"{step_id}.execute",
                        "capability": capability,
                        "parameters": parameters,
                    }
                ],
            }
        )
    return {
        "schema_version": "1.0",
        "mission_id": f"ScrewAssembly{n_parts}",
        "scene": {"id": "screw-assembly", "parts": n_parts},
        "provenance": {
            "kind": "human",
            "generator": "screw_domain.build_plan_document",
        },
        "initial_state": list(initial_state or []),
        "root": {
            "type": "sequence",
            "id": "mission",
            "label": f"Screw assembly, {n_parts} part(s)",
            "children": children,
        },
    }


def blocks_by_label(n_parts: int) -> dict[str, dict[str, Any]]:
    return {block["label"]: block for block in build_mission(n_parts)}
