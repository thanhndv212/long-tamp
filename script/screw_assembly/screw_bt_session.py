"""The screw-assembly cell as a BehaviorTree.CPP host session (#58).

A short, seeded TaskPlan on the real scene, driven through the session the
C++ host (``examples/behaviortree``, ``agimus_taskplan_bt``) calls: the right
arm picks the driver, goes home while the left arm grasps part 1 (a
``parallel`` node: two lanes, lowered to BT.CPP's ``Parallel``), then racks
the driver. It plans only; nothing executes.

``build_screw_session(options_json)`` is what the host's allowlisted
``create_screw_session`` factory calls. Options: ``seed`` (default 1) and
``plan``: ``"short"`` (default, the plan above) or ``"full"``, the whole
mission (every part clamped and screwed, the driver racked, the parts
released), as the Python mission plans it.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

#: The mission's blocks this plan runs, by label.
BLOCKS = (
    "bootstrap: pick driver",
    "ur10_right home (bootstrap)",
    "part1 A0: grasp",
    "return: rack driver",
)


def plan_document(n_parts: int) -> dict[str, Any]:
    """The short plan: those blocks, in order, as one sequence (``build_mission``
    block indices keep the step ids of the full mission)."""
    from screw_domain import _transaction, build_mission

    blocks = build_mission(n_parts)
    steps = [
        _transaction(i, block)
        for i, block in enumerate(blocks)
        if block["label"] in BLOCKS
    ]
    if len(steps) != len(BLOCKS):
        raise ValueError(
            f"expected blocks {BLOCKS}, found {[s['label'] for s in steps]}"
        )
    return {
        "schema_version": "1.0",
        "mission_id": "ScrewCellBT",
        "scene": {"id": "screw-assembly", "parts": n_parts},
        "provenance": {"kind": "human", "generator": "screw_bt_session"},
        "label": "Screw cell: driver out and back, part 1 grasped",
        "root": {"type": "sequence", "id": "root", "children": steps},
    }


def build_screw_session(options_json: str = "{}"):
    """A ``HostSession`` for the short plan, independent steps in parallel."""
    import task_screw_assembly as T

    from long_tamp.tasks.task_planning.host import HostSession
    from long_tamp.tasks.task_planning.world_state import RecordedFacts

    options = json.loads(options_json or "{}")
    T.seed_everything(int(options.get("seed", 1)))
    task, planner = T.setup()
    n_parts = sum(1 for o in task.task_config.OBJECTS if o.startswith("part"))
    recorded = RecordedFacts(None, predicates=T.RECORDED_PREDICATES)
    ctx: dict[str, Any] = {
        "q": list(task.q_init),
        "records": [],
        "trajectory": None,
        "checkpoint": None,
        "live_viewer": None,
        "n_parts": n_parts,
        "backend_displays": False,
        "inject": [],
        "concurrent": True,
    }
    if options.get("plan", "short") == "full":
        document = T.build_plan_document(n_parts)
    else:
        document = plan_document(n_parts)
    session = T.mission_session(
        task, planner, ctx, recorded, verbose=False, document=document
    )
    host = HostSession(session.plan, session.registry, session.world_state, recorded)
    host.ctx = ctx  # what the steps planned (records), for checks
    return host
