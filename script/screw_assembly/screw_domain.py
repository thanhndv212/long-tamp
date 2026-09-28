"""The screw-assembly mission as blocks and as a TaskPlan (pure Python, no HPP).

``build_mission(n)`` is the geometric definition: a chain of short blocks, each
a grasp sequence refined with ``GraspSequenceRefiner`` (or a home move);
``refinement_step(block)`` turns a block into the refiner's step.
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

from long_tamp.tasks.refiner import Lookahead, RefinementStep
from long_tamp.tasks.task_planning import CapabilityDescriptor
from long_tamp.tasks.task_planning.pddl import PddlExport, to_pddl
from long_tamp.tasks.task_planning.skeleton import skeleton_document

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


def refinement_step(block: dict[str, Any]) -> RefinementStep:
    """``block`` (a grasp-sequence block, not a home move) as a refiner step."""
    lookahead = None
    if "lookahead" in block:
        lookahead = Lookahead(
            pair=tuple(block["lookahead"]["pair"]),
            also=tuple(block["lookahead"]["also"]),
            # Path-check the clamp move too: a clamp target the arm can't
            # reach by path gets redrawn in the real plan, which voids the
            # hints and costs a block replan.
            verify_paths=True,
        )
    return RefinementStep(
        label=block["label"],
        sequence=tuple(block["seq"]),
        frozen=block["frozen"],
        lookahead=lookahead,
    )


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


def descriptors(n_parts: int) -> dict[str, CapabilityDescriptor]:
    """``DESCRIPTORS`` plus the mission's two guard conditions.

    ``part_done`` holds once a part is clamped, both holes are screwed and
    the arm that carried it has let go; ``all_parts_done`` once every part
    is. The release belongs in it: a run killed after the screws but before
    the release must still release the part on resume (found by
    ``kill_resume.py``, #12). They guard the part blocks and the
    driver pickup, whose own effects are undone later in the mission (the
    part is released, the driver racked), so a resumed or scenario run skips
    finished work instead of redoing it (without them, a run resumed after a
    part's release would grasp and release that clamped part again).
    """
    done = []
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        done += [
            f"holds(fixtures/clamp{i}, {p}/h_seat)",
            f"screwed({p}, {p}/h_hole1)",
            f"screwed({p}, {p}/h_hole2)",
            f"not holds({LEFT}/gripper, {p}/h_grasp)",
        ]
    return {
        **DESCRIPTORS,
        "part_done": CapabilityDescriptor(
            "part_done",
            "1.0",
            {
                "part": str,
                "clamp": str,
                "seat": str,
                "hole1": str,
                "hole2": str,
                "holder": str,
                "held": str,
            },
            preconditions=(
                "holds(?clamp, ?seat)",
                "screwed(?part, ?hole1)",
                "screwed(?part, ?hole2)",
                "not holds(?holder, ?held)",
            ),
        ),
        "all_parts_done": CapabilityDescriptor(
            "all_parts_done", f"1.{n_parts}", {}, preconditions=tuple(done)
        ),
    }


def _transaction(index: int, block: dict[str, Any]) -> dict[str, Any]:
    capability, parameters = _step(index, block)
    step_id = f"b{index:02d}-{capability}"
    return {
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


def build_plan_document(
    n_parts: int, initial_state: list[str] | None = None
) -> dict[str, Any]:
    """The mission as a TaskPlan: one transaction per ``build_mission`` block.

    Structure (block order unchanged)::

        sequence
          fallback  all parts done?  else  sequence[pick driver, home]
          fallback  part i done?     else  sequence[A0, A, home, B]   (per part)
          rack driver

    ``initial_state`` (ground atoms) is what the plan is checked against at
    load time; the default is the nominal start, where nothing is held.
    """
    blocks = build_mission(n_parts)
    steps = [_transaction(i, b) for i, b in enumerate(blocks)]
    children: list[dict[str, Any]] = [
        {
            "type": "fallback",
            "id": "tool-ready",
            "label": "Driver picked (unless every part is done)",
            "children": [
                {
                    "type": "condition",
                    "id": "all-parts-done",
                    "label": "All parts done",
                    "capability": "all_parts_done",
                    "parameters": {},
                },
                {
                    "type": "sequence",
                    "id": "pick-driver",
                    "label": "Pick driver",
                    "children": steps[0:2],
                },
            ],
        }
    ]
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        first = 2 + 4 * (i - 1)
        children.append(
            {
                "type": "fallback",
                "id": f"{p}",
                "label": f"Part {i} assembled",
                "children": [
                    {
                        "type": "condition",
                        "id": f"{p}-done",
                        "label": f"{p} assembled",
                        "capability": "part_done",
                        "parameters": {
                            "part": p,
                            "clamp": f"fixtures/clamp{i}",
                            "seat": f"{p}/h_seat",
                            "hole1": f"{p}/h_hole1",
                            "hole2": f"{p}/h_hole2",
                            "holder": f"{LEFT}/gripper",
                            "held": f"{p}/h_grasp",
                        },
                    },
                    {
                        "type": "sequence",
                        "id": f"{p}-assemble",
                        "label": f"Assemble {p}",
                        "children": steps[first : first + 4],
                    },
                ],
            }
        )
    children.append(steps[-1])
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


def transactions(document: dict[str, Any]) -> list[dict[str, Any]]:
    """The plan's transactions in execution order."""
    found = []

    def visit(node: dict[str, Any]) -> None:
        if node["type"] == "transaction":
            found.append(node)
        for child in node.get("children", []):
            visit(child)

    visit(document["root"])
    return found


def blocks_by_label(n_parts: int) -> dict[str, dict[str, Any]]:
    return {block["label"]: block for block in build_mission(n_parts)}


# --- The mission as a planning problem (M3, #13) ------------------------------

#: Export-only preconditions over static facts (``static_facts``): which
#: gripper grasps which handle, which clamp takes which part's seat, which
#: holes are the part's, which tool drives screws and where it is docked.
#: They bound what a planner may bind each parameter to; the capabilities'
#: run-time preconditions are unchanged.
STATIC_PRECONDITIONS: dict[str, tuple[str, ...]] = {
    "grasp": ("can_grasp(?gripper, ?handle)",),
    "clamp_and_screw": (
        "clamp_takes(?clamp, ?seat)",
        "seat_of(?part, ?seat)",
        "carry_handle(?part, ?held)",
        "first_hole(?part, ?hole1)",
        "second_hole(?part, ?hole2)",
        "screw_tool(?tool_gripper, ?tool)",
    ),
    "rack": ("docks(?gripper, ?dock, ?dock_handle)",),
}


def static_facts(n_parts: int) -> list[str]:
    """The facts ``STATIC_PRECONDITIONS`` refer to, for ``n_parts`` parts."""
    facts = [
        f"can_grasp({RIGHT}/gripper, driver/h_grip)",
        f"screw_tool({RIGHT}/gripper, driver/h_grip)",
        f"docks({RIGHT}/gripper, fixtures/rack_hold, driver/h_rack)",
    ]
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        facts += [
            f"can_grasp({LEFT}/gripper, {p}/h_grasp)",
            f"carry_handle({p}, {p}/h_grasp)",
            f"clamp_takes(fixtures/clamp{i}, {p}/h_seat)",
            f"seat_of({p}, {p}/h_seat)",
            f"first_hole({p}, {p}/h_hole1)",
            f"second_hole({p}, {p}/h_hole2)",
        ]
    return facts


def mission_goal(n_parts: int) -> list[str]:
    """Every part clamped and screwed, released, and the driver back on its dock."""
    goal = []
    for i in range(1, n_parts + 1):
        p = f"part{i}"
        goal += [
            f"holds(fixtures/clamp{i}, {p}/h_seat)",
            f"screwed({p}, {p}/h_hole1)",
            f"screwed({p}, {p}/h_hole2)",
        ]
    return goal + [
        f"not holds({LEFT}/gripper, _)",
        "holds(fixtures/rack_hold, driver/h_rack)",
    ]


def pddl_problem(n_parts: int, state: list[str] | None = None) -> PddlExport:
    """The mission as PDDL: the capabilities, ``state`` (default: nothing
    held, nothing screwed) plus the static facts, and ``mission_goal``.

    Home moves have no effects, so they are not actions: a plan found for
    this problem is the mission's grasp/clamp/release/rack skeleton.
    """
    return to_pddl(
        descriptors(n_parts),
        init=[*static_facts(n_parts), *(state or [])],
        goal=mission_goal(n_parts),
        domain_name="screw-assembly",
        problem_name=f"screw-assembly-{n_parts}",
        static_preconditions=STATIC_PRECONDITIONS,
    )


# --- From a transaction to its block, and from a skeleton to a plan (#14) ------


def _arm(gripper: str) -> str:
    return gripper.split("/")[0]


def _other(arm: str) -> str:
    return LEFT if arm == RIGHT else RIGHT


def block_for(capability: str, parameters: dict[str, Any]) -> dict[str, Any]:
    """The geometric block a transaction runs, from its capability and parameters.

    The inverse of ``_step``: ``block_for(*_step(i, block)) == block`` for every
    ``build_mission`` block, so a plan built by a task planner (whose steps
    aren't in ``build_mission``, e.g. part 1 in clamp 2) runs the same way.
    The arm a phase doesn't move is frozen.
    """
    label = parameters["block"]
    if capability == "home":
        return {"label": label, "move": (parameters["arm"], "driver")}
    if capability == "grasp":
        gripper = parameters["gripper"]
        return {
            "label": label,
            "seq": [(gripper, parameters["handle"])],
            "frozen": {0: [_other(_arm(gripper))]},
        }
    if capability == "release":
        gripper = parameters["gripper"]
        return {
            "label": label,
            "seq": [(gripper, None)],
            "frozen": {0: [_other(_arm(gripper))]},
        }
    if capability == "rack":
        other = _other(_arm(parameters["gripper"]))
        return {
            "label": label,
            "seq": [
                (parameters["dock"], parameters["dock_handle"]),
                (parameters["gripper"], None),
            ],
            "frozen": {0: [other], 1: [other]},
        }
    if capability == "clamp_and_screw":
        # Phase 0 moves the holder's arm (it carries the part into the
        # clamp); the screw phases move the tool arm.
        holder, tool = _arm(parameters["holder"]), _arm(parameters["tool_gripper"])
        return {
            "label": label,
            "seq": [
                (parameters["clamp"], parameters["seat"]),
                (DRIVER_TIP, parameters["hole1"]),
                (DRIVER_TIP, None),
                (DRIVER_TIP, parameters["hole2"]),
                (DRIVER_TIP, None),
            ],
            "frozen": {0: [tool], 1: [holder], 2: [holder], 3: [holder], 4: [holder]},
            # The clamp candidate must leave hole 1 (phase 1) AND hole 2
            # (phase 3) reachable.
            "lookahead": {"pair": (0, 1), "also": (3,)},
        }
    raise ValueError(f"no block for capability {capability!r}")


def expand_step(
    index: int, capability: str, parameters: dict[str, Any], held: dict[str, str]
) -> list[tuple[str, dict[str, Any], str]]:
    """A skeleton step as transactions: labels and ``block`` filled in, and a
    move home for the tool arm after the pickup and after each part's
    screws (those moves have no effects, so a planner never plans them).

    ``held`` (gripper -> handle, updated here) names what a release lets go.
    """
    p = dict(parameters)
    out: list[tuple[str, dict[str, Any], str]] = []

    def add(cap: str, params: dict[str, Any], label: str) -> None:
        out.append((cap, {**params, "block": label}, label))

    def home(tag: str, arm: str) -> None:
        add("home", {"arm": arm}, f"{arm} home ({tag})")

    if capability == "grasp":
        held[p["gripper"]] = p["handle"]
        if p["handle"] == "driver/h_grip":
            add("grasp", p, "bootstrap: pick driver")
            home("bootstrap", _arm(p["gripper"]))
        else:
            add("grasp", p, f"{p['handle'].split('/')[0]} A0: grasp")
    elif capability == "clamp_and_screw":
        part, clamp = p["part"], p["clamp"]
        slot = clamp.split("/")[-1]
        default = f"clamp{part.removeprefix('part')}"
        suffix = "" if slot == default else f" (in {slot})"
        add("clamp_and_screw", p, f"{part} A: clamp + screw{suffix}")
        home(part, _arm(p["tool_gripper"]))
    elif capability == "release":
        what = held.pop(p["gripper"], "")
        add("release", p, f"{what.split('/')[0] or p['gripper']} B: release")
    elif capability == "rack":
        held.pop(p["gripper"], None)
        add("rack", p, "return: rack driver")
    else:
        raise ValueError(f"unexpected skeleton step {capability!r}")
    return out


def planned_document(
    steps: list[tuple[str, dict[str, Any]]],
    n_parts: int,
    state: list[str] | None = None,
    generator: str = "unified-planning",
) -> dict[str, Any]:
    """A TaskPlan document for a planner's skeleton (see ``pddl_problem``).

    ``state`` is the start the skeleton was planned from; it is also the
    plan's ``initial_state``, so the document is validated from there.
    """
    held: dict[str, str] = {}
    for atom in state or []:
        if atom.startswith("holds("):
            gripper, handle = atom[len("holds(") : -1].split(", ")
            held[gripper] = handle
    return skeleton_document(
        steps,
        mission_id=f"ScrewAssembly{n_parts}",
        expand=lambda index, cap, params: expand_step(index, cap, params, held),
        initial_state=state or [],
        scene={"id": "screw-assembly", "parts": n_parts},
        generator=generator,
        label=f"Screw assembly, {n_parts} part(s) (planned)",
    )
