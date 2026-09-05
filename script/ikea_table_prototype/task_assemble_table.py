#!/usr/bin/env python3
"""IKEA LACK table assembly — 2x UR10+Robotiq, YAML-driven task script.

Local prototype only — see README.md (this whole directory only exists on
the local/ikea-furniture-prototype branch). pyhpp only imports inside the
hpp-agimus-arm64 container (see debug_view_frames.py's docstring), which
this host shell doesn't have — every actual run of this script happens
there. Has been run repeatedly there: fixed several real bugs live
(double-prefixed handle/contact names, missing arm/gripper self-collision
exclusions in ur10_robotiq.srdf, a zero-gap resting collision, a wrong
pregrasp approach-direction sign — see build_assets.py's urdf-srdf stage
docstring for that one — an unseeded/deterministic RNG, and the arm
pedestal height). Phase 1's pregrasp target now converges reliably but a
full 12-phase success hasn't landed yet — see GRASP_SEQUENCE's own comment
below for where that stood.

Copied from script/templates/task_yaml_template.py (the generic YAML-driven
task template) with one deliberate structural deviation, explained below.

WHY with_grasp_goals() IS NOT USED HERE
----------------------------------------
The template's usual pattern narrows the loaded config to just the active
grasp pairs via ``_loader.task_config.with_grasp_goals(GRASP_GOALS)``. That
filter (``yaml_loader._yaml_with_grasp_goals``) keeps only the OBJECTS
whose *handle* appears in a goal string — but `table` only ever appears as
the HANDLE side of leg-into-table docking (table/socketN_hole, see build_
assets.py's urdf-srdf stage), never as a grasp target itself in
GRASP_SEQUENCE. Verified empirically (back when leg->table was a placement
<contact> pair rather than today's docking-handle pair, but the mechanism
is the same shape): calling with_grasp_goals() with only leg grasp goals
silently drops `table` from OBJECTS, taking its handles/contacts out of
the graph with it — with nowhere for a leg's peg to dock, the sequence
would be unsatisfiable. So this script uses ``_loader.task_config``
unfiltered instead, keeping all 5 objects (table + 4 legs) and every
declared handle present regardless of which pairs this run's
GRASP_SEQUENCE touches.

WHAT "PLACE INTO A SOCKET" ACTUALLY MEANS HERE
------------------------------------------------
leg1 -> socket1, leg2 -> socket2, etc. is a deterministic gripper/handle
docking pair now (legN/peg <-> table/socketN_hole, restricted 1:1 by
ikea_table_config.yaml's valid_pairs) — NOT the placement-<contact>
mechanism this section used to describe (ConstraintGraphFactory picking an
emergent leaf of a combinatorial leg/socket placement manifold on a plain
`(gripper, None)` release). See build_assets.py's urdf-srdf stage
docstring for why that was replaced: the old mechanism left which socket a
released leg landed in emergent from the transport path, not a value this
script set. GRASP_SEQUENCE below now has 3 phases per leg instead of 2 —
grasp the handle, dock the peg into its named socket, then release the
handle (the leg stays put, held by the docking grasp alone).

GRASP_SEQUENCE below alternates arms across the 4 legs (mostly to exercise
both arms of the two-UR10 setup this scene was built for) — an arbitrary
first-draft ordering, not derived from any reachability or collision
analysis. Likely the first thing to change if planning stalls.

STATUS: after the fixes listed above, phase 1's pregrasp target
(ur10_left/gripper > leg1/handle) converges reliably (residual ~1e-10 on
many attempts) but hasn't yet landed a fully collision-free candidate
within the ~30s/attempt-batch budget ConfigGenerator.generate_via_edge
hardcodes (not exposed through plan_sequence() to override). Dominant
remaining failures: ur10_left/upper_arm_link vs ground, and
ur10_left/wrist_2_link or forearm_link vs workbench — genuine geometric
tightness given this arm's reach into this specific spot on the
workbench, not a further bug found so far. Paused here to let a human
look at the scene directly (debug_view_frames.py) rather than keep
burning compute on blind retries.

RNG SEEDING — load-bearing, not cosmetic
------------------------------------------
Target generation (``ConfigGenerator.generate_via_edge``) draws its random
restarts via pinocchio's ``randomConfiguration``, backed by a C++ RNG that
nothing in this pipeline ever seeds. Confirmed live: two completely
separate ``python3`` process runs produced bit-for-bit identical
"random" draws (same 608 attempts, same final residual, same collision
pair) — every retry during this task's debugging was silently replaying
the exact same candidate sequence, never sampling anything new. Seeding
from wall-clock time here is what makes repeated runs actually explore
different candidates.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import pinocchio as pin

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner
from long_tamp.tasks import ManipulationTask
from long_tamp.config.yaml_loader import YamlTaskLoader

pin.seed(time.time_ns() % (2**31))


# ---------------------------------------------------------------------------
TASK_NAME = "IKEA LACK Table Assembly (2x UR10+Robotiq-2F85)"

_YAML_PATH = Path(__file__).parent / "config" / "ikea_table_config.yaml"

# ---------------------------------------------------------------------------
# OPTIONAL TUNING
# ---------------------------------------------------------------------------

# The Robotiq's 5 mimic-tagged finger joints are independent DOF here (see
# ikea_table_config.yaml's joint_groups comment — no <mimic> support
# anywhere in this workspace's HPP stack) but purely cosmetic: the grasp
# itself is a rigid gripper_tcp<->handle constraint, not simulated finger
# closing. Frozen the same way twin/task_lift_ball.py freezes Panda's
# finger joints, via substring match on the joint name.
FREEZE_JOINT_SUBSTRINGS: List[str] = ["finger_joint", "knuckle_joint"]

# Cosmetic replay-only overlay (_replay_sequence_with_grasp_visuals):
# these joints stay frozen open per FREEZE_JOINT_SUBSTRINGS during actual
# planning, so nothing closes them for real. Mimic multipliers from
# robotiq_arg2f_85_model_macro.xacro's <mimic> tags, duplicated from
# debug_view_frames.py's GRIPPER_MIMIC_JOINTS since this pairing is
# specific to this demo's gripper + leg geometry.
GRIPPER_MIMIC_JOINTS: List[Tuple[str, int]] = [
    ("finger_joint", 1),
    ("left_inner_knuckle_joint", 1),
    ("left_inner_finger_joint", -1),
    ("right_outer_knuckle_joint", 1),
    ("right_inner_knuckle_joint", 1),
    ("right_inner_finger_joint", -1),
]

# finger_joint value that closes the inner-finger pads to ~30mm apart,
# matching the leg's 0.03x0.03 cross-section (LEG_HALF_EXTENT in
# build_assets.py). Found via an FK sweep: 0 rad -> 0.0924m open, 0.8 rad
# -> 0.0079m (pads touching), 0.6 rad -> 0.0307m (closest to 0.03m).
GRIPPER_CLOSED_VALUE: float = 0.60

# Found live, first run: IK for leg1's pregrasp kept landing in
# "Collision between object ur10_left/pedestal_0 and
# ur10_left/upper_arm_link_0" (23/26 attempts were solver-non-convergence,
# but the 3 that did converge hit exactly this pair). pedestal is a coarse
# 0.3x0.3x0.3 box build_assets.py's merge-arm stage fixes under the
# shoulder (see its PEDESTAL_LINK) — two joints away from upper_arm_link
# in the chain (world -[shoulder_pan]-
# shoulder_link -[shoulder_lift]- upper_arm_link), so not auto-excluded as
# an adjacent pair. Excluding it as a known-coarse-proxy false positive,
# same category task_yaml_template.py's COLLISION_EXCLUSIONS comment
# describes — not verified against the real UR10 mesh silhouette (no
# visual inspection was done here, only that planning proceeds past this
# error). Revisit if a resulting path visibly runs the real arm mesh
# through the real pedestal mesh once someone views a replay.
COLLISION_EXCLUSIONS: List[Tuple[str, str]] = [
    ("ur10_left/pedestal", "ur10_left/shoulder_lift_joint"),
    ("ur10_right/pedestal", "ur10_right/shoulder_lift_joint"),
    # Same story, one link further down the chain: after fixing the
    # workbench-clearance/pregrasp-direction bugs, phase 1 started hitting
    # "pedestal_0 and forearm_link_0" instead — forearm_link is attached
    # to elbow_joint, a different joint than upper_arm_link's
    # shoulder_lift_joint, so the exclusion above never covered it.
    ("ur10_left/pedestal", "ur10_left/elbow_joint"),
    ("ur10_right/pedestal", "ur10_right/elbow_joint"),
    # Same story, another link further down the chain: random_config's
    # per-attempt failure tally (long_tamp.backends.pyhpp) surfaced
    # "pedestal_0 and wrist_1_link_0" as a dominant failure reason
    # (78/1000 in one batch) — wrist_1_link is attached to wrist_1_joint,
    # still a different joint than the two exclusions above cover.
    ("ur10_left/pedestal", "ur10_left/wrist_1_joint"),
    ("ur10_right/pedestal", "ur10_right/wrist_1_joint"),
    # One more link down again, same rerun: "pedestal_0 and
    # wrist_2_link_0" (65/1000). At this point every joint from
    # shoulder_lift through wrist_1 has independently shown up here one
    # rerun at a time — a coarse 0.3m box under the shoulder is going to
    # overlap *something* in most arm poses regardless of which specific
    # downstream link it is, so this joint-by-joint whack-a-mole will
    # likely keep finding one more link each run. If wrist_3_joint (the
    # last one before the gripper) turns up next, consider excluding the
    # whole pedestal-vs-arm-chain in one shot instead of a 5th entry here.
    ("ur10_left/pedestal", "ur10_left/wrist_2_joint"),
    ("ur10_right/pedestal", "ur10_right/wrist_2_joint"),
]

# Pick up each leg, dock its peg into the matching table socket, release the
# arm's grip (the leg stays put, held by the docking grasp) -- 3 phases per
# leg. leg1 -> socket1, leg2 -> socket2, etc. is now a deterministic
# gripper/handle pairing (see ikea_table_config.yaml's valid_pairs and
# build_assets.py's urdf-srdf stage docstring for why/how), not an emergent
# placement choice -- see the module docstring's old "WHAT 'PLACE INTO A
# SOCKET' ACTUALLY MEANS HERE" section, superseded by this change.
GRASP_SEQUENCE: List[Tuple[str, Optional[str]]] = [
    ("ur10_right/gripper", "leg1/handle"),  # arm grasps leg1
    # ("leg1/peg", "table/socket1_hole"),  # dock leg1's peg into socket1
    # ("ur10_right/gripper", None),  # arm releases leg1 (stays docked)
    # ("ur10_right/gripper", "leg2/handle"),
    # ("leg2/peg", "table/socket2_hole"),
    # ("ur10_right/gripper", None),
    # ("ur10_right/gripper", "leg3/handle"),
    # ("leg3/peg", "table/socket3_hole"),
    # ("ur10_right/gripper", None),
    # ("ur10_right/gripper", "leg4/handle"),
    # ("leg4/peg", "table/socket4_hole"),
    # ("ur10_right/gripper", None),
]

# Phase 0 (grasp leg1 with ur10_right) reliably fails at path planning, not
# target generation: ✓ SUCCESS target-config generation on every attempt,
# immediately followed by "Maximal number of iterations reached" on
# computePath, repeated across 9 freshly-regenerated targets in one run.
# That pattern -- reachable endpoint, unreachable path, every time -- points
# to a static obstacle blocking the corridor rather than sampling bad luck.
# Prime suspect: ur10_left. Its initial pose (ikea_table_config.yaml's
# UR10_LEFT joint_groups) was the pose whose reach into leg1's pregrasp spot
# was actually IK-verified (see that file's comment above joint_groups,
# 300/300 restarts) -- but GRASP_SEQUENCE above sends ur10_right after leg1
# instead, while frozen_arms_mode="auto" (the plan_sequence default) locks
# ur10_left rigidly at that same pose for the whole run. ur10_left never
# grasps anything in this sequence, so nothing is lost by letting it move
# out of ur10_right's way for this one phase.
#
# Scoped to phase 0 only (dict keys not listed default to frozen-nothing in
# "manual" mode -- see GraspSequencePlanner.plan_sequence's per_phase_frozen_arms
# docstring) so every other phase keeps "auto"'s normal freeze-inactive-arm
# behavior -- including the new legN/peg docking phases (index 1, 4, 7, 10):
# ikea_table_config.yaml's arm_groups now lists legN/peg under ur10_right
# (it's the arm actually carrying the leg), so "freeze ur10_left" is the
# correct auto-equivalent there too, same as the plain grasp/release phases.
# If leg2-4's grasp phases (index 3, 6, 9) hit the same corridor-blocking
# failure phase 0 did, unfreeze ur10_left there as well.
PER_PHASE_FROZEN_ARMS: dict[int, List[str]] = {
    i: (["ur10_left"])
    for i in range(len(GRASP_SEQUENCE))
}


# ---------------------------------------------------------------------------
# Loader (module-level singleton — parsed once, reused if imported)
# ---------------------------------------------------------------------------

_loader = YamlTaskLoader(_YAML_PATH)


# ---------------------------------------------------------------------------
# Task class
# ---------------------------------------------------------------------------

class TableAssemblyTask(ManipulationTask):
    """2x UR10+Robotiq assembling a 4-legged IKEA LACK table prototype."""

    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name=TASK_NAME,
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        # Unfiltered — see module docstring for why with_grasp_goals()
        # would silently break table-socket placement here.
        self.task_config = _loader.task_config
        self.use_factory = True

    def build_initial_config(self) -> List[float]:
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


# ---------------------------------------------------------------------------
# Run function (called from main, importable for tests)
# ---------------------------------------------------------------------------

def run_task(backend: str = "pyhpp") -> bool:
    """Set up the task, run the planner, offer interactive replay.

    Returns:
        True if planning succeeded, False otherwise.
    """
    task = TableAssemblyTask(backend=backend)

    print("\n" + "=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  Backend   : {backend}")
    print(f"  YAML cfg  : {_YAML_PATH.name}")
    print(f"  Objects   : {task.task_config.OBJECTS}")
    print(f"  Frozen    : {task.FREEZE_JOINT_SUBSTRINGS}")
    print("=" * 70 + "\n")

    print("Setting up task (loading URDF/SRDF, building scene)...")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,
        )
    except Exception as exc:
        import traceback
        print(f"✗ Setup failed: {exc}")
        traceback.print_exc()
        return False
    print("✓ Task set up")

    if COLLISION_EXCLUSIONS:
        print("\nDisabling collision exclusions...")
        removed = 0
        for body_a, body_b in COLLISION_EXCLUSIONS:
            try:
                task.scene_builder.disable_collision_pair(
                    obstacle_name=body_a, joint_name=body_b
                )
                removed += 1
            except Exception as exc:
                print(f"  ⚠ Could not disable {body_a} <-> {body_b}: {exc}")
        print(f"✓ Disabled {removed}/{len(COLLISION_EXCLUSIONS)} pair(s)")

    q_init = task.q_init
    if not q_init:
        print("✗ Failed to get initial configuration")
        return False
    print(f"\n✓ Initial config: {len(q_init)} DOF")

    try:
        task.planner.visualize(q_init)
        print("✓ Initial scene displayed")
    except Exception as exc:
        print(f"⚠ Visualization skipped: {exc}")

    print("\nCreating GraspSequencePlanner...")
    seq_planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        graph_constraints=getattr(task, "_graph_constraints", None),
        freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
        auto_save_dir=None,
        run_logger=getattr(task, "run_logger", None),
    )

    print(f"\nPlanning sequence: {GRASP_SEQUENCE}")
    try:
        result = seq_planner.plan_sequence(
            grasp_sequence=GRASP_SEQUENCE,
            q_init=q_init,
            verbose=True,
            frozen_arms_mode="manual",
            per_phase_frozen_arms=PER_PHASE_FROZEN_ARMS,
        )
    except Exception as exc:
        import traceback
        print(f"\n✗ Planning error: {exc}")
        traceback.print_exc()
        return False

    if not result["success"]:
        print("\n" + "=" * 70)
        print("✗ PLANNING FAILED")
        print("=" * 70)
        print(f"  Reason: {result.get('error', 'Unknown')}")
        return False

    print("\n" + "=" * 70)
    print("✓ PLANNING SUCCEEDED")
    print("=" * 70)
    print(seq_planner.get_phase_summary())

    all_paths = [
        p
        for phase in seq_planner.phase_results
        for p in phase.get("paths", [])
        if p is not None
    ]
    if all_paths:
        print(f"\n✓ {len(all_paths)} path(s) generated")
        _interactive_replay(task, seq_planner)
    return True


# ---------------------------------------------------------------------------
# Interactive replay helpers
# ---------------------------------------------------------------------------

# gripper name -> arm side, for real UR10 grippers only -- excludes a
# carried object's own pseudo-gripper (e.g. "leg1/peg"), which has no
# fingers to animate.
_REAL_GRIPPER_SIDE = {
    "ur10_left/gripper": "ur10_left",
    "ur10_right/gripper": "ur10_right",
}


def _gripper_override(
    q: np.ndarray, rank: dict, side: str, closedness: float
) -> np.ndarray:
    """Return a copy of `q` with `side`'s 6 gripper joints set to a given
    closedness (0.0 = open, 1.0 = GRIPPER_CLOSED_VALUE), mimic-consistent
    across all 6 via GRIPPER_MIMIC_JOINTS' multipliers.
    """
    q = q.copy()
    for j, mult in GRIPPER_MIMIC_JOINTS:
        q[rank[f"{side}/{j}"]] = mult * (closedness * GRIPPER_CLOSED_VALUE)
    return q


def _animate_gripper(
    task: TableAssemblyTask,
    q: np.ndarray,
    rank: dict,
    side: str,
    from_closedness: float,
    to_closedness: float,
    steps: int = 20,
    dt: float = 0.02,
) -> np.ndarray:
    """Animate `side`'s gripper between two closedness values.

    Visualization only. Returns the final full q for the caller to track.
    """
    for i in range(steps + 1):
        frac = from_closedness + (to_closedness - from_closedness) * i / steps
        q = _gripper_override(q, rank, side, frac)
        task.planner.visualize(q)
        time.sleep(dt)
    return q


def _replay_sequence_with_grasp_visuals(
    task: TableAssemblyTask,
    seq_planner: GraspSequencePlanner,
    n_samples: int = 60,
    dt: float = 0.02,
) -> None:
    """Replay phase_results with a cosmetic gripper close/open overlay.

    HPP's planned paths keep gripper joints frozen open throughout (the
    grasp is a rigid TCP constraint, not simulated finger closing), so
    played as-is every leg would float in an open gripper. This closes a
    side's fingers after it grasps a real handle, opens them before it
    releases, and re-applies the closed pose over every frame in between
    (including non-gripper phases like peg->socket docking) since the raw
    path always carries the open value.
    """
    rank = task.robot.rankInConfiguration
    held: dict[str, float] = {"ur10_left": 0.0, "ur10_right": 0.0}

    q = np.array(task.q_init, dtype=float)
    task.planner.visualize(q)

    for phase in seq_planner.phase_results:
        if phase.get("skipped") or not phase.get("paths"):
            continue

        gripper = phase.get("gripper")
        handle = phase.get("handle")
        side = _REAL_GRIPPER_SIDE.get(gripper)

        print(f"\nPhase {phase['phase']}: {gripper} -> {handle}")

        if side is not None and handle is None and held.get(side, 0.0) > 0.0:
            print(f"  opening {side}'s gripper")
            q = _animate_gripper(task, q, rank, side, held[side], 0.0, dt=dt)
            held[side] = 0.0

        for path_obj in phase.get("paths", []):
            if path_obj is None:
                continue
            path = (
                task.planner.get_path(path_obj)
                if isinstance(path_obj, int)
                else path_obj
            )
            length = path.length()
            for i in range(n_samples + 1):
                q_path, ok = path.call(length * i / n_samples)
                if not ok:
                    continue
                q = np.array(q_path, dtype=float)
                # The raw path always carries open (0) fingers -- redraw
                # any still-held side's closed pose over it every frame.
                for held_side, closedness in held.items():
                    if closedness > 0.0:
                        q = _gripper_override(q, rank, held_side, closedness)
                task.planner.visualize(q)
                time.sleep(dt)

        if side is not None and handle is not None:
            print(f"  closing {side}'s gripper")
            q = _animate_gripper(task, q, rank, side, 0.0, 1.0, dt=dt)
            held[side] = 1.0

    print("\n✓ Replay complete")


def _interactive_replay(
    task: TableAssemblyTask, seq_planner: GraspSequencePlanner
) -> None:
    """Menu-driven replay of the generated paths."""
    path_items: List[Tuple[str, object]] = []
    for phase in seq_planner.phase_results:
        for idx, path_obj in enumerate(phase.get("paths", [])):
            if path_obj is not None:
                edge_names = phase.get("edges", [])
                label = (
                    edge_names[idx]
                    if idx < len(edge_names)
                    else f"phase {phase['phase']} path {idx}"
                )
                path_items.append((label, path_obj))

    print("\n" + "-" * 50)
    print("Replay menu")
    print(f"  {len(path_items)} path(s) available")
    for i, (label, _) in enumerate(path_items):
        print(f"    [{i}]  {label}")
    print("  [a]  replay all in sequence")
    print("  [q]  quit")
    print("-" * 50)

    while True:
        try:
            raw = input("replay> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if raw in ("q", "quit", "exit", ""):
            break

        if raw in ("a", "all"):
            print("\nReplaying full sequence (with gripper open/close overlay)...")
            try:
                _replay_sequence_with_grasp_visuals(task, seq_planner)
            except Exception as exc:
                print(f"  ⚠ replay with grasp visuals failed: {exc}")
                _replay_fallback(task, path_items)
            continue

        try:
            idx = int(raw)
        except ValueError:
            print(f"  Unknown '{raw}'. Enter index, 'a' for all, 'q' to quit.")
            continue

        if idx < 0 or idx >= len(path_items):
            print(f"  Index {idx} out of range (0 – {len(path_items) - 1})")
            continue

        label, path_obj = path_items[idx]
        print(f"\nReplaying [{idx}] {label} ...")
        try:
            if isinstance(path_obj, int):
                task.planner.play_path(path_obj)
            else:
                task.planner.play_path_vector(path_obj)
            print("✓ Done")
        except Exception as exc:
            print(f"  ⚠ Failed: {exc}")


def _replay_fallback(task: TableAssemblyTask, path_items: List) -> None:
    """Replay all paths individually when replay_sequence() raises."""
    for label, path_obj in path_items:
        print(f"  Playing: {label}")
        try:
            if isinstance(path_obj, int):
                task.planner.play_path(path_obj)
            else:
                task.planner.play_path_vector(path_obj)
        except Exception as exc:
            print(f"    ⚠ Failed: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=TASK_NAME)
    parser.add_argument(
        "--backend",
        default="pyhpp",
        choices=["pyhpp"],
        help="HPP backend to use (default: pyhpp)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip gepetto-viewer display",
    )
    parser.add_argument(
        "--show-joints",
        action="store_true",
        help="Print all joint names and DOF ranks, then exit",
    )
    args = parser.parse_args()

    if args.show_joints:
        task = TableAssemblyTask(backend=args.backend)
        task.setup(skip_graph=True)
        from long_tamp.visualization import print_joint_info
        print_joint_info(task.robot)
        return 0

    success = run_task(backend=args.backend)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
