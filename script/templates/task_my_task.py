#!/usr/bin/env python3
"""My Task — template grasp-sequence script (YAML-driven).

QUICK START
===========
Step 1: Copy the YAML config template and fill in your robot details.

    cp script/templates/task_config_template.yaml \
       script/config/my_robot_config.yaml
    # Edit my_robot_config.yaml — replace all <PLACEHOLDER> values.

Step 2: Copy this file next to your other task scripts.

    cp script/templates/task_my_task.py \
       script/my_robot/task_my_task.py

Step 3: Edit the five sections marked  # <-- EDIT  below.

Step 4: Run.

    # Print joint names (useful when filling joint_groups in the YAML):
    python script/my_robot/task_my_task.py --show-joints

    # Plan with the PyHPP backend:
    python script/my_robot/task_my_task.py --backend pyhpp

    # Same, without the viewer:
    python script/my_robot/task_my_task.py --backend pyhpp --no-viz

HOW IT WORKS
============
1. YamlTaskLoader reads my_robot_config.yaml and exposes:
      _loader.file_paths          →  URDF/SRDF locations for SceneBuilder
      _loader.joint_bounds_class  →  joint limits used by the planner
      _loader.task_config         →  grippers, handles, valid pairs
      _loader.build_initial_config() → flat q_init vector

2. MyTask.__init__() passes these to ManipulationTask, then calls
   task_config.with_grasp_goals() to restrict the factory graph to the
   specific pairs listed in GRASP_GOALS.

3. run_task() drives the full lifecycle:
      setup()  →  disable collisions  →  plan  →  replay

4. GraspSequencePlanner.plan_sequence() handles multi-phase sequences
   (approach, grasp, retract, release) automatically.

EXTENDING
=========
Grasp + release in one run:
    GRASP_SEQUENCE = [
        ("my_robot/gripper", "my_object/handle"),   # grasp phase
        ("my_robot/gripper", None),                 # release phase
    ]

Multi-arm sequence:
    GRASP_SEQUENCE = [
        ("arm1/gripper", "object1/handle"),
        ("arm2/gripper", "object2/handle"),
        ("arm1/gripper", None),
        ("arm2/gripper", None),
    ]

Auto-save planned paths to disk:
    auto_save_dir=Path("/tmp/my_task_paths")   # in GraspSequencePlanner(...)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional, Tuple

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner
from long_tamp.tasks import ManipulationTask
from long_tamp.config.yaml_loader import YamlTaskLoader


# ===========================================================================
# SECTION 1 — Task identity                                      # <-- EDIT
# ===========================================================================

# Human-readable label printed in all log output.
TASK_NAME = "My Robot: My Task"


# ===========================================================================
# SECTION 2 — YAML config path                                   # <-- EDIT
# ===========================================================================

# Adjust the filename to match your YAML config file.
# The default assumes this script lives in  script/<robot>/task_*.py
# and the YAML lives in                     script/config/*_config.yaml
_YAML_PATH = Path(__file__).parent.parent / "config" / "my_robot_config.yaml"


# ===========================================================================
# SECTION 3 — Grasp goals                                        # <-- EDIT
# ===========================================================================

# List of "<gripper_frame> grasps <handle_frame>" strings.
# The factory constraint graph will be restricted to exactly these pairs.
# Gripper and handle names must match what is defined in the robot/object SRDFs.
GRASP_GOALS: List[str] = [
    "my_robot/gripper grasps my_object/handle",
    # Add more pairs here for multi-grasp tasks.
]

# Full planning sequence — each entry is (gripper_frame, handle_frame_or_None).
# None encodes a release phase.  Mirror the pairs above (plus optional releases).
GRASP_SEQUENCE: List[Tuple[str, Optional[str]]] = [
    ("my_robot/gripper", "my_object/handle"),
    # ("my_robot/gripper", None),   # Uncomment to add a release phase.
]


# ===========================================================================
# SECTION 4 — Arm freezing (optional)                            # <-- EDIT
# ===========================================================================

# Joint name substrings identifying arms to freeze while another arm is active.
# Example: ["second_arm"] freezes all joints whose name contains "second_arm".
# Leave as [] for single-arm setups or if you do not want auto-freezing.
FREEZE_JOINT_SUBSTRINGS: List[str] = []


# ===========================================================================
# SECTION 5 — Collision exclusions (optional)                    # <-- EDIT
# ===========================================================================

# List of (body_a, body_b) link-name pairs to exclude from collision checking.
# Use this to silence known false positives at valid configurations.
# Example: [("ground/base_link", "my_robot/link_3")]
COLLISION_EXCLUSIONS: List[Tuple[str, str]] = [
    # ("environment_link", "robot_link"),
]


# ===========================================================================
# Framework code — no need to edit below this line
# ===========================================================================

# Module-level singleton: YAML is parsed once even if this module is imported
# multiple times (e.g. from a test).
_loader = YamlTaskLoader(_YAML_PATH)


class MyTask(ManipulationTask):
    """Grasp-sequence task for my robot, driven by YAML configuration."""

    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name=TASK_NAME,
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        # Restrict the factory graph to GRASP_GOALS only.
        self.task_config = _loader.task_config.with_grasp_goals(GRASP_GOALS)
        self.use_factory = True

    def build_initial_config(self) -> List[float]:
        """Return the initial configuration for the active object subset."""
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


def run_task(backend: str = "pyhpp") -> bool:
    """Run the full task lifecycle.  Returns True on planning success."""
    task = MyTask(backend=backend)

    # --- Summary ---------------------------------------------------------
    print("\n" + "=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  Backend   : {backend}")
    print(f"  YAML cfg  : {_YAML_PATH.name}")
    print(f"  Goals     : {GRASP_GOALS}")
    print(f"  Sequence  : {GRASP_SEQUENCE}")
    print(f"  Frozen    : {task.FREEZE_JOINT_SUBSTRINGS}")
    print("=" * 70 + "\n")

    # --- 1. Setup --------------------------------------------------------
    print("Setting up task...")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,   # GraspSequencePlanner owns its own graph.
        )
    except Exception as exc:
        import traceback
        print(f"✗ Setup failed: {exc}")
        traceback.print_exc()
        return False
    print("✓ Task set up")

    # --- 2. Collision exclusions -----------------------------------------
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

    # --- 3. Initial configuration ----------------------------------------
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

    # --- 4. Plan ---------------------------------------------------------
    print("\nCreating GraspSequencePlanner...")
    seq_planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        graph_constraints=getattr(task, "_graph_constraints", None),
        auto_save_dir=None,          # Set to a Path to auto-save planned paths.
        run_logger=getattr(task, "run_logger", None),
    )

    print(f"\nPlanning: {GRASP_SEQUENCE}")
    try:
        result = seq_planner.plan_sequence(
            grasp_sequence=GRASP_SEQUENCE,
            q_init=q_init,
            verbose=True,
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

    # --- 5. Replay -------------------------------------------------------
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
# Interactive replay
# ---------------------------------------------------------------------------

def _interactive_replay(task: MyTask, seq_planner: GraspSequencePlanner) -> None:
    """Menu-driven path replay after successful planning."""
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
    for i, (label, _) in enumerate(path_items):
        print(f"    [{i}]  {label}")
    print("  [a]  replay all")
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
            print("Replaying full sequence...")
            try:
                seq_planner.replay_sequence(speed=1.0)
                print("✓ Done")
            except Exception as exc:
                print(f"  ⚠ replay_sequence failed: {exc}")
                _replay_fallback(task, path_items)
            continue

        try:
            idx = int(raw)
        except ValueError:
            print(f"  Unknown '{raw}'. Enter index, 'a' for all, or 'q' to quit.")
            continue

        if not 0 <= idx < len(path_items):
            print(f"  Index {idx} out of range (0 – {len(path_items) - 1})")
            continue

        label, path_obj = path_items[idx]
        print(f"Replaying [{idx}] {label} ...")
        try:
            if isinstance(path_obj, int):
                task.planner.play_path(path_obj)
            else:
                task.planner.play_path_vector(path_obj)
            print("✓ Done")
        except Exception as exc:
            print(f"  ⚠ Failed: {exc}")


def _replay_fallback(task: MyTask, path_items: List) -> None:
    """Play each path individually when replay_sequence() raises."""
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
        help="HPP backend (default: pyhpp)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip gepetto-viewer display",
    )
    parser.add_argument(
        "--show-joints",
        action="store_true",
        help="Print all joint names and ranks, then exit",
    )
    args = parser.parse_args()

    if args.show_joints:
        # Load the scene without planning — useful for finding joint names
        # to put in joint_groups in the YAML config.
        task = MyTask(backend=args.backend)
        task.setup(skip_graph=True)
        from long_tamp.visualization import print_joint_info
        print_joint_info(task.robot)
        return 0

    success = run_task(backend=args.backend)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
