#!/usr/bin/env python3
"""Generic task script template — YAML-driven, robot-agnostic.

HOW TO USE
----------
1.  Copy this file to  script/<your_robot>/task_<name>.py
2.  Copy the YAML template to  script/config/<your_robot>_config.yaml  and
    fill in every <PLACEHOLDER> with your robot's actual values.
3.  Edit the three constants at the top of this file:
        TASK_NAME  — human-readable description shown in logs
        _YAML_PATH — path to your YAML config (relative to this script's location)
        GRASP_GOALS — list of "gripper grasps handle" strings to plan
4.  Set FREEZE_JOINT_SUBSTRINGS to freeze arms that are not involved in the
    active grasp (optional, can be []).
5.  Populate COLLISION_EXCLUSIONS with (body_a, body_b) pairs to remove
    known false-positive collisions (optional, can be []).
6.  Run:
        python script/<your_robot>/task_<name>.py --backend pyhpp
        python script/<your_robot>/task_<name>.py --backend corba

ARCHITECTURE
------------
YAML config
    └─ YamlTaskLoader ──► file_paths          → ManipulationTask (SceneBuilder)
                       ──► joint_bounds_class → ManipulationTask (SceneBuilder)
                       ──► task_config        → GraspSequencePlanner
                       ──► build_initial_config() → q_init

ManipulationTask.setup()
    └─ builds HPP scene, constraint graph, planner

GraspSequencePlanner.plan_sequence()
    └─ plans each (gripper, handle) pair in the given sequence
       using the factory constraint graph built by setup()

ADDING A RELEASE STEP
---------------------
A release is encoded as (gripper, None) in the grasp sequence:

    grasp_sequence = [
        ("<robot>/gripper",  "<object>/handle"),   # approach + grasp
        ("<robot>/gripper",  None),                # retract + release
    ]

MULTI-GRIPPER SEQUENCES
-----------------------
Just extend the list.  Each entry is an independent (grasp or release) phase:

    grasp_sequence = [
        ("arm1/gripper", "obj1/handle"),
        ("arm2/gripper", "obj2/handle"),
        ("arm1/gripper", None),
        ("arm2/gripper", None),
    ]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Optional

from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.config.yaml_loader import YamlTaskLoader


# ---------------------------------------------------------------------------
# EDIT THESE THREE CONSTANTS  ← ← ←
# ---------------------------------------------------------------------------

# Human-readable label printed in log output.
TASK_NAME = "My Robot: <Task Description>"

# Path to YAML config relative to this script's parent's config/ directory.
# Adjust the filename to match your config file.
_YAML_PATH = Path(__file__).parent.parent / "config" / "my_robot_config.yaml"

# Grasp goals to plan.  Each string must match "<gripper_frame> grasps <handle_frame>".
# The factory graph will be restricted to only these pairs.
GRASP_GOALS: List[str] = [
    "<robot_name>/gripper grasps <object_name>/handle",
    # Add more pairs if planning a multi-grasp sequence.
]

# ---------------------------------------------------------------------------
# OPTIONAL TUNING  ← edit as needed
# ---------------------------------------------------------------------------

# Joint substrings for arms to freeze while another arm is active.
# Example: ["second_arm"] freezes all joints whose name contains "second_arm".
# Leave empty if you have a single arm or do not want auto-freezing.
FREEZE_JOINT_SUBSTRINGS: List[str] = []

# Collision exclusions: list of (body_a, body_b) link name pairs to ignore.
# Use when the URDF geometry causes false-positive collisions at valid configs.
COLLISION_EXCLUSIONS: List[Tuple[str, str]] = [
    # ("<env_link>", "<robot_link>"),
]

# Grasp sequence to plan.  Each entry is (gripper_frame, handle_frame_or_None).
# None as the second element encodes a release phase.
GRASP_SEQUENCE: List[Tuple[str, Optional[str]]] = [
    ("<robot_name>/gripper", "<object_name>/handle"),
    # ("<robot_name>/gripper", None),   # Uncomment to add a release phase.
]


# ---------------------------------------------------------------------------
# Loader (module-level singleton — parsed once, reused if imported)
# ---------------------------------------------------------------------------

_loader = YamlTaskLoader(_YAML_PATH)


# ---------------------------------------------------------------------------
# Task class
# ---------------------------------------------------------------------------

class MyRobotTask(ManipulationTask):
    """Rename this class to something descriptive, e.g. PickAndPlaceTask."""

    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name=TASK_NAME,
            backend=backend,
            # These two come straight from the YAML loader — no robot-specific code.
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        # Restrict the factory graph to only the active grasp goals.
        self.task_config = _loader.task_config.with_grasp_goals(GRASP_GOALS)
        # Always True for YAML-driven scripts: the factory builds the graph.
        self.use_factory = True
        # PyHPP backend may need a constraint dict (leave {} for CORBA).
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> List[float]:
        """Initial configuration from YAML for the active object subset."""
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


# ---------------------------------------------------------------------------
# Run function (called from main, importable for tests)
# ---------------------------------------------------------------------------

def run_task(backend: str = "pyhpp") -> bool:
    """Set up the task, run the planner, offer interactive replay.

    Returns:
        True if planning succeeded, False otherwise.
    """
    task = MyRobotTask(backend=backend)

    # ------------------------------------------------------------------
    # Print a short summary of what we are about to do.
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  Backend   : {backend}")
    print(f"  YAML cfg  : {_YAML_PATH.name}")
    print(f"  Goals     : {GRASP_GOALS}")
    print(f"  Frozen    : {task.FREEZE_JOINT_SUBSTRINGS}")
    print("=" * 70 + "\n")

    # ------------------------------------------------------------------
    # 1. Build the HPP scene, constraint graph, and planner.
    #    skip_graph=True because GraspSequencePlanner owns its own graph.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 2. Disable false-positive collision pairs (if any).
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 3. Retrieve and display initial configuration.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 4. Build GraspSequencePlanner and run.
    # ------------------------------------------------------------------
    print("\nCreating GraspSequencePlanner...")
    seq_planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        pyhpp_constraints=getattr(task, "pyhpp_constraints", {}),
        graph_constraints=getattr(task, "_graph_constraints", None),
        auto_save_dir=None,                       # Change to a Path to auto-save paths.
        run_logger=getattr(task, "run_logger", None),
    )

    print(f"\nPlanning sequence: {GRASP_SEQUENCE}")
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

    # ------------------------------------------------------------------
    # 5. Success — report and interactive replay.
    # ------------------------------------------------------------------
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

def _interactive_replay(task: MyRobotTask, seq_planner: GraspSequencePlanner) -> None:
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
            print("\nReplaying full sequence...")
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


def _replay_fallback(task: MyRobotTask, path_items: List) -> None:
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
        choices=["pyhpp", "corba"],
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
        # Useful for finding correct joint names for joint_groups in the YAML.
        task = MyRobotTask(backend=args.backend)
        task.setup(skip_graph=True)
        from agimus_spacelab.visualization import print_joint_info
        print_joint_info(task.robot)
        return 0

    success = run_task(backend=args.backend)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
