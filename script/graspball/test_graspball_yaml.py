#!/usr/bin/env python3
"""GraspBall task: UR5 grasps pokeball — driven by YAML config.

This script demonstrates the YAML-based configuration architecture.
All robot/object paths, initial poses, joint bounds, gripper names, and
valid pairs are declared in ``script/config/graspball_config.yaml``; no
Python-level config class references SpaceLab-specific code.

Architecture:
    graspball_config.yaml
        └─ YamlTaskLoader ──► file_paths       → SceneBuilder
                           ──► joint_bounds_class
                           ──► task_config      → ManipulationTask
                           ──► build_initial_config()

Usage::

    python script/graspball/test_graspball_yaml.py [--backend pyhpp|corba]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.config.yaml_loader import YamlTaskLoader

# ---------------------------------------------------------------------------
# Load YAML config (module-level singleton — cheap / idempotent)
# ---------------------------------------------------------------------------

_YAML_PATH = Path(__file__).parent.parent / "config" / "graspball_config.yaml"
_loader = YamlTaskLoader(_YAML_PATH)


# ---------------------------------------------------------------------------
# Task class
# ---------------------------------------------------------------------------

class GraspBallYamlTask(ManipulationTask):
    """UR5 grasps pokeball, configured entirely from YAML.

    Demonstrates:
    * Framework-agnostic file_paths / joint_bounds loaded from YAML.
    * task_config built by YamlTaskLoader — no SpaceLab-specific imports.
    * build_initial_config() draws initial poses directly from YAML data.
    * Factory-mode constraint graph for the single ur5/gripper → pokeball/handle pair.
    """

    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name="GraspBall (YAML-driven)",
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        # Use the full config filtered to the single grasp of interest.
        self.task_config = _loader.task_config.with_grasp_goals([
            "ur5/gripper grasps pokeball/handle"
        ])
        self.use_factory = True
        self.pyhpp_constraints = {}

    def build_initial_config(self) -> List[float]:
        """Build initial config from YAML: UR5 joints + pokeball pose."""
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Project into the grasp goal state and return configs."""
        cg = self.config_gen
        cg.max_attempts = getattr(
            self.task_config, "MAX_RANDOM_ATTEMPTS", cg.max_attempts
        )

        gripper = self.task_config.GRIPPERS[0]
        handle = self.task_config.HANDLES_PER_OBJECT[0][0]
        desired_grasp = f"{gripper} grasps {handle}"

        all_states = list(self.graph_builder.get_states().keys())
        goal_candidates = [s for s in all_states if s == desired_grasp]
        if not goal_candidates:
            goal_candidates = [s for s in all_states if desired_grasp in s]
        if not goal_candidates:
            raise RuntimeError(
                f"Could not find state matching '{desired_grasp}'. "
                f"Available states: {all_states}"
            )

        cg.project_on_node(goal_candidates[0], q_init, "q_goal")
        cg.configs.setdefault("q_init", list(q_init))
        return cg.configs


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _print_config_summary() -> None:
    """Print a human-readable summary of the loaded YAML config."""
    tc = _loader.task_config
    print("\n" + "=" * 60)
    print("YAML Config Summary")
    print("=" * 60)
    print(f"  Robots       : {tc.ROBOT_NAMES}")
    print(f"  Environments : {tc.ENVIRONMENT_NAMES}")
    print(f"  Objects      : {tc.OBJECTS}")
    print(f"  Joint groups : {tc.ROBOTS}")
    print(f"  Grippers     : {tc.GRIPPERS}")
    print(f"  Valid pairs  :")
    for gripper, handles in tc.VALID_PAIRS.items():
        print(f"    {gripper} → {handles}")
    q0 = _loader.build_initial_config()
    print(f"  q_init ({len(q0)} DOF): {[round(v, 4) for v in q0]}")
    print("=" * 60)


def _interactive_replay(task: GraspBallYamlTask, configs: Dict[str, List[float]]) -> None:
    """Simple interactive replay for the generated configurations."""
    labels = list(configs.keys())
    if not labels:
        print("No configurations to replay.")
        return

    print("\n" + "-" * 50)
    print("Replay menu")
    for i, label in enumerate(labels):
        print(f"  [{i}]  visualise config '{label}'")
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

        try:
            idx = int(raw)
        except ValueError:
            print(f"  Unknown command '{raw}'. Enter index or 'q'.")
            continue

        if idx < 0 or idx >= len(labels):
            print(f"  Index {idx} out of range (0 – {len(labels) - 1})")
            continue

        label = labels[idx]
        q = configs[label]
        print(f"\nDisplaying config '{label}' ...")
        try:
            task.planner.visualize(q)
            print("✓ Done")
        except Exception as e:
            print(f"  ⚠ Failed: {e}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="GraspBall task with YAML-driven config"
    )
    parser.add_argument(
        "--backend",
        default="pyhpp",
        choices=["pyhpp", "corba"],
        help="Planning backend (default: pyhpp)",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Print YAML config summary and exit without running planning",
    )
    args = parser.parse_args()

    _print_config_summary()

    if args.summary_only:
        return 0

    print(f"\n{'='*60}")
    print(f"GraspBall YAML Task  ({args.backend} backend)")
    print(f"{'='*60}\n")

    task = GraspBallYamlTask(backend=args.backend)

    # ------------------------------------------------------------------
    # Setup: load scene, build factory constraint graph
    # ------------------------------------------------------------------
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
        )
    except Exception as e:
        print(f"\n✗ Setup failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print("✓ Task initialized")
    print(f"  Initial config ({len(task.q_init)} DOF)")

    # ------------------------------------------------------------------
    # Display initial scene
    # ------------------------------------------------------------------
    try:
        task.planner.visualize(task.q_init)
        print("✓ Initial scene displayed")
    except Exception as e:
        print(f"⚠ Visualization failed: {e}")

    # ------------------------------------------------------------------
    # Generate configurations
    # ------------------------------------------------------------------
    print("\nGenerating configurations...")
    try:
        configs = task.generate_configurations(task.q_init)
        print(f"✓ Generated {len(configs)} configuration(s):")
        for name, q in configs.items():
            print(f"    {name}: {len(q)} DOF")
    except Exception as e:
        print(f"✗ Configuration generation failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    # ------------------------------------------------------------------
    # Interactive replay
    # ------------------------------------------------------------------
    _interactive_replay(task, configs)

    return 0


if __name__ == "__main__":
    sys.exit(main())
