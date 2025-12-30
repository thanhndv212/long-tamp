#!/usr/bin/env python3
"""Task template for agimus_spacelab.

Copy this file into `script/<your_package>/task_<name>.py` and adapt:
- `TASK_NAME` (display name)
- `initialize_task_config()` import/module/class
- collision setup (optional)
- manual waypoint generation or factory mode goal

This template mirrors the structure of
`script/spacelab/task_grasp_frame_gripper.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List

from agimus_spacelab.tasks import ManipulationTask


TASK_NAME = "Spacelab Manipulation: <YOUR TASK NAME>"

# Edit these two constants after copying the template.
# Defaults are set so this template can run immediately.
CONFIG_MODULE = "spacelab_config"
CONFIG_CLASS = "GraspFrameGripper"


# Add config directory (so `script/config/<...>_config.py` can be imported).
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))


def initialize_task_config():
    """Import and initialize the task configuration.

        Expected:
    - a config module in `script/config/`.
    - a `TaskConfigurations` container with a `TaskConfigClass` inside.

        See `script/templates/task_config_template.py` for a starting point.
    """
    import importlib

    # Replace CONFIG_MODULE/CONFIG_CLASS after copying.
    module = importlib.import_module(CONFIG_MODULE)
    task_configurations = getattr(module, "TaskConfigurations")
    task_config = getattr(task_configurations, CONFIG_CLASS)
    task_config.init_poses()
    return task_config


class TemplateTask(ManipulationTask):
    """Default task template."""

    FREEZE_JOINT_SUBSTRINGS: List[str] = []

    def __init__(self, use_factory: bool = False, backend: str = "corba"):
        super().__init__(task_name=TASK_NAME, backend=backend)
        self.task_config = initialize_task_config()
        self.use_factory = use_factory

    def setup_collision_management(self) -> None:
        """Optional: disable problematic collisions for this task."""

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate waypoint configurations.

        Contract: return a dict of named configurations.
        Common keys: q_init, q_goal, plus intermediate q_wp_*.
        """
        cg = self.config_gen
        cg.max_attempts = getattr(
            self.task_config,
            "MAX_RANDOM_ATTEMPTS",
            cg.max_attempts,
        )

        if self.use_factory:
            gripper = self.task_config.GRIPPERS[0]
            handle = self.task_config.HANDLES_PER_OBJECT[0][0]
            desired_grasp = f"{gripper} grasps {handle}"

            all_states = list(self.graph_builder.get_states().keys())
            goal_candidates = [s for s in all_states if s == desired_grasp]
            if not goal_candidates:
                goal_candidates = [s for s in all_states if desired_grasp in s]
            if not goal_candidates:
                raise RuntimeError(
                    "Could not find a grasp state containing '%s'."
                    % desired_grasp
                )
            goal_state = goal_candidates[0]

            # Minimal fallback: project directly into goal.
            cg.project_on_node(goal_state, q_init, "q_goal")
            cg.configs.setdefault("q_init", list(q_init))
            return cg.configs

        raise NotImplementedError(
            "Manual (non-factory) waypoint generation not implemented "
            "in template."
        )


def main(
    visualize: bool = True,
    solve: bool = False,
    show_joints: bool = False,
    use_factory: bool = False,
    backend: str = "corba",
):
    task = TemplateTask(use_factory=use_factory, backend=backend)

    task.setup(
        validation_step=getattr(
            task.task_config,
            "PATH_VALIDATION_STEP",
            0.01,
        ),
        projector_step=getattr(task.task_config, "PATH_PROJECTOR_STEP", 0.1),
    )

    if show_joints:
        from agimus_spacelab.visualization import print_joint_info

        print_joint_info(task.robot)

    result = task.run(
        visualize=visualize,
        solve=solve,
        preferred_configs=[],
        max_iterations=5000,
    )

    return task, result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=TASK_NAME)
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization",
    )
    parser.add_argument(
        "--solve",
        action="store_true",
        help="Solve planning problem",
    )
    parser.add_argument(
        "--show-joints",
        action="store_true",
        help="Print joint info",
    )
    parser.add_argument(
        "--factory",
        action="store_true",
        help="Use ConstraintGraphFactory (automatic graph generation)",
    )
    parser.add_argument(
        "--backend",
        type=str,
        default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use",
    )

    args = parser.parse_args()

    main(
        visualize=not args.no_viz,
        solve=args.solve,
        show_joints=args.show_joints,
        use_factory=args.factory,
        backend=args.backend,
    )
