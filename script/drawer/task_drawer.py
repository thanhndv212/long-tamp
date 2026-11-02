#!/usr/bin/env python3
"""Drawer open/close: a Panda opens a cabinet drawer and pushes it shut.

The cabinet is an articulated object (#161): anchored in the world by
``root: anchor`` in the YAML, with one prismatic joint, ``cabinet/drawer``.
The constraint-graph factory locks that joint whenever the handle is free,
at the value it was left at, and frees it while the gripper holds the
handle, so the drawer moves only with the hand.

The run is six primitives of GraspSequencePlanner:

    grasp -> plan_loop (pull the drawer open) -> release
    grasp -> plan_loop (push it shut)         -> release

Each ``plan_loop`` target keeps the grasp: the drawer joint moves by the
opening and the arm follows by IK, so the TCP stays on the handle.

Run:
    python script/drawer/task_drawer.py
    python script/drawer/task_drawer.py --open 0.12 --no-viewer
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

from long_tamp.config.yaml_loader import YamlTaskLoader
from long_tamp.eval.robocerebra_plan import PandaKinematics, robot_files
from long_tamp.tasks import ManipulationTask
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

TASK_NAME = "Drawer open/close"

_HERE = Path(__file__).parent
_YAML_PATH = _HERE / "config" / "drawer_config.yaml"
_TWIN_ASSETS = _HERE.parent / "twin" / "assets"

GRIPPER = "panda/gripper"
HANDLE = "cabinet/handle"
DRAWER = "cabinet/drawer"
# The drawer slides along the cabinet's +x, which its pose turns to -x in
# the world: towards the robot.
OPEN_DIRECTION = np.array([-1.0, 0.0, 0.0])
OPEN_DEFAULT = 0.15  # m, of the joint's 0.2

_loader = YamlTaskLoader(_YAML_PATH)
_PANDA_URDF, _PANDA_SRDF = robot_files(_TWIN_ASSETS)
_loader.file_paths["robot"]["panda"]["urdf"] = _PANDA_URDF
_loader.file_paths["robot"]["panda"]["srdf"] = _PANDA_SRDF


class DrawerTask(ManipulationTask):
    """One Panda and a one-drawer cabinet, from YAML."""

    def __init__(self, viewer_type: str = "none"):
        super().__init__(
            task_name=TASK_NAME,
            backend="pyhpp",
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
            viewer_type=viewer_type,
            log_dir=None,
        )
        self.task_config = _loader.task_config
        self.use_factory = True

    def build_initial_config(self) -> list[float]:
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


class DrawerRun:
    """The task, its planner, and the six steps of an open/close run."""

    def __init__(self, timeout: float = 30.0, viewer_type: str = "none"):
        self.task = DrawerTask(viewer_type=viewer_type)
        self.task.setup(skip_graph=True)
        self.sequence = GraspSequencePlanner(
            graph_builder=self.task.graph_builder,
            config_gen=self.task.config_gen,
            planner=self.task.planner,
            task_config=self.task.task_config,
            backend="pyhpp",
            graph_constraints=self.task._graph_constraints,
            auto_save_dir=None,
        )
        self.kin = PandaKinematics(_PANDA_URDF, [0, 0, 0, 0, 0, 0, 1])
        model = self.task.planner.model()
        self._arm = [
            model.joints[model.getJointId(f"panda/panda_joint{k}")].idx_q
            for k in range(1, 8)
        ]
        self._drawer = model.joints[model.getJointId(DRAWER)].idx_q
        self.q0 = list(self.task.q_init)
        self.timeout = timeout

    def drawer(self, q: list[float]) -> float:
        """The drawer's opening in ``q``, in metres."""
        return float(q[self._drawer])

    def slid(self, q: list[float], opening: float) -> list[float] | None:
        """``q`` with the drawer at ``opening`` and the hand still on the
        handle; ``None`` if IK fails."""
        arm = np.asarray(q, dtype=float)[self._arm]
        T = self.kin.tcp(arm)
        T[:3, 3] += (opening - self.drawer(q)) * OPEN_DIRECTION
        arm = self.kin.ik(arm, T)
        if arm is None:
            return None
        out = list(q)
        for i, value in zip(self._arm, arm):
            out[i] = float(value)
        out[self._drawer] = opening
        return out

    def _kwargs(self) -> dict[str, Any]:
        return {
            "q_scene_init": self.q0,
            "timeout_per_edge": self.timeout,
            "verbose": False,
        }

    def grasp(self, q: list[float]) -> dict[str, Any]:
        return self.sequence.grasp(GRIPPER, HANDLE, q, **self._kwargs())

    def slide(self, q: list[float], opening: float) -> dict[str, Any]:
        target = self.slid(q, opening)
        if target is None:
            return {"success": False, "message": f"no IK to opening {opening}"}
        return self.sequence.plan_loop(GRIPPER, q, target, **self._kwargs())

    def release(self, q: list[float]) -> dict[str, Any]:
        return self.sequence.release(GRIPPER, q, verbose=False)

    def run(self, opening: float = OPEN_DEFAULT) -> dict[str, Any]:
        """Open the drawer to ``opening`` and close it again.

        Returns ``success``, the failed step's ``message``, and per step the
        drawer's opening where it ended (``steps``)."""
        q, steps = list(self.q0), []
        for name, step in (
            ("grasp", self.grasp),
            ("open", lambda q: self.slide(q, opening)),
            ("release", self.release),
            ("grasp", self.grasp),
            ("close", lambda q: self.slide(q, 0.0)),
            ("release", self.release),
        ):
            result = step(q)
            if not result["success"]:
                return {
                    "success": False,
                    "message": f"{name}: {result.get('message')}",
                    "steps": steps,
                }
            q = list(result["final_config"])
            steps.append((name, round(self.drawer(q), 4)))
        return {"success": True, "message": None, "steps": steps, "final_config": q}


def main() -> int:
    parser = argparse.ArgumentParser(description=TASK_NAME)
    parser.add_argument("--open", type=float, default=OPEN_DEFAULT)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--no-viz", "--no-viewer", action="store_true")
    parser.add_argument("--viewer-port", type=int, default=8081)
    args = parser.parse_args()

    run = DrawerRun(timeout=args.timeout)
    result = run.run(args.open)
    for name, opening in result["steps"]:
        print(f"  {name:8s} drawer at {opening:.3f} m")
    if not result["success"]:
        print(f"✗ {result['message']}")
        return 1
    print("✓ drawer opened and closed")

    if not args.no_viz:
        from long_tamp.visualization.mission_viewer import MissionViewer

        viewer = MissionViewer(run.task, args.viewer_port, run.q0)
        try:
            viewer.completed(run.sequence.phase_results)
            viewer.finish()
        finally:
            viewer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
