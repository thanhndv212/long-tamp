#!/usr/bin/env python3
"""Load the ikea_table_config scene: viser view, FK check.

See README.md. Debug helper, not part of the task
pipeline. The only viewer script in this directory — it used to be three
(view_full_scene.py, view_arm_gripper.py, this file), all loading
overlapping scenes; consolidated here since debug_view_frames.py already
loaded the full scene (arms + environment + objects) via the YAML config
(the single source of truth config/ikea_table_config.yaml now is) and
just needed the other two's remaining unique behavior folded in:

  - view_full_scene.py's FK placement check (below): print each object's
    *actual* world position (via forward kinematics, not the raw q_init
    values) next to its YAML-declared target, so a placement bug shows up
    here rather than only being caught by eyeballing the viewer. Every
    object matched exactly the one time this was checked (see git
    history) — kept as a standing sanity check for future scene edits,
    not because a mismatch is expected.
  - view_arm_gripper.py's narrower "just the arm, no clutter" scene had
    no behavior this file doesn't already have as a strict superset
    (same arm+gripper, plus everything else) — nothing to fold in there.

Run inside the hpp-agimus-arm64 container: pyhpp only imports there (see
build_assets.py's docstring for the container-vs-host path split this
implies for every generated URDF/SRDF this loads).
"""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin

from long_tamp.tasks import ManipulationTask
from long_tamp.config.yaml_loader import YamlTaskLoader

HERE = Path(__file__).parent
_YAML_PATH = HERE / "config" / "ikea_table_config.yaml"
_loader = YamlTaskLoader(_YAML_PATH)


class _ViewTask(ManipulationTask):
    def __init__(self, backend: str = "pyhpp"):
        super().__init__(
            task_name="ikea_table_assembly (frame debug view)",
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
        )
        self.task_config = _loader.task_config

    def build_initial_config(self):
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


def _target_pose_xyzquat(obj_data: dict) -> list[float]:
    """Mirror YamlTaskLoader.build_initial_config's own pose handling."""
    if "initial_pose_xyzquat" in obj_data:
        return [float(v) for v in obj_data["initial_pose_xyzquat"]]
    from long_tamp.utils.transforms import xyzrpy_to_xyzquat

    rpy = [float(v) for v in obj_data["initial_pose_xyzrpy"]]
    return xyzrpy_to_xyzquat(rpy).tolist()


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _write_idle_pose_to_yaml(
    side: str, joint_ranks: list[tuple[str, int]], q: np.ndarray
) -> None:
    """Patch joint_groups' `initial:` field for `side`'s 6 arm joints.

    Regex substitution on the raw text rather than a yaml load+dump round
    trip, since the latter would strip this file's hand-written comments.
    Only touches the file on disk -- takes effect on the next run, not
    this one (the in-memory _loader/task_config were already parsed).
    """
    text = _YAML_PATH.read_text()
    for j, r in joint_ranks:
        pattern = re.compile(
            rf"(joint:\s*{re.escape(side)}/{re.escape(j)},\s*initial:\s*)"
            rf"[-+]?\d+\.?\d*"
        )
        value = float(q[r])
        new_text, n = pattern.subn(
            lambda m, v=value: f"{m.group(1)}{v:.4f}", text
        )
        if n != 1:
            print(
                f"✗ Aborting: expected exactly 1 match for {side}/{j}'s "
                f"initial: field, found {n}. YAML left unmodified."
            )
            return
        text = new_text
    _YAML_PATH.write_text(text)
    print(
        f"✓ Wrote {side}'s current pose as the new idle pose into "
        f"{_YAML_PATH.name} (takes effect on the next run)"
    )


def _add_arm_sliders(task: _ViewTask, q: np.ndarray) -> None:
    """Add a per-joint slider for each arm's 6 UR10 joints to the viser GUI.

    For manually hunting a better idle pose (see task_assemble_table.py's
    PER_PHASE_FROZEN_ARMS comment on why ur10_left's default pose needs
    one). Slider range comes from the YAML's own joint_groups bounds, so a
    slider can't drive a joint past what the scene allows.
    """
    import yaml

    gui = task.planner.viewer.viewer.gui
    rank = task.robot.rankInConfiguration
    raw_groups = yaml.safe_load(_YAML_PATH.read_text())["joint_groups"]

    bounds_by_joint = {
        entry["joint"]: entry["bounds"]
        for entries in raw_groups.values()
        for entry in entries
    }

    for side in ("ur10_left", "ur10_right"):
        joint_ranks = [(j, rank[f"{side}/{j}"]) for j in ARM_JOINTS]
        sliders: dict[str, Any] = {}

        with gui.add_folder(f"{side} arm"):
            for j, r in joint_ranks:
                lo, hi = bounds_by_joint[f"{side}/{j}"]
                slider = gui.add_slider(
                    j, min=lo, max=hi, step=0.01, initial_value=float(q[r])
                )
                sliders[j] = slider

                @slider.on_update
                def _on_update(_, r=r, slider=slider) -> None:
                    q[r] = slider.value
                    task.planner.visualize(q)

            print_btn = gui.add_button("Print pose")

            @print_btn.on_click
            def _on_click(_, side=side, joint_ranks=joint_ranks) -> None:
                lines = ", ".join(
                    f"{{joint: {side}/{j}, initial: {q[r]: .2f}}}"
                    for j, r in joint_ranks
                )
                print(f"{side} pose:\n  {lines}")

            set_idle_btn = gui.add_button("Set as idle pose")

            @set_idle_btn.on_click
            def _on_set_idle(_, side=side, joint_ranks=joint_ranks) -> None:
                _write_idle_pose_to_yaml(side, joint_ranks, q)

            plan_btn = gui.add_button("Plan to random config")

            @plan_btn.on_click
            def _on_plan(
                _, side=side, joint_ranks=joint_ranks, sliders=sliders
            ) -> None:
                _plan_random_arm_config(
                    task, q, side, joint_ranks, bounds_by_joint, sliders
                )


def _ensure_collision_validation(ps: Any) -> None:
    """Idempotently register collision + joint-bound config validation.

    ``Problem.isConfigValid()`` starts with zero registered validators and
    trivially returns ``(True, "")`` for anything, collisions included --
    see ``PyHPPBackend.random_config()``, which has the same guard. Path
    validation (set up during scene setup) is a separate registry and
    doesn't cover this.
    """
    cv = ps.configValidation()
    if (
        hasattr(cv, "numberConfigValidations")
        and cv.numberConfigValidations() == 0
    ):
        ps.addConfigValidation("CollisionValidation")
        ps.addConfigValidation("JointBoundValidation")


def _plan_random_arm_config(
    task: _ViewTask,
    q: np.ndarray,
    side: str,
    joint_ranks: list[tuple[str, int]],
    bounds_by_joint: dict[str, list[float]],
    sliders: dict[str, Any],
    n_steps: int = 60,
    max_attempts: int = 30,
) -> None:
    """Animate `side`'s 6 arm joints moving to a random valid configuration.

    Not a graph-based RRT plan -- this script runs with skip_graph=True,
    and every real planner entry point here is scoped to a manipulation-
    graph transition. Instead: draws a random target for just this arm
    (every other entry of q left untouched), validates a straight-line
    interpolation to it step-by-step via Problem.isConfigValid(), and
    redraws on failure. Finds a valid taut string through open space; does
    not route around an obstacle.
    """
    _ensure_collision_validation(task.ps)
    q_start = q.copy()

    for attempt in range(1, max_attempts + 1):
        q_goal = q_start.copy()
        for j, r in joint_ranks:
            lo, hi = bounds_by_joint[f"{side}/{j}"]
            q_goal[r] = np.random.uniform(lo, hi)

        steps = [
            q_start + (q_goal - q_start) * (i / n_steps)
            for i in range(n_steps + 1)
        ]
        if all(task.ps.isConfigValid(s)[0] for s in steps):
            for s in steps:
                q[:] = s
                task.planner.visualize(q)
                time.sleep(0.02)
            for j, r in joint_ranks:
                slider = sliders.get(j)
                if slider is not None:
                    slider.value = float(q[r])
            print(f"{side}: reached random config in {attempt} attempt(s)")
            return

    print(
        f"{side}: no collision-free straight-line target found in "
        f"{max_attempts} attempts (scene may be too cluttered for a "
        "straight-line move — this button doesn't route around obstacles)"
    )


# (joint, mimic multiplier relative to finger_joint) — ground truth from
# assets/robotiq_2f85/urdf/robotiq_arg2f_85_model_macro.xacro's own
# <mimic> tags. Both *_inner_finger_joint mimic with multiplier="-1"
# there, which is why they need the negated bounds in
# config/ikea_table_config.yaml (that xacro declares them all-positive
# regardless of multiplier — a bug confirmed against
# github.com/PickNikRobotics/ros2_robotiq_gripper's actively-maintained
# equivalent, which correctly flips a mimic="-1" joint's own <limit> sign
# too). finger_joint itself is the master, multiplier 1 by definition.
GRIPPER_MIMIC_JOINTS = [
    ("finger_joint", 1),
    ("left_inner_knuckle_joint", 1),
    ("left_inner_finger_joint", -1),
    ("right_outer_knuckle_joint", 1),
    ("right_inner_knuckle_joint", 1),
    ("right_inner_finger_joint", -1),
]


def _add_gripper_sliders(task: _ViewTask, q: np.ndarray) -> None:
    """Add an open/close slider per arm's gripper to the viser GUI.

    finger_joint is the Robotiq 2F-85's only real actuated DOF; the other
    5 are <mimic> joints in the vendored xacro (see build_assets.py's
    merge-arm stage) that this workspace drives as independent DOF — no
    <mimic> support anywhere in the HPP stack here. GRIPPER_MIMIC_JOINTS'
    multipliers reproduce that xacro's own mimic relationships, so this
    slider's "closed" end should curl all 5 in mechanically-consistent
    directions rather than driving any of them backwards.
    """
    gui = task.planner.viewer.viewer.gui
    rank = task.robot.rankInConfiguration

    for side in ("ur10_left", "ur10_right"):
        ranks_mults = [
            (rank[f"{side}/{j}"], mult) for j, mult in GRIPPER_MIMIC_JOINTS
        ]

        with gui.add_folder(f"{side} gripper"):
            slider = gui.add_slider(
                "Open (0) / Close (0.8)",
                min=0.0,
                max=0.8,
                step=0.01,
                initial_value=0.0,
            )

            @slider.on_update
            def _on_update(_, ranks_mults=ranks_mults, slider=slider) -> None:
                for r, mult in ranks_mults:
                    q[r] = mult * slider.value
                task.planner.visualize(q)


def _check_object_placement(task: _ViewTask) -> None:
    """FK-verify every object's root_joint against its YAML target pose.

    Not assumed from q_init: the loaded pinocchio model is queried
    directly, the same check view_full_scene.py used to do (see this
    file's own module docstring for why it moved here).
    """
    import yaml

    raw_objects = yaml.safe_load(_YAML_PATH.read_text())["objects"]

    model = task.robot.model()
    data = model.createData()
    q = np.asarray(task.q_init, dtype=float)
    pin.forwardKinematics(model, data, q)

    print("--- object placement check (FK, not assumed) ---")
    for name in task.task_config.OBJECTS:
        target = _target_pose_xyzquat(raw_objects[name])
        jid = model.getJointId(f"{name}/root_joint")
        actual = [round(float(v), 6) for v in data.oMi[jid].translation]
        print(f"{name:8s} target={target[:3]}  actual={actual}")


def main() -> None:
    task = _ViewTask(backend="pyhpp")
    task.setup(skip_graph=True)

    q_init = task.q_init
    task.planner.setup_viewer(viewer_type="viser")

    q = np.array(q_init, dtype=float)
    task.planner.visualize(q)
    _add_arm_sliders(task, q)
    _add_gripper_sliders(task, q)

    _check_object_placement(task)

    print("nq:", len(q_init))
    print("Viewer running — check the (viser) HTTP line above for the "
          "actual port. Ctrl-C to stop.")

    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
