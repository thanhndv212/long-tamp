#!/usr/bin/env python3
"""Load the ikea_table_config scene: viser view, FK check.

Local prototype only — see README.md. Debug helper, not part of the task
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

import time
from pathlib import Path

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
    task.planner.visualize(q_init)

    _check_object_placement(task)

    print("nq:", len(q_init))
    print("Viewer running — check the (viser) HTTP line above for the "
          "actual port. Ctrl-C to stop.")

    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
