"""Articulated objects (#161): anchored objects whose joints move.

An object declared ``root: anchor`` in a YAML config is fixed at its
initial pose; its prismatic and revolute joints (a drawer's slide, a door's
hinge) are its configuration. The unit tests cover the loader, the scene
builder and the task's initial configuration without HPP; the PyHPP tests
load script/drawer's cabinet, and the slow one opens and closes its drawer.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from long_tamp.config.yaml_loader import YamlTaskLoader
from long_tamp.planning.scene import SceneBuilder
from long_tamp.tasks.base import ManipulationTask

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_DRAWER_DIR = Path(__file__).resolve().parent.parent / "script" / "drawer"

_CABINET_URDF = """<?xml version="1.0"?>
<robot name="cabinet">
  <link name="base_link"/>
  <link name="top"/>
  <link name="bottom"/>
  <link name="door"/>
  <link name="plate"/>
  <joint name="top_drawer" type="prismatic">
    <parent link="base_link"/><child link="top"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="0.2" effort="1" velocity="1"/>
  </joint>
  <joint name="bottom_drawer" type="prismatic">
    <parent link="base_link"/><child link="bottom"/>
    <axis xyz="1 0 0"/><limit lower="-0.005" upper="0.16" effort="1" velocity="1"/>
  </joint>
  <joint name="door" type="revolute">
    <parent link="base_link"/><child link="door"/>
    <axis xyz="0 0 1"/><limit lower="-2.094" upper="-0.1" effort="1" velocity="1"/>
  </joint>
  <joint name="plate" type="fixed">
    <parent link="door"/><child link="plate"/>
  </joint>
</robot>
"""


def _loader(tmp_path: Path, cabinet: dict, urdf: str = _CABINET_URDF):
    (tmp_path / "cabinet.urdf").write_text(urdf)
    config = {
        "paths": {
            "robot": {"arm": {"urdf": "arm.urdf", "srdf": "arm.srdf"}},
            "environment": {},
            "objects": {
                "cabinet": {"urdf": "cabinet.urdf", "srdf": ""},
                "box": {"urdf": "box.urdf", "srdf": ""},
            },
        },
        "joint_groups": {
            "ARM": [{"joint": "arm/j1", "initial": 0.5, "bounds": [-1, 1]}]
        },
        "objects": {
            "box": {"initial_pose_xyzquat": [0.1, 0.2, 0.3, 0, 0, 0, 1]},
            "cabinet": cabinet,
        },
    }
    path = tmp_path / "task.yaml"
    path.write_text(yaml.safe_dump(config))
    return YamlTaskLoader(path)


_ANCHORED = {
    "root": "anchor",
    "initial_pose_xyzquat": [0.75, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    "joints": {"cabinet/top_drawer": 0.1, "door": -1.0},
    "handles": ["cabinet/handle"],
}


class TestLoader:
    def test_anchored_object_is_loaded_at_its_pose(self, tmp_path):
        paths = _loader(tmp_path, _ANCHORED).file_paths["objects"]
        assert paths["cabinet"]["root_joint_type"] == "anchor"
        assert paths["cabinet"]["pose"] == [0.75, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        assert "root_joint_type" not in paths["box"]

    def test_every_movable_joint_gets_a_value(self, tmp_path):
        joints = _loader(tmp_path, _ANCHORED).articulated_joints
        # The fixed joint is left out; bottom_drawer is not given and starts
        # at 0, inside its limits.
        assert joints == {
            "cabinet": {
                "cabinet/top_drawer": 0.1,
                "cabinet/bottom_drawer": 0.0,
                "cabinet/door": -1.0,
            }
        }

    def test_a_joint_left_out_is_clamped_into_its_limits(self, tmp_path):
        joints = _loader(tmp_path, {**_ANCHORED, "joints": {}}).articulated_joints
        assert joints["cabinet"]["cabinet/door"] == -0.1

    def test_initial_config_has_one_value_per_joint(self, tmp_path):
        loader = _loader(tmp_path, _ANCHORED)
        q = loader.build_initial_config(objects=["box", "cabinet"])
        # arm joint, box pose, then the cabinet's three joints (not a pose)
        assert q == [0.5, 0.1, 0.2, 0.3, 0, 0, 0, 1, 0.1, 0.0, -1.0]

    def test_task_config_lists_the_joints(self, tmp_path):
        config = _loader(tmp_path, _ANCHORED).task_config
        assert config.ARTICULATED_JOINTS["cabinet"]["cabinet/top_drawer"] == 0.1
        assert config.OBJECTS == ["box", "cabinet"]

    def test_freeflyer_objects_are_unchanged(self, tmp_path):
        loader = _loader(tmp_path, {"initial_pose_xyzquat": [1, 2, 3, 0, 0, 0, 1]})
        assert loader.articulated_joints == {}
        assert loader.build_initial_config()[-7:] == [1, 2, 3, 0, 0, 0, 1]

    @pytest.mark.parametrize(
        "cabinet, match",
        [
            ({**_ANCHORED, "joints": {"lid": 0.0}}, "not a movable joint"),
            ({**_ANCHORED, "joints": {"top_drawer": 0.5}}, "outside its limits"),
            ({**_ANCHORED, "root": "planar"}, "root must be"),
        ],
    )
    def test_bad_declarations_raise(self, tmp_path, cabinet, match):
        with pytest.raises(ValueError, match=match):
            _loader(tmp_path, cabinet).articulated_joints

    def test_a_continuous_joint_is_rejected(self, tmp_path):
        urdf = _CABINET_URDF.replace(
            'name="door" type="revolute"', 'name="door" type="continuous"'
        )
        with pytest.raises(ValueError, match="continuous"):
            _loader(tmp_path, _ANCHORED, urdf).articulated_joints


class _Planner:
    def __init__(self):
        self.loaded, self.bounded = [], []

    def load_object(self, **kwargs):
        self.loaded.append(kwargs)

    def set_joint_bounds(self, joint, bounds):
        self.bounded.append(joint)


def _scene(objects):
    scene = SceneBuilder.__new__(SceneBuilder)
    scene.FILE_PATHS = {"objects": objects}
    scene.planner = _Planner()
    scene.loaded_objects, scene.freeflyer_objects = [], []
    scene.joint_bounds = SimpleNamespace(
        all_robot_bounds=lambda: {}, freeflyer_bounds=lambda: [0, 1] * 7
    )
    return scene


class TestSceneBuilder:
    def test_anchored_object_loads_with_its_root_and_pose(self):
        scene = _scene(
            {
                "box": {"urdf": "box.urdf", "srdf": ""},
                "cabinet": {
                    "urdf": "cabinet.urdf",
                    "srdf": "",
                    "root_joint_type": "anchor",
                    "pose": [0.75, 0, 0, 0, 0, 1, 0],
                },
            }
        )
        scene.load_objects(["box", "cabinet"]).set_joint_bounds()
        box, cabinet = scene.planner.loaded
        assert box["root_joint_type"] == "freeflyer" and "pose" not in box
        assert cabinet["root_joint_type"] == "anchor"
        np.testing.assert_allclose(cabinet["pose"].translation, [0.75, 0, 0])
        # Only the free flyer has a root_joint to bound.
        assert scene.planner.bounded == ["box/root_joint"]


class TestInitialConfig:
    def _task(self, articulated, objects=("cabinet",)):
        return SimpleNamespace(
            task_config=SimpleNamespace(
                ARTICULATED_JOINTS=articulated, OBJECTS=list(objects)
            ),
            robot=SimpleNamespace(
                rankInConfiguration={"arm/j1": 0, "cabinet/b": 1, "cabinet/a": 2}
            ),
        )

    def test_values_go_to_their_joints_ranks(self):
        # Listed a, b (URDF order); the model ranks b before a.
        task = self._task({"cabinet": {"cabinet/a": 0.1, "cabinet/b": 0.2}})
        q = ManipulationTask._place_articulated_joints(task, [0.5, 0.1, 0.2])
        assert q == [0.5, 0.2, 0.1]

    def test_unloaded_objects_are_skipped(self):
        task = self._task({"cabinet": {"cabinet/a": 0.1}}, objects=())
        assert ManipulationTask._place_articulated_joints(task, [0.5]) == [0.5]

    def test_a_joint_missing_from_the_model_raises(self):
        task = self._task({"cabinet": {"cabinet/c": 0.1}})
        with pytest.raises(ValueError, match="not in the model"):
            ManipulationTask._place_articulated_joints(task, [0.5, 0.0, 0.0])


def _drawer_module():
    if str(_DRAWER_DIR) not in sys.path:
        sys.path.insert(0, str(_DRAWER_DIR))
    import task_drawer  # noqa: PLC0415 -- test-local, path-dependent import

    return task_drawer


@requires_pyhpp
class TestDrawerScene:
    def test_cabinet_is_anchored_with_its_drawer_joint(self):
        import pinocchio as pin

        run = _drawer_module().DrawerRun()
        model = run.task.planner.model()
        assert not model.existJointName("cabinet/root_joint")
        assert model.joints[model.getJointId("cabinet/drawer")].nq == 1
        assert len(run.q0) == model.nq
        assert run.drawer(run.q0) == 0.0
        data = model.createData()
        pin.framesForwardKinematics(model, data, np.asarray(run.q0))
        base = data.oMf[model.getFrameId("cabinet/base_link")]
        np.testing.assert_allclose(base.translation, [0.75, 0, 0], atol=1e-9)

    @pytest.mark.slow_planning
    def test_drawer_opens_and_closes(self):
        drawer = _drawer_module()
        run = drawer.DrawerRun()
        result = run.run(drawer.OPEN_DEFAULT)
        assert result["success"], result["message"]
        assert [opening for _, opening in result["steps"]] == pytest.approx(
            [0.0, 0.15, 0.15, 0.15, 0.0, 0.0], abs=1e-3
        )
        # The drawer moves only while held: grasps and releases leave it where
        # it was, and while it slides the hand stays on the handle.
        phases = run.sequence.phase_results
        assert len(phases) == 6
        for name, phase in zip(("grasp", "open", "release", "grasp", "close"), phases):
            if name in ("open", "close"):
                continue
            for path in phase["paths"]:
                openings = _configs(run, path)[:, run._drawer]
                assert np.ptp(openings) < 1e-6, f"the drawer moved during {name}"
        handle = _handle_offsets(run, phases[1]) + _handle_offsets(run, phases[4])
        assert max(handle) < 1e-3


def _configs(run, path, samples=60):
    if isinstance(path, int):
        path = run.task.planner.get_path(path)
    t0 = 0.0
    if hasattr(path, "timeRange"):
        tr = path.timeRange()
        t0 = tr.first if hasattr(tr, "first") else tr[0]
    out = []
    for i in range(samples):
        q, ok = path.eval(t0 + path.length() * i / (samples - 1))
        if ok:
            out.append(np.asarray(q, dtype=float))
    return np.array(out)


def _handle_offsets(run, phase):
    """Distances between the TCP and the handle along ``phase``'s paths."""
    import pinocchio as pin

    model = run.task.planner.model()
    data = model.createData()
    tcp = model.getFrameId("panda/panda_hand_tcp")
    drawer = model.getFrameId("cabinet/drawer_link")
    handle = np.array([0.207, 0.0, 0.6])  # cabinet.srdf, in the drawer's frame
    out = []
    for path in phase["paths"]:
        for q in _configs(run, path):
            pin.framesForwardKinematics(model, data, q)
            target = data.oMf[drawer].act(handle)
            out.append(float(np.linalg.norm(data.oMf[tcp].translation - target)))
    return out
