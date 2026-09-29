"""The screw-assembly scene exported to MuJoCo (#17).

Needs the ``sim`` extra (MuJoCo, trimesh); the kinematics check also needs the
PyHPP backend. Both skip otherwise.
"""

import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("trimesh")

from long_tamp.sim.mjcf import export_mjcf, fk_mismatch

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

SCREW_DIR = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"
CONFIG = SCREW_DIR / "config" / "screw_assembly_config.yaml"


@pytest.fixture(scope="module")
def export(tmp_path_factory):
    return export_mjcf(CONFIG, tmp_path_factory.mktemp("mjcf"))


def test_the_scene_keeps_hpp_names(export):
    model = export.load()
    for name in (
        "ur10_left/shoulder_pan_joint",
        "ur10_right/wrist_3_joint",
        "ur10_right/robotiq_85_left_knuckle_joint",
    ):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
    for name in ("ur10_left/tool0", "fixtures/base_link", "part1/base_link"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0


def test_objects_are_free_at_their_initial_pose(export):
    model = export.load()
    assert set(export.free_joints) >= {"part1", "driver"}
    joint = model.joint(export.free_joints["driver"])
    assert joint.type == mujoco.mjtJoint.mjJNT_FREE
    adr = model.jnt_qposadr[joint.id]
    # driver: initial_pose_xyzquat [1.08, 0.36, 0.436, 0, 0, 1, 0] (xyzw)
    np.testing.assert_allclose(
        model.qpos0[adr : adr + 7], [1.08, 0.36, 0.436, 0, 0, 0, 1]
    )


def test_mimic_joints_become_equalities(export):
    model = export.load()
    # 5 mimic joints per Robotiq 2F-85 (PickNik), two grippers.
    assert len(export.mimics) == 10
    assert model.neq == 10
    assert all(model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT for i in range(model.neq))


def test_the_export_folder_can_move(export, tmp_path):
    moved = tmp_path / "moved"
    shutil.copytree(export.path.parent, moved)
    model = mujoco.MjModel.from_xml_path(str(moved / export.path.name))
    assert model.nmesh > 0


def test_the_scene_steps_under_gravity(export):
    model = export.load()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()


@pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")
def test_kinematics_match_hpp(export, tmp_path):
    import pinocchio as pin

    if str(SCREW_DIR) not in sys.path:
        sys.path.insert(0, str(SCREW_DIR))
    import task_screw_assembly as T

    task, _ = T.setup(log_dir=str(tmp_path))
    pin_model = task.robot.model()
    model = export.load()

    errors = fk_mismatch(model, pin_model, np.array(task.q_init))
    # Every link of both arms, the fixtures and every object is compared.
    assert len(errors) >= 50
    assert max(errors.values()) < 1e-6

    pin.seed(0)
    lower = np.maximum(pin_model.lowerPositionLimit, -3.0)
    upper = np.minimum(pin_model.upperPositionLimit, 3.0)
    for _ in range(20):
        q = pin.randomConfiguration(pin_model, lower, upper)
        assert max(fk_mismatch(model, pin_model, q).values()) < 1e-6
