"""
Tests for the TWIN-style bimanual examples (script/twin/) and the
underlying multi-independent-robot loading fix (`pose` param on
`BackendBase.load_robot`) they depend on.

The dual-robot tests load two copies of the repo's own vendored UR10
(script/ikea_table_prototype/assets/ur10/official/), so they need no
external robot-description package — only the PyHPP backend.
"""

import tempfile
from pathlib import Path

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP, PyHPPBackend
except ImportError:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_UR10_DIR = (
    Path(__file__).resolve().parent.parent
    / "script" / "ikea_table_prototype" / "assets" / "ur10" / "official"
)


def _render_ur10_urdf() -> str:
    """Write the vendored UR10 URDF to a temp file with absolute mesh paths.

    Its mesh paths are relative to the URDF's own directory, but pinocchio's
    URDF loader resolves non-`package://` paths against the process's CWD
    (same reason script/twin/task_lift_ball.py renders its Panda URDF).
    """
    text = (_UR10_DIR / "ur10.urdf").read_text()
    text = text.replace('filename="meshes/', f'filename="{_UR10_DIR}/meshes/')
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", prefix="ur10_", delete=False
    )
    tmp.write(text)
    tmp.close()
    return tmp.name


_UR10_URDF = _render_ur10_urdf()


def _load_two_ur10s(pose_right=None):
    """Load two independent UR10s into one PyHPP scene."""
    backend = PyHPPBackend()
    backend.load_robot(robot_name="ur10_left", urdf_path=_UR10_URDF, srdf_path="")
    backend.load_robot(
        robot_name="ur10_right", urdf_path=_UR10_URDF, srdf_path="", pose=pose_right
    )
    return backend


@requires_pyhpp
class TestMultiRobotLoading:
    """Loading two independent robots into one PyHPP scene (Step 0 fix).

    Before the fix, `PyHPPBackend.load_robot` unconditionally recreated
    `self.device`/`self.problem` on every call, so a second robot silently
    replaced the first instead of being inserted into the same composite
    device.
    """

    def test_second_robot_does_not_replace_the_first(self):
        from pinocchio import SE3
        import numpy as np

        backend = PyHPPBackend()
        backend.load_robot(robot_name="ur10_left", urdf_path=_UR10_URDF, srdf_path="")
        device_after_first = backend.device

        backend.load_robot(
            robot_name="ur10_right",
            urdf_path=_UR10_URDF,
            srdf_path="",
            pose=SE3(np.eye(3), np.array([0.7, 0.0, 0.0])),
        )

        assert backend.device is device_after_first, (
            "a second load_robot() call must insert into the existing "
            "composite device, not replace it"
        )

    def test_joint_names_are_namespaced_per_robot(self):
        backend = _load_two_ur10s()

        names = list(backend.device.model().names)
        left = [n for n in names if n.startswith("ur10_left/")]
        right = [n for n in names if n.startswith("ur10_right/")]

        assert len(left) == 6, f"expected 6 ur10_left/* joints, got {left}"
        assert len(right) == 6, f"expected 6 ur10_right/* joints, got {right}"

    def test_pose_places_the_second_robots_root(self):
        from pinocchio import SE3
        import numpy as np

        offset = np.array([0.7, 0.0, 0.0])
        backend = _load_two_ur10s(pose_right=SE3(np.eye(3), offset))

        model = backend.device.model()
        left_placement = model.jointPlacements[model.getJointId("ur10_left/shoulder_pan_joint")]
        right_placement = model.jointPlacements[model.getJointId("ur10_right/shoulder_pan_joint")]

        # Both share the URDF-native z offset; only the right arm carries
        # the extra world-pose translation composed on top of it.
        assert np.allclose(right_placement.translation - left_placement.translation, offset)
