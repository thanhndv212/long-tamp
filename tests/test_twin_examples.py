"""
Tests for the TWIN-style bimanual examples (script/twin/) and the
underlying multi-independent-robot loading fix (`pose` param on
`BackendBase.load_robot`) they depend on.

The dual-robot tests here need `hpp_practicals`'s `package://` resources
to resolve, which this ament-based HPP build does via the
`share/ament_index/resource_index/packages/<name>` convention — point
`AMENT_PREFIX_PATH` at a prefix providing that for `hpp_practicals` (see
`script/twin/README.md` for the one-time dev-environment setup). Tests
skip cleanly, rather than failing, when that isn't configured.
"""

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP, PyHPPBackend
except ImportError:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_UR5_URDF = "package://hpp_practicals/urdf/ur5_gripper.urdf"


def _load_two_ur5s(pose_right=None):
    """Load two independent hpp_practicals UR5s, or return None if the
    `hpp_practicals` package can't be resolved in this environment."""
    backend = PyHPPBackend()
    try:
        backend.load_robot(robot_name="ur5_left", urdf_path=_UR5_URDF, srdf_path="")
    except Exception:
        return None
    backend.load_robot(
        robot_name="ur5_right", urdf_path=_UR5_URDF, srdf_path="", pose=pose_right
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
        try:
            backend.load_robot(robot_name="ur5_left", urdf_path=_UR5_URDF, srdf_path="")
        except Exception:
            pytest.skip("hpp_practicals package:// not resolvable in this environment")
        device_after_first = backend.device

        backend.load_robot(
            robot_name="ur5_right",
            urdf_path=_UR5_URDF,
            srdf_path="",
            pose=SE3(np.eye(3), np.array([0.7, 0.0, 0.0])),
        )

        assert backend.device is device_after_first, (
            "a second load_robot() call must insert into the existing "
            "composite device, not replace it"
        )

    def test_joint_names_are_namespaced_per_robot(self):
        backend = _load_two_ur5s()
        if backend is None:
            pytest.skip("hpp_practicals package:// not resolvable in this environment")

        names = list(backend.device.model().names)
        left = [n for n in names if n.startswith("ur5_left/")]
        right = [n for n in names if n.startswith("ur5_right/")]

        assert len(left) == 6, f"expected 6 ur5_left/* joints, got {left}"
        assert len(right) == 6, f"expected 6 ur5_right/* joints, got {right}"

    def test_pose_places_the_second_robots_root(self):
        from pinocchio import SE3
        import numpy as np

        offset = np.array([0.7, 0.0, 0.0])
        backend = _load_two_ur5s(pose_right=SE3(np.eye(3), offset))
        if backend is None:
            pytest.skip("hpp_practicals package:// not resolvable in this environment")

        model = backend.device.model()
        left_placement = model.jointPlacements[model.getJointId("ur5_left/shoulder_pan_joint")]
        right_placement = model.jointPlacements[model.getJointId("ur5_right/shoulder_pan_joint")]

        # Both share the URDF-native z offset; only the right arm carries
        # the extra world-pose translation composed on top of it.
        assert np.allclose(right_placement.translation - left_placement.translation, offset)
