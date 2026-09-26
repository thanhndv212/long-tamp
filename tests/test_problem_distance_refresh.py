"""The problem's distance must cover every joint loaded after the first robot.

``Problem(device)`` builds its ``WeighedDistance`` from the device as it is
when the problem is created -- in ``PyHPPBackend.load_robot``, right after
the *first* robot. Joints loaded afterwards (a second robot, every object's
freeflyer) got no weight, so distances read uninitialized memory. When that
came out NaN the roadmap's nearest-node search found nothing: RRT planning
failed on "Maximal number of iterations" within seconds, or
``Roadmap::addNode`` dereferenced the null nearest node and segfaulted at the
"target generated -> path planning begins" transition -- intermittently,
since it depended on memory layout.
"""

from pathlib import Path

import numpy as np
import pytest

from long_tamp.backends import PyHPPBackend
from long_tamp.backends.pyhpp import HAS_PYHPP

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_SCRIPT = Path(__file__).parents[1] / "script"
# Resolvable in the HPP container (mesh paths are container-absolute).
_ARM = _SCRIPT / "ikea_table_prototype" / "generated" / "ur10_robotiq.container.urdf"
_BALL = _SCRIPT / "twin" / "assets"


def _scene():
    from pinocchio import SE3

    backend = PyHPPBackend()
    backend.load_robot(robot_name="arm_left", urdf_path=str(_ARM), srdf_path="")
    pose = SE3.Identity()
    pose.translation = np.array([1.5, 0.0, 0.0])
    backend.load_robot(
        robot_name="arm_right", urdf_path=str(_ARM), srdf_path="", pose=pose
    )
    backend.load_object(
        "ball",
        str(_BALL / "pokeball_bimanual.urdf"),
        str(_BALL / "pokeball_bimanual.srdf"),
    )
    return backend


def _neutral(backend):
    import pinocchio

    return np.asarray(pinocchio.neutral(backend.device.model()), dtype=float)


@requires_pyhpp
def test_distance_is_zero_between_a_configuration_and_itself():
    backend = _scene()
    q = _neutral(backend)
    assert backend.problem.distance().compute(q, q) == 0.0


@requires_pyhpp
def test_distance_is_finite_for_joints_loaded_after_the_problem():
    backend = _scene()
    q0 = _neutral(backend)
    q1 = q0.copy()
    q1[-7] += 0.2  # the object's freeflyer: loaded last, after the problem
    d = backend.problem.distance().compute(q0, q1)
    assert np.isfinite(d) and d > 0.0
