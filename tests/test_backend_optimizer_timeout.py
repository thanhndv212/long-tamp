"""configure_transition_planner(path_optimizer_timeout=...) sets the cap."""

import pytest

from long_tamp.backends import PyHPPBackend
from long_tamp.backends.pyhpp import HAS_PYHPP

pytestmark = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")


def test_default_is_30_seconds():
    assert PyHPPBackend()._path_optimizer_timeout == 30.0


def test_configure_overrides_it_and_leaves_it_alone_when_omitted():
    backend = PyHPPBackend()
    backend.configure_transition_planner(path_optimizer_timeout=5)
    assert backend._path_optimizer_timeout == 5.0
    backend.configure_transition_planner(time_out=10.0)
    assert backend._path_optimizer_timeout == 5.0


def test_spline_optimizer_can_be_dropped_from_every_profile():
    backend = PyHPPBackend()
    lists = (
        "_transit_edge_optimizers",
        "_waypoint_pregrasp_optimizers",
        "_waypoint_grasp_optimizers",
        "_transition_default_optimizers",
    )
    assert any(
        n.startswith("SplineGradientBased")
        for attr in lists
        for n in getattr(backend, attr)
    )
    backend.configure_transition_planner(spline_optimizer=False)
    for attr in lists:
        assert not any(
            n.startswith("SplineGradientBased") for n in getattr(backend, attr)
        ), attr
        assert getattr(backend, attr), f"{attr} must keep its shortcut optimizers"
