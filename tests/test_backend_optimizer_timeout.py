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
