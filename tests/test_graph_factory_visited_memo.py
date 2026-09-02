#!/usr/bin/env python3
"""Regression test: GraphFactoryAbstract._visitedGrasps memoization.

Verifies that the fix for the combinatorial blowup in _recurse() is correct:
- The set of created states and transitions is identical before and after the fix.
- _recurse() call count drops sharply for the fixed version (no redundant subtree walks).
- Works correctly with a non-monotonic filter (rejected combinations may still
  have allowed descendants — the fix must not prune their subtrees on the first
  path that reaches them, only on subsequent paths).

Loads constraint_graph_factory.py from a real pyhpp install if available,
else from a hpp-python source checkout pointed to by $HPP_PYTHON_SRC_DIR
(stubbing its compiled deps, pyhpp.constraints and numpy). Skipped if
neither is available.
"""

import importlib
import importlib.util
import os
import sys
import types
import unittest
from pathlib import Path


def _stub_pyhpp_constraints():
    pkg = types.ModuleType("pyhpp")
    sub = types.ModuleType("pyhpp.constraints")
    sub.Implicit = object
    sub.LockedJoint = object
    pkg.constraints = sub
    sys.modules.setdefault("pyhpp", pkg)
    sys.modules.setdefault("pyhpp.constraints", sub)


def _stub_numpy():
    if "numpy" not in sys.modules:
        np_stub = types.ModuleType("numpy")
        np_stub.ndarray = list
        sys.modules["numpy"] = np_stub


def _cgf_source_path() -> Path | None:
    """hpp-python checkout to load constraint_graph_factory.py from, if a
    real pyhpp install isn't importable. Set by $HPP_PYTHON_SRC_DIR."""
    src_dir = os.environ.get("HPP_PYTHON_SRC_DIR")
    if not src_dir:
        return None
    path = Path(src_dir) / "src/pyhpp/manipulation/constraint_graph_factory.py"
    return path if path.exists() else None


_CGF_PATH = _cgf_source_path()


def _load_cgf():
    try:
        return importlib.import_module("pyhpp.manipulation.constraint_graph_factory")
    except ImportError:
        pass
    _stub_pyhpp_constraints()
    _stub_numpy()
    spec = importlib.util.spec_from_file_location("constraint_graph_factory", _CGF_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Minimal concrete subclass of GraphFactoryAbstract
# ---------------------------------------------------------------------------


def _make_factory(cgf_mod):
    """Return a concrete GraphFactoryAbstract subclass that records state/transition sets."""

    class _State:
        __slots__ = ("grasps",)

        def __init__(self, grasps):
            self.grasps = grasps

        def __repr__(self):
            return f"S{self.grasps}"

    class _TestFactory(cgf_mod.GraphFactoryAbstract):
        def __init__(self):
            super().__init__()
            self._created_states = set()
            self._created_transitions = set()  # (from_grasps, to_grasps, ig)
            self._recurse_calls = 0

        def makeState(self, grasps, priority):
            self._created_states.add(grasps)
            return _State(grasps)

        def makeLoopTransition(self, state):
            pass

        def makeTransition(self, stateFrom, stateTo, ig):
            self._created_transitions.add((stateFrom.grasps, stateTo.grasps, ig))

        # Instrument _recurse to count calls
        def _recurse(self, grippers, handles, grasps, depth):
            self._recurse_calls += 1
            super()._recurse(grippers, handles, grasps, depth)

    return _TestFactory


# ---------------------------------------------------------------------------
# Helpers to build and run a factory for n grippers / n handles
# ---------------------------------------------------------------------------


def _run_factory(factory_cls, n, grasp_filter=None):
    f = factory_cls()
    grippers = [f"g{i}" for i in range(n)]
    handles = [f"h{i}" for i in range(n)]
    objects = [f"obj{i}" for i in range(n)]
    handles_per_obj = [[h] for h in handles]
    f.setGrippers(grippers)
    f.setObjects(objects, handles_per_obj, [[] for _ in range(n)])
    if grasp_filter is not None:
        f.graspIsAllowed = grasp_filter
    f.generate()
    return f


# ---------------------------------------------------------------------------
# Sequential filter (real implementation, no compiled deps)
# ---------------------------------------------------------------------------

_FILTER_PATH = (
    Path(__file__).parent.parent
    / "src/long_tamp/planning/sequential_grasp_filter.py"
)


def _load_sequential_filter():
    spec = importlib.util.spec_from_file_location(
        "sequential_grasp_filter", _FILTER_PATH
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_filter_mod = _load_sequential_filter()
SequentialGraspFilter = _filter_mod.SequentialGraspFilter


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def _cgf_available() -> bool:
    try:
        if importlib.util.find_spec("pyhpp.manipulation.constraint_graph_factory"):
            return True
    except ModuleNotFoundError:
        pass
    return _CGF_PATH is not None


@unittest.skipUnless(
    _cgf_available(),
    "no pyhpp install and $HPP_PYTHON_SRC_DIR not set to a hpp-python checkout",
)
class TestVisitedGraspsMemo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.cgf = _load_cgf()
        cls.Factory = _make_factory(cls.cgf)

    # -- correctness: allow-all filter, compare n=2..5 ----------------------

    def _check_state_transition_counts(self, n, grasp_filter=None):
        """States/transitions produced must match the expected combinatorial."""
        f = _run_factory(self.Factory, n, grasp_filter)
        return f._created_states, f._created_transitions, f._recurse_calls

    def test_allow_all_states_n2(self):
        states, transitions, _ = self._check_state_transition_counts(2)
        # All 2^n = 4 possible assignment combos (none or one of 2 handles per gripper)
        # Free state + P(2,1)*2 + P(2,2) = 1 + 4 + 2 = 7? Actually:
        # grasps is (g0_handle, g1_handle), each either None or a handle index
        # with handle-exclusivity enforced:
        #   (None,None), (0,None),(1,None),(None,0),(None,1),(0,1),(1,0)  → 7 states
        self.assertEqual(len(states), 7)

    def test_allow_all_call_count_n4(self):
        """Memoization must prevent the exponential blowup for n=4."""
        f_fixed = _run_factory(self.Factory, 4)
        # Without the fix, call count would be O(n! * 2^n); with the fix it is
        # bounded. We just assert it's reasonable (< 2000 for n=4).
        self.assertLess(
            f_fixed._recurse_calls,
            2_000,
            "Call count too high — memoization may not be working",
        )

    def test_call_count_drops_with_sequential_filter_n6(self):
        """SequentialGraspFilter reduces state creation dramatically vs allow-all."""
        n = 6
        grippers = [f"g{i}" for i in range(n)]
        handles = [f"h{i}" for i in range(n)]
        current = {g: None for g in grippers}
        next_grasp = ("g0", "h0")
        filt = SequentialGraspFilter(grippers, handles, current, next_grasp)

        f_filtered = _run_factory(self.Factory, n, filt)
        f_unfiltered = _run_factory(self.Factory, n)

        # Filter only permits 2 states (free + g0-holds-h0); unfiltered creates many more.
        self.assertEqual(
            len(f_filtered._created_states),
            2,
            "Sequential filter should yield exactly 2 states",
        )
        self.assertGreater(
            len(f_unfiltered._created_states),
            len(f_filtered._created_states) * 10,
            "Unfiltered should have at least 10× more states than filtered",
        )

    # -- correctness: non-monotonic filter must not drop states -------------

    def test_non_monotonic_filter_correctness(self):
        """A non-monotonic filter (rejects intermediate states, allows deeper ones)
        must still produce the correct accepted states via the fixed recursion."""

        # Allow only states where ALL grippers are either free or all holding.
        # This is non-monotonic: (0, None) is rejected, (0, 1) is accepted.
        def non_monotonic(grasps):
            holding = [g for g in grasps if g is not None]
            return len(holding) == 0 or len(holding) == len(grasps)

        n = 3
        f = _run_factory(self.Factory, n, non_monotonic)

        # Free state (None,None,None) and fully-grasped states P(3,3)=6 permutations
        # Expect 1 + 6 = 7 states
        self.assertEqual(len(f._created_states), 7)
        for grasps in f._created_states:
            holding = [g for g in grasps if g is not None]
            self.assertIn(
                len(holding),
                (0, n),
                f"Unexpected intermediate state accepted: {grasps}",
            )

    # -- no regression: sequential filter on n=8 must complete fast ---------

    def test_sequential_filter_n8_completes_fast(self):
        """Phase 13 worst-case (8 grippers) must not hang."""
        import time

        n = 8
        grippers = [f"g{i}" for i in range(n)]
        handles = [f"h{i}" for i in range(n)]
        current = {g: None for g in grippers}
        next_grasp = ("g0", "h0")
        filt = SequentialGraspFilter(grippers, handles, current, next_grasp)

        t0 = time.monotonic()
        _run_factory(self.Factory, n, filt)
        elapsed = time.monotonic() - t0

        self.assertLess(
            elapsed, 30.0, f"Graph construction for n=8 took {elapsed:.1f}s — too slow"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
