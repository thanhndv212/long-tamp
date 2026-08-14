#!/usr/bin/env python3
"""Regression test: PrunedRecursionMixin bounds the factory's recursion.

`GraphFactoryAbstract._recurse` walks every distinct partial gripper-to-handle
assignment; `graspIsAllowed` only decides whether a *state* is created, so the
walk itself is the full combinatorial space.  Live on RS4 A that was a single
`build_phase_graph` call at 13 GB and climbing after ten minutes -- 10
grippers, for a graph with 3 states.

The mixin prunes to assignments a target state can still be reached from.
This test pins the two properties that matter:

- **Same graph.**  Pruned and unpruned produce identical states and
  transitions, including for a *release* target (where the target has fewer
  grasps than the state it is reached from) and for a non-monotonic filter.
- **Bounded work.**  `_recurse` call count stops growing combinatorially with
  gripper count, which is the actual bug.

Loads `constraint_graph_factory` the same way `test_graph_factory_visited_memo`
does: a real pyhpp install if importable, else a hpp-python checkout named by
$HPP_PYTHON_SRC_DIR.  Skipped when neither is available.
"""

import importlib
import importlib.util
import os
import sys
import types
import unittest
from pathlib import Path

SRC = Path(__file__).parent.parent / "src"


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


def _cgf_source_path():
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
    spec = importlib.util.spec_from_file_location(
        "constraint_graph_factory", _CGF_PATH
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    from agimus_spacelab.planning.sequential_graph_factory import (
        PrunedRecursionMixin,
        pruned_factory_class,
    )
    from agimus_spacelab.planning.sequential_grasp_filter import (
        SequentialGraspFilter,
    )

    HAVE_MIXIN = True
except ImportError:  # no backend to import the base factory from
    HAVE_MIXIN = False


def _cgf_available() -> bool:
    try:
        if importlib.util.find_spec("pyhpp.manipulation.constraint_graph_factory"):
            return True
    except ModuleNotFoundError:
        pass
    return _CGF_PATH is not None


def _make_factories(cgf_mod, mixin):
    """A plain recording factory and the same one with the mixin applied."""

    class _State:
        __slots__ = ("grasps",)

        def __init__(self, grasps):
            self.grasps = grasps

    class _Base(cgf_mod.GraphFactoryAbstract):
        def __init__(self):
            super().__init__()
            self.created_states = set()
            self.created_transitions = set()
            self.recurse_calls = 0

        def makeState(self, grasps, priority):
            self.created_states.add(grasps)
            return _State(grasps)

        def makeLoopTransition(self, state):
            pass

        def makeTransition(self, stateFrom, stateTo, ig):
            self.created_transitions.add((stateFrom.grasps, stateTo.grasps, ig))

    class _Counting:
        """Must sit ahead of the mixin in the MRO: the mixin reimplements
        `_recurse` and recurses through `self`, so a counter placed *behind*
        it would only ever see the outermost call."""

        def _recurse(self, grippers, handles, grasps, depth):
            self.recurse_calls += 1
            super()._recurse(grippers, handles, grasps, depth)

    plain = type("_Plain", (_Counting, _Base), {})
    pruned = type("_Pruned", (_Counting, mixin, _Base), {})
    return plain, pruned


def _run(factory_cls, n, grasp_filter=None, targets=None):
    f = factory_cls()
    grippers = [f"g{i}" for i in range(n)]
    handles = [f"h{i}" for i in range(n)]
    f.setGrippers(grippers)
    f.setObjects(
        [f"obj{i}" for i in range(n)],
        [[h] for h in handles],
        [[] for _ in range(n)],
    )
    if grasp_filter is not None:
        f.graspIsAllowed = grasp_filter
    if targets is not None:
        f.set_target_grasps(targets)
    f.generate()
    return f


@unittest.skipUnless(
    _cgf_available() and HAVE_MIXIN,
    "needs a pyhpp install (or $HPP_PYTHON_SRC_DIR) and an importable "
    "agimus_spacelab.planning",
)
class TestPrunedRecursion(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.cgf = _load_cgf()
        cls.mixin = PrunedRecursionMixin
        cls.pruned_factory_class = staticmethod(pruned_factory_class)
        cls.Plain, cls.Pruned = _make_factories(cls.cgf, cls.mixin)

    def _sequential(self, n, current, next_grasp):
        grippers = [f"g{i}" for i in range(n)]
        handles = [f"h{i}" for i in range(n)]
        return SequentialGraspFilter(grippers, handles, current, next_grasp)

    def _assert_same_graph(self, n, filt, targets):
        plain = _run(self.Plain, n, filt)
        pruned = _run(self.Pruned, n, filt, targets)
        self.assertEqual(
            plain.created_states,
            pruned.created_states,
            "pruning changed which states are created",
        )
        self.assertEqual(
            plain.created_transitions,
            pruned.created_transitions,
            "pruning changed which transitions are created",
        )
        return plain, pruned

    def test_same_graph_for_a_grasp_target(self):
        """A grasp phase: pruned and unpruned agree, pruned does less work."""
        n = 5
        current = {f"g{i}": None for i in range(n)}
        current["g1"] = "h1"
        filt = self._sequential(n, current, ("g0", "h0"))
        plain, pruned = self._assert_same_graph(
            n, filt, (filt.current_grasps, filt.next_grasps)
        )
        self.assertEqual(len(pruned.created_states), 2)
        self.assertLess(pruned.recurse_calls, plain.recurse_calls)

    def test_same_graph_for_a_release_target(self):
        """A release target has *fewer* grasps than the state it is reached
        from, so the guard must not prune the path that carries it."""
        n = 4
        current = {f"g{i}": None for i in range(n)}
        current["g0"] = "h0"
        current["g2"] = "h2"
        filt = self._sequential(n, current, ("g0", None))
        _, pruned = self._assert_same_graph(
            n, filt, (filt.current_grasps, filt.next_grasps)
        )
        self.assertEqual(len(pruned.created_states), 2)

    def test_same_graph_for_a_non_monotonic_filter(self):
        """Targets, not the filter, drive the prune, so a filter that rejects
        intermediate states but accepts deeper ones is still reproduced
        exactly -- as long as the targets name every state it accepts.

        (That proviso is the contract: targets must cover the allowed set.
        `_apply_sequential_filter` satisfies it by construction, since the
        filter accepts exactly the two tuples handed to `set_target_grasps`.)
        """
        from itertools import permutations

        def all_or_nothing(grasps):
            return all(g is None for g in grasps) or all(
                g is not None for g in grasps
            )

        n = 3
        targets = [(None,) * n, *permutations(range(n))]
        self._assert_same_graph(n, all_or_nothing, targets)

    def test_targets_must_cover_the_allowed_set(self):
        """The flip side of the contract: name only some allowed states and
        the others are legitimately pruned away.  Pinned so the requirement
        stays visible rather than being discovered as a bug."""

        def all_or_nothing(grasps):
            return all(g is None for g in grasps) or all(
                g is not None for g in grasps
            )

        n = 3
        partial = [(None,) * n, tuple(range(n))]
        plain = _run(self.Plain, n, all_or_nothing)
        pruned = _run(self.Pruned, n, all_or_nothing, partial)
        self.assertEqual(pruned.created_states, set(partial))
        self.assertLess(len(pruned.created_states), len(plain.created_states))

    def test_no_targets_leaves_the_walk_untouched(self):
        """With no targets registered the mixin must be a no-op."""
        n = 4
        plain = _run(self.Plain, n)
        pruned = _run(self.Pruned, n)
        self.assertEqual(plain.created_states, pruned.created_states)
        self.assertEqual(plain.created_transitions, pruned.created_transitions)
        self.assertEqual(plain.recurse_calls, pruned.recurse_calls)

    def test_work_stays_bounded_as_grippers_are_added(self):
        """The actual bug: unpruned work explodes with gripper count while the
        graph stays at 2 states.  Pruned work must not."""
        counts = {}
        for n in (4, 6, 8):
            current = {f"g{i}": None for i in range(n)}
            filt = self._sequential(n, current, ("g0", "h0"))
            pruned = _run(
                self.Pruned, n, filt, (filt.current_grasps, filt.next_grasps)
            )
            self.assertEqual(len(pruned.created_states), 2)
            counts[n] = pruned.recurse_calls

        plain_8 = _run(
            self.Plain,
            8,
            self._sequential(8, {f"g{i}": None for i in range(8)}, ("g0", "h0")),
        )
        self.assertLess(
            counts[8],
            plain_8.recurse_calls / 100,
            "pruned walk is not meaningfully smaller than the full walk",
        )
        # Growth from 4 to 8 grippers must be nowhere near combinatorial.
        self.assertLess(counts[8], counts[4] * 10)

    def test_pruned_factory_class_is_cached_and_woven(self):
        cls_a = self.pruned_factory_class(self.Plain)
        cls_b = self.pruned_factory_class(self.Plain)
        self.assertIs(cls_a, cls_b, "class factory should be cached")
        self.assertTrue(issubclass(cls_a, self.mixin))
        self.assertTrue(issubclass(cls_a, self.Plain))


if __name__ == "__main__":
    unittest.main()
