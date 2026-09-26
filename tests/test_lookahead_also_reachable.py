"""Unit tests for ``find_feasible_phase_target()``'s ``also_reachable``.

A phase-N grasp can pin more than phase N+1 depends on: a grasp that fixes
a part's orientation fixes it for every later contact on that part, so a
candidate that passes the N+1 probe can still leave a later grasp
unreachable -- and no retry of that later phase can undo the commitment.
``also_reachable`` makes the candidate prove those later grasps too.

The scene is faked at the graph-builder / config-generator boundary: what
is under test is which candidate the search accepts and which held set
each probe is built with, neither of which needs a solver.

Importing long_tamp's planner requires pyhpp, so these run in the hpp-arm64
container like the rest of the suite.
"""

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

PHASE_N = ("arm1/g_wb", "part/h_wb")
PHASE_N1 = ("tool/g_tip", "part/h_contact1")
LATER = ("tool/g_tip", "part/h_contact2")


class _Tracker:
    """Just what the lookahead touches: held grasps and edge names."""

    def __init__(self, grasps):
        self.current_grasps = dict(grasps)

    def copy(self):
        return _Tracker(self.current_grasps)

    def update_grasp(self, gripper, handle):
        self.current_grasps[gripper] = handle

    def get_grasp_edge_sequence(self, gripper, handle):
        return [f"{handle}|01", f"{handle}|12"]


class _GraphBuilder:
    def __init__(self):
        self.builds = []  # (next_grasp, held set) per phase-graph build

    def build_phase_graph(self, held_grasps, next_grasp, **_):
        self.builds.append((next_grasp, dict(held_grasps)))

    def get_graph(self):
        return object()


class _ConfigGen:
    """Phase N's candidates are numbered 1, 2, ... in draw order; the later
    grasp is unreachable from every candidate in ``dead_for_later``."""

    def __init__(self, dead_for_later=()):
        self.dead_for_later = set(dead_for_later)
        self.candidate = 0
        self.frozen_sets = []  # set_frozen_joints() calls, in order

    def set_frozen_joints(self, names):
        self.frozen_sets.append(list(names))

    def update_graph(self, graph):
        pass

    def generate_via_edge(self, edge_name, q_from, timeout=None):
        handle = edge_name.split("|")[0]
        if edge_name == f"{PHASE_N[1]}|01":
            self.candidate += 1
        if handle == LATER[1] and self.candidate in self.dead_for_later:
            return False, None
        return True, [float(self.candidate)]


class _PathPlanner:
    """plan_transition_edge stand-in: no path to any candidate in ``blocked``
    (identified by the candidate number the fake generator stamps into q)."""

    def __init__(self, blocked=()):
        self.blocked = set(blocked)
        self.calls = 0

    def plan_transition_edge(self, edge, q1, q2):
        self.calls += 1
        if int(q2[0]) in self.blocked:
            raise RuntimeError("no path")
        return object(), object()


def _planner(config_gen, path_planner=None):
    p = object.__new__(GraspSequencePlanner)
    p.grasp_tracker = _Tracker({"arm1/g_wb": None, "tool/g_tip": None})
    p.graph_builder = _GraphBuilder()
    p.config_gen = config_gen
    p.planner = path_planner or object()
    p.task_config = None
    return p


def _search(p, also_reachable=(), verify_paths=False):
    return p.find_feasible_phase_target(
        phase_n=PHASE_N,
        phase_n1=PHASE_N1,
        q_current=[0.0],
        q_scene_init=[0.0],
        frozen_arms_n=[],
        frozen_arms_n1=[],
        max_candidates=5,
        verbose=False,
        also_reachable=also_reachable,
        verify_paths=verify_paths,
    )


class TestAlsoReachable:
    def test_rejects_a_candidate_that_strands_the_later_grasp(self):
        p = _planner(_ConfigGen(dead_for_later={1}))

        chain = _search(p, also_reachable=[(LATER, [])])

        assert chain == [[2.0], [2.0]], "candidate 1 strands LATER; take 2"

    def test_without_it_the_first_n1_reachable_candidate_wins(self):
        """The failure the parameter exists for: candidate 1 passes the
        N+1 probe and is accepted, although LATER is dead from it."""
        p = _planner(_ConfigGen(dead_for_later={1}))

        assert _search(p) == [[1.0], [1.0]]

    def test_each_probe_holds_only_phase_n(self):
        """The later grasp runs after N+1's grasp/release pair, which
        restores the held set -- so its probe must not see N+1 held."""
        p = _planner(_ConfigGen())

        _search(p, also_reachable=[(LATER, [])])

        held_by_next = {nxt: held for nxt, held in p.graph_builder.builds}
        assert held_by_next[PHASE_N1] == {PHASE_N[0]: PHASE_N[1]}
        assert held_by_next[LATER] == {PHASE_N[0]: PHASE_N[1]}

    def test_the_real_tracker_is_never_touched(self):
        p = _planner(_ConfigGen(dead_for_later={1}))
        before = dict(p.grasp_tracker.current_grasps)

        _search(p, also_reachable=[(LATER, [])])

        assert p.grasp_tracker.current_grasps == before

    def test_exhausting_the_budget_returns_none(self):
        p = _planner(_ConfigGen(dead_for_later={1, 2, 3, 4, 5}))

        assert _search(p, also_reachable=[(LATER, [])]) is None

    def test_every_probe_build_resets_the_generators_frozen_joints(self):
        """The generator holds its frozen joints at q_from across restarts.
        If a probe build doesn't set them, it inherits the previous *real*
        phase's set -- live, the driver's arm stayed frozen while probing
        a driver-tip grasp, and every probe failed in the solver."""
        gen = _ConfigGen()
        gen.frozen_sets.append(["stale/arm_joint"])  # left by a real phase
        p = _planner(gen)

        _search(p, also_reachable=[(LATER, [])])

        builds = len(p.graph_builder.builds)
        assert gen.frozen_sets[-builds:] == [[]] * builds


class TestVerifyPaths:
    def test_rejects_a_candidate_with_no_path_to_it(self):
        paths = _PathPlanner(blocked={1})
        p = _planner(_ConfigGen(), paths)

        assert _search(p, verify_paths=True) == [[2.0], [2.0]]

    def test_is_off_by_default(self):
        paths = _PathPlanner(blocked={1})
        p = _planner(_ConfigGen(), paths)

        assert _search(p) == [[1.0], [1.0]]
        assert paths.calls == 0

    def test_rejects_a_candidate_that_blocks_a_protected_phases_path(self):
        """The protected grasps' paths are checked from the candidate too:
        the fake stamps probe configs with the candidate number, so
        blocking candidate 1 blocks every edge probed from it."""
        paths = _PathPlanner(blocked={1})
        p = _planner(_ConfigGen(), paths)

        chain = _search(p, also_reachable=[(LATER, [])], verify_paths=True)

        assert chain == [[2.0], [2.0]]
        # candidate 2: its own 2 edges + N+1's 2 + LATER's 2
        assert paths.calls >= 6

    def test_path_checks_run_on_a_short_budget_that_is_restored(self):
        class _Budgeted(_PathPlanner):
            _transition_time_out = 60.0
            _transition_max_iterations = 10000

            def __init__(self):
                super().__init__()
                self.budgets = []

            def configure_transition_planner(self, time_out, max_iterations):
                self.budgets.append((time_out, max_iterations))
                self._transition_time_out = time_out
                self._transition_max_iterations = max_iterations

        paths = _Budgeted()
        p = _planner(_ConfigGen(), paths)
        _search(p, verify_paths=True)

        assert paths.budgets[0] == (8.0, 3000)
        assert paths.budgets[-1] == (60.0, 10000)
        assert (paths._transition_time_out, paths._transition_max_iterations) == (
            60.0,
            10000,
        )
