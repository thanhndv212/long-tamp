"""plan_loop's security margins and optimizer override (#211).

Importing long_tamp requires pyhpp, so these run inside the hpp-arm64
container; the backend and the graph are doubles.
"""

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

EDGE = "Loop | 0-0"


class _Graph:
    def __init__(self):
        self.calls = []

    def getTransition(self, name):
        return ("transition", name)

    def setSecurityMarginForTransition(self, transition, first, second, margin):
        self.calls.append(("margin", transition, first, second, margin))

    def initialize(self):
        self.calls.append(("initialize",))


class _Backend:
    def __init__(self, fail=False):
        self.graph = _Graph()
        self._transition_optimizers_by_edge_name = {}
        self.planned_with = []
        self.fail = fail

    def plan_transition_edge(self, edge, q1, q2):
        self.planned_with.append(self._transition_optimizers_by_edge_name.get(edge))
        if self.fail:
            raise RuntimeError("no path")
        return 7, None


class _Tracker:
    def get_loop_edge(self):
        return EDGE

    def get_current_state_name(self):
        return "held"


class _Builder:
    def apply_state_constraints(self, state_name, q, max_iterations, error_threshold):
        return True, q, 0.0


def _planner(backend):
    planner = object.__new__(GraspSequencePlanner)
    planner.planner = backend
    planner.grasp_tracker = _Tracker()
    planner.graph_builder = _Builder()
    planner.phase_results = []
    planner._MAX_COLLISION_RETRIES = 2
    planner._build_phase_graph_and_constraints = lambda **kwargs: None
    return planner


def test_a_loop_keeps_the_margins_on_its_edge():
    backend = _Backend()
    result = _planner(backend).plan_loop(
        "panda/gripper",
        [0.0],
        [1.0],
        verbose=False,
        security_margins=[("pan/root_joint", "universe", 0.02)],
    )
    assert result["success"]
    # The margin invalidates the graph: it is initialized again.
    assert backend.graph.calls == [
        ("margin", ("transition", EDGE), "pan/root_joint", "universe", 0.02),
        ("initialize",),
    ]


def test_a_loop_without_margins_leaves_the_graph_alone():
    backend = _Backend()
    _planner(backend).plan_loop("panda/gripper", [0.0], [1.0], verbose=False)
    assert backend.graph.calls == []
    assert backend.planned_with == [None]


def test_the_optimizer_override_holds_only_while_the_loop_plans():
    backend = _Backend()
    backend._transition_optimizers_by_edge_name[EDGE] = ["Spline"]
    _planner(backend).plan_loop(
        "panda/gripper", [0.0], [1.0], verbose=False, optimizers=("Shortcut",)
    )
    assert backend.planned_with == [["Shortcut"]]
    assert backend._transition_optimizers_by_edge_name == {EDGE: ["Spline"]}


def test_the_optimizer_override_is_undone_when_the_loop_fails():
    backend = _Backend(fail=True)
    result = _planner(backend).plan_loop(
        "panda/gripper", [0.0], [1.0], verbose=False, optimizers=("Shortcut",)
    )
    assert not result["success"]
    assert backend.planned_with == [["Shortcut"], ["Shortcut"]]
    assert backend._transition_optimizers_by_edge_name == {}
