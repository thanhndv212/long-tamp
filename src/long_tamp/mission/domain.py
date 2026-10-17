"""The MissionDomain interface (ADR-0011).

A *mission domain* is everything the planner -> refiner -> executor ->
repair -> supervisor loop needs to know about one robot and one kind of task:

- **symbolic**: its capabilities (``descriptors``) and guard conditions
  (``guards``), the objects and regions goals may name, the static facts and
  extra preconditions over them, the recorded predicates (facts no sensor
  shows, ADR-0002), and a default goal;
- **skeleton -> plan**: how a planned step expands into transactions
  (``expand_step``, e.g. a move home after a pickup) and how a transaction
  becomes something to refine (``step_for``: a grasp sequence,
  :class:`~long_tamp.tasks.refiner.RefinementStep`, or a :class:`Move`);
- **repair**: which bindings a failure rules out (``repair_policy``) and
  which failures are retried in place (``retry``);
- **geometry and execution**: building the scene (``setup``), planning a
  move (``plan_move``), what a planned path is sent to the executor as
  (``execution_payload``), and the backends it runs on (``make_backend``);
- **the runner's hooks**: the world state guards and effects are checked
  on (``world_state``), where to replan from when the backend observes something
  else than planned (``observed_start``), opening hands in a configuration
  (``opened_config``), and domain-specific execution metrics
  (``execution_metrics``).

The generic runner (``long_tamp.mission.runner``, #132) drives any
:class:`MissionDomain`. :class:`BaseMissionDomain` gives the parts with a
generic answer (PDDL export, vocabulary, skeleton document, repair policy),
so a domain writes only what is specific to it. ``script/screw_assembly/
screw_mission.py`` is the reference implementation.

The interface is provisional until a second domain implements it (the
RoboCerebra goal translator, #159): expect it to change then.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, Union, runtime_checkable

from long_tamp.tasks.refiner import RefinementStep
from long_tamp.tasks.task_planning.capabilities import (
    CapabilityDescriptor,
    CapabilityRegistry,
)
from long_tamp.tasks.task_planning.language import Vocabulary, check_goal
from long_tamp.tasks.task_planning.model import PlanValidationError, TaskPlan
from long_tamp.tasks.task_planning.pddl import PddlExport, to_pddl
from long_tamp.tasks.task_planning.predicates import Atom, parse_atom
from long_tamp.tasks.task_planning.repair import (
    Binding,
    Failure,
    retry_missed_grasp,
)
from long_tamp.tasks.task_planning.session import WorldState
from long_tamp.tasks.task_planning.skeleton import Step, skeleton_document
from long_tamp.tasks.task_planning.world_state import (
    CompositeWorldState,
    GraspTrackerState,
    RecordedFacts,
)

from .goals import GRASPED, ObjectLevelState, check_object_goal

#: The predicates saying what a gripper holds: ``holds(gripper, handle)``,
#: ``grasped(gripper, object)``.
HELD_PREDICATES = frozenset({"holds", GRASPED})

#: One expanded step: ``(capability, parameters, label)`` (see
#: ``skeleton.Expand``).
Transaction = tuple[str, dict[str, Any], str]


@dataclass(frozen=True)
class Move:
    """A step that is a plain motion, not a grasp sequence (an arm going
    home). ``spec`` is the domain's own description, read back by its
    ``plan_move``."""

    label: str
    spec: Mapping[str, Any] = field(default_factory=dict)


#: What one transaction is refined as.
MissionStep = Union[RefinementStep, Move]


@dataclass(frozen=True)
class Scenario:
    """An initial-state scenario (validation level V4): reach a start state
    by running the ``setup`` steps (transaction labels) from the nominal
    start, then run the mission from there.

    ``facts``: the recorded facts after the setup; ``skip``: the labels the
    mission must skip; ``atoms``: the start state, for a load-time check.
    """

    name: str
    setup: tuple[str, ...] = ()
    facts: tuple[str, ...] = ()
    skip: tuple[str, ...] = ()
    atoms: tuple[str, ...] = ()


@dataclass
class MissionScene:
    """A built scene: the task (robot, objects, constraint graph), its
    planner, the start configuration, and anything else the domain keeps
    (``extras``)."""

    task: Any
    planner: Any
    q_init: Sequence[float]
    extras: dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class MissionDomain(Protocol):
    """One robot and one kind of task, as the mission loop sees it."""

    #: Short id, used for the PDDL domain name and mission ids.
    name: str
    #: Predicates only completed execution writes (``RecordedFacts``).
    recorded_predicates: frozenset[str]
    #: Notes on what the predicates mean, for goal writers (LLM prompts).
    goal_notes: str
    #: One paragraph introducing the domain to a chat user.
    intro: str

    # -- symbolic ------------------------------------------------------------

    def scene_info(self) -> dict[str, Any]:
        """The ``scene`` block of the plan documents (``{"id": ...}``)."""
        ...

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        """Every capability, guards included, by id."""
        ...

    def guards(self) -> frozenset[str]:
        """The capability ids that are guard conditions (no effects):
        checked against the world state, never executed."""
        ...

    def objects(self) -> tuple[str, ...]:
        """The objects a goal may name."""
        ...

    def regions(self) -> tuple[str, ...]:
        """The regions ``placed_at(object, region)`` may name (may be
        empty)."""
        ...

    def static_facts(self) -> tuple[str, ...]:
        """Ground atoms true for the whole mission (``seat(clamp1, part1)``)."""
        ...

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        """Extra preconditions over static facts, by capability id."""
        ...

    def default_goal(self) -> list[str]:
        """The mission's own goal, as ground literals."""
        ...

    def pddl_problem(
        self,
        state: Iterable[str | Atom] = (),
        blocked: Iterable[Binding] = (),
        goal: Sequence[str] | None = None,
    ) -> PddlExport:
        """The PDDL export from ``state`` to ``goal`` (default: the default
        goal), avoiding the ``blocked`` bindings."""
        ...

    def goal_vocabulary(self, state: Iterable[str | Atom] = ()) -> Vocabulary:
        """What a goal may say, for goal writers and ``check_goal``."""
        ...

    def plan_document(self, initial_state: Iterable[str] = ()) -> dict[str, Any] | None:
        """A hand-written TaskPlan document for the default goal, or None
        when the domain only plans from goals."""
        ...

    # -- skeleton -> plan ----------------------------------------------------

    def expand_step(
        self,
        index: int,
        capability: str,
        parameters: dict[str, Any],
        held: Mapping[str, str],
    ) -> list[Transaction]:
        """The transactions for one planned step (``skeleton.Expand``), given
        what each gripper ``held`` before it."""
        ...

    def planned_document(
        self,
        steps: Sequence[Step],
        state: Iterable[str | Atom] = (),
        generator: str = "unified-planning",
    ) -> dict[str, Any]:
        """The TaskPlan document for a planned skeleton from ``state``."""
        ...

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> MissionStep:
        """What one transaction is refined as."""
        ...

    # -- repair --------------------------------------------------------------

    def repair_policy(self, failure: Failure) -> list[Binding]:
        """The bindings a failure rules out for the next plan."""
        ...

    def retry(self, failure: Failure) -> bool:
        """Whether to retry the failed step in place before replanning."""
        ...

    # -- geometry and execution ----------------------------------------------

    def scenarios(self) -> dict[str, Scenario]:
        """Named initial-state scenarios (may be empty)."""
        ...

    def setup(self, seed: int | None = None, log_dir: str = "auto") -> MissionScene:
        """Build the scene (robot, objects, constraint graph, planner)."""
        ...

    def plan_move(
        self,
        scene: MissionScene,
        move: Move,
        q: Sequence[float],
        verbose: bool = True,
    ) -> dict[str, Any]:
        """Plan ``move`` from ``q``: a result dict like a refinement's
        (``success``, ``final_config``, ``resumes``, ``replans``,
        ``message``) plus the planned ``phase_results``."""
        ...

    def execution_payload(self, phase: Mapping[str, Any], edge: Any, path: Any) -> Any:
        """What a planned ``path`` of ``phase`` (along graph ``edge``) is
        sent to the executor as: the path, or a skill command around it."""
        ...

    def make_backend(self, name: str, scene: MissionScene, **options: Any) -> Any:
        """The execution backend called ``name`` (``"mock"``,
        ``"playback"``, ...), or None to plan without executing."""
        ...

    # -- the runner's hooks (#132) -------------------------------------------

    def world_state(self, scene: MissionScene, recorded: RecordedFacts) -> WorldState:
        """The world-state source the runner checks guards and effects on:
        what the scene's grasp tracker holds, plus the ``recorded`` facts."""
        ...

    def observed_start(
        self, scene: MissionScene, planned: Sequence[float], observed: Sequence[float]
    ) -> list[float]:
        """The configuration to replan a step from when the backend observes
        ``observed`` where ``planned`` was expected (drift, a missed grasp)."""
        ...

    def opened_config(
        self, scene: MissionScene, q: Sequence[float], grippers: Sequence[str]
    ) -> list[float]:
        """``q`` with the hands of ``grippers`` open, for hands the backend
        could not open itself (so a retried approach does not arrive closed)."""
        ...

    def execution_metrics(self, executions: Sequence[Any]) -> dict[str, Any]:
        """Domain-specific entries for the run's execution summary
        (``runner.execution_summary``), from ``PlanRun.executions``."""
        ...


class BaseMissionDomain:
    """Generic answers for the parts of :class:`MissionDomain` that have
    one. A subclass sets ``name`` and implements ``descriptors``,
    ``expand_step`` (if it inserts steps), ``step_for``, ``setup`` and
    ``plan_move``; the rest has defaults to override as needed."""

    name: str = "mission"
    recorded_predicates: frozenset[str] = frozenset()
    goal_notes: str = ""
    intro: str = ""

    # -- symbolic ------------------------------------------------------------

    def scene_info(self) -> dict[str, Any]:
        return {"id": self.name}

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        raise NotImplementedError

    def guards(self) -> frozenset[str]:
        """By default, the capabilities that change nothing: no effects and
        no ``writes`` (a move home has no effects but writes the arm pose)."""
        return frozenset(
            cap
            for cap, d in self.descriptors().items()
            if not d.effect_literals and not d.writes
        )

    def objects(self) -> tuple[str, ...]:
        """By default, every argument of the static facts."""
        found = {a for fact in self.static_facts() for a in parse_atom(fact).args}
        return tuple(sorted(found))

    def regions(self) -> tuple[str, ...]:
        return ()

    def static_facts(self) -> tuple[str, ...]:
        return ()

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        return {}

    def default_goal(self) -> list[str]:
        return []

    def pddl_problem(
        self,
        state: Iterable[str | Atom] = (),
        blocked: Iterable[Binding] = (),
        goal: Sequence[str] | None = None,
    ) -> PddlExport:
        return to_pddl(
            self.descriptors(),
            init=[*state, *self.static_facts()],
            goal=self.default_goal() if goal is None else goal,
            domain_name=self.name,
            problem_name=f"{self.name}-mission",
            objects=(*self.objects(), *self.regions()),
            static_preconditions=self.static_preconditions(),
            blocked=blocked,
        )

    def goal_vocabulary(self, state: Iterable[str | Atom] = ()) -> Vocabulary:
        return Vocabulary.from_domain(
            self.descriptors(),
            [str(a) for a in state],
            objects=(*self.objects(), *self.regions()),
            notes=self.goal_notes,
        )

    def plan_document(self, initial_state: Iterable[str] = ()) -> dict[str, Any] | None:
        return None

    # -- skeleton -> plan ----------------------------------------------------

    def expand_step(
        self,
        index: int,
        capability: str,
        parameters: dict[str, Any],
        held: Mapping[str, str],
    ) -> list[Transaction]:
        args = ", ".join(str(v) for v in parameters.values())
        return [(capability, parameters, f"{capability}({args})")]

    def planned_document(
        self,
        steps: Sequence[Step],
        state: Iterable[str | Atom] = (),
        generator: str = "unified-planning",
    ) -> dict[str, Any]:
        initial_state = [str(a) for a in state]
        held = held_by_gripper(initial_state)

        def expand(index: int, capability: str, parameters: dict[str, Any]):
            transactions = self.expand_step(index, capability, parameters, held)
            update_held(held, self.descriptors()[capability], parameters)
            return transactions

        return skeleton_document(
            steps,
            mission_id=self.name,
            expand=expand,
            initial_state=initial_state,
            scene=self.scene_info(),
            generator=generator,
        )

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> MissionStep:
        raise NotImplementedError

    # -- repair --------------------------------------------------------------

    def repair_policy(self, failure: Failure) -> list[Binding]:
        """Block the failed step's binding (the implementation parameter
        ``block``, a transaction label, left out); nothing to block when it
        has no other parameter."""
        binding = {
            k: str(v) for k, v in failure.get("parameters", {}).items() if k != "block"
        }
        return [(failure["capability"], binding)] if binding else []

    def retry(self, failure: Failure) -> bool:
        return retry_missed_grasp(failure)

    # -- geometry and execution ----------------------------------------------

    def scenarios(self) -> dict[str, Scenario]:
        return {}

    def setup(self, seed: int | None = None, log_dir: str = "auto") -> MissionScene:
        raise NotImplementedError

    def plan_move(
        self,
        scene: MissionScene,
        move: Move,
        q: Sequence[float],
        verbose: bool = True,
    ) -> dict[str, Any]:
        raise NotImplementedError(f"{self.name}: no plain moves")

    def execution_payload(self, phase: Mapping[str, Any], edge: Any, path: Any) -> Any:
        return path

    def make_backend(self, name: str, scene: MissionScene, **options: Any) -> Any:
        """``"none"`` (plan only) and ``"mock"``; a domain adds its own."""
        if name == "none":
            return None
        if name == "mock":
            from long_tamp.execution import MockBackend

            return MockBackend(**options)
        raise ValueError(f"{self.name}: unknown backend {name!r}")

    def world_state(self, scene: MissionScene, recorded: RecordedFacts) -> WorldState:
        """The grasp tracker's ``holds`` atoms if a capability uses
        ``holds``, the ``grasped`` atoms derived from them if one uses
        ``grasped``, and the recorded facts."""
        used = {
            literal.atom.name
            for descriptor in self.descriptors().values()
            for literal in (
                *descriptor.precondition_literals,
                *descriptor.effect_literals,
            )
        }
        tracker = GraspTrackerState(scene.planner)
        sources: list[Callable[[], Iterable[Any]]] = []
        if "holds" in used:
            sources.append(tracker)
        if GRASPED in used:
            sources.append(ObjectLevelState(tracker))
        return CompositeWorldState(*sources, recorded)

    def observed_start(
        self, scene: MissionScene, planned: Sequence[float], observed: Sequence[float]
    ) -> list[float]:
        """Everything where it is observed."""
        return [float(v) for v in observed]

    def opened_config(
        self, scene: MissionScene, q: Sequence[float], grippers: Sequence[str]
    ) -> list[float]:
        """No finger joints to open."""
        return [float(v) for v in q]

    def execution_metrics(self, executions: Sequence[Any]) -> dict[str, Any]:
        return {}


def held_by_gripper(state: Iterable[str | Atom]) -> dict[str, str]:
    """``{gripper: handle or object}`` from the ``holds`` / ``grasped``
    atoms (``HELD_PREDICATES``)."""
    atoms = (parse_atom(a) if isinstance(a, str) else a for a in state)
    return {a.args[0]: a.args[1] for a in atoms if a.name in HELD_PREDICATES}


def update_held(
    held: dict[str, str],
    descriptor: CapabilityDescriptor,
    parameters: Mapping[str, Any],
) -> None:
    """Apply a step's ``holds`` / ``grasped`` effects to ``held``, in place."""
    for literal in descriptor.effect_literals:
        if literal.atom.name not in HELD_PREDICATES:
            continue
        gripper, handle = (
            str(parameters.get(a[1:], a)) if a.startswith("?") else a
            for a in literal.atom.args
        )
        if literal.positive:
            held[gripper] = handle
        else:
            held.pop(gripper, None)


def capability_registry(
    domain: MissionDomain,
    handler: Callable[..., Any],
    guard: Callable[..., Any] | None = None,
) -> CapabilityRegistry:
    """A registry of the domain's capabilities: guards bound to ``guard``
    (default: ``handler``), the others to ``handler``."""
    registry = CapabilityRegistry()
    guards = domain.guards()
    for cap, descriptor in domain.descriptors().items():
        registry.register(descriptor, (guard or handler) if cap in guards else handler)
    return registry


def check_domain(domain: MissionDomain) -> list[str]:
    """Why ``domain`` is inconsistent (empty: it isn't). Checks what can be
    checked without geometry: guards and static preconditions name known
    capabilities, guards change nothing, recorded predicates are some
    capability's effect, the default goal is in the domain's vocabulary, and
    the hand-written plan, if any, validates."""
    errors: list[str] = []
    descriptors = domain.descriptors()
    for cap, descriptor in descriptors.items():
        if cap != descriptor.capability_id:
            errors.append(f"descriptor {cap!r} has id {descriptor.capability_id!r}")
    for cap in sorted(domain.guards()):
        if cap not in descriptors:
            errors.append(f"guard {cap!r}: unknown capability")
        elif descriptors[cap].effect_literals or descriptors[cap].writes:
            errors.append(f"guard {cap!r} changes the state")
    for cap in sorted(domain.static_preconditions()):
        if cap not in descriptors:
            errors.append(f"static precondition on unknown capability {cap!r}")
    effects = {
        literal.atom.name
        for descriptor in descriptors.values()
        for literal in descriptor.effect_literals
    }
    for predicate in sorted(domain.recorded_predicates - effects):
        errors.append(f"recorded predicate {predicate!r} is no capability's effect")
    goal = domain.default_goal()
    errors += check_goal(goal, domain.goal_vocabulary())
    errors += check_object_goal(goal, domain.objects(), domain.regions())
    document = domain.plan_document()
    if document is not None:
        registry = capability_registry(domain, lambda **_: None)
        try:
            TaskPlan.from_dict(document, registry)
        except PlanValidationError as error:
            errors.append(f"plan document: {error}")
    return errors


__all__ = [
    "BaseMissionDomain",
    "MissionDomain",
    "MissionScene",
    "MissionStep",
    "Move",
    "Scenario",
    "HELD_PREDICATES",
    "Transaction",
    "capability_registry",
    "check_domain",
    "held_by_gripper",
    "update_held",
]
