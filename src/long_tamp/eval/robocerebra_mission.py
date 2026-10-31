"""RoboCerebra as a mission domain (#186, R9): the whole task planned by
long_tamp's mission loop instead of one benchmark segment at a time.

A case's ``goal.json`` is one ordered chain of goals per object
(``popcorn_1: pick > pour plate_1 > in coffee_table_table_region``). The
benchmark's checker (``_check_success``) is a counter per object: the
object's next goal counts once it holds, and only that one. There is no
order across objects. The domain says exactly that to a task planner:

- ``progress(object, sK)``: the object's counter is at ``K`` (the env's
  ``check()["progress"]``, observed);
- ``goal_pick(object, sK)``, ``goal_place(object, sK, region)``,
  ``goal_pour(object, sK, receiver)``: the object's goal number ``K``
  (static facts, from ``goal.json``); ``next(sK, sK+1)`` orders them;
- ``grasped(gripper, object)``: what the hand holds (observed from the
  finger contacts);
- ``rest(object, sK)``: at ``K`` the object may be let go of: its next goal
  is a pick, it has none left, or it is given up on (static).

``pick``, ``place`` and ``pour`` move an object's counter on when its next
goal is theirs; ``regrasp`` and ``putdown`` take and release an object
without moving it on: ``regrasp`` an object that must be held and is not
(it fell), ``putdown`` one that need not be (to free the hand). So a plan
never puts down what it is carrying to a goal, nor picks up what it need
not hold. The goal is every object's counter at the end of its chain, cut
before the first goal the domain has no capability for (``open``,
``turnon``: #161): the checker never counts past it.

Each step is refined by the HPP planner of rows A to C
(:meth:`HppPolicy.plan <long_tamp.eval.robocerebra_policy.HppPolicy.plan>`,
on a fresh scene export) and runs on :class:`RoboCerebraBackend`, which
steps its OSC tracker in the env and fails the step when the checker did
not count its goal. :func:`evaluate_mission_case` runs a case as one
mission (row D): ``MissionRunner.run_with_repair`` replans from the
observed world after every failure, and an object whose step keeps failing
is given up on. The whole rollout shares one budget, 150 env steps per
annotated task step, instead of 150 a segment.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from long_tamp.execution import ExecutionCommand, ExecutionStatus, Feedback

from long_tamp.mission.domain import BaseMissionDomain, MissionScene, Move
from long_tamp.mission.goals import grasped
from long_tamp.mission.template import KinematicPlanner
from long_tamp.tasks.task_planning.capabilities import CapabilityDescriptor
from long_tamp.tasks.task_planning.language import Vocabulary
from long_tamp.tasks.task_planning.predicates import Atom
from long_tamp.tasks.task_planning.world_state import CompositeWorldState

from .robocerebra_plan import GRIPPER, held_object
from .robocerebra_policy import (
    HppPolicy,
    LookaheadFailed,
    ReleaseController,
    SegmentController,
    SegmentGoal,
    UnsupportedGoal,
)

#: ``goal.json`` predicate -> the capability that reaches it.
GOAL_CAPABILITIES = {"pick": "pick", "in": "place", "on": "place", "pour": "pour"}


def stage(index: int) -> str:
    """The constant for an object's counter at ``index``."""
    return f"s{index}"


def chains(goal: Mapping[str, Sequence[Sequence[str]]]) -> dict[str, list[list[str]]]:
    """Each object's goals, lower-cased, cut before the first unsupported
    one (the checker never counts past it)."""
    out = {}
    for obj, specs in goal.items():
        kept = []
        for spec in specs:
            spec = [str(spec[0]).lower(), *(str(a) for a in spec[1:])]
            if spec[0] not in GOAL_CAPABILITIES:
                break
            kept.append(spec)
        out[obj] = kept
    return out


def observed_atoms(
    progress: Mapping[str, int], held: str | None, gripper: str = GRIPPER
) -> list[str]:
    """The world as the domain's predicates: each object's counter (the
    env's ``progress``) and what the hand holds (an object without goals at
    ``s0``)."""
    atoms = [f"progress({obj}, {stage(int(k))})" for obj, k in progress.items()]
    if held is not None:
        atoms.append(grasped(held, gripper))
        if held not in progress:
            atoms.append(f"progress({held}, {stage(0)})")
    return atoms


class RoboCerebraDomain(BaseMissionDomain):
    """One RoboCerebra case: one Panda hand, the case's objects, and their
    chains of goals (``goal``: ``load``'s ``goal``, per object its
    predicates in order)."""

    name = "robocerebra"
    goal_notes = (
        "progress(object, sK) says K of the object's goals are reached; its "
        "goal number K is goal_pick, goal_place or goal_pour at sK. "
        "grasped(gripper, object) while the hand holds it."
    )
    intro = (
        "A Panda arm in a RoboCerebra scene moves objects between regions and "
        "pours them over receivers, reaching each object's goals in order."
    )

    def __init__(
        self,
        goal: Mapping[str, Sequence[Sequence[str]]],
        case: str = "robocerebra",
        gripper: str = GRIPPER,
        others: Iterable[str] = (),
        lookahead: bool = True,
    ) -> None:
        self.case = case
        self.gripper = gripper
        self.full_goal = {obj: [list(s) for s in specs] for obj, specs in goal.items()}
        self.chains = chains(goal)
        #: The scene's other objects: no goals, but the hand may hold one.
        self.others = tuple(o for o in others if o not in self.full_goal)
        #: The objects given up on (``give_up``).
        self.given_up: set[str] = set()
        #: Whether a pick checks the place or pour after it (``plan_move``).
        self.lookahead = lookahead
        #: The objects left for last (``defer``), and those whose pick is
        #: no longer checked (``resume``).
        self.deferred: set[str] = set()
        self.unchecked: set[str] = set()

    @classmethod
    def from_info(
        cls, info: Mapping[str, Any], lookahead: bool = True
    ) -> RoboCerebraDomain:
        """From the env server's ``load`` reply."""
        return cls(
            info["goal"],
            case=str(info.get("case", "robocerebra")),
            others=info.get("objects", ()),
            lookahead=lookahead,
        )

    def give_up(self, obj: str) -> None:
        """Stop planning ``obj``'s goals: they leave the goal, and the
        object may be put down wherever its chain stands."""
        self.given_up.add(obj)

    def defer(self, obj: str) -> None:
        """Leave ``obj`` out of the goal until the others are done or given
        up on (its pick plans, the place after it does not, from here)."""
        self.deferred.add(obj)

    def resume(self) -> bool:
        """Put the deferred objects back in the goal, their picks no longer
        checked: a pick still counts, and the object is put down last.
        False when none was deferred."""
        if not self.deferred:
            return False
        self.unchecked |= self.deferred
        self.deferred.clear()
        return True

    def then(self, obj: str, at: str) -> SegmentGoal | None:
        """The place or pour after ``obj``'s pick at stage ``at``, if its
        pick is to be checked against it (``lookahead``)."""
        if not self.lookahead or obj in self.unchecked:
            return None
        specs = self.chains.get(obj, [])
        k = int(at[1:]) + 1
        if k >= len(specs) or GOAL_CAPABILITIES[specs[k][0]] == "pick":
            return None
        return SegmentGoal.from_spec(specs[k])

    # -- symbolic --------------------------------------------------------------

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        advance = ("progress(?object, ?next)", "not progress(?object, ?stage)")
        pick = CapabilityDescriptor(
            "pick",
            "1.0",
            {"gripper": str, "object": str, "stage": str, "next": str},
            preconditions=("not grasped(?gripper, _)", "progress(?object, ?stage)"),
            effects=("grasped(?gripper, ?object)", *advance),
            writes=("grasp_state",),
            restartable=True,
        )
        place = CapabilityDescriptor(
            "place",
            "1.0",
            {"gripper": str, "object": str, "region": str, "stage": str, "next": str},
            preconditions=("grasped(?gripper, ?object)", "progress(?object, ?stage)"),
            effects=("not grasped(?gripper, ?object)", *advance),
            writes=("grasp_state",),
            restartable=True,
        )
        pour = CapabilityDescriptor(
            "pour",
            "1.0",
            {
                "gripper": str,
                "object": str,
                "receiver": str,
                "stage": str,
                "next": str,
            },
            preconditions=("grasped(?gripper, ?object)", "progress(?object, ?stage)"),
            effects=advance,
            writes=("object_pose",),
            restartable=True,
        )
        regrasp = CapabilityDescriptor(
            "regrasp",
            "1.0",
            {"gripper": str, "object": str, "stage": str},
            preconditions=("not grasped(?gripper, _)", "progress(?object, ?stage)"),
            effects=("grasped(?gripper, ?object)",),
            writes=("grasp_state",),
            restartable=True,
        )
        putdown = CapabilityDescriptor(
            "putdown",
            "1.0",
            {"gripper": str, "object": str, "stage": str},
            preconditions=("grasped(?gripper, ?object)", "progress(?object, ?stage)"),
            effects=("not grasped(?gripper, ?object)",),
            writes=("grasp_state",),
            restartable=True,
        )
        return {d.capability_id: d for d in (pick, place, pour, regrasp, putdown)}

    def objects(self) -> tuple[str, ...]:
        """The objects with goals (receivers are objects too), then the
        others."""
        found = dict.fromkeys((*self.full_goal, *self.others))
        for specs in self.chains.values():
            for spec in specs:
                if spec[0] == "pour":
                    found.setdefault(spec[2])
        return tuple(found)

    def regions(self) -> tuple[str, ...]:
        found = dict.fromkeys(
            spec[2]
            for specs in self.chains.values()
            for spec in specs
            if GOAL_CAPABILITIES[spec[0]] == "place"
        )
        return tuple(found)

    def stages(self) -> tuple[str, ...]:
        longest = max((len(specs) for specs in self.chains.values()), default=0)
        return tuple(stage(k) for k in range(longest + 1))

    def static_facts(self) -> tuple[str, ...]:
        facts = [f"gripper({self.gripper})"]
        stages = self.stages()
        facts += [f"next({a}, {b})" for a, b in zip(stages, stages[1:])]
        facts += [f"rest({obj}, {stage(0)})" for obj in self.others]
        for obj, specs in self.chains.items():
            if obj in self.given_up:
                # No goals left: the hand only puts it down.
                facts += [f"rest({obj}, {stage(k)})" for k in range(len(specs) + 1)]
                continue
            facts.append(f"rest({obj}, {stage(len(specs))})")
            for k, spec in enumerate(specs):
                capability = GOAL_CAPABILITIES[spec[0]]
                if capability == "pick":
                    facts.append(f"goal_pick({obj}, {stage(k)})")
                    facts.append(f"rest({obj}, {stage(k)})")
                elif capability == "place":
                    facts.append(f"goal_place({obj}, {stage(k)}, {spec[2]})")
                else:
                    facts.append(f"goal_pour({obj}, {stage(k)}, {spec[2]})")
        return tuple(facts)

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        """The chains, which no step changes: the planner's only (the
        capabilities' run-time preconditions are what the world says)."""
        hand = "gripper(?gripper)"
        step = "next(?stage, ?next)"
        return {
            "pick": (hand, "goal_pick(?object, ?stage)", step),
            "place": (hand, "goal_place(?object, ?stage, ?region)", step),
            "pour": (hand, "goal_pour(?object, ?stage, ?receiver)", step),
            "regrasp": (hand, "not rest(?object, ?stage)"),
            "putdown": (hand, "rest(?object, ?stage)"),
        }

    def default_goal(self) -> list[str]:
        return self.goal()

    def goal(self, without: Iterable[str] = ()) -> list[str]:
        """Every object's counter at the end of its chain, but for the
        objects given up on, deferred, and ``without``."""
        skip = self.given_up | self.deferred | set(without)
        return [
            f"progress({obj}, {stage(len(specs))})"
            for obj, specs in self.chains.items()
            if specs and obj not in skip
        ]

    def initial_atoms(self) -> list[str]:
        """The start: no goal reached, the hand empty."""
        return observed_atoms({obj: 0 for obj in self.full_goal}, None, self.gripper)

    def unsupported(self) -> dict[str, list[list[str]]]:
        """Per object, the goals cut from its chain."""
        return {
            obj: specs[len(self.chains[obj]) :]
            for obj, specs in self.full_goal.items()
            if len(specs) > len(self.chains[obj])
        }

    def goal_vocabulary(self, state: Iterable[str | Atom] = ()) -> Vocabulary:
        return Vocabulary.from_domain(
            self.descriptors(),
            [str(a) for a in state],
            objects=(*self.objects(), *self.regions(), *self.stages(), self.gripper),
            notes=self.goal_notes,
        )

    def scene_info(self) -> dict[str, Any]:
        return {"id": self.name, "case": self.case, "objects": list(self.objects())}

    def expand_step(
        self,
        index: int,
        capability: str,
        parameters: dict[str, Any],
        held: Mapping[str, str],
    ) -> list[tuple[str, dict[str, Any], str]]:
        """One transaction per step, labelled without the counters."""
        shown = [
            str(parameters[k])
            for k in ("object", "region", "receiver")
            if k in parameters
        ]
        return [(capability, parameters, f"{capability} {' '.join(shown)}")]

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> Move:
        """A motion the HPP policy plans when the step runs (``plan_move``)."""
        binding = {k: v for k, v in parameters.items() if k != "block"}
        label = self.expand_step(0, capability, dict(binding), {})[0][2]
        return Move(label, {"capability": capability, "parameters": binding})

    # -- repair --------------------------------------------------------------

    def repair_policy(self, failure: Mapping[str, Any]) -> list[Any]:
        """Nothing to block: every goal names its region or receiver, so a
        failed binding is the only one. A replan from the observed world is
        the repair (the geometry samples again)."""
        return []

    def retry(self, failure: Mapping[str, Any]) -> bool:
        """Plan again from the world as observed after an execution miss (a
        stalled grasp, a goal not reached). A step that did not plan
        (``refinement_failed``, ``lookahead_failed``) is not retried: the
        planner is given the same geometry again, and in row D none of 235
        such retries planned. Nor is one past the budget or with no way to
        reach it (``unsupported``)."""
        final = (
            "budget_spent(",
            "unsupported(",
            "refinement_failed(",
            "lookahead_failed(",
        )
        return not any(f.startswith(final) for f in failure["facts"])

    # -- geometry and execution ----------------------------------------------

    def world_state(self, scene: MissionScene, recorded: Any) -> Any:
        """The env's counters and what the fingers hold, as atoms."""
        world: EnvWorld = scene.extras["world"]
        known = set(self.objects())

        def observed() -> list[str]:
            held = world.held()
            return observed_atoms(
                world.progress(), held if held in known else None, self.gripper
            )

        return CompositeWorldState(observed, recorded)

    def plan_move(
        self,
        scene: MissionScene,
        move: Move,
        q: Sequence[float],
        verbose: bool = True,
    ) -> dict[str, Any]:
        """Plan the step from the scene as it stands: HPP for a pick, place
        or pour (a ``regrasp`` is a pick that the checker does not count),
        a :class:`ReleaseController` for a ``putdown``. The one path is an
        :class:`EnvMotion` saying what the step must leave behind. A pick
        is checked against the place or pour after it (:meth:`then`); when
        that does not plan, the step fails with ``lookahead_failed``."""
        world: EnvWorld = scene.extras["world"]
        capability = move.spec["capability"]
        parameters = move.spec["parameters"]
        obj = str(parameters["object"])
        record: dict[str, Any] = {"step": move.label}
        expect: dict[str, Any]
        if capability == "putdown":
            motion = EnvMotion(ReleaseController(info=record), {"released": obj})
            return _planned(motion, "release where the hand is", q)
        if capability in ("pick", "regrasp"):
            goal = SegmentGoal("grasped", obj)
        elif capability == "place":
            goal = SegmentGoal("placed", obj, str(parameters["region"]))
        else:
            goal = SegmentGoal("poured", obj, receiver=str(parameters["receiver"]))
        if capability == "regrasp":
            expect = {"held": obj}
        else:
            expect = {"progress": (obj, int(str(parameters["next"])[1:]))}
        then = None
        if capability == "pick":
            then = self.then(obj, str(parameters["stage"]))
        policy: HppPolicy = scene.extras["policy"]
        name = move.label.replace(" ", "_").replace("/", "_")
        started = time.time()
        try:
            plan = policy.plan(world.env, goal, name, record, then=then)
        except UnsupportedGoal as error:
            result = _unplanned(f"unsupported({obj})", str(error), q, record)
        except LookaheadFailed as error:
            record.update(error.info)
            result = _unplanned(
                f"lookahead_failed({obj})", f"LookaheadFailed: {error}", q, record
            )
        except Exception as error:
            record.update(getattr(error, "info", {}))
            result = _unplanned(
                f"refinement_failed({name})",
                f"{type(error).__name__}: {error}",
                q,
                record,
            )
        else:
            record.update(plan.info, plan_seconds=round(time.time() - started, 3))
            controller = SegmentController(plan.stages, plan.tcp_start, record)
            return _planned(EnvMotion(controller, expect), "planned", q)
        world.unplanned.append(
            {**record, "message": result["message"], "facts": result["facts"]}
        )
        return result


def _planned(motion: EnvMotion, message: str, q: Sequence[float]) -> dict[str, Any]:
    return {
        "success": True,
        "final_config": list(q),
        "resumes": 0,
        "replans": 0,
        "message": message,
        "phase_results": [{"paths": [motion], "edges": [None]}],
    }


def _unplanned(
    fact: str, message: str, q: Sequence[float], record: dict[str, Any]
) -> dict[str, Any]:
    return {
        "success": False,
        "final_config": list(q),
        "resumes": 0,
        "replans": 0,
        "message": message,
        "facts": [fact],
        "info": record,
    }


@dataclass
class EnvWorld:
    """The env as the mission sees it: the checker's counters and the
    object both fingers touch, read again only after the env stepped, and
    the env steps left of the rollout's ``budget``. ``unplanned`` keeps the
    record of each step that did not plan, which the runner drops."""

    env: Any
    budget: int
    obs: dict[str, Any] = field(default_factory=dict)
    used: int = 0
    result: dict[str, Any] | None = None
    unplanned: list[dict[str, Any]] = field(default_factory=list)
    _progress: dict[str, int] | None = field(default=None, repr=False)
    _held: tuple[str | None] | None = field(default=None, repr=False)

    @property
    def left(self) -> int:
        return self.budget - self.used

    def progress(self) -> dict[str, int]:
        if self._progress is None:
            self.result = self.env.check()
            self._progress = dict(self.result["progress"])
        return dict(self._progress)

    def held(self) -> str | None:
        if self._held is None:
            self._held = (held_object(self.env.export_scene()["manifest"]),)
        return self._held[0]

    @property
    def all_done(self) -> bool:
        self.progress()
        return bool(self.result and self.result["all_done"])

    def step(self, action: Any) -> dict[str, Any]:
        """One env step, within the budget."""
        if self.left <= 0:
            raise RuntimeError("the rollout's budget is spent")
        result = self.env.step(np.asarray(action, dtype=float))
        self.used += 1
        self.obs = result.get("obs", self.obs)
        self.result = result
        self._progress = dict(result["progress"])
        self._held = None
        return result


@dataclass
class EnvMotion:
    """A planned step for the env: the controller, and what must hold once
    it is done (``progress``: an object's counter reached; ``held``: the
    object in the hand; ``released``: not in it)."""

    controller: Callable[[dict[str, Any]], Any]
    expect: Mapping[str, Any]

    def length(self) -> None:
        """No duration: the backend sends a heartbeat every few env steps."""
        return None


class RoboCerebraBackend:
    """Runs :class:`EnvMotion` commands in the env (the execution
    contract): ``chunk`` env steps a poll, until the controller is done, the
    case is done, the step has used ``max_steps`` (a tracker that does not
    converge) or the budget is spent. A step that ends without what it must
    leave behind fails with ``goal_missed(object, sK)``,
    ``grasp_missed(gripper, object)`` or, cut at ``max_steps``,
    ``stalled(gripper)``, which the mission replans from."""

    def __init__(
        self,
        world: EnvWorld,
        chunk: int = 10,
        gripper: str = GRIPPER,
        max_steps: int = 150,
    ):
        self.world = world
        self.chunk = chunk
        self.gripper = gripper
        self.max_steps = max_steps
        self.motion: EnvMotion | None = None
        self.steps = 0
        #: One record per command: its step, env steps and outcome.
        self.log: list[dict[str, Any]] = []

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        self.motion = command.payload
        self.steps = 0
        self.log.append({"step": command.step_id, "steps_used": 0})
        return ExecutionStatus.RUNNING

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        motion = self.motion
        if motion is None:
            return ExecutionStatus.BUSY, None
        world, controller = self.world, motion.controller
        for _ in range(self.chunk):
            if getattr(controller, "done", True) or world.all_done:
                return self._finish()
            if self.steps >= self.max_steps:
                return self._finish(stalled=True)
            if world.left <= 0:
                return self._end(
                    ExecutionStatus.FAILURE, "budget spent", ("budget_spent()",)
                )
            world.step(controller(world.obs))
            self.steps += 1
        self.log[-1]["steps_used"] = self.steps
        return ExecutionStatus.RUNNING, Feedback(message=f"{self.steps} env steps")

    def cancel(self) -> None:
        if self.motion is not None:
            self._end(ExecutionStatus.FAILURE, "cancelled")

    def _finish(self, stalled: bool = False) -> tuple[ExecutionStatus, Feedback]:
        """What the step left behind; cut short (``stalled``), it still
        succeeds if that holds (a retreat that does not converge after the
        release counted)."""
        assert self.motion is not None
        expect, world = self.motion.expect, self.world
        missed: tuple[str, ...] = ()
        if "progress" in expect:
            obj, k = expect["progress"]
            if world.progress().get(obj, 0) < k:
                missed = (f"goal_missed({obj}, {stage(k - 1)})",)
        elif "held" in expect and world.held() != expect["held"]:
            missed = (f"grasp_missed({self.gripper}, {expect['held']})",)
        elif "released" in expect and world.held() == expect["released"]:
            missed = (f"still_held({self.gripper}, {expect['released']})",)
        if missed:
            if stalled:
                missed = (f"stalled({self.gripper})", *missed)
            return self._end(ExecutionStatus.FAILURE, "missed", missed)
        return self._end(ExecutionStatus.SUCCESS, "stalled" if stalled else "done")

    def _end(
        self, status: ExecutionStatus, message: str, facts: tuple[str, ...] = ()
    ) -> tuple[ExecutionStatus, Feedback]:
        assert self.motion is not None
        info = getattr(self.motion.controller, "info", {})
        self.log[-1].update(
            steps_used=self.steps,
            status=status.value,
            message=message,
            facts=list(facts),
            **({"policy": dict(info)} if info else {}),
        )
        self.motion = None
        return status, Feedback(message=message, facts=facts)


def mission_scene(env: Any, policy: HppPolicy, budget: int) -> MissionScene:
    """The scene the mission runs in: the env (``extras["world"]``) and the
    policy refining each step; no configuration of its own (the env holds
    the robot's)."""
    world = EnvWorld(env, budget)
    return MissionScene(
        task=None,
        planner=KinematicPlanner(),
        q_init=[],
        extras={"world": world, "policy": policy},
    )


def run_mission_case(
    env: Any,
    info: Mapping[str, Any],
    seed: int,
    policy: HppPolicy,
    *,
    step_factor: int = 150,
    retries: int = 2,
    lookahead: bool = True,
    runner_factory: Callable[..., Any] | None = None,
) -> dict[str, Any]:
    """One rollout of a loaded case as one mission (row D).

    Reset as :func:`~long_tamp.eval.robocerebra_runner.evaluate_case` does,
    then plan the whole goal from the observed world and run it with
    ``run_with_repair``: a failed step is planned again from where the
    world is, up to ``retries`` times; past that its object is given up on
    and the rest of the goal is planned again, until nothing is left, the
    case is done or the ``step_factor`` x task steps budget is spent.

    With ``lookahead``, an object whose pick plans but whose place or pour
    after it does not is deferred: the others go first, then it is picked
    without the check (the pick counts) and put down when its place fails.
    """
    from long_tamp.mission.runner import MissionRunner

    obs = env.reset(step=0, seed=int(seed)).get("obs") or {}
    domain = RoboCerebraDomain.from_info(info, lookahead=lookahead)
    budget = step_factor * len(info["steps"])
    scene = mission_scene(env, policy, budget)
    world: EnvWorld = scene.extras["world"]
    world.obs = obs
    backend = RoboCerebraBackend(world, gripper=domain.gripper, max_steps=step_factor)
    make = runner_factory or MissionRunner
    runner = make(domain, scene, verbose=False)
    rounds: list[dict[str, Any]] = []
    started = time.time()
    goals = sum(len(specs) for specs in domain.chains.values())
    deferred: list[str] = []
    while world.left > 0 and not world.all_done:
        if not domain.goal():
            if domain.resume():
                continue
            break
        result = runner.run_with_repair(
            None,
            rounds=goals * (retries + 1) + 1,
            mission={"backend": backend},
            goal=domain.goal(),
            retries=retries,
        )
        failure = result.get("failure") or {}
        rounds.append(
            {
                "success": result["success"],
                "message": result["repair_message"],
                "task_replans": result["task_replans"],
                "retries": result["retries"],
                "failure": failure or None,
                "blocks": result["blocks"],
            }
        )
        if result["success"]:
            if domain.resume():
                continue
            break
        obj = failure.get("parameters", {}).get("object")
        if obj not in domain.chains or obj in domain.given_up:
            break  # nothing left to give up on
        if any(f.startswith("lookahead_failed(") for f in failure.get("facts", ())):
            if obj not in domain.unchecked:
                domain.defer(obj)
                deferred.append(obj)
                continue
        domain.give_up(obj)
    final = env.check()
    total = final["total"]
    return {
        "seed": int(seed),
        "success": bool(final["all_done"]),
        "total_completed": final["total_completed"],
        "total": total,
        "completed_share": (
            round(final["total_completed"] / total, 4) if total else 0.0
        ),
        "segments": [],
        "mission": {
            "budget": budget,
            "steps_used": world.used,
            "seconds": round(time.time() - started, 2),
            "given_up": sorted(domain.given_up),
            "deferred": deferred,
            "unsupported": domain.unsupported(),
            "rounds": rounds,
            "executions": backend.log,
            "unplanned": world.unplanned,
        },
    }


def evaluate_mission_case(
    env: Any,
    case: str,
    *,
    seeds: Sequence[int],
    policy: HppPolicy | None = None,
    step_factor: int = 150,
    retries: int = 2,
    lookahead: bool = True,
) -> dict[str, Any]:
    """:func:`run_mission_case` over ``seeds``, as an ``evaluate_case`` row
    (``protocol: "mission"``) that ``summarize`` and ``merge_results`` take."""
    info = env.load(case)
    policy = policy or HppPolicy()
    rollouts = [
        run_mission_case(
            env,
            info,
            seed,
            policy,
            step_factor=step_factor,
            retries=retries,
            lookahead=lookahead,
        )
        for seed in seeds
    ]
    return {
        "case": case,
        "model": info.get("model", "bddl"),
        "step_limit": step_factor,
        "resume": False,
        "protocol": "mission",
        "rollouts": rollouts,
        "success_rate": (
            sum(r["success"] for r in rollouts) / len(rollouts) if rollouts else 0.0
        ),
    }


__all__ = [
    "GOAL_CAPABILITIES",
    "EnvMotion",
    "EnvWorld",
    "RoboCerebraBackend",
    "RoboCerebraDomain",
    "chains",
    "evaluate_mission_case",
    "mission_scene",
    "observed_atoms",
    "run_mission_case",
    "stage",
]
