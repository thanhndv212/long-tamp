"""An HPP policy for RoboCerebra's segmented protocol (#159, row A "Ideal").

Per task segment, :class:`HppPolicy` reads the segment's goal predicates
from the case's ``goal.json`` (as the env server loads it), exports the
scene as it stands, plans the segment in HPP
(:class:`~long_tamp.eval.robocerebra_plan.SegmentPlanner`) and returns a
:class:`SegmentController`, which tracks the plan with ``OSC_POSE`` actions
(:class:`~long_tamp.eval.robocerebra_tracker.OscTracker`) from each
observation. :func:`~long_tamp.eval.robocerebra_runner.evaluate_case` steps
it under the benchmark's budget and resume.

Predicates map onto what the planner does:

``pick``
    ``grasped``: grasp one of the object's generated handles and lift it.
    A flat object none of whose handles plans (a plate) is pushed to the
    table's edge and pinched by its overhang instead
    (:mod:`~long_tamp.eval.robocerebra_push`).
``in`` / ``on``
    ``placed``: the object is in the hand (the official resume puts the
    demo's state at the segment start); carry it over the region, release.
    ``on`` over another object releases it on that object's top
    (:func:`~long_tamp.eval.robocerebra_plan.object_top_region`).
``pour``
    ``poured``: the object is in the hand, as for a place (a pour always
    follows the pick of what it pours); carry it over the receiving object,
    tilt it and turn it back.

Anything else (``open``, ``close``, ``turnon``) is rejected: the segment holds
still with the gripper closed, so an object a later segment resumes in the
hand is not dropped while the gripper command ramps (robosuite keeps the
gripper command across the resume).

With ``recover`` (row C, the continuous variant, where no resume repairs
the scene between segments) the policy plans the next action towards the
goal from the scene as it is: an object in the hand that the goal does not
want is released (:class:`ReleaseController`), and an object to place or
pour that is not in the hand is picked first. The runner calls the policy
again when that controller is done, within the segment's budget.

HPP keeps native memory from every problem it builds, so a long run in one
process grows by a few hundred MB a segment; by default each segment is
planned in a forked child process (``isolate``), which returns the plan.
"""

from __future__ import annotations

import logging
import multiprocessing
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from long_tamp.grasping import xyzquat_to_pose

from .robocerebra_plan import PlanningError, object_top_region, tcp_to_eef
from .robocerebra_push import EdgePinch, EdgePinchController, edge_pinch
from .robocerebra_tracker import POS_SCALE, OscTracker, Stage, obs_pose

logger = logging.getLogger(__name__)

PREDICATES = {"pick": "grasped", "in": "placed", "on": "placed", "pour": "poured"}
ASSETS = Path(__file__).resolve().parents[3] / "script" / "twin" / "assets"


class UnsupportedGoal(ValueError):
    """A goal predicate the planner has no translation for."""


@dataclass(frozen=True)
class SegmentGoal:
    """One goal predicate in planner terms: ``grasped`` an object,
    ``placed`` it in ``region`` (a region name of the exported manifest, or
    for ``on`` an object's), or ``poured`` it over ``receiver`` (an object
    name)."""

    kind: str
    obj: str
    region: str | None = None
    receiver: str | None = None

    @classmethod
    def from_spec(cls, spec: list[str]) -> SegmentGoal:
        """Translate one ``goal.json`` predicate, e.g. ``["in", obj, region]``."""
        name = str(spec[0]).lower()
        if name not in PREDICATES:
            raise UnsupportedGoal(
                f"predicate {spec[0]!r} is not supported ({list(spec)})"
            )
        kind = PREDICATES[name]
        if kind == "grasped":
            if len(spec) != 2:
                raise UnsupportedGoal(f"pick takes one object: {list(spec)}")
            return cls(kind, str(spec[1]))
        if kind == "poured":
            if len(spec) != 3:
                raise UnsupportedGoal(f"pour takes two objects: {list(spec)}")
            return cls(kind, str(spec[1]), receiver=str(spec[2]))
        if len(spec) != 3:
            raise UnsupportedGoal(f"{name} takes an object and a region: {list(spec)}")
        return cls(kind, str(spec[1]), str(spec[2]))


def segment_specs(info: dict[str, Any], segment: int) -> list[list[str]]:
    """The goal predicates annotated for task step ``segment`` (``load``'s
    ``goal`` and ``goal_steps``: per object, its predicates and their steps)."""
    specs = []
    for obj, steps in info.get("goal_steps", {}).items():
        for index, step in enumerate(steps):
            if step == segment:
                specs.append(list(info["goal"][obj][index]))
    return specs


@dataclass
class SegmentController:
    """Closed-loop actions for one segment: ``controller(obs) -> (7,)``.

    Tracks ``stages`` from the first observation, which fixes the planner
    TCP's offset to the observed end effector (same joints at the segment
    start). With no stages (planning failed, or the goal is unsupported) it
    holds still, gripper ``hold_gripper``. ``info`` goes into the runner's
    record of the segment, with the target's observed position at the first
    and the latest step (``target_start`` / ``target_end``).
    """

    stages: list[Stage] | None
    tcp_start: np.ndarray | None = None
    info: dict[str, Any] = field(default_factory=dict)
    hold_gripper: float = 1.0
    tracker: OscTracker | None = field(default=None, init=False)

    @property
    def done(self) -> bool:
        """Whether every stage has run (holding still counts as done), or
        tracking stalled (:attr:`stalled`)."""
        return not self.stages or (self.tracker is not None and self.tracker.done)

    @property
    def stalled(self) -> bool:
        """Whether tracking gave up in a stage it made no progress in."""
        return self.tracker is not None and self.tracker.stalled is not None

    def __call__(self, obs: dict[str, Any]) -> np.ndarray:
        position = obs.get(f"{self.info.get('target')}_pos")
        if position is not None:
            position = [round(float(v), 4) for v in position]
            self.info.setdefault("target_start", position)
            self.info["target_end"] = position
        if not self.stages:
            action = np.zeros(7)
            action[3] = 1e-6  # nonzero: re-anchors OSC's orientation goal
            action[6] = self.hold_gripper
            return action
        if self.tracker is None:
            self.tracker = OscTracker(
                self.stages, tcp_to_eef(self.tcp_start, obs_pose(obs))
            )
        action = self.tracker(obs)
        self.info["stage"] = self.tracker.stage_name
        self.info["stages"] = self.tracker.log
        return action


@dataclass
class ReleaseController:
    """Open the gripper where the hand is, then rise ``lift`` metres: drops
    an object the goal does not want in the hand. Closed loop on the
    observed end effector; ``done`` once risen, or after ``max_steps``."""

    info: dict[str, Any] = field(default_factory=dict)
    open_steps: int = 10
    lift: float = 0.12
    max_steps: int = 40
    steps: int = field(default=0, init=False)
    start: np.ndarray | None = field(default=None, init=False)
    risen: bool = field(default=False, init=False)

    @property
    def done(self) -> bool:
        return self.risen or self.steps >= self.max_steps

    def __call__(self, obs: dict[str, Any]) -> np.ndarray:
        position = np.asarray(obs["robot0_eef_pos"], dtype=float)
        if self.start is None:
            self.start = position.copy()
        goal = self.start.copy()
        if self.steps >= self.open_steps:
            goal[2] += self.lift
            self.risen = position[2] >= goal[2] - 0.01
        self.steps += 1
        action = np.zeros(7)
        action[:3] = np.clip((goal - position) / POS_SCALE, -1.0, 1.0)
        action[3] = 1e-6  # nonzero: re-anchors OSC's orientation goal
        action[6] = -1.0
        self.info["stage"] = "release" if self.steps <= self.open_steps else "rise"
        return action


class HppPolicy:
    """``policy(env, info, segment, step_limit) -> SegmentController``.

    Plans one segment from the scene the env server exports at its start.
    Only the segment's first predicate is planned; the others are recorded.
    With ``isolate``, the planning runs in a forked child process. With
    ``recover``, a hand holding the wrong object (or none, for a place or
    pour) is put right first: see the module notes.
    """

    def __init__(
        self,
        assets: str | Path = ASSETS,
        work_dir: str | Path | None = None,
        timeout: float = 20.0,
        max_iterations: int = 3000,
        lift: float = 0.10,
        isolate: bool = True,
        recover: bool = False,
    ) -> None:
        self.assets = Path(assets)
        self.work_dir = (
            Path(work_dir) if work_dir else Path(tempfile.mkdtemp(prefix="rc_policy_"))
        )
        self.timeout = timeout
        self.max_iterations = max_iterations
        self.lift = lift
        self.isolate = isolate
        self.recover = recover
        self._calls = 0
        self._robot: tuple[str, str] | None = None

    def __call__(
        self, env: Any, info: dict[str, Any], segment: int, step_limit: int
    ) -> SegmentController | ReleaseController | EdgePinchController:
        specs = segment_specs(info, segment)
        record: dict[str, Any] = {"goals": specs}
        if not specs:
            record["status"] = "no goal"
            return SegmentController(None, info=record)
        if len(specs) > 1:
            record["unplanned"] = specs[1:]
        try:
            goal = SegmentGoal.from_spec(specs[0])
        except UnsupportedGoal as error:
            record.update(status="unsupported", error=str(error))
            return SegmentController(None, info=record)
        start = time.time()
        try:
            plan = self.plan(env, goal, f"s{segment}", record)
            if plan is None:
                return ReleaseController(info=record)
        except UnsupportedGoal as error:  # the scene has no such region
            record.update(status="unsupported", error=str(error))
            return SegmentController(None, info=record)
        except Exception as error:
            record.update(
                status="planning failed", error=f"{type(error).__name__}: {error}"
            )
            record.update(getattr(error, "info", {}))
            record["plan_seconds"] = round(time.time() - start, 3)
            logger.warning("segment %d %s: %s", segment, specs[0], error)
            return SegmentController(None, info=record)
        record.update(
            plan.info, status="planned", plan_seconds=round(time.time() - start, 3)
        )
        return plan_controller(plan, record)

    def plan(
        self,
        env: Any,
        goal: SegmentGoal,
        name: str,
        record: dict[str, Any],
        then: SegmentGoal | None = None,
    ) -> Any:
        """The plan reaching ``goal`` from the scene as it stands (exported
        under ``name``), or ``None`` to release what the hand holds (with
        ``recover`` only). ``record`` receives what the hand held and the
        handles generated. A pick none of whose handles plans is an
        :class:`~long_tamp.eval.robocerebra_push.EdgePinch` when the object
        is flat (the HPP failure in ``record["hpp"]``); :func:`plan_controller`
        runs either.

        ``then``, the place or pour that follows a pick, is a lookahead: it
        is planned too, from the scene the pick leaves (the object held at
        the top of the lift), and when it does not plan the pick fails with
        a :class:`LookaheadFailed`. A ``then`` the scene has no region or
        receiver for is not checked: it fails on its own."""
        from .robocerebra_plan import held_object, robot_files
        from .robocerebra_scene import held_export

        if self._robot is None:
            self._robot = robot_files(self.assets)
        self._calls += 1  # a fresh directory per call: a segment may replan
        scene = env.export_scene(self.work_dir / f"{name}-{self._calls}")
        held = held_object(scene.manifest)
        record["held"] = held
        if self.recover and held is not None and held != goal.obj:
            record.update(status="release", recovery=f"release {held}")
            return None
        if self.recover and goal.kind != "grasped" and held is None:
            record["recovery"] = f"pick {goal.obj}"
            goal = SegmentGoal("grasped", goal.obj)
        if goal.kind == "grasped":
            if held is not None:
                raise RuntimeError(f"{held} is already in the hand")
            region = None
        elif held != goal.obj:
            raise RuntimeError(f"{goal.obj} is not in the hand (holding {held})")
        else:
            region = _region(scene, goal)
        try:
            plan, handles = self._solve(scene.root, goal, region)
        except PlanningError as error:
            if goal.kind != "grasped":
                raise
            try:
                way = edge_pinch(scene, goal.obj)
            except PlanningError:
                raise error from None
            record["hpp"] = error.info
            return way  # scripted from the observations: no lookahead
        if handles is not None:
            record["handles"] = handles
        if goal.kind != "grasped" or then is None or plan.held_after is None:
            return plan
        try:
            region = _region(scene, then)
        except UnsupportedGoal:
            return plan
        after = plan.held_after
        ahead = held_export(
            scene, goal.obj, after["joints"], after["pose"], f"{scene.root}-then"
        )
        start = time.time()
        try:
            self._solve(ahead.root, then, region)
        except Exception as error:
            lookahead = {
                "then": [then.kind, then.region or then.receiver],
                "message": f"{type(error).__name__}: {error}",
                **getattr(error, "info", {}),
                "plan_seconds": round(time.time() - start, 3),
            }
            raise LookaheadFailed(
                f"{then.kind} of {goal.obj} after the pick does not plan",
                {**plan.info, "lookahead": lookahead},
            ) from None
        plan.info["lookahead"] = {
            "then": [then.kind, then.region or then.receiver],
            "plan_seconds": round(time.time() - start, 3),
        }
        return plan

    def _solve(
        self, root: Path, goal: SegmentGoal, region: dict[str, Any] | None
    ) -> tuple[Any, Any]:
        assert self._robot is not None
        urdf, srdf = self._robot
        args = (
            root,
            urdf,
            srdf,
            goal.obj,
            region,
            self.timeout,
            self.max_iterations,
            self.lift,
            goal.receiver,
        )
        return _in_child(_solve, *args) if self.isolate else _solve(*args)


class LookaheadFailed(PlanningError):
    """A pick that plans, but the place or pour after it does not, from the
    scene the pick leaves: ``info["lookahead"]`` says why."""


def plan_controller(
    plan: Any, record: dict[str, Any]
) -> SegmentController | EdgePinchController:
    """The controller running a plan from :meth:`HppPolicy.plan`, with
    ``record`` as its ``info``."""
    if isinstance(plan, EdgePinch):
        return EdgePinchController(plan, info=record)
    return SegmentController(plan.stages, plan.tcp_start, record)


def _region(scene: Any, goal: SegmentGoal) -> dict[str, Any] | None:
    """The region a place goes to, the exported manifest's or, failing
    that, an object's top (None for a pick or a pour);
    :class:`UnsupportedGoal` when the scene has no such region or receiver."""
    manifest = scene.manifest
    if goal.kind == "poured":
        if goal.receiver not in manifest.get("objects", {}):
            raise UnsupportedGoal(f"no object {goal.receiver!r} in the scene")
        return None
    if goal.kind != "placed":
        return None
    regions = {region["region"]: region for region in manifest.get("regions", [])}
    if goal.region in regions:
        return regions[goal.region]
    if goal.region in manifest.get("objects", {}) and goal.region != goal.obj:
        return object_top_region(
            goal.region,
            scene.file_paths()["objects"][goal.region]["urdf"],
            xyzquat_to_pose(scene.object_config(goal.region)),
        )
    raise UnsupportedGoal(f"no region {goal.region!r} in the scene")


def _solve(
    scene_dir: Path,
    urdf: str,
    srdf: str,
    target: str,
    region: dict[str, Any] | None,
    timeout: float,
    max_iterations: int,
    lift: float,
    receiver: str | None = None,
) -> tuple[Any, Any]:
    """Plan a pick of ``target`` (no ``region`` nor ``receiver``), its place
    into ``region`` or its pour over ``receiver`` on the exported scene: the
    plan and, for a pick, the handles generated."""
    from .robocerebra_grasps import add_grasp_handles
    from .robocerebra_plan import SegmentPlanner
    from .robocerebra_scene import load_export

    scene = load_export(scene_dir)
    if region is None and receiver is None:
        handles = add_grasp_handles(scene, objects=[target]).get(target)
        planner = SegmentPlanner(
            scene, urdf, srdf, target, timeout=timeout, max_iterations=max_iterations
        )
        return planner.plan_pick(lift=lift), handles
    planner = SegmentPlanner(
        scene,
        urdf,
        srdf,
        target,
        held=True,
        timeout=timeout,
        max_iterations=max_iterations,
    )
    if receiver is not None:
        return planner.plan_pour(receiver), None
    return planner.plan_place(region), None


def _in_child(function: Any, *args: Any) -> Any:
    """``function(*args)`` in a forked process, so the memory it takes goes
    with the process. Exceptions come back as their message (and a
    :class:`PlanningError`'s ``info``)."""
    context = multiprocessing.get_context("fork")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(
        target=_child_main, args=(sender, function, args), daemon=True
    )
    process.start()
    sender.close()
    try:
        kind, payload = receiver.recv()
    except EOFError:
        process.join()
        raise RuntimeError(
            f"the planning process died (exit code {process.exitcode})"
        ) from None
    finally:
        receiver.close()
    process.join()
    if kind == "ok":
        return payload
    name, message, info = payload
    if info is not None:
        raise PlanningError(message, info)
    raise RuntimeError(f"{name}: {message}")


def _child_main(sender: Any, function: Any, args: tuple[Any, ...]) -> None:
    try:
        result = ("ok", function(*args))
    except BaseException as error:
        result = (
            "error",
            (type(error).__name__, str(error), getattr(error, "info", None)),
        )
    try:
        sender.send(result)
    finally:
        sender.close()
        # Skip interpreter teardown: HPP's destructors are slow and the
        # parent's atexit handlers and open files are not this process's.
        os._exit(0)


_DEFAULT: HppPolicy | None = None


def hpp_policy(
    env: Any, info: dict[str, Any], segment: int, step_limit: int
) -> SegmentController:
    """:class:`HppPolicy` with its defaults, for ``evaluate.py --policy``."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = HppPolicy()
    return _DEFAULT(env, info, segment, step_limit)


_CONTINUOUS: HppPolicy | None = None


def hpp_policy_continuous(
    env: Any, info: dict[str, Any], segment: int, step_limit: int
) -> SegmentController | ReleaseController:
    """:class:`HppPolicy` with ``recover``, for row C (``evaluate.py
    --continuous``)."""
    global _CONTINUOUS
    if _CONTINUOUS is None:
        _CONTINUOUS = HppPolicy(recover=True)
    return _CONTINUOUS(env, info, segment, step_limit)


__all__ = [
    "PREDICATES",
    "HppPolicy",
    "ReleaseController",
    "SegmentController",
    "SegmentGoal",
    "UnsupportedGoal",
    "plan_controller",
    "hpp_policy",
    "hpp_policy_continuous",
    "segment_specs",
]
