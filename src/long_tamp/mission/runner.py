"""The generic mission runner (ADR-0012, #132).

The planner -> refiner -> executor -> repair -> supervisor loop, for any
:class:`~long_tamp.mission.MissionDomain`. It was written once, against the
screw assembly (``script/screw_assembly/task_screw_assembly.py``); this is
the same loop with every screw fact moved behind the domain:

- :meth:`MissionRunner.run_mission` runs one TaskPlan: each step is refined
  (a grasp sequence on the ``refiner``, or the domain's ``plan_move``) and
  its motion executed on the backend, with drift replanning and missed-grasp
  recovery;
- :meth:`MissionRunner.run_with_repair` plans a goal, runs it, and replans
  around failures (``plan_execute_repair``);
- :meth:`MissionRunner.run_supervised` puts repair loops under the execution
  supervisor;
- :meth:`MissionRunner.plan_from_goal` and
  :meth:`MissionRunner.understand_instruction` plan from a goal or an
  operator's instruction.

The run folder (``MissionCheckpoint``: ``mission.json``, ``checkpoint.json``,
the per-phase dumps, ``trajectory.json``), the event stream and the result
dicts are those of the screw runner. Nothing here imports HPP: the scene's
planner and the refiner are duck-typed, so a test domain runs on a fake
planner and the ``MockBackend``.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from long_tamp.execution import ExecutionCommand, ExecutionPolicy, PlanExecutor
from long_tamp.tasks.mission_checkpoint import MissionCheckpoint
from long_tamp.tasks.refiner import RefinementStep
from long_tamp.tasks.task_planning import (
    CapabilityRegistry,
    RecordedFacts,
    TaskPlan,
    TaskPlanningSession,
    parallelize,
)
from long_tamp.tasks.task_planning.events import plan_event
from long_tamp.tasks.task_planning.predicates import holds, parse_atom, parse_state

from .domain import MissionDomain, MissionScene, Move

#: Seconds of path time between two samples of ``trajectory.json``.
TRAJECTORY_DT = 0.05


@dataclass
class _Run:
    """One ``run_mission`` call's running state."""

    q: list[float]
    trajectory: list[dict[str, Any]] | None = None
    checkpoint: MissionCheckpoint | None = None
    live_viewer: Any = None
    backend_displays: bool = False
    inject: list[dict[str, Any]] = field(default_factory=list)
    concurrent: bool = False
    records: list[dict[str, Any]] = field(default_factory=list)
    #: label -> the grasps held when the step started (to replan it from).
    step_start: dict[str, dict[str, str]] = field(default_factory=dict)
    #: label -> (capability, binding), for failures execution reports later.
    steps: dict[str, tuple[str, dict[str, Any]]] = field(default_factory=dict)
    block_index: int = 0
    failure: dict[str, Any] | None = None
    last_phases: list[dict[str, Any]] = field(default_factory=list)
    executor: PlanExecutor | None = None


class MissionRunner:
    """Runs missions of ``domain`` in ``scene`` (``domain.setup()``).

    ``recorded`` holds the facts only completed execution writes (default:
    empty, for the domain's ``recorded_predicates``); it persists across
    calls, as the world does. ``refiner`` refines grasp sequences (default: a
    ``GraspSequenceRefiner`` on ``scene.planner``, built per step).
    """

    def __init__(
        self,
        domain: MissionDomain,
        scene: MissionScene,
        recorded: RecordedFacts | None = None,
        refiner: Any = None,
        max_replans: int = 10,
        verbose: bool = True,
    ) -> None:
        self.domain = domain
        self.scene = scene
        self.recorded = (
            recorded
            if recorded is not None
            else RecordedFacts(None, predicates=domain.recorded_predicates)
        )
        self.refiner = refiner
        self.max_replans = max_replans
        self.verbose = verbose

    @property
    def planner(self) -> Any:
        return self.scene.planner

    # -- world and plans -----------------------------------------------------

    def world(self) -> Callable[[], Any]:
        """The world state (the domain's ``world_state``: by default the
        grasp tracker plus the recorded facts)."""
        return self.domain.world_state(self.scene, self.recorded)

    def world_atoms(self) -> list[str]:
        """The world state as sorted ground atoms."""
        return sorted(str(atom) for atom in parse_state(self.world()()))

    def plan_from_goal(
        self,
        state: list[str] | None = None,
        engine: str = "auto",
        blocked: list[tuple[str, dict[str, str]]] | None = None,
        goal: list[str] | None = None,
    ) -> dict[str, Any]:
        """Plan ``goal`` (default: the domain's) from ``state`` (default: the
        world) with a task planner, as a TaskPlan document."""
        from long_tamp.tasks.task_planning.skeleton import default_planner, planner_name

        state = self.world_atoms() if state is None else state
        t0 = time.time()
        task_planner = default_planner(engine)
        name = planner_name(task_planner)
        steps = task_planner.solve(self.domain.pddl_problem(state, blocked or [], goal))
        print(
            f"task planner ({name}): {len(steps)} steps in {time.time() - t0:.2f}s",
            flush=True,
        )
        return self.domain.planned_document(steps, state, generator=name)

    def understand_instruction(
        self,
        instruction: str,
        state: list[str] | None = None,
        model: str | None = None,
        on_call: Callable[..., Any] | None = None,
        client: Any = None,
    ) -> tuple[list[str], list[tuple[str, dict[str, str]]]]:
        """The goal and the constraints in an operator's ``instruction``
        (#22, #87), through three model roles on one client (``model``:
        ``"<api>:<model>"``):

        1. the grounder finds the objects the instruction refers to;
        2. the goal writer writes the goal, checked against the domain's
           vocabulary and reachable by the task planner from ``state``;
        3. the plan reviewer turns what the instruction rules out into
           blocked bindings, kept only if the planner still reaches the goal.

        No role writes steps. ``on_call`` receives each model call's record.
        """
        from long_tamp.ai import make_client
        from long_tamp.tasks.task_planning.language import (
            ModelGoalWriter,
            goal_from_instruction,
            ground_instruction,
            with_grounding,
        )
        from long_tamp.tasks.task_planning.review import (
            ReviewRequest,
            review_constraints,
        )
        from long_tamp.tasks.task_planning.skeleton import default_planner

        state = self.world_atoms() if state is None else state
        task_planner = default_planner("auto")

        def solve(goal, blocked=None):
            return task_planner.solve(
                self.domain.pddl_problem(state, blocked or [], goal)
            )

        def why_not(goal, blocked=None) -> str | None:
            return _verdict(lambda: solve(goal, blocked))

        client = client or make_client(model, on_call=on_call)
        vocabulary = self.domain.goal_vocabulary(state)
        t0 = time.time()
        grounding = ground_instruction(instruction, vocabulary, client)
        vocabulary = with_grounding(vocabulary, grounding.value)
        goal = goal_from_instruction(
            instruction, ModelGoalWriter(client), vocabulary, lambda g: why_not(g)
        )
        plan = tuple(
            f"{cap}({', '.join(f'{k}={v}' for k, v in params.items() if k != 'block')})"
            for cap, params in solve(goal)
        )
        review = review_constraints(
            ReviewRequest(instruction, vocabulary, self.domain.descriptors(), plan),
            client,
            replannable=lambda blocked: why_not(goal, blocked),
        )
        print(
            f"instruction {instruction!r} ({client.spec}, {time.time() - t0:.1f}s):\n"
            f"  refers to: {', '.join(grounding.value.objects) or '-'}"
            + (f" (fallback: {grounding.fallback})" if grounding.fallback else "")
            + "\n  goal:\n    "
            + "\n    ".join(goal)
            + "\n  constraints: "
            + (
                "; ".join(f"no {c}{b}" for c, b in review.value)
                if review.value
                else "none"
                + (f" (fallback: {review.fallback})" if review.fallback else "")
            ),
            flush=True,
        )
        for name, outcome in (("grounder", grounding), ("plan reviewer", review)):
            if outcome.fallback and outcome.attempts:
                print(f"  {name} attempts rejected:", flush=True)
                for attempt in outcome.attempts:
                    print(f"    {attempt}", flush=True)
        return goal, list(review.value)

    # -- one plan ------------------------------------------------------------

    def run_mission(
        self,
        q_start: Sequence[float] | None = None,
        document: dict[str, Any] | None = None,
        backend: Any = None,
        trajectory: list[dict[str, Any]] | None = None,
        checkpoint: MissionCheckpoint | None = None,
        live_viewer: Any = None,
        on_event: Callable[[dict[str, Any]], None] | None = None,
        inject: list[dict[str, Any]] | None = None,
        plan_ahead: bool = False,
        max_drift: float | None = None,
        concurrent: bool = False,
        control: Any = None,
    ) -> dict[str, Any]:
        """Run the TaskPlan ``document`` (default: the domain's hand-written
        ``plan_document()``) from ``q_start`` (default: the scene's start).

        Each step refines its transaction, then its motion executes on
        ``backend`` (None: planning only). A step whose effect already holds
        in the world is skipped, which is how a resume continues a run. With
        ``trajectory`` (a list), each completed step's motion is sampled into
        it (and written to ``checkpoint``'s ``trajectory.json``); with
        ``checkpoint``, every step is logged to the run folder. ``on_event``
        receives the event stream; ``inject`` holds ``--inject-failure``
        specs (``parse_injection``); ``control`` (an ``ExecutionControl``)
        pauses, resumes or stops the run between steps.
        """
        run = _Run(
            q=[
                float(v)
                for v in (q_start if q_start is not None else self.scene.q_init)
            ],
            trajectory=trajectory,
            checkpoint=checkpoint,
            live_viewer=live_viewer,
            backend_displays=getattr(backend, "display", None) is not None,
            inject=inject if inject is not None else [],
            concurrent=concurrent,
        )
        session = self.session(run, document)
        config_gen = getattr(self.planner, "config_gen", None)
        executor = PlanExecutor(
            session,
            backend=backend,
            policy=ExecutionPolicy(max_start_drift=max_drift),
            control=control,
            on_skip=lambda label, why: print(f"\n=== {label} === skipped ({why})"),
            on_event=on_event,
            on_drift=lambda node, observe: self._on_drift(run, node, observe),
            plan_ahead=plan_ahead,
            concurrent=concurrent,
            # merged motion of two lanes: checked for collisions as a whole
            validate_config=(
                (lambda q: config_gen.is_config_valid(list(q))[0])
                if config_gen is not None
                else None
            ),
        )
        run.executor = executor
        if on_event is not None:
            on_event(plan_event(session.plan))  # the viewer draws this plan
        t_mission = time.time()
        result = executor.run()
        if not result.success:
            print(f"\n=== {result.failed_step} stopped the mission: {result.message}")
        failure = None
        if not result.success:
            failure = (
                run.failure
                or execution_failure(result, run.steps)
                or stopped_failure(result, session)
            )
        if failure and self.domain.retry(failure) and backend is not None:
            self.recover_missed_grasp(run, backend, failure)
        return {
            "success": result.success,
            "seconds": round(time.time() - t_mission, 2),
            "blocks": run.records,
            "skipped": result.skipped,
            "executed": len(result.executions),
            "execution": execution_summary(result.executions, self.domain),
            "timing": result.timing or None,
            "final_config": run.q,
            "failure": failure,
        }

    def session(
        self, run: _Run, document: dict[str, Any] | None = None
    ) -> TaskPlanningSession:
        """``document`` bound to this scene: guards evaluated on the world,
        every other capability refined and executed by ``_run_step``."""
        domain = self.domain
        world = self.world()

        def runner(capability: str):
            def run_it(parameters: dict[str, Any]) -> dict[str, Any]:
                return self._run_step(run, capability, parameters)

            return run_it

        def check(descriptor):
            """A guard condition: true when its literals hold in the world."""

            def evaluate(parameters: dict[str, Any]) -> bool:
                state = parse_state(world())
                return all(
                    holds(literal.ground(parameters), state)
                    for literal in descriptor.precondition_literals
                )

            return evaluate

        registry = CapabilityRegistry()
        guards = domain.guards()
        for name, descriptor in domain.descriptors().items():
            registry.register(
                descriptor, check(descriptor) if name in guards else runner(name)
            )
        document = document or domain.plan_document()
        if document is None:
            raise ValueError(
                f"{domain.name}: no hand-written plan; plan one from a goal "
                "(plan_from_goal) and pass it as the document"
            )
        if run.concurrent:
            # Independent steps (another arm's) in parallel lanes (#21).
            document = parallelize(document, registry)
        plan = TaskPlan.from_dict(document, registry)
        return TaskPlanningSession(
            plan, registry, world_state=world, recorded=self.recorded
        )

    def _run_step(
        self, run: _Run, capability: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        """Refine one transaction and submit its motion; a failure raises,
        leaving ``run.q`` and the grasps as they were (the transaction
        contract)."""
        step = self.domain.step_for(capability, parameters)
        label = step.label
        tracker = self.planner.grasp_tracker
        run.step_start[label] = {g: h for g, h in tracker.current_grasps.items() if h}
        index = run.block_index
        run.block_index = index + 1
        binding = {k: v for k, v in parameters.items() if k != "block"}
        run.steps[label] = (capability, binding)
        if _injected(run.inject, capability, binding):
            # --inject-failure: fail as if its first phase couldn't be
            # reached, without planning (the M3 exit test).
            facts = [f"refinement_failed({label.replace(' ', '_')})"]
            reached = ""
            if isinstance(step, RefinementStep) and step.sequence:
                gripper, handle = step.sequence[0]
                facts.append(f"cannot_reach({gripper}, {handle})")
                reached = f": cannot_reach({gripper}, {handle})"
            run.failure = {
                "step": label,
                "capability": capability,
                "parameters": binding,
                "facts": facts,
            }
            print(f"\n=== {label} === INJECTED FAILURE{reached}")
            raise RuntimeError(f"{label}: injected failure")
        r = self.refine(run, index, step)
        if not r["success"]:
            # Undo whatever the step's phases committed to the grasp tracker.
            reset = getattr(self.planner, "reset_grasp_tracker_to_call_start", None)
            if reset is not None and isinstance(step, RefinementStep):
                reset()
            run.failure = {
                "step": label,
                "capability": capability,
                "parameters": binding,
                "facts": list(r.get("facts", [])),
            }
            raise RuntimeError(f"{label}: {r['message']}")
        if run.executor is not None:
            for command in self.step_commands(label, run.last_phases):
                run.executor.submit(command)
        return {"replans": r["replans"], "resumes": r["resumes"]}

    def refine(
        self, run: _Run, index: int, step: RefinementStep | Move
    ) -> dict[str, Any]:
        """Plan one step from ``run.q``, then log and record it (``run.q``
        advances only on success)."""
        label = step.label
        checkpoint = run.checkpoint
        if checkpoint is not None:
            checkpoint.phase_dump_dir(index, label)
        t0 = time.time()
        print(f"\n=== {label} ===", flush=True)
        is_move = isinstance(step, Move)
        if isinstance(step, Move):
            r = self.domain.plan_move(self.scene, step, run.q, verbose=self.verbose)
            phases = r.get("phase_results", [])
        else:
            refined = self._refiner().refine(step, run.q)
            r = refined.as_dict()
            phases = refined.phases
        record = {
            "label": label,
            "success": r["success"],
            "replans": r["replans"],
            "resumes": r["resumes"],
            "seconds": round(time.time() - t0, 2),
            "message": r["message"],
        }
        run.records.append(record)
        if is_move:
            detail = f"{r['resumes']} retries) {r['message']}"
        else:
            detail = f"{r['resumes']} resumes, {r['replans']} replans)"
        print(
            f"--- {label}: {'ok' if r['success'] else 'FAILED'} "
            f"({record['seconds']}s, {detail}",
            flush=True,
        )
        if checkpoint is not None:
            checkpoint.record(
                index,
                label,
                r,
                record["seconds"],
                r["final_config"],
                self.planner.grasp_tracker.current_grasps,
            )
        if r["success"]:
            if run.trajectory is not None:
                run.trajectory += sample_phases(phases, label, self.get_path)
                # After every step, like the run log, so a killed run keeps
                # its motion and a resume can append to it.
                if checkpoint is not None:
                    write_trajectory(
                        checkpoint.dir / "trajectory.json",
                        run.trajectory,
                        self.domain.scene_info(),
                    )
            if run.live_viewer is not None:
                # With a playback backend the backend drives the viewer.
                run.live_viewer.completed(phases, play=not run.backend_displays)
            run.q = list(r["final_config"])
            run.last_phases = phases
        return r

    def _refiner(self) -> Any:
        if self.refiner is not None:
            return self.refiner
        from long_tamp.tasks.refiner import GraspSequenceRefiner

        return GraspSequenceRefiner(
            self.planner,
            q_scene_init=self.scene.q_init,
            max_replans=self.max_replans,
            verbose=self.verbose,
        )

    def get_path(self, path: Any) -> Any:
        """A phase's path: an HPP path, or its index in the planner's store."""
        if isinstance(path, int):
            store = getattr(self.scene.task, "planner", None) or self.planner
            return store.get_path(path)
        return path

    def step_commands(
        self, label: str, phases: list[dict[str, Any]]
    ) -> list[ExecutionCommand]:
        """The motion a step produced, one command per completed path; the
        domain's ``execution_payload`` decides what each path is sent as (the
        path, or a skill command around it)."""
        commands = []
        for phase in phases:
            if not phase.get("complete", True) or phase.get("skipped"):
                continue
            paths = [p for p in phase.get("paths", []) if p is not None]
            edges = list(phase.get("edges", []))
            # A resumed phase planned only its last edges: align from the end.
            edges = (
                [None] * (len(paths) - len(edges)) + edges[-len(paths) :]
                if paths
                else []
            )
            for path, edge in zip(paths, edges):
                path = self.get_path(path)
                payload = self.domain.execution_payload(phase, edge, path)
                commands.append(
                    ExecutionCommand(
                        step_id=label, duration=path.length(), payload=payload
                    )
                )
        return commands

    # -- recovery ------------------------------------------------------------

    def _on_drift(
        self, run: _Run, node: dict[str, Any], observe: Any
    ) -> list[ExecutionCommand]:
        """The robot drifted before this step's motion: plan the step again
        from where it is (the grasps it started with, the observed config)."""
        operation = node["children"][0] if node["type"] == "transaction" else node
        capability = operation["capability"]
        parameters = dict(operation.get("parameters", {}))
        label = self.domain.step_for(capability, parameters).label
        MissionCheckpoint.restore_grasps(
            self.planner.grasp_tracker, run.step_start[label]
        )
        observed = observe(run.q) if observe is not None else None
        if observed is None:
            raise RuntimeError("the backend can't report the observed configuration")
        run.q = self.domain.observed_start(self.scene, run.q, observed)
        print(f"\n=== {label} === the robot drifted: replanning from where it is")
        executor = run.executor
        assert executor is not None
        submitted = len(executor._pending)
        self._run_step(run, capability, parameters)
        commands = executor._pending[submitted:]
        del executor._pending[submitted:]
        return commands

    def recover_missed_grasp(
        self, run: _Run, backend: Any, failure: dict[str, Any]
    ) -> None:
        """After ``grasp_missed`` (#128): open the hand that missed, undo the
        step's grasps and the missed one in the grasp tracker, and take the
        robot and the objects where the backend observes them, so the retry
        picks the object where it is."""
        missed = [
            atom.args
            for atom in map(parse_atom, failure.get("facts", []))
            if atom.name == "grasp_missed"
        ]
        unopened = []  # hands the backend can't open: their fingers stay closed
        for gripper, _ in missed:
            if not hasattr(backend, "open_gripper"):
                unopened.append(gripper)
                continue
            try:
                backend.open_gripper(gripper)
            except (KeyError, RuntimeError) as e:
                # No hand bound (weld grasps): nothing to open. In its own
                # process the backend's KeyError comes back as a RuntimeError.
                if "no hand for" not in str(e):
                    raise
                unopened.append(gripper)
        tracker = self.planner.grasp_tracker
        held = dict(run.step_start.get(failure["step"]) or {})
        if not held:
            held = {g: h for g, h in tracker.current_grasps.items() if h}
        for gripper, obj in missed:
            if str(held.get(gripper, "")).startswith(f"{obj}/"):
                del held[gripper]
        MissionCheckpoint.restore_grasps(tracker, held)
        observe = getattr(backend, "observed_config", None)
        # With the objects: the one the hand missed has moved.
        observed = observe(run.q, True) if observe is not None else None
        if observed is not None:
            run.q = self.domain.observed_start(self.scene, run.q, observed)
        if unopened:
            run.q = self.domain.opened_config(self.scene, run.q, unopened)
        print(
            f"\n=== {failure['step']} === {', '.join(failure['facts'])}: hand opened, "
            f"world {'observed' if observed is not None else 'NOT observed'}",
            flush=True,
        )

    # -- repair and supervision ----------------------------------------------

    def run_with_repair(
        self,
        q_start: Sequence[float] | None,
        rounds: int,
        mission: dict[str, Any],
        goal: list[str] | None = None,
        constraints: list[tuple[str, dict[str, str]]] | None = None,
        retries: int = 2,
    ) -> dict[str, Any]:
        """Plan, run, and replan around failures (#15).

        Every round plans the goal from the current world state with the
        bindings blocked so far, and runs it (``run_mission`` with the
        options in ``mission``) from where the last round stopped. A failure
        the domain retries (a missed grasp) is retried as it was, up to
        ``retries`` times per step (#128).
        """
        from long_tamp.tasks.task_planning.repair import plan_execute_repair

        state: dict[str, Any] = {
            "q": q_start,
            "result": None,
            "seconds": 0.0,
            "blocks": [],
            "skipped": [],
        }

        def plan(blocked):
            return self.plan_from_goal(
                self.world_atoms(),
                blocked=[*(constraints or []), *blocked],
                goal=goal,
            )

        def execute(document):
            result = self.run_mission(q_start=state["q"], document=document, **mission)
            state["q"] = result["final_config"]
            state["seconds"] += result["seconds"]
            state["blocks"] += result["blocks"]
            state["skipped"] += result["skipped"]
            state["result"] = result
            return result["failure"]

        def on_replan(failure, blocked):
            if not blocked:
                print(
                    f"\n=== retrying: {failure['step']} failed "
                    f"({', '.join(failure.get('facts', []))}); replanning it "
                    "from the observed world",
                    flush=True,
                )
                return
            print(
                f"\n=== replanning: {failure['step']} failed "
                f"({', '.join(failure.get('facts', [])[1:]) or 'no facts'}); "
                f"blocking {blocked}",
                flush=True,
            )

        outcome = plan_execute_repair(
            plan,
            execute,
            self.domain.repair_policy,
            max_rounds=rounds,
            on_replan=on_replan,
            retry=self.domain.retry,
            max_retries=retries,
        )
        # No first plan at all: nothing ran.
        result = dict(state["result"] or {"failure": None, "final_config": state["q"]})
        result.update(
            success=outcome.success,
            repair_message=outcome.message,
            seconds=round(state["seconds"], 2),
            blocks=state["blocks"],
            skipped=state["skipped"],
            task_replans=len(outcome.rounds) - 1,
            retries=sum(r.retried for r in outcome.rounds),
            blocked=outcome.blocked,
            failures=outcome.failures,
        )
        if not outcome.success:
            print(f"\n=== repair gave up: {outcome.message}")
        return result

    def run_supervised(
        self,
        q_start: Sequence[float] | None,
        rounds: int,
        mission: dict[str, Any],
        instruction: str,
        goal: list[str],
        constraints: list[tuple[str, dict[str, str]]],
        client: Any,
        run_dir: Path,
        retries: int = 2,
    ) -> dict[str, Any]:
        """Repair loops under the execution supervisor (#88).

        Each repair loop (``run_with_repair``) handles what the refiner can
        explain. When one gives up, the supervisor decides: retry, relax the
        goal, abort, or escalate (``escalation.md`` / ``escalation.json`` in
        ``run_dir``).
        """
        from long_tamp.tasks.task_planning.skeleton import default_planner
        from long_tamp.tasks.task_planning.supervisor import (
            Limits,
            Situation,
            supervise,
        )

        where = {"q": q_start}
        #: What every repair loop ran, for the mission's summary.
        total: dict[str, Any] = {
            "seconds": 0.0,
            "blocks": [],
            "skipped": [],
            "loops": 0,
        }
        task_planner = default_planner("auto")

        def run_repair(current_goal):
            result = self.run_with_repair(
                where["q"], rounds, mission, current_goal, constraints, retries
            )
            where["q"] = result.get("final_config") or where["q"]
            total["seconds"] += result.get("seconds", 0.0)
            total["blocks"] += result.get("blocks", [])
            total["skipped"] += result.get("skipped", [])
            total["loops"] += 1
            return result

        def situation_of(result, current_goal):
            return Situation(
                instruction=instruction,
                original_goal=goal,
                goal=current_goal,
                failure=result.get("failure"),
                message=result.get("repair_message", ""),
                blocked=result.get("blocked", []),
                vocabulary=self.domain.goal_vocabulary(self.world_atoms()),
                failures=result.get("failures", []),
            )

        def reachable(candidate):
            return _verdict(
                lambda: task_planner.solve(
                    self.domain.pddl_problem(self.world_atoms(), constraints, candidate)
                )
            )

        outcome = supervise(
            run_repair,
            situation_of,
            instruction,
            goal,
            client,
            Limits(max_decisions=3, max_tokens=200_000),
            reachable,
            report_dir=run_dir,
        )
        for i, d in enumerate(outcome.decisions, 1):
            print(
                f"\n=== supervisor decision {i}: {d['action']}"
                + (f" (fallback: {d['fallback']})" if d["fallback"] else "")
                + f"\n    {d['reason']}"
                + (
                    f"\n    goal now: {d['goal']}"
                    if d["action"] == "relax_goal"
                    else ""
                ),
                flush=True,
            )
        if outcome.report is not None:
            print(f"\n=== escalation report: {outcome.report}", flush=True)
        result = dict(outcome.last)
        result.update(
            success=outcome.success,
            seconds=round(total["seconds"], 2),
            blocks=total["blocks"],
            skipped=total["skipped"],
        )
        result["supervisor"] = {
            "original_goal": goal,
            "final_goal": outcome.goal,
            "decisions": outcome.decisions,
            "stopped": outcome.stopped,
            "repair_loops": total["loops"],
            "report": str(outcome.report) if outcome.report else None,
        }
        return result


# -- helpers -------------------------------------------------------------------


def _verdict(solve: Callable[[], Any]) -> str | None:
    """None if ``solve`` finds a plan, else the planner's first line."""
    try:
        solve()
    except Exception as error:  # noqa: BLE001 - the planner's verdict
        return str(error).splitlines()[0][:300] if str(error) else type(error).__name__
    return None


def parse_injection(text: str) -> dict[str, Any]:
    """``capability:key=value,key=value`` -> an ``--inject-failure`` spec."""
    capability, _, rest = text.partition(":")
    match = dict(item.split("=", 1) for item in rest.split(",") if item)
    return {"capability": capability, "match": match, "fired": False}


def _injected(
    inject: list[dict[str, Any]], capability: str, binding: dict[str, Any]
) -> bool:
    """Whether an ``--inject-failure`` spec targets this step (each fires once)."""
    for spec in inject:
        if spec["fired"] or spec["capability"] != capability:
            continue
        if all(str(binding.get(k)) == v for k, v in spec["match"].items()):
            spec["fired"] = True
            return True
    return False


def sample_phases(
    phases: list[dict[str, Any]],
    label: str,
    get_path: Callable[[Any], Any] = lambda path: path,
    dt: float = TRAJECTORY_DT,
) -> list[dict[str, Any]]:
    """The completed phases' paths sampled every ``dt`` seconds of path time
    (paths are time-parameterized), for ``trajectory.json``. ``get_path``
    resolves a path stored by index."""
    segments = []
    for phase in phases:
        if not phase.get("complete", True) or phase.get("skipped"):
            continue
        configs: list[list[float]] = []
        for path in phase.get("paths", []):
            if path is None:
                continue
            path = get_path(path)
            length = path.length()
            t0 = path.timeRange().first if hasattr(path, "timeRange") else 0.0
            n = max(2, int(length / dt) + 1)
            for i in range(n):
                q, ok = path.eval(t0 + length * i / (n - 1))
                if ok:
                    configs.append([round(float(v), 5) for v in q])
        if configs:
            segments.append(
                {
                    "block": label,
                    "gripper": phase.get("gripper"),
                    "handle": phase.get("handle"),
                    "configs": configs,
                }
            )
    return segments


def write_trajectory(
    path: Path, segments: list[dict[str, Any]], header: dict[str, Any]
) -> None:
    """Write the replay input (atomically: it is rewritten after each step).
    ``header`` is the domain's ``scene_info()``."""
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({**header, "dt": TRAJECTORY_DT, "segments": segments}))
    tmp.replace(path)


def stopped_failure(run: Any, session: TaskPlanningSession) -> dict[str, Any] | None:
    """Where a run ended that neither planning nor execution reported as a
    failure: a step aborted or stopped (#108). ``step`` is its label."""
    if run.failed_step is None:
        return None
    labels, stack = {}, [session.plan.document["root"]]
    while stack:
        node = stack.pop()
        labels[node["id"]] = node.get("label", node["id"])
        stack += node.get("children", []) + ([node["child"]] if "child" in node else [])
    return {
        "step": labels.get(run.failed_step, run.failed_step),
        "facts": [],
        "message": run.message,
    }


def execution_failure(
    run: Any, steps: dict[str, tuple[str, dict[str, Any]]] | None = None
) -> dict[str, Any] | None:
    """The failure facts of a step whose execution failed (a skill reporting
    a misalignment, a grasp ``grasp_missed``), in the shape planning failures
    have; ``steps`` (label -> capability, parameters) adds the binding."""
    failed = [e for e in run.executions if e.result.status.value != "success"]
    if not failed:
        return None
    last = failed[-1]
    failure = {
        "step": last.command.step_id,
        "facts": list(last.result.facts),
        "message": last.result.message,
    }
    if steps and last.command.step_id in steps:
        capability, parameters = steps[last.command.step_id]
        failure.update(capability=capability, parameters=dict(parameters))
    return failure


def metric_max(executions: Sequence[Any], key: str) -> float | None:
    """The largest ``key`` metric the backend reported, if any."""
    values = [e.result.metrics[key] for e in executions if key in e.result.metrics]
    return max(values) if values else None


def fact_count(executions: Sequence[Any], prefix: str) -> int:
    """How many reported facts start with ``prefix`` (``"holds("``)."""
    return sum(1 for e in executions for f in e.result.facts if f.startswith(prefix))


def execution_summary(
    executions: Sequence[Any], domain: MissionDomain | None = None
) -> dict[str, Any] | None:
    """What the backend measured over the mission (tracking error, drift,
    grasps), from ``PlanRun.executions``, plus the domain's
    ``execution_metrics``."""
    if not executions:
        return None
    summary = {
        "commands": len(executions),
        "failed": sum(1 for e in executions if e.result.status.value != "success"),
        "sim_seconds": round(
            sum(e.result.metrics.get("sim_seconds", 0.0) for e in executions), 2
        ),
        "max_tracking_error": metric_max(executions, "tracking_error"),
        "max_drift": metric_max(executions, "drift"),
        "max_object_drift": metric_max(executions, "object_drift"),
        "max_start_drift": metric_max(executions, "start_drift"),
        # each close verified against the hand model (#127)
        "grasps_held": fact_count(executions, "holds("),
        "grasps_missed": fact_count(executions, "grasp_missed("),
    }
    if domain is not None:
        summary.update(domain.execution_metrics(executions))
    return summary


__all__ = [
    "TRAJECTORY_DT",
    "MissionRunner",
    "execution_failure",
    "execution_summary",
    "fact_count",
    "metric_max",
    "parse_injection",
    "sample_phases",
    "stopped_failure",
    "write_trajectory",
]
