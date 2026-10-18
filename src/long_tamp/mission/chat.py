"""A mission as gated chat tools, for any domain (#89, #133).

:class:`MissionChat` keeps an operator's working mission (goal, constraints,
plan, last run) on a :class:`~long_tamp.mission.MissionRunner` and exposes it
as ``long_tamp.ai.chat`` tools. The tools come from the domain: what a goal
may say is its vocabulary, what a constraint may name its capabilities, and
the plan is its task planner's. Every tool checks its arguments the way the
rest of long_tamp does, and refuses with the reason (``ToolRejected``):

- ``state``: the world state, the goal, the constraints, the plan;
- ``write_goal``: the goal writer role (#22) turns an instruction into a
  checked goal;
- ``set_goal``: goal literals, checked against the vocabulary and reachable
  by the task planner under the constraints;
- ``add_constraint`` / ``remove_constraint``: blocked bindings, checked like
  the plan reviewer's (real names, the goal still reachable);
- ``plan``: the task planner's plan for the goal, optionally in stages
  (``first``: literals of the goal to reach before the rest); the planner
  chooses every step;
- ``run``: execute the plan on the mission's backend;
- ``explain_failure``: the last run's failure, as the refiner reported it.

:func:`run_chat` is the operator's chat around it: stdin, and the web
viewer's chat panel with its Start and Reset buttons (#90, #104, #114).
"""

from __future__ import annotations

import sys
from collections.abc import Callable
from typing import Any

from long_tamp.ai.chat import Tool, ToolRejected
from long_tamp.tasks.task_planning.language import check_goal
from long_tamp.tasks.task_planning.model import grounded_literals
from long_tamp.tasks.task_planning.predicates import apply_effects, parse_state
from long_tamp.tasks.task_planning.review import ReviewRequest, check_constraints

#: The implementation parameter the runner adds to every step (a transaction
#: label): never something an operator names.
BLOCK = "block"


class MissionChat:
    """The working mission behind the chat tools (see the module docstring).

    ``runner`` runs it (a :class:`~long_tamp.mission.MissionRunner`, or
    anything with its ``domain``, ``world_atoms`` and ``run_mission``);
    ``mission`` holds the ``run_mission`` options (backend, checkpoint,
    events ...); ``client`` is the model, for ``write_goal``.
    """

    def __init__(
        self,
        runner: Any,
        mission: dict[str, Any] | None = None,
        q_start: Any = None,
        client: Any = None,
        engine: str = "auto",
    ) -> None:
        from long_tamp.tasks.task_planning.skeleton import default_planner

        self.runner, self.domain = runner, runner.domain
        self.mission, self.q = dict(mission or {}), q_start
        #: The model client, for the goal-writer role (``write_goal``).
        self.client = client
        self.descriptors = self.domain.descriptors()
        self.task_planner = default_planner(engine)
        self.goal: list[str] | None = None
        self.constraints: list[tuple[str, dict[str, str]]] = []
        self.steps: list[tuple[str, dict[str, Any]]] = []
        self.document: dict[str, Any] | None = None
        self.last: dict[str, Any] | None = None

    # -- helpers -------------------------------------------------------------------

    def world(self) -> list[str]:
        return self.runner.world_atoms()

    def vocabulary(self):
        return self.domain.goal_vocabulary(self.world())

    def _solve(self, goal, state=None, constraints=None):
        return self.task_planner.solve(
            self.domain.pddl_problem(
                self.world() if state is None else state,
                self.constraints if constraints is None else constraints,
                goal,
            )
        )

    def _why_not(self, goal, constraints=None) -> str | None:
        try:
            self._solve(goal, constraints=constraints)
        except Exception as error:  # noqa: BLE001 - the planner's verdict
            return str(error).splitlines()[0][:300] or type(error).__name__
        return None

    def _after(self, state: list[str], steps) -> list[str]:
        """The symbolic state after ``steps`` (their declared effects)."""
        atoms = parse_state(state)
        for capability, parameters in steps:
            node = {"capability": capability, "parameters": dict(parameters)}
            _, effects = grounded_literals(node, self.descriptors[capability])
            atoms = apply_effects(atoms, effects)
        return sorted(str(a) for a in atoms)

    @staticmethod
    def _label(step) -> str:
        capability, parameters = step
        args = ", ".join(f"{k}={v}" for k, v in parameters.items() if k != BLOCK)
        return f"{capability}({args})"

    def actions(self) -> dict[str, tuple[str, ...]]:
        """The capabilities a plan is made of (those with effects), with the
        parameters a constraint may bind."""
        return {
            cap: tuple(p for p in d.required_parameters if p != BLOCK)
            for cap, d in self.descriptors.items()
            if d.effect_literals
        }

    def domain_text(self) -> str:
        """The domain, for the chat model's instructions: what a goal or a
        constraint may name."""
        v = self.vocabulary()
        lines = [
            "Goal predicates: "
            + ", ".join(f"{name}/{arity}" for name, arity in v.predicates.items()),
            "Objects: " + ", ".join(v.objects),
            "Capabilities (for constraints): "
            + "; ".join(
                f"{cap}({', '.join(ps)})" for cap, ps in self.actions().items()
            ),
        ]
        if v.notes:
            lines += ["Notes:", v.notes.strip()]
        return "\n".join(lines)

    def instructions(self) -> str:
        """The chat model's instructions: the domain's intro and vocabulary."""
        return (self.domain.intro.strip() + "\n\n" + self.domain_text()).strip()

    # -- tools ---------------------------------------------------------------------

    def state(self) -> dict[str, Any]:
        return {
            "world": self.world(),
            "goal": self.goal,
            "constraints": self.constraints,
            "plan": [self._label(s) for s in self.steps],
        }

    def write_goal(self, instruction: str) -> dict[str, Any]:
        """The goal writer role turns ``instruction`` into a checked goal."""
        from long_tamp.tasks.task_planning.language import (
            GoalError,
            ModelGoalWriter,
            goal_from_instruction,
        )

        if self.client is None:
            raise ToolRejected("no model for the goal writer: use set_goal")
        try:
            goal = goal_from_instruction(
                str(instruction),
                ModelGoalWriter(self.client),
                self.vocabulary(),
                lambda g: self._why_not(g),
            )
        except GoalError as error:
            raise ToolRejected(str(error)) from error
        self.goal, self.steps, self.document = goal, [], None
        return {"goal": goal}

    def set_goal(self, literals: list[str]) -> dict[str, Any]:
        goal = [str(g) for g in (literals or [])]
        errors = check_goal(goal, self.vocabulary())
        if not errors:
            why = self._why_not(goal)
            if why:
                errors = [f"no plan reaches it under the constraints: {why}"]
        if errors:
            raise ToolRejected("; ".join(errors))
        self.goal, self.steps, self.document = goal, [], None
        return {"goal": goal}

    def add_constraint(
        self, capability: str, parameters: dict[str, str]
    ) -> dict[str, Any]:
        blocked = (
            str(capability),
            {str(k): str(v) for k, v in (parameters or {}).items()},
        )
        request = ReviewRequest("", self.vocabulary(), self.descriptors)
        replannable = (
            (lambda b: self._why_not(self.goal, [*self.constraints, *b]))
            if self.goal
            else None
        )
        errors = check_constraints([blocked], request, replannable)
        if errors:
            raise ToolRejected("; ".join(errors))
        self.constraints.append(blocked)
        self.steps, self.document = [], None
        return {"constraints": self.constraints}

    def remove_constraint(self, index: int) -> dict[str, Any]:
        try:
            removed = self.constraints.pop(int(index))
        except (IndexError, ValueError) as error:
            raise ToolRejected(
                f"no constraint {index!r} (there are {len(self.constraints)})"
            ) from error
        self.steps, self.document = [], None
        return {"removed": removed, "constraints": self.constraints}

    def plan(self, first: list[str] | None = None) -> dict[str, Any]:
        if not self.goal:
            raise ToolRejected("set a goal first (set_goal)")
        state = self.world()
        stages = []
        if first:
            first = [str(g) for g in first]
            extra = [g for g in first if g not in self.goal]
            if extra:
                raise ToolRejected(
                    f"'first' must be literals of the goal; not in it: {extra}"
                )
            stages.append(first)
        stages.append(self.goal)
        steps = []
        for stage in stages:
            try:
                part = self._solve(stage, state=state)
            except Exception as error:  # noqa: BLE001 - the planner's verdict
                raise ToolRejected(
                    f"no plan reaches {stage}: {str(error).splitlines()[0][:300]}"
                ) from error
            steps += part
            state = self._after(state, part)
        self.steps = steps
        self.document = self.domain.planned_document(
            steps, self.world(), generator="chat"
        )
        return {"plan": [self._label(s) for s in steps]}

    def run(self) -> dict[str, Any]:
        if self.document is None:
            raise ToolRejected("there is no plan to run (plan first)")
        control = self.mission.get("control")
        if control is not None:
            control.reset()  # a stop ends one run, not the chat
        result = self.runner.run_mission(
            q_start=self.q, document=self.document, **self.mission
        )
        self.q = result.get("final_config") or self.q
        self.last = result
        self.steps, self.document = [], None
        failure = result.get("failure") or {}
        return {
            "success": result["success"],
            "seconds": result["seconds"],
            "failed_step": failure.get("step"),
            "world": self.world(),
        }

    def explain_failure(self) -> dict[str, Any]:
        if self.last is None:
            raise ToolRejected("nothing has run yet")
        failure = self.last.get("failure")
        if not failure:
            return {"failure": None, "note": "the last run succeeded"}
        return {
            "step": failure.get("step"),
            "capability": failure.get("capability"),
            "parameters": failure.get("parameters"),
            "facts": failure.get("facts", []),
            "message": failure.get("message", ""),
        }

    def _examples(self) -> tuple[str, str, str]:
        """A goal literal, a capability and a binding of it, from the domain,
        for the tools' argument hints."""
        goal = self.domain.default_goal()
        literal = goal[-1] if goal else "a ground literal"
        actions = self.actions()
        cap = next((c for c, ps in actions.items() if ps), next(iter(actions), "-"))
        params = actions.get(cap, ())
        binding = f'{{"{params[-1]}": "<object>"}}' if params else "{}"
        return literal, cap, binding

    def tools(self) -> list[Tool]:
        literal, cap, binding = self._examples()
        return [
            Tool(
                "state",
                "the world state, goal, constraints and current plan",
                self.state,
            ),
            Tool(
                "write_goal",
                "turn the operator's words into a checked goal (the goal writer); "
                "use it for new or changed goals",
                self.write_goal,
                {"instruction": "what the operator wants as the end result"},
            ),
            Tool(
                "set_goal",
                "set the mission's goal (checked: known predicates and objects, reachable)",
                self.set_goal,
                {"literals": f"list of goal literals, e.g. {literal}"},
            ),
            Tool(
                "add_constraint",
                "forbid a choice: a capability with some of its parameters",
                self.add_constraint,
                {
                    "capability": f"e.g. {cap}",
                    "parameters": f"object, e.g. {binding}",
                },
            ),
            Tool(
                "remove_constraint",
                "remove a constraint by its index in state()['constraints']",
                self.remove_constraint,
                {"index": "integer"},
            ),
            Tool(
                "plan",
                "ask the task planner for a plan to the goal; optionally reach some goal "
                "literals first",
                self.plan,
                {"first": "optional list of goal literals to reach before the rest"},
            ),
            Tool("run", "execute the current plan on the robot (simulation)", self.run),
            Tool(
                "explain_failure",
                "why the last run failed, as the motion planner reported it",
                self.explain_failure,
            ),
        ]


def run_chat(
    work: MissionChat,
    client: Any,
    events: Callable[[dict[str, Any]], None],
    web: Any = None,
    watchdog: Any = None,
) -> dict[str, Any]:
    """An operator's chat with a model acting through ``work``'s tools
    (#89): reads operator messages from stdin until EOF or "quit", and writes
    every tool call to ``events``. With ``web`` (a ``ViewerServer``), the
    same chat is also in the page's chat panel (#90), with a Start mission
    and a Reset mission button (#104, #114); after stdin closes, it goes on
    there until "quit" or Ctrl-C. ``watchdog`` (a ``StepWatchdog``) also
    reports its decisions in the chat (#109).

    Returns the last run's result plus ``chat`` (the turns, goal and
    constraints); ``reset`` is set when the operator pressed Reset.
    """
    from long_tamp.ai.chat import ChatSession, ChatTurn
    from long_tamp.tasks.task_planning.events import make_event, plan_event
    from long_tamp.viewer import ChatBridge

    def on_tool(call):
        events(
            make_event(
                "chat",
                "tool",
                call.tool,
                "SUCCESS" if call.ok else "FAILURE",
                message=call.error,
                metrics={"arguments": call.arguments},
            )
        )
        if call.tool == "plan" and call.ok and work.document is not None:
            events(plan_event(work.document, "planned in the chat"))

    session = ChatSession(client, work.tools(), work.instructions(), on_tool=on_tool)
    bridge = ChatBridge(session)
    if watchdog is not None:
        printed = watchdog.on_decision

        def on_decision(watch, decision):
            if printed is not None:
                printed(watch, decision)
            bridge.note(
                f"watchdog ({decision.by}) on {watch.label}: {decision.action}"
                + (
                    f" {decision.wait_s / 60:.1f} min more"
                    if decision.action == "wait" and decision.wait_s
                    else ""
                )
                + f": {decision.reason}"
            )

        watchdog.on_decision = on_decision

    def start():
        """The page's Start button: run the plan, no model involved; the
        model sees it in the history, like a run it called."""
        call = session.call("run", {})
        session.turns.append(
            ChatTurn(user="(the operator pressed Start mission)", say="", calls=[call])
        )
        return call

    bridge.add_action(
        "start", "Start mission", start, enabled=lambda: work.document is not None
    )
    reset = []

    def start_over():
        """The page's Reset button: end this mission; the launcher starts a
        fresh one."""
        reset.append(True)
        bridge.ended.set()
        return "resetting: a fresh mission starts in a few seconds"

    bridge.add_action("reset", "Reset mission", start_over)
    if web is not None:
        bridge.attach(web)
        print(f"chat: also in the web viewer, {web.url}", flush=True)
    echo = not sys.stdin.isatty()  # piped messages: show them in the log
    print("chat: type an instruction, 'quit' or Ctrl-D to end", flush=True)
    while True:
        try:
            line = input("operator> ")
        except EOFError:
            if web is not None and not bridge.ended.is_set():
                print(
                    "chat: stdin closed; the chat goes on in the web viewer "
                    "('quit' there or Ctrl-C ends it)",
                    flush=True,
                )
                try:
                    bridge.ended.wait()
                except KeyboardInterrupt:
                    pass
                bridge.wait()
            break
        if echo:
            print(line, flush=True)
        if line.strip().lower() in ("quit", "exit"):
            break
        if not line.strip():
            continue
        turn = bridge.turn(line.strip())
        if turn is None:  # the error is in the transcript
            print(f"  [error] {bridge.transcript()[-1]['text']}", flush=True)
            continue
        for call in turn.calls:
            print(f"  [tool] {call.as_text()[:400]}", flush=True)
        if turn.error:
            print(f"  [error] {turn.error}", flush=True)
        print(f"model> {turn.say}", flush=True)
    last = work.last or {"success": True, "seconds": 0.0, "failure": None}
    result = dict(last)
    result.setdefault("final_config", work.q)
    result["chat"] = {
        "turns": [
            {"user": t.user, "say": t.say, "calls": [c.as_text() for c in t.calls]}
            for t in session.turns
        ],
        "goal": work.goal,
        "constraints": work.constraints,
    }
    if reset:
        result["reset"] = True
    return result


__all__ = ["MissionChat", "run_chat"]
