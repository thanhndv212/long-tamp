"""The screw-assembly mission as gated chat tools (``--chat``, #89).

``MissionChat`` keeps an operator's working mission (goal, constraints, plan,
last run) and exposes it as ``long_tamp.ai.chat`` tools. Every tool checks
its arguments the way the rest of long_tamp does, and refuses with the
reason (``ToolRejected``):

- ``state``: the world state, the goal, the constraints, the plan;
- ``write_goal``: the goal writer role (#22) turns an instruction into a
  checked goal;
- ``set_goal``: goal literals, checked against the vocabulary and reachable
  by the task planner under the constraints;
- ``add_constraint`` / ``remove_constraint``: blocked bindings, checked like
  the plan reviewer's (real names, the goal still reachable);
- ``plan``: the task planner's plan for the goal, optionally in stages
  (``first``: literals of the goal to reach before the rest, e.g. part 2
  first); the planner chooses every step;
- ``run``: execute the plan on the mission's backend;
- ``explain_failure``: the last run's failure, as the refiner reported it.
"""

from __future__ import annotations

from typing import Any

from long_tamp.ai.chat import Tool, ToolRejected
from long_tamp.tasks.task_planning.language import check_goal
from long_tamp.tasks.task_planning.model import grounded_literals
from long_tamp.tasks.task_planning.predicates import (
    apply_effects,
    parse_state,
)
from long_tamp.tasks.task_planning.review import ReviewRequest, check_constraints

INTRO = """\
You are the operator's assistant for a two-arm screw-assembly cell: the left arm \
(ur10_left) carries parts into jig clamps, the right arm (ur10_right) screws them \
with a screwdriver ("driver"). A mission is a goal (final-state literals), optional \
constraints (choices the planner must avoid), and a plan the task planner finds."""


class MissionChat:
    """The working mission behind the chat tools (see the module docstring)."""

    def __init__(
        self,
        T: Any,
        task,
        planner,
        recorded,
        n_parts: int,
        mission: dict,
        q_start=None,
        client: Any = None,
    ):
        self.T, self.task, self.planner, self.recorded = T, task, planner, recorded
        self.n_parts, self.mission, self.q = n_parts, mission, q_start
        #: The model client, for the goal-writer role (``write_goal``).
        self.client = client
        self.clamps = T.clamp_seats(task.task_config.VALID_PAIRS)
        self.descriptors = T.descriptors(n_parts)
        from long_tamp.tasks.task_planning.skeleton import default_planner

        self.task_planner = default_planner("auto")
        self.goal: list[str] | None = None
        self.constraints: list[tuple[str, dict[str, str]]] = []
        self.steps: list[tuple[str, dict[str, Any]]] = []
        self.document: dict[str, Any] | None = None
        self.last: dict[str, Any] | None = None

    # -- helpers -------------------------------------------------------------------

    def world(self) -> list[str]:
        return self.T.world_atoms(self.planner, self.recorded)

    def vocabulary(self):
        return self.T.goal_vocabulary(self.n_parts, self.world(), self.clamps)

    def _solve(self, goal, state=None, constraints=None):
        return self.task_planner.solve(
            self.T.pddl_problem(
                self.n_parts,
                self.world() if state is None else state,
                self.constraints if constraints is None else constraints,
                self.clamps,
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
        args = ", ".join(f"{k}={v}" for k, v in parameters.items() if k != "block")
        return f"{capability}({args})"

    def domain(self) -> str:
        """The domain, for the chat model's instructions: what a goal or a
        constraint may name."""
        v = self.vocabulary()
        lines = [
            "Goal predicates: "
            + ", ".join(f"{name}/{arity}" for name, arity in v.predicates.items()),
            "Objects: " + ", ".join(v.objects),
            "Capabilities (for constraints): "
            + "; ".join(
                f"{cap}({', '.join(p for p in d.required_parameters if p != 'block')})"
                for cap, d in self.descriptors.items()
                if d.effect_literals
            ),
        ]
        if v.notes:
            lines += ["Notes:", v.notes.strip()]
        return "\n".join(lines)

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
        self.document = self.T.planned_document(
            steps, self.n_parts, self.world(), generator="chat"
        )
        return {"plan": [self._label(s) for s in steps]}

    def run(self) -> dict[str, Any]:
        if self.document is None:
            raise ToolRejected("there is no plan to run (plan first)")
        control = self.mission.get("control")
        if control is not None:
            control.reset()  # a stop ends one run, not the chat
        result = self.T.run_mission(
            self.task,
            self.planner,
            self.n_parts,
            q_start=self.q,
            document=self.document,
            **self.mission,
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

    def tools(self) -> list[Tool]:
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
                {
                    "literals": "list of goal literals, e.g. screwed(part2, part2/h_hole1)"
                },
            ),
            Tool(
                "add_constraint",
                "forbid a choice: a capability with some of its parameters",
                self.add_constraint,
                {
                    "capability": "e.g. clamp_and_screw",
                    "parameters": 'object, e.g. {"clamp": "fixtures/clamp1"}',
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
