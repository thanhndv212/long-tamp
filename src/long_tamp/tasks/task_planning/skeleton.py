"""Task planning: from a goal to a plan skeleton to a TaskPlan (ADR-0001, M3).

A task planner turns a PDDL export (``pddl.to_pddl``) into a *skeleton*: the
sequence of capability calls, ``[(capability_id, parameters), ...]``.
``skeleton_document`` turns a skeleton into a TaskPlan document (a sequence
of transactions) that goes through the same validation as a hand-written plan
(``TaskPlan.from_dict``: capability contracts, then plan-time simulation from
``initial_state``).

``UnifiedPlanningPlanner`` is the reference planner: Unified Planning with a
classical engine, Fast Downward where its wheels exist (Linux x86-64, macOS),
else pyperplan after Unified Planning compiles the problem down to STRIPS
(quantifiers, disjunctions, conditional effects and negative conditions
removed; the plan is mapped back). It is an optional dependency
(``pip install long-tamp[planning]``); importing this module doesn't need it.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import warnings
from collections.abc import Callable, Iterable, Sequence
from pathlib import Path
from typing import Any, Protocol

from .pddl import PddlExport, from_pddl_plan

Step = tuple[str, dict[str, Any]]
#: ``expand(index, capability, parameters)`` -> the transactions for one
#: skeleton step, as ``(capability, parameters, label)``: the step itself
#: with its implementation parameters filled in, plus any steps the domain
#: inserts around it (e.g. a move home after a pickup).
Expand = Callable[[int, str, dict[str, Any]], list[tuple[str, dict[str, Any], str]]]


class NoPlanFound(RuntimeError):
    """The planner proved, or gave up proving, that the goal is unreachable."""


class TaskPlanner(Protocol):
    def solve(self, export: PddlExport) -> list[Step]:
        """A skeleton reaching the export's goal; raises ``NoPlanFound``."""
        ...


class UnifiedPlanningPlanner:
    """Plan with a Unified Planning one-shot engine.

    ``engine="auto"`` uses Fast Downward if installed, else pyperplan. When
    the engine can't handle the problem's features (pyperplan: plain STRIPS
    only), the problem is compiled down first and the plan mapped back.
    """

    #: Compilations applied, in order, until the engine supports the problem.
    COMPILATIONS = (
        "QUANTIFIERS_REMOVING",
        "DISJUNCTIVE_CONDITIONS_REMOVING",
        "CONDITIONAL_EFFECTS_REMOVING",
        "NEGATIVE_CONDITIONS_REMOVING",
    )

    def __init__(self, engine: str = "auto", timeout: float | None = None):
        self.engine = engine
        self.timeout = timeout

    def engine_name(self) -> str:
        if self.engine != "auto":
            return self.engine
        for module, name in (
            ("up_fast_downward", "fast-downward"),
            ("up_pyperplan", "pyperplan"),
        ):
            try:
                __import__(module)
                return name
            except ImportError:
                continue
        raise ImportError(
            "no Unified Planning engine installed: pip install long-tamp[planning]"
        )

    def solve(self, export: PddlExport) -> list[Step]:
        try:
            from unified_planning.engines import CompilationKind
            from unified_planning.io import PDDLReader
            from unified_planning.shortcuts import (
                Compiler,
                OneshotPlanner,
                get_environment,
            )
        except ImportError as error:  # pragma: no cover - depends on the extra
            raise ImportError(
                "UnifiedPlanningPlanner needs the 'planning' extra: "
                "pip install long-tamp[planning]"
            ) from error
        get_environment().credits_stream = None
        name = self.engine_name()
        if self.engine == "auto" and name == "pyperplan":
            warnings.warn(
                "Unified Planning fell back to pyperplan, which is slow and "
                "can hang on larger problems; use FastDownwardPlanner or "
                "install up-fast-downward",
                stacklevel=2,
            )
        problem = PDDLReader().parse_problem_string(export.domain, export.problem)
        with OneshotPlanner(name=name) as planner:
            map_backs = []
            for kind_name in self.COMPILATIONS:
                if planner.supports(problem.kind):
                    break
                kind = getattr(CompilationKind, kind_name)
                with Compiler(problem_kind=problem.kind, compilation_kind=kind) as c:
                    compiled = c.compile(problem, kind)
                problem = compiled.problem
                map_backs.append(compiled.map_back_action_instance)
            result = planner.solve(problem, timeout=self.timeout)
        if result.plan is None:
            raise NoPlanFound(f"{name}: {result.status.name}")
        plan = result.plan
        for map_back in reversed(map_backs):
            plan = plan.replace_action_instances(map_back)
        actions = [
            (a.action.name, *(str(p) for p in a.actual_parameters))
            for a in plan.actions
        ]
        return from_pddl_plan(export, actions)


class FastDownwardPlanner:
    """Run Fast Downward (https://github.com/aibasel/downward) on the export.

    Fast Downward reads the exported PDDL as is (quantifiers, negative
    preconditions, conditional effects), so nothing is compiled away. The
    executable (``fast-downward.py`` from a source build, or the one bundled
    with ``up-fast-downward``) is found from ``executable``, the
    ``LONG_TAMP_FAST_DOWNWARD`` environment variable, ``PATH``, then the
    ``up_fast_downward`` package. ``alias`` is a Fast Downward search alias.
    """

    def __init__(
        self,
        executable: str | None = None,
        alias: str = "lama-first",
        timeout: float = 120.0,
    ) -> None:
        self.executable = executable or self.find()
        if self.executable is None:
            raise FileNotFoundError(
                "Fast Downward not found: build it (github.com/aibasel/downward) "
                "and set LONG_TAMP_FAST_DOWNWARD to its fast-downward.py, or "
                "pip install long-tamp[planning]"
            )
        self.alias = alias
        self.timeout = timeout

    @staticmethod
    def find() -> str | None:
        candidates = [os.environ.get("LONG_TAMP_FAST_DOWNWARD")]
        candidates += [shutil.which(n) for n in ("fast-downward.py", "fast-downward")]
        try:
            import up_fast_downward

            bundled = Path(up_fast_downward.__file__).parent / "downward"
            candidates.append(str(bundled / "fast-downward.py"))
        except ImportError:
            pass
        return next((c for c in candidates if c and Path(c).exists()), None)

    def solve(self, export: PddlExport) -> list[Step]:
        with tempfile.TemporaryDirectory(prefix="long-tamp-fd-") as tmp:
            domain, problem = Path(tmp) / "domain.pddl", Path(tmp) / "problem.pddl"
            plan = Path(tmp) / "sas_plan"
            domain.write_text(export.domain)
            problem.write_text(export.problem)
            command = [self.executable]
            if self.executable.endswith(".py"):
                command.insert(0, sys.executable)
            command += ["--plan-file", str(plan), "--alias", self.alias]
            command += [str(domain), str(problem)]
            try:
                run = subprocess.run(
                    command,
                    cwd=tmp,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout,
                )
            except subprocess.TimeoutExpired as error:
                raise NoPlanFound(
                    f"fast-downward: timeout after {self.timeout}s"
                ) from error
            if not plan.exists():
                tail = (run.stdout + run.stderr).strip().splitlines()[-3:]
                raise NoPlanFound(
                    f"fast-downward exit {run.returncode}: {' | '.join(tail)}"
                )
            actions = [
                line.strip()
                for line in plan.read_text().splitlines()
                if line.strip() and not line.startswith(";")
            ]
        return from_pddl_plan(export, actions)


def default_planner(engine: str = "auto") -> TaskPlanner:
    """Fast Downward run directly when it can be found (``engine`` "auto" or
    "fast-downward-direct"), else Unified Planning (``engine`` names its
    engine, "auto" picks one)."""
    if engine in ("auto", "fast-downward-direct") and FastDownwardPlanner.find():
        return FastDownwardPlanner()
    return UnifiedPlanningPlanner(
        "auto" if engine == "fast-downward-direct" else engine
    )


def planner_name(planner: TaskPlanner) -> str:
    if isinstance(planner, FastDownwardPlanner):
        return "fast-downward (direct)"
    if isinstance(planner, UnifiedPlanningPlanner):
        return f"unified-planning/{planner.engine_name()}"
    return type(planner).__name__


def _transaction(
    index: int, capability: str, parameters: dict[str, Any], label: str
) -> dict[str, Any]:
    step_id = f"b{index:02d}-{capability}"
    return {
        "type": "transaction",
        "id": step_id,
        "label": label,
        "restart_state": ["q_current", "grasp_state"],
        "children": [
            {
                "type": "operation",
                "id": f"{step_id}.execute",
                "capability": capability,
                "parameters": dict(parameters),
            }
        ],
    }


def skeleton_document(
    steps: Sequence[Step],
    mission_id: str,
    expand: Expand | None = None,
    initial_state: Iterable[str] = (),
    scene: dict[str, Any] | None = None,
    generator: str = "skeleton_document",
    label: str | None = None,
) -> dict[str, Any]:
    """A TaskPlan document running ``steps`` in order, one transaction each.

    ``expand`` fills in each step's implementation parameters and label, and
    may insert steps around it (see ``Expand``); by default a step becomes
    one transaction labelled ``capability(args)``. Validate the result with
    ``TaskPlan.from_dict(document, registry)``.
    """

    def default(index: int, capability: str, parameters: dict[str, Any]):
        args = ", ".join(str(v) for v in parameters.values())
        return [(capability, parameters, f"{capability}({args})")]

    expand = expand or default
    children: list[dict[str, Any]] = []
    for index, (capability, parameters) in enumerate(steps):
        for cap, params, step_label in expand(index, capability, dict(parameters)):
            children.append(_transaction(len(children), cap, params, step_label))
    return {
        "schema_version": "1.0",
        "mission_id": mission_id,
        "scene": dict(scene or {"id": mission_id}),
        "provenance": {"kind": "planner", "generator": generator},
        "initial_state": list(initial_state),
        "root": {
            "type": "sequence",
            "id": "mission",
            "label": label or mission_id,
            "children": children,
        },
    }


__all__ = [
    "Expand",
    "FastDownwardPlanner",
    "NoPlanFound",
    "Step",
    "TaskPlanner",
    "UnifiedPlanningPlanner",
    "default_planner",
    "planner_name",
    "skeleton_document",
]
