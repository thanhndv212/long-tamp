"""The screw assembly as a ``MissionDomain`` (ADR-0011, #131).

The reference implementation of ``long_tamp.mission.MissionDomain``. It
delegates to ``screw_domain`` (and, for the geometry, ``task_screw_assembly``),
so nothing about the mission changes: ``task_screw_assembly.py`` still calls
those functions directly until the generic runner replaces its loop (#132,
#134).

Importing this module needs no HPP; ``setup``, ``plan_move`` and
``make_backend`` import ``task_screw_assembly`` (HPP) when called.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from typing import Any

import screw_domain as D
from long_tamp.mission import (
    BaseMissionDomain,
    MissionScene,
    MissionStep,
    Move,
    Scenario,
    Transaction,
)
from long_tamp.tasks.task_planning.capabilities import CapabilityDescriptor
from long_tamp.tasks.task_planning.language import Vocabulary
from long_tamp.tasks.task_planning.pddl import PddlExport
from long_tamp.tasks.task_planning.predicates import Atom
from long_tamp.tasks.task_planning.repair import Binding, Failure
from long_tamp.tasks.task_planning.skeleton import Step

INTRO = """\
Two UR10 arms assemble parts on a jig: the left arm carries each part to a
clamp, the right arm drives its two screws with a screwdriver it picks from
a rack, and puts the driver back at the end."""


class ScrewAssemblyDomain(BaseMissionDomain):
    """``n_parts`` parts screwed in the jig's clamps. ``clamps`` are the
    scene's (clamp, seat) pairs (``screw_domain.clamp_seats``); by default
    clamp i takes part i. ``setup`` reads them from the scene."""

    name = "screw-assembly"
    recorded_predicates = D.RECORDED_PREDICATES
    goal_notes = D.GOAL_NOTES
    intro = INTRO

    def __init__(
        self, n_parts: int, clamps: list[tuple[str, str]] | None = None
    ) -> None:
        self.n_parts = n_parts
        self.clamps = clamps

    # -- symbolic ------------------------------------------------------------

    def scene_info(self) -> dict[str, Any]:
        return {"id": self.name, "parts": self.n_parts}

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        return D.descriptors(self.n_parts)

    def static_facts(self) -> tuple[str, ...]:
        return tuple(D.static_facts(self.n_parts, self.clamps))

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        return dict(D.STATIC_PRECONDITIONS)

    def default_goal(self) -> list[str]:
        return D.mission_goal(self.n_parts)

    def pddl_problem(
        self,
        state: Iterable[str | Atom] = (),
        blocked: Iterable[Binding] = (),
        goal: Sequence[str] | None = None,
    ) -> PddlExport:
        return D.pddl_problem(
            self.n_parts,
            [str(a) for a in state],
            blocked=list(blocked),
            clamps=self.clamps,
            goal=None if goal is None else list(goal),
        )

    def goal_vocabulary(self, state: Iterable[str | Atom] = ()) -> Vocabulary:
        return D.goal_vocabulary(self.n_parts, [str(a) for a in state], self.clamps)

    def plan_document(
        self, initial_state: Iterable[str] = ()
    ) -> dict[str, Any] | None:
        return D.build_plan_document(self.n_parts, list(initial_state) or None)

    # -- skeleton -> plan ----------------------------------------------------

    def expand_step(
        self,
        index: int,
        capability: str,
        parameters: dict[str, Any],
        held: Mapping[str, str],
    ) -> list[Transaction]:
        # screw_domain.expand_step keeps ``held`` up to date itself.
        return D.expand_step(index, capability, parameters, dict(held))

    def planned_document(
        self,
        steps: Sequence[Step],
        state: Iterable[str | Atom] = (),
        generator: str = "unified-planning",
    ) -> dict[str, Any]:
        return D.planned_document(
            list(steps), self.n_parts, [str(a) for a in state], generator
        )

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> MissionStep:
        block = D.block_for(capability, dict(parameters))
        if "move" in block:
            arm, carried = block["move"]
            return Move(block["label"], {"arm": arm, "carried": carried})
        return D.refinement_step(block)

    # -- repair --------------------------------------------------------------

    def repair_policy(self, failure: Failure) -> list[Binding]:
        return D.repair_policy(failure)

    # -- geometry and execution ----------------------------------------------

    def scenarios(self) -> dict[str, Scenario]:
        from scenarios import SCENARIOS

        return {
            name: Scenario(
                name,
                setup=tuple(spec["setup"]),
                facts=tuple(spec["facts"]),
                skip=tuple(spec["skip"]),
                atoms=tuple(spec["atoms"]),
            )
            for name, spec in SCENARIOS.items()
        }

    def setup(self, seed: int | None = None, log_dir: str = "auto") -> MissionScene:
        import task_screw_assembly as T

        if seed is not None:
            T.seed_everything(seed)
        task, planner = T.setup(log_dir=log_dir)
        config: Any = task.task_config
        self.clamps = D.clamp_seats(config.VALID_PAIRS)
        return MissionScene(task, planner, list(task.q_init))

    def plan_move(
        self,
        scene: MissionScene,
        move: Move,
        q: Sequence[float],
        verbose: bool = True,
    ) -> dict[str, Any]:
        import task_screw_assembly as T

        return T.run_home_move(
            scene.task,
            scene.planner,
            list(q),
            move.spec["arm"],
            move.spec.get("carried"),
            verbose=verbose,
        )

    def execution_payload(self, phase: Mapping[str, Any], edge: Any, path: Any) -> Any:
        """Driving a screw is a skill: the driver tip's insertion into a hole
        (grasp edge ``_12``) is a ``SkillCommand`` for the screw skill, as in
        ``task_screw_assembly.block_commands``."""
        if (
            phase.get("gripper") == D.DRIVER_TIP
            and edge is not None
            and str(edge).endswith("_12")
        ):
            from long_tamp.sim.skills import SCREW
            from long_tamp.tasks.task_planning.skills import SkillCommand

            hole = phase.get("handle") or ""
            return SkillCommand(
                SCREW,
                {"tool": "driver", "part": hole.split("/")[0], "hole": hole},
                approach=path,
            )
        return path

    def make_backend(self, name: str, scene: MissionScene, **options: Any) -> Any:
        if name == "none":
            return None
        import task_screw_assembly as T

        return T.make_backend(name, scene.task, **options)
