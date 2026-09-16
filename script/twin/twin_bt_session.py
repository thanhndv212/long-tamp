#!/usr/bin/env python3
"""TWIN "lift ball" mission adapter for the BehaviorTree.CPP host.

The generic ``task_planning/`` layer (``src/long_tamp/tasks/task_planning/``)
knows nothing about any specific robot or mission -- it only ships
``create_fake_session``, a mission-agnostic conformance adapter (see its
own docstring). Per ``docs/usage/behaviortree-integration.md`` §9, a real
mission adapter -- its own capability registry, ``TaskPlan`` document,
session -- lives in its own module and registers a factory in ``host.py``.
This is that module for TWIN's real bimanual "lift ball" scene
(``task_lift_ball.py``), the first from-scratch (no SpaceLab content)
example of a real mission driving this pipeline end to end.

Deliberately does NOT subclass ``HostSession`` with mission-specific state
(a ``q_current``/``seq_planner`` attribute, etc.) -- the capability
implementations below close over a plain mutable ``state`` dict instead.
``HostSession`` itself stays exactly as generic as ``create_fake_session``
already needs it to be; nothing mission-specific leaks into it.

Each capability (``grasp``, ``release``) is a thin wrapper around
``GraspSequencePlanner.grasp()``/``.release()`` (see ``grasp_sequence.py``'s
module docstring on why those primitives exist): reads ``state["q_current"]``,
calls the primitive, and only writes ``state["q_current"]`` back on success
-- raising without mutating state on failure, matching the atomicity
contract every ``transaction``-wrapped capability must uphold (see
``TaskPlanningSession.execute_step``'s docstring). A failed BT
``RetryUntilSuccessful`` attempt therefore always restarts the grasp from
its true entry configuration, never from wherever a failed attempt gave
up -- the same lesson ``resume_sequence()`` had to learn the hard way (see
``tests/test_resume_start_config.py``), here true by construction since
``grasp()``/``release()`` take ``q_current`` as an explicit argument rather
than tracking their own resume state.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_TWIN_SCRIPT_DIR = Path(__file__).resolve().parent


def build_twin_session(options_json: str = "{}") -> Any:
    """Build a real ``HostSession`` driving TWIN's bimanual lift-ball scene.

    Constructs the actual scene (``LiftBallTask`` + ``GraspSequencePlanner``,
    ``backend="pyhpp"`` -- real RRT/IK planning, no mocks) exactly as
    ``task_lift_ball.py``'s own ``run_task()`` does, registers ``grasp``/
    ``release`` capabilities against it, and compiles a plan mirroring
    ``GRASP_SEQUENCE`` (both grasps, no release -- TWIN's real mission
    never needs to release either gripper).
    """
    if str(_TWIN_SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(_TWIN_SCRIPT_DIR))
    import task_lift_ball as twin  # local import: path-dependent, see above

    from long_tamp.tasks.grasp_sequence import GraspSequencePlanner
    from long_tamp.tasks.task_planning.capabilities import (
        CapabilityDescriptor,
        CapabilityRegistry,
    )
    from long_tamp.tasks.task_planning.host import HostSession
    from long_tamp.tasks.task_planning.model import TaskPlan

    task = twin.LiftBallTask(backend="pyhpp")
    task.setup(
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
        skip_graph=True,
    )
    q_init = task.q_init
    if not q_init:
        raise RuntimeError("TWIN scene setup did not produce an initial configuration")

    seq_planner = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        graph_constraints=getattr(task, "_graph_constraints", None),
        auto_save_dir=None,
        run_logger=getattr(task, "run_logger", None),
    )
    state: dict[str, Any] = {"q_current": list(q_init)}

    def grasp_impl(parameters: dict) -> dict:
        result = seq_planner.grasp(
            parameters["gripper"], parameters["handle"], state["q_current"]
        )
        if not result["success"]:
            raise RuntimeError(result["message"])
        state["q_current"] = result["final_config"]
        return {"gripper": parameters["gripper"], "handle": parameters["handle"]}

    def release_impl(parameters: dict) -> dict:
        result = seq_planner.release(parameters["gripper"], state["q_current"])
        if not result["success"]:
            raise RuntimeError(result["message"])
        state["q_current"] = result["final_config"]
        return {"gripper": parameters["gripper"]}

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            capability_id="grasp",
            version="1.0",
            required_parameters={"gripper": str, "handle": str},
            effects=("grasp_state",),
            max_attempts=3,
            max_timeout=300.0,
            restartable=True,
        ),
        grasp_impl,
    )
    registry.register(
        CapabilityDescriptor(
            capability_id="release",
            version="1.0",
            required_parameters={"gripper": str},
            effects=("grasp_state",),
            max_attempts=3,
            max_timeout=300.0,
            restartable=True,
        ),
        release_impl,
    )

    document = _build_plan_document(twin.GRASP_SEQUENCE)
    plan = TaskPlan.from_dict(document, registry)
    return HostSession(plan, registry)


def _build_plan_document(grasp_sequence: list[tuple[str, str]]) -> dict[str, Any]:
    """TaskPlan IR: a sequence of transactions, one grasp each, in order.

    Mirrors ``GRASP_SEQUENCE`` from ``task_lift_ball.py`` exactly -- this
    document is the single source of truth the compiler turns into the BT
    XML the C++ host ticks; keeping it a direct transliteration of
    ``GRASP_SEQUENCE`` is what makes this a faithful BT-driven equivalent
    of calling ``run_sequence()``/``plan_sequence()`` with the same list,
    not a parallel, independently-maintained mission definition.
    """
    children = []
    for index, (gripper, handle) in enumerate(grasp_sequence):
        node_id = f"grasp-{index}"
        children.append(
            {
                "type": "transaction",
                "id": node_id,
                "label": f"Grasp {handle} with {gripper}",
                "restart_state": ["q_current"],
                "children": [
                    {
                        "type": "operation",
                        "id": f"{node_id}.execute",
                        "capability": "grasp",
                        "parameters": {"gripper": gripper, "handle": handle},
                    }
                ],
            }
        )
    return {
        "schema_version": "1.0",
        "mission_id": "TwinLiftBall",
        "scene": {"id": "twin-lift-ball", "robots": ["panda_left", "panda_right"]},
        "provenance": {"kind": "human", "generator": "twin-bt-adapter"},
        "root": {
            "type": "sequence",
            "id": "root",
            "label": "TWIN lift ball",
            "children": children,
        },
    }
