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


def build_twin_regrasp_session(options_json: str = "{}") -> Any:
    """Build a ``HostSession`` that forces a real release before a real grasp.

    ``build_twin_session()``'s document is a flat two-transaction sequence
    (§4/§11 "flat-sequence case already verified") -- both grippers grasp
    once each, nothing ever releases, because TWIN's actual bimanual
    mission never needs it (see that function's docstring). This factory
    exercises the gap identified in ``docs/usage/behaviortree-integration.md``
    §11 item 1: a single gripper (``panda_left/gripper`` only --
    ``panda_right`` stays idle) grasps ``ball/handle``, then a ``fallback``
    forces a real ``release()`` and a real re-``grasp()`` of that *same*
    handle.

    Deliberately the *same* handle: two other shapes were tried and
    rejected while building this scenario, both real findings and both
    orthogonal to what item 1 asks to prove, so worth recording here
    rather than re-discovering them:

    1. Regrasping onto ``ball/handle2`` with ``panda_left/gripper`` (the
       handle TWIN's real mission always gives to ``panda_right/gripper``)
       is kinematically unreachable from that gripper's approach direction
       -- consistent solver-residual non-convergence (``Last validity
       error: None``, not a collision) across retries.
    2. Regrasping onto ``ball/handle2`` with ``panda_right/gripper`` (i.e.
       the "easy" grasp TWIN's own flat mission plans in ~18s) turns out to
       only be easy as the *second* phase built on an already-partially-
       built dual-grasp graph (edge name ``0-0_01``): built standalone, as
       the only phase in a single-gripper mission like this one (edge name
       ``f_01``, same naming ``panda_left/gripper``'s own first grasp
       below uses), it failed 6/6 draws across two independent processes,
       residuals scattered from 0.01 to 8.9 with no collision
       (``Last validity error: None``) -- i.e. "easy" here was an artifact
       of phase-build order, not the handle/gripper pairing itself.

    ``panda_left/gripper`` regrasping ``ball/handle`` (this function's
    actual choice) sidesteps both: its first grasp reliably reaches the
    target standalone (2/2 real runs, ~90-95s of RRT planning each). The
    regrasp step (release, then grasp the same target again) does hit a
    real snag of its own, though a *known* one: it lands on the exact
    ``f_12`` pregrasp -> grasp waypoint collision already documented as
    intermittently flaky in ``tests/test_grasp_release_use_case_twin.py``
    (``panda_left/panda_*finger_*`` vs ``ball/base_link_0``) -- observed
    100% of regrasp draws across two independent verification runs before
    the ``grasp`` capability's ``max_attempts`` below was raised from 3 to
    8 to give the BT-level ``RetryUntilSuccessful`` more real budget
    against it (consistent with how the *first* grasp above also sometimes
    needs several draws, just never zero for zero). This is a pre-existing
    scene-asset clearance question, not a compiler/session bug -- see that
    test's own docstring for why it's tracked but not fixed here.

    The plan IR expresses the forced cycle as a ``fallback``: an ``empty``
    *condition* checking whether the gripper currently holds nothing (false
    right after ``grasp-handle1`` -- it's still holding ``ball/handle``)
    guards a ``sequence`` of ``release`` then ``grasp`` transactions. This
    is the first plan document in the repo to compile a top-level
    ``fallback``/``condition`` composed with nested ``transaction``s rather
    than a flat run of transactions, proving the compiler's per-node
    ``Fallback``/``retry`` shapes (§4) compose correctly one level up, not
    just individually.
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

    def empty_impl(parameters: dict) -> bool:
        return (
            seq_planner.grasp_tracker.current_grasps.get(parameters["gripper"]) is None
        )

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            capability_id="grasp",
            version="1.0",
            required_parameters={"gripper": str, "handle": str},
            effects=("grasp_state",),
            # Higher than create_twin_session's grasp (3): this scenario's
            # regrasp step deterministically re-lands on ball/handle's known
            # flaky f_12 waypoint (see this function's docstring) -- more
            # BT-level RetryUntilSuccessful budget compensates for real
            # target-generation variance on that specific marginal edge.
            max_attempts=8,
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
    registry.register(
        CapabilityDescriptor(
            capability_id="empty",
            version="1.0",
            required_parameters={"gripper": str},
        ),
        empty_impl,
    )

    document = _build_regrasp_plan_document()
    plan = TaskPlan.from_dict(document, registry)
    return HostSession(plan, registry)


def _build_regrasp_plan_document() -> dict[str, Any]:
    """IR: grasp ``ball/handle``, then a ``fallback``-guarded forced
    release+regrasp of that *same* handle, with ``panda_left/gripper`` only.

    See ``build_twin_regrasp_session()``'s docstring for why this shape
    (rather than a flat sequence, and rather than the other
    gripper/handle combinations tried first) is the point of this document.
    """
    gripper = "panda_left/gripper"
    handle = "ball/handle"
    return {
        "schema_version": "1.0",
        "mission_id": "TwinRegrasp",
        "scene": {"id": "twin-lift-ball", "robots": ["panda_left"]},
        "provenance": {"kind": "human", "generator": "twin-bt-regrasp-adapter"},
        "root": {
            "type": "sequence",
            "id": "root",
            "label": "TWIN regrasp",
            "children": [
                {
                    "type": "transaction",
                    "id": "grasp-handle1",
                    "label": "Grasp ball/handle with panda_left/gripper",
                    "restart_state": ["q_current"],
                    "children": [
                        {
                            "type": "operation",
                            "id": "grasp-handle1.execute",
                            "capability": "grasp",
                            "parameters": {"gripper": gripper, "handle": handle},
                        }
                    ],
                },
                {
                    "type": "fallback",
                    "id": "already-cycled",
                    "label": "Already released and regrasped ball/handle",
                    "children": [
                        {
                            "type": "condition",
                            "id": "gripper-empty",
                            "label": "panda_left/gripper currently holds nothing",
                            "capability": "empty",
                            "parameters": {"gripper": gripper},
                        },
                        {
                            "type": "sequence",
                            "id": "release-then-regrasp",
                            "label": "Release ball/handle, regrasp ball/handle",
                            "children": [
                                {
                                    "type": "transaction",
                                    "id": "release-gripper",
                                    "label": "Release panda_left/gripper",
                                    "restart_state": ["q_current"],
                                    "children": [
                                        {
                                            "type": "operation",
                                            "id": "release-gripper.execute",
                                            "capability": "release",
                                            "parameters": {"gripper": gripper},
                                        }
                                    ],
                                },
                                {
                                    "type": "transaction",
                                    "id": "grasp-handle1-again",
                                    "label": "Regrasp ball/handle with panda_left/gripper",
                                    "restart_state": ["q_current"],
                                    "children": [
                                        {
                                            "type": "operation",
                                            "id": "grasp-handle1-again.execute",
                                            "capability": "grasp",
                                            "parameters": {
                                                "gripper": gripper,
                                                "handle": handle,
                                            },
                                        }
                                    ],
                                },
                            ],
                        },
                    ],
                },
            ],
        },
    }


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
