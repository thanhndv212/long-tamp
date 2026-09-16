#!/usr/bin/env python3
"""External orchestrator over ``GraspSequencePlanner``'s capability primitives.

``plan_sequence()`` fuses two concerns: per-phase motion planning and the
policy deciding what to do next (auto-inserting a release when a gripper
holds the wrong object, in ``_plan_auto_release_if_needed``, deep inside
``_run_phase_loop``). That policy is not swappable -- it's hardcoded control
flow in a 4000+ line class full of hard-won, scene-specific fixes.

``run_sequence()`` is the same auto-release policy, reimplemented as a
standalone function driven purely by the ``grasp()``/``release()``
capability primitives (see ``grasp_sequence.py``'s module docstring on why
those exist) instead of touching ``GraspSequencePlanner`` internals. It does
NOT replace or modify ``plan_sequence()``/``resume_sequence()``/
``_run_phase_loop`` -- those stay exactly as they are for every existing
caller. Its purpose is narrower and additive: proving the capability layer
is sufficient for an external orchestrator to reproduce today's common-case
sequencing behavior, so a BT tree or a PDDL planner emitting the same
``grasp``/``release`` calls is a direct, drop-in alternative to this
function -- not a hypothetical one.

Deliberately does not reproduce every ``plan_sequence()`` behavior:
collision-retry-count/timeout knobs, ``phase_q_hints``/lookahead warm-starts,
resume-from-failure, and RunLogger event emission all stay inside
``plan_sequence()``'s own path (already exercised by ``grasp()``/
``release()`` via the shared per-phase helpers) -- this module is the
sequencing policy layer only, not a second implementation of motion
planning.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Sequence

from long_tamp.logging import get_logger

if TYPE_CHECKING:
    from .grasp_sequence import GraspSequencePlanner

logger = get_logger("tasks.sequence_orchestrator")


def run_sequence(
    seq_planner: "GraspSequencePlanner",
    grasp_sequence: Sequence[tuple[str, str | None]],
    q_init: Sequence[float],
    verbose: bool = True,
) -> dict[str, Any]:
    """Drive ``grasp_sequence`` to completion via ``grasp()``/``release()``.

    Unlike ``plan_sequence()`` (which raises on the first phase failure and
    never returns ``success: False``), this never raises -- a phase
    failure stops the sequence and returns a failure dict, matching
    ``grasp()``/``release()``'s own contract. That's a deliberate choice
    for an orchestrator meant to be one of several pluggable
    implementations: exceptions are a poor interface for a policy a caller
    might want to swap out.

    Args:
        seq_planner: A ``GraspSequencePlanner`` already constructed against
            a real scene (same setup ``plan_sequence()`` itself needs).
        grasp_sequence: ``(gripper, handle)`` pairs in order; ``handle is
            None`` means an explicit release of that gripper (same shape
            ``plan_sequence()`` accepts).
        q_init: Starting configuration.
        verbose: Log progress.

    Returns:
        ``success``, ``message``, ``phase_results`` (concatenated from
        every ``grasp()``/``release()`` call, in order), ``final_config``,
        ``grasp_tracker``, ``completed_phases`` (count before any
        failure), and ``failed_phase_idx`` (``None`` on success).
    """
    q_current = list(q_init)
    phase_results: list[dict[str, Any]] = []

    for phase_idx, (gripper, handle) in enumerate(grasp_sequence):
        if handle is None:
            if verbose:
                logger.info("[Orchestrator] Phase %d: release '%s'", phase_idx, gripper)
            result = seq_planner.release(gripper, q_current)
            phase_results.extend(result["phase_results"])
            if not result["success"]:
                return _failure(
                    f"Phase {phase_idx}: {result['message']}",
                    phase_results,
                    q_current,
                    seq_planner,
                    phase_idx,
                )
            q_current = result["final_config"]
            continue

        currently_held = seq_planner.grasp_tracker.current_grasps.get(gripper)
        if currently_held is not None and currently_held != handle:
            # The auto-release policy decision: plan_sequence() makes this
            # same call internally (_plan_auto_release_if_needed); here it
            # is ordinary orchestration code calling the same primitive
            # plan_sequence() itself is built on, not a special case.
            if verbose:
                logger.info(
                    "[Orchestrator] Phase %d: '%s' holds '%s', releasing "
                    "before grasping '%s'",
                    phase_idx,
                    gripper,
                    currently_held,
                    handle,
                )
            release_result = seq_planner.release(gripper, q_current)
            phase_results.extend(release_result["phase_results"])
            if not release_result["success"]:
                return _failure(
                    f"Phase {phase_idx}: auto-release failed: "
                    f"{release_result['message']}",
                    phase_results,
                    q_current,
                    seq_planner,
                    phase_idx,
                )
            q_current = release_result["final_config"]

        if verbose:
            logger.info(
                "[Orchestrator] Phase %d: grasp '%s' with '%s'",
                phase_idx,
                handle,
                gripper,
            )
        grasp_result = seq_planner.grasp(gripper, handle, q_current)
        phase_results.extend(grasp_result["phase_results"])
        if not grasp_result["success"]:
            return _failure(
                f"Phase {phase_idx}: {grasp_result['message']}",
                phase_results,
                q_current,
                seq_planner,
                phase_idx,
            )
        q_current = grasp_result["final_config"]

    return {
        "success": True,
        "message": f"Sequence completed: {len(grasp_sequence)} phases",
        "phase_results": phase_results,
        "final_config": q_current,
        "grasp_tracker": seq_planner.grasp_tracker,
        "completed_phases": len(grasp_sequence),
        "failed_phase_idx": None,
    }


def _failure(
    message: str,
    phase_results: list[dict[str, Any]],
    q_current: list[float],
    seq_planner: "GraspSequencePlanner",
    failed_phase_idx: int,
) -> dict[str, Any]:
    return {
        "success": False,
        "message": message,
        "phase_results": phase_results,
        "final_config": q_current,
        "grasp_tracker": seq_planner.grasp_tracker,
        "completed_phases": failed_phase_idx,
        "failed_phase_idx": failed_phase_idx,
    }


__all__ = ["run_sequence"]
