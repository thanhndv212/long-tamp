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
collision-retry-count/timeout knobs, resume-from-failure, and RunLogger
event emission all stay inside ``plan_sequence()``'s own path (already
exercised by ``grasp()``/``release()`` via the shared per-phase helpers)
-- this module is the sequencing policy layer only, not a second
implementation of motion planning.

Lookahead (``lookahead_pairs``) is the one exception: a real scene
(``script/ikea_table_prototype/task_assemble_table.py``'s leg-pickup ->
peg-into-socket docking pairs) hit exactly the failure class
``GraspSequencePlanner.find_feasible_phase_target()`` exists to fix --
phase N's randomly-drawn grasp target left phase N+1 geometrically
unreachable, ~1850/1850 solver failures, never a collision, residual
never shrinking -- and unlike ``plan_sequence()``, this module had no way
to opt a phase pair into that protection at all (``grasp()`` itself
couldn't even accept a pre-found candidate to commit). Both gaps are
closed here: ``grasp()`` now takes an optional ``q_hint``, and
``lookahead_pairs`` tells this function which phase indices to probe with
``find_feasible_phase_target()`` before committing, threading the winning
candidate through as that ``q_hint`` instead of letting ``grasp()`` draw
an unvalidated random one.
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
    per_phase_frozen_arms: dict[int, list[str]] | None = None,
    lookahead_pairs: Sequence[int] = (),
    q_scene_init: Sequence[float] | None = None,
    lookahead_probe_timeout: float = 5.0,
    lookahead_max_candidates: int = 100,
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
        per_phase_frozen_arms: Optional ``{phase_idx: [arm, ...]}`` manual
            frozen-arms override, same shape ``plan_sequence()`` accepts.
            Threaded into every ``grasp()``/``release()`` call as that
            phase's own list (both primitives build their own graph at
            their internal ``phase_idx=0``, so this function remaps: phase
            5's entry here becomes ``per_phase_frozen_arms={0: [...]}`` for
            that one call) -- ``None`` (the default) leaves every phase on
            ``grasp()``/``release()``'s own ``frozen_arms_mode="auto"``.
        lookahead_pairs: Phase indices ``i`` where phase ``i``'s grasp
            target should be probed with
            ``seq_planner.find_feasible_phase_target()`` against phase
            ``i + 1`` before committing, instead of letting ``grasp()``
            draw an unvalidated random one (see module docstring). Both
            phase ``i`` and phase ``i + 1`` must be grasps (``handle is
            not None``) -- ``find_feasible_phase_target()`` doesn't support
            release phases. A candidate search that finds nothing
            (``None``) logs a warning and falls back to an unprotected
            ``grasp()`` call rather than failing the whole sequence outright
            -- lookahead is a best-effort improvement to the odds, not a
            feasibility proof.
        q_scene_init: True scene-initial configuration for
            ``find_feasible_phase_target()``'s own ``q_scene_init`` --
            defaults to ``q_init`` (this function's own starting
            configuration) when omitted, matching ``grasp()``/``release()``
            defaulting to ``q_current`` when their own ``q_scene_init`` is
            unset.
        lookahead_probe_timeout: Forwarded to
            ``find_feasible_phase_target()``.
        lookahead_max_candidates: Forwarded to
            ``find_feasible_phase_target()``.

    Returns:
        ``success``, ``message``, ``phase_results`` (concatenated from
        every ``grasp()``/``release()`` call, in order), ``final_config``,
        ``grasp_tracker``, ``completed_phases`` (count before any
        failure), and ``failed_phase_idx`` (``None`` on success).
    """
    q_current = list(q_init)
    q_scene_init = list(q_scene_init) if q_scene_init is not None else list(q_init)
    lookahead_pairs = set(lookahead_pairs)
    phase_results: list[dict[str, Any]] = []

    def _frozen_for(phase_idx: int) -> dict[int, list[str]] | None:
        if per_phase_frozen_arms is None:
            return None
        return {0: per_phase_frozen_arms.get(phase_idx, [])}

    for phase_idx, (gripper, handle) in enumerate(grasp_sequence):
        if handle is None:
            if verbose:
                logger.info("[Orchestrator] Phase %d: release '%s'", phase_idx, gripper)
            result = seq_planner.release(
                gripper,
                q_current,
                frozen_arms_mode="manual" if per_phase_frozen_arms else "auto",
                per_phase_frozen_arms=_frozen_for(phase_idx),
            )
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
            release_result = seq_planner.release(
                gripper,
                q_current,
                frozen_arms_mode="manual" if per_phase_frozen_arms else "auto",
                per_phase_frozen_arms=_frozen_for(phase_idx),
            )
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

        q_hint = None
        if phase_idx in lookahead_pairs:
            next_gripper, next_handle = grasp_sequence[phase_idx + 1]
            if next_handle is None:
                raise ValueError(
                    f"lookahead_pairs entry {phase_idx} requires phase "
                    f"{phase_idx + 1} to also be a grasp (got a release) -- "
                    "find_feasible_phase_target() only supports grasp/grasp "
                    "pairs"
                )
            if verbose:
                logger.info(
                    "[Orchestrator] Phase %d: probing lookahead against "
                    "phase %d ('%s' -> '%s') before committing",
                    phase_idx,
                    phase_idx + 1,
                    next_gripper,
                    next_handle,
                )
            q_hint = seq_planner.find_feasible_phase_target(
                phase_n=(gripper, handle),
                phase_n1=(next_gripper, next_handle),
                q_current=q_current,
                q_scene_init=q_scene_init,
                frozen_arms_n=(per_phase_frozen_arms or {}).get(phase_idx, []),
                frozen_arms_n1=(per_phase_frozen_arms or {}).get(phase_idx + 1, []),
                probe_timeout=lookahead_probe_timeout,
                max_candidates=lookahead_max_candidates,
                verbose=verbose,
            )
            if q_hint is None and verbose:
                logger.warning(
                    "[Orchestrator] Phase %d: lookahead found no candidate "
                    "leaving phase %d reachable within %d probes; falling "
                    "back to an unprotected grasp",
                    phase_idx,
                    phase_idx + 1,
                    lookahead_max_candidates,
                )

        if verbose:
            logger.info(
                "[Orchestrator] Phase %d: grasp '%s' with '%s'",
                phase_idx,
                handle,
                gripper,
            )
        grasp_result = seq_planner.grasp(
            gripper,
            handle,
            q_current,
            frozen_arms_mode="manual" if per_phase_frozen_arms else "auto",
            per_phase_frozen_arms=_frozen_for(phase_idx),
            q_scene_init=q_scene_init,
            q_hint=q_hint,
        )
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
