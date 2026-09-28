"""Block-level failure recovery over ``GraspSequencePlanner``.

A *block* is a short grasp sequence planned as one unit -- e.g. "secure a
part, drive two screws into it". Long missions are chains of blocks, and a
chain is only as reliable as its weakest block, so each block needs to
recover from failure on its own rather than failing the whole mission.

``run_block_with_recovery()`` escalates through three levels, each covering
what the level below it cannot:

1. **Within a phase** -- ``plan_sequence()``'s own target redraws after a
   failed edge (not controlled here).
2. **Resume** -- ``resume_sequence()`` re-plans the failing phase from the
   last completed phase's final configuration, keeping every completed
   phase. Cheap, but it can only redraw the failing phase's own targets.
3. **Replan the block** -- discard everything the block committed
   (``reset_grasp_tracker_to_call_start()``), draw fresh lookahead hints,
   and plan the block again from its entry configuration. This is the only
   level that can undo an earlier phase's commitment.

Level 3 fires in three cases:

- the lookahead's hint chain broke at planning time (a hinted edge's target
  was redrawn, so the "next phase stays reachable" guarantee is void);
- the failing phase is *unreachable* from the block's commitments: target
  generation failed ``unreachable_resumes`` times in a row on the same edge
  with only solver failures and no collision (``ConfigGenerator.
  last_edge_failure``), so no redraw of that phase can succeed;
- ``resume_limit`` resumes in a row failed for any other reason.

Distilled from agimus_spacelab's full-mission runner, where the ladder took
a 22-block multi-arm assembly mission to 10/10 completions over 10 seeded
runs (0.9% of blocks replanned, 29/29 failures recovered). Unlike that
runner, this one is bounded (``max_replans``) and returns a result instead
of retrying until interrupted.
"""

from __future__ import annotations

import gc
from typing import TYPE_CHECKING, Any, Callable, Sequence

from long_tamp.logging import get_logger

if TYPE_CHECKING:
    from .grasp_sequence import GraspSequencePlanner

logger = get_logger("tasks.block_recovery")

HintsFactory = Callable[[], "dict[int, list[list[float]]] | None"]


def run_block_with_recovery(
    seq_planner: GraspSequencePlanner,
    block_seq: Sequence[tuple[str, str | None]],
    q_init: Sequence[float],
    q_scene_init: Sequence[float] | None = None,
    per_phase_frozen_arms: dict[int, list[str]] | None = None,
    label: str = "block",
    hints_factory: HintsFactory | None = None,
    resume_limit: int = 40,
    unreachable_resumes: int = 3,
    max_replans: int | None = 10,
    unfreeze_after: int | None = 30,
    unfreezable_arms: Sequence[str] = (),
    verbose: bool = True,
) -> dict[str, Any]:
    """Plan ``block_seq`` from ``q_init``, recovering from failures.

    Args:
        seq_planner: A ``GraspSequencePlanner`` set up against the scene.
        block_seq: ``(gripper, handle)`` phases; ``handle=None`` releases.
        q_init: The block's entry configuration. Every replan restarts here.
        q_scene_init: True scene-initial configuration (defaults to
            ``q_init``), forwarded to the planner.
        per_phase_frozen_arms: ``{phase_idx: [arm keyword, ...]}``, used in
            ``"manual"`` frozen-arms mode. ``None`` uses ``"auto"``.
        label: Name used in log lines and the result.
        hints_factory: Returns fresh ``phase_q_hints`` (see
            ``make_lookahead_hints_factory()``); called once per attempt at
            the block, so a replan gets a genuinely new candidate.
        resume_limit: Failed resumes in a row before replanning the block.
        unreachable_resumes: Resumes in a row that fail as *unreachable* on
            the same edge before replanning the block early. ``None`` or 0
            disables the fast path.
        max_replans: Replan budget; ``None`` means unbounded.
        unfreeze_after: After this many failures of one phase, remove
            ``unfreezable_arms`` from that phase's frozen list. Cheaper than
            a replan, so keep it below ``resume_limit``.
        unfreezable_arms: Arm keywords safe to unfreeze (frozen by default
            as a margin, not because the phase needs them held).
        verbose: Log progress.

    Returns:
        ``success``, ``final_config``, ``replans``, ``resumes``,
        ``message``, ``label`` and ``failure``. ``final_config`` is
        ``q_init`` on failure. ``failure`` is ``None`` on success, else the
        last failure before giving up: ``{"kind", "phase_idx", "edge"}``
        with ``kind`` one of ``"unreachable"``, ``"stuck"`` (resume limit),
        ``"hint_chain_broken"`` (``phase_idx`` the first broken phase,
        ``edge`` ``None``) and ``"no_resumable_state"`` (both ``None``).
    """
    frozen = (
        {i: list(arms) for i, arms in per_phase_frozen_arms.items()}
        if per_phase_frozen_arms is not None
        else None
    )
    mode = "manual" if frozen is not None else "auto"
    q_init = list(q_init)
    q_scene_init = list(q_scene_init) if q_scene_init is not None else q_init
    fail_counts: dict[int, int] = {}
    replans = 0
    total_resumes = 0
    failure: dict[str, Any] | None = None

    def _log(msg: str, *args: Any) -> None:
        if verbose:
            logger.info("[%s] " + msg, label, *args)

    def _maybe_unfreeze(phase_idx: int) -> None:
        fail_counts[phase_idx] = fail_counts.get(phase_idx, 0) + 1
        if (
            frozen is None
            or unfreeze_after is None
            or fail_counts[phase_idx] != unfreeze_after
        ):
            return
        before = frozen.get(phase_idx, [])
        after = [a for a in before if a not in unfreezable_arms]
        if after != before:
            _log(
                "phase %d failed %d times; unfreezing %s",
                phase_idx + 1,
                unfreeze_after,
                sorted(set(before) - set(after)),
            )
            frozen[phase_idx] = after

    def _result(success: bool, q: Sequence[float], message: str) -> dict[str, Any]:
        return {
            "success": success,
            "final_config": list(q),
            "replans": replans,
            "resumes": total_resumes,
            "message": message,
            "label": label,
            "failure": None if success else failure,
        }

    hints = hints_factory() if hints_factory else None

    while True:
        # --- one attempt at the block, from its entry configuration -----
        try:
            result = seq_planner.plan_sequence(
                grasp_sequence=list(block_seq),
                q_init=q_init,
                q_scene_init=q_scene_init,
                frozen_arms_mode=mode,
                per_phase_frozen_arms=frozen,
                verbose=verbose,
                phase_q_hints=hints,
            )
        except KeyboardInterrupt:
            raise
        except Exception as e:  # plan_sequence reports failure by raising
            _log("plan_sequence failed: %s", e)
            result = {"success": False}

        if result.get("success"):
            return _result(True, result["final_config"], "planned")

        broken = _hint_chain_broken(seq_planner, hints)
        if broken is not None:
            reason, failure = broken
        else:
            reason = _resume_until_stuck(
                seq_planner,
                mode,
                frozen,
                hints,
                resume_limit,
                unreachable_resumes,
                _maybe_unfreeze,
                verbose,
                _log,
            )
            total_resumes += reason["resumes"]
            if reason["success"]:
                return _result(True, reason["final_config"], "resumed")
            failure = reason["failure"]
            reason = reason["why"]

        # --- level 3: discard the block's commitments and start over ----
        if max_replans is not None and replans >= max_replans:
            return _result(False, q_init, f"gave up after {replans} replans ({reason})")
        replans += 1
        _log("replanning from entry (replan %d): %s", replans, reason)
        seq_planner.reset_grasp_tracker_to_call_start()
        if replans % 5 == 0:
            # Each replan rebuilds phase graphs and a lookahead round; the
            # native bindings leave reference cycles gc must break.
            gc.collect()
        hints = hints_factory() if hints_factory else None


def _hint_chain_broken(
    seq_planner: GraspSequencePlanner, hints: dict | None
) -> tuple[str, dict[str, Any]] | None:
    broken = set(getattr(seq_planner, "invalidated_phase_hints", ())) & set(hints or {})
    if not broken:
        return None
    return (
        f"hint chain broken for phase(s) {sorted(i + 1 for i in broken)} "
        "(a hinted target was redrawn)",
        {"kind": "hint_chain_broken", "phase_idx": min(broken), "edge": None},
    )


def _resume_until_stuck(
    seq_planner: GraspSequencePlanner,
    mode: str,
    frozen: dict[int, list[str]] | None,
    hints: dict | None,
    resume_limit: int,
    unreachable_resumes: int | None,
    maybe_unfreeze: Callable[[int], None],
    verbose: bool,
    log: Callable[..., None],
) -> dict[str, Any]:
    """Resume the failed block until it succeeds or resuming can't help."""
    resumes = 0
    streak = 0
    streak_edge: str | None = None
    while True:
        state = seq_planner.get_resumable_state()
        if state is None:
            # Nothing left to resume: every phase completed.
            q = (
                seq_planner.phase_results[-1].get("final_config")
                if seq_planner.phase_results
                else None
            )
            return {
                "success": q is not None,
                "final_config": q,
                "resumes": resumes,
                "why": "no resumable state",
                "failure": (
                    None
                    if q is not None
                    else {"kind": "no_resumable_state", "phase_idx": None, "edge": None}
                ),
            }

        resumes += 1
        should_log = resumes == 1 or resumes % 10 == 0
        if should_log:
            log(
                "phase %d edge %d failed (%s); resume %d",
                state["phase_idx"] + 1,
                state["edge_idx"] + 1,
                state["error"],
                resumes,
            )
        maybe_unfreeze(state["phase_idx"])
        try:
            result = seq_planner.resume_sequence(
                retry_from_edge=-1,
                frozen_arms_mode=mode,
                per_phase_frozen_arms=frozen,
                verbose=verbose and should_log,
                phase_q_hints=hints,
            )
        except KeyboardInterrupt:
            raise
        except Exception as e:
            log("resume_sequence failed: %s", e)
            result = {"success": False}
        if result.get("success"):
            return {
                "success": True,
                "final_config": result["final_config"],
                "resumes": resumes,
                "why": "",
                "failure": None,
            }

        edge = unreachable_failed_edge(seq_planner)
        streak = (
            streak + 1
            if edge is not None and edge == streak_edge
            else (1 if edge is not None else 0)
        )
        streak_edge = edge
        if unreachable_resumes and streak >= unreachable_resumes:
            failed = seq_planner.get_resumable_state() or state
            return {
                "success": False,
                "final_config": None,
                "resumes": resumes,
                "why": f"phase {failed['phase_idx'] + 1} unreachable "
                f"(solver-only failures, no collisions, {streak} resumes "
                f"on {failed['edge_name']})",
                "failure": {
                    "kind": "unreachable",
                    "phase_idx": failed["phase_idx"],
                    "edge": failed["edge_name"],
                },
            }
        if resumes >= resume_limit:
            return {
                "success": False,
                "final_config": None,
                "resumes": resumes,
                "why": f"phase {state['phase_idx'] + 1} still failing after "
                f"{resumes} resumes ({state['edge_name']})",
                "failure": {
                    "kind": "stuck",
                    "phase_idx": state["phase_idx"],
                    "edge": state["edge_name"],
                },
            }
        if resumes % 10 == 0:
            gc.collect()


def unreachable_failed_edge(seq_planner: GraspSequencePlanner) -> str | None:
    """Edge the block just failed on, if the failure means *unreachable*.

    That is: target generation failed on a phase with a committed phase
    before it in the block, and every attempt failed in the constraint
    solver -- none even reached collision checking -- on that same edge.
    Redrawing that phase cannot help; only undoing an earlier commitment
    can. ``None`` for any other failure.
    """
    state = seq_planner.get_resumable_state()
    gen = getattr(seq_planner, "config_gen", None)
    fail = getattr(gen, "last_edge_failure", None)
    if (
        state is None
        or fail is None
        or not state.get("completed_phases")
        or not str(state.get("error")).startswith("Target generation failed")
        or fail["edge_name"] != state["edge_name"]
        or fail["collision_invalid"] > 0
        or fail["solver_failed"] == 0
    ):
        return None
    return state["edge_name"]


def make_lookahead_hints_factory(
    seq_planner: GraspSequencePlanner,
    block_seq: Sequence[tuple[str, str | None]],
    q_current: Sequence[float],
    q_scene_init: Sequence[float] | None = None,
    per_phase_frozen_arms: dict[int, list[str]] | None = None,
    phase_pair: tuple[int, int] = (0, 1),
    also_protect: Sequence[int] = (),
    max_rounds: int | None = 5,
    probe_timeout: float = 5.0,
    max_candidates: int = 100,
    verify_paths: bool = False,
    verbose: bool = True,
) -> HintsFactory:
    """Build a ``hints_factory`` that probes the block's lookahead.

    Each call searches ``find_feasible_phase_target()`` for a phase
    ``phase_pair[0]`` candidate leaving ``phase_pair[1]`` -- and every phase
    in ``also_protect`` -- reachable, retrying up to ``max_rounds`` fresh
    rounds. Returns ``{phase_pair[0]: chain}``, or ``None`` when no round
    found one (the block then plans unhinted; the recovery ladder remains).
    ``verify_paths`` also path-plans phase ``phase_pair[0]`` to each candidate
    (see ``find_feasible_phase_target``), so fewer hint chains break later.
    """
    frozen = per_phase_frozen_arms or {}
    n_idx, n1_idx = phase_pair
    q_current = list(q_current)
    q_scene = list(q_scene_init) if q_scene_init is not None else q_current
    also = [
        (block_seq[i], frozen.get(i, []))
        for i in also_protect
        if i < len(block_seq) and block_seq[i][1] is not None
    ]

    def factory() -> dict[int, list[list[float]]] | None:
        rounds = 0
        while max_rounds is None or rounds < max_rounds:
            rounds += 1
            chain = seq_planner.find_feasible_phase_target(
                phase_n=block_seq[n_idx],
                phase_n1=block_seq[n1_idx],
                q_current=q_current,
                q_scene_init=q_scene,
                frozen_arms_n=frozen.get(n_idx, []),
                frozen_arms_n1=frozen.get(n1_idx, []),
                probe_timeout=probe_timeout,
                max_candidates=max_candidates,
                verbose=verbose,
                also_reachable=also,
                verify_paths=verify_paths,
            )
            if chain is not None:
                return {n_idx: chain}
            if rounds % 5 == 0:
                gc.collect()
        if verbose:
            logger.warning(
                "lookahead found no candidate in %d round(s); planning unhinted",
                rounds,
            )
        return None

    return factory


__all__ = [
    "make_lookahead_hints_factory",
    "run_block_with_recovery",
    "unreachable_failed_edge",
]
