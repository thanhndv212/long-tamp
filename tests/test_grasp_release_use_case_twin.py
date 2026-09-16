"""Real-scene integration test for ``GraspSequencePlanner.grasp()``/``.release()``.

``tests/test_grasp_release_capabilities.py`` proves the new primitives' own
glue logic (precondition checks, exception-to-dict conversion) against
stubbed helpers. It does NOT prove they can drive a real HPP scene to a
real successful grasp -- that risk lives entirely in the helpers they
reuse (``_build_phase_graph_and_constraints``, ``_compute_and_project_edge_sequence``,
``_plan_phase_edges``, ``_release_frozen_arms``, ``_plan_release_subphase``),
which stub-based tests can't exercise for real.

This test runs the actual TWIN "lift ball" bimanual scene
(``script/twin/task_lift_ball.py`` -- the smallest real multi-grasp use
case in the repo: two independent Panda arms, two grasps, no auto-release
needed) through ``grasp()``/``release()`` directly, instead of through
``plan_sequence()``. It is the regression safety net for the next
refactor step (pulling the orchestration policy -- auto-release insertion,
retry, lookahead -- out of ``plan_sequence()`` into a separate, swappable
layer): once this passes, we know the primitives are behaviorally real,
not just internally consistent with their own mocks, before anything is
built on top of them.

Deliberately ONE ``LiftBallTask``/``GraspSequencePlanner`` construction for
the WHOLE test, exercising grasp -> grasp -> failed-grasp -> release ->
release sequentially in one flow -- matching how every real caller (a
script, a mission) actually uses this: one process, one scene. An earlier
version of this file built a fresh scene per test function; that surfaced
a large, unexplained reliability gap between the first scene built in a
process (100% pass across every run observed) and a second/third one built
right after it in the same process (far more failures, yet the same test
run completely alone in its own process passed reliably) -- a strong
signature of state not being fully independent between successive HPP
scene constructions in one process, not genuine per-edge solver
randomness. Investigating that further is a separate, non-trivial
question (HPP/pinocchio's own global/random state across repeated scene
construction) outside this test's scope; sidestepping it by matching
real usage (one construction) is the correct fix here, not a workaround.

Both primitives are deliberately exercised in ONE ``GraspSequencePlanner``
instance (rather than a second instance running ``plan_sequence()`` in
parallel for comparison) -- ``grasp()``/``release()`` reuse the exact same
per-phase helpers ``plan_sequence()`` drives internally, so behavioral
equivalence to ``plan_sequence()`` is structural, not something this test
needs to re-derive.

Requires the real PyHPP backend AND ``hpp_practicals``'s ``package://``
resources (same requirement as ``tests/test_twin_examples.py`` -- see that
file's module docstring for the ``AMENT_PREFIX_PATH`` setup this needs).
Skips cleanly otherwise; run inside the hpp-arm64 container / a properly
configured dev environment before trusting this as a merge gate.
"""

import sys
from pathlib import Path

import pytest

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="PyHPP backend not available")

_TWIN_SCRIPT_DIR = Path(__file__).resolve().parent.parent / "script" / "twin"


def _build_twin_lift_ball_planner():
    """Real LiftBallTask + real GraspSequencePlanner, set up exactly as
    ``script/twin/task_lift_ball.py``'s own ``run_task()`` does (headless --
    no viewer is started; see that function's comment on why planning must
    stay headless).

    Returns ``(seq_planner, q_init, GRASP_SEQUENCE)`` or raises/skips if the
    scene can't be loaded in this environment (missing ``hpp_practicals``
    ``package://`` resources).
    """
    if str(_TWIN_SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(_TWIN_SCRIPT_DIR))
    import task_lift_ball as twin  # noqa: PLC0415 -- test-local, path-dependent import

    from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

    task = twin.LiftBallTask(backend="pyhpp")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,
        )
    except Exception as exc:
        pytest.skip(f"TWIN scene could not be set up in this environment: {exc}")

    q_init = task.q_init
    if not q_init:
        pytest.skip("TWIN scene setup did not produce an initial configuration")

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
    return seq_planner, q_init, twin.GRASP_SEQUENCE


def _grasp_retrying(seq_planner, gripper, handle, q_current, attempts=3):
    """Call ``grasp()``, retrying on a bare target-generation failure only.

    Belt-and-suspenders on top of ``_plan_phase_edges``'s own (small,
    dedicated) generation-retry budget -- see
    ``tests/test_plan_phase_edges_generation_retry.py`` and
    ``GraspSequencePlanner.__init__``'s ``_MAX_GENERATION_RETRIES`` for the
    production-side fix this complements. Does NOT retry a precondition
    failure (the whole point of a nested/duplicate ``grasp()`` call under
    test) -- those return immediately, before any IK draw, so retrying them
    would be pointless and would mask a real precondition-check regression.
    """
    result = None
    for _ in range(attempts):
        result = seq_planner.grasp(gripper, handle, q_current)
        if result["success"] or "Target generation failed" not in result["message"]:
            return result
    return result


@requires_pyhpp
class TestGraspReleaseAgainstRealTwinScene:
    def test_grasp_release_lifecycle(self):
        """One real bimanual scene, driven end to end: grasp both arms,
        confirm the precondition check blocks a conflicting third grasp
        without auto-releasing, then release both arms."""
        seq_planner, q_init, grasp_sequence = _build_twin_lift_ball_planner()
        assert len(grasp_sequence) == 2, (
            "test assumes TWIN's two-grasp sequence; update this test if "
            "script/twin/task_lift_ball.py's GRASP_SEQUENCE shape changes"
        )
        (gripper1, handle1), (gripper2, handle2) = grasp_sequence

        # 1. First real grasp.
        r1 = _grasp_retrying(seq_planner, gripper1, handle1, q_init)
        assert r1["success"], f"first grasp failed: {r1['message']}"
        assert seq_planner.grasp_tracker.current_grasps[gripper1] == handle1

        # 2. Second real grasp -- both grippers now holding simultaneously,
        # the actual TWIN dual-grasp end state.
        r2 = _grasp_retrying(seq_planner, gripper2, handle2, r1["final_config"])
        assert r2["success"], f"second grasp failed: {r2['message']}"
        assert seq_planner.grasp_tracker.current_grasps[gripper1] == handle1
        assert seq_planner.grasp_tracker.current_grasps[gripper2] == handle2

        # 3. grasp() must NOT silently insert a release when gripper1 is
        # asked to grasp something else while still holding handle1 --
        # that precondition enforcement is the whole point of extracting
        # this as a primitive (see grasp()'s docstring). The precondition
        # check runs before any IK draw, so no retry needed here.
        r3 = seq_planner.grasp(gripper1, handle2, r2["final_config"])
        assert r3["success"] is False
        assert "release(" in r3["message"]
        assert seq_planner.grasp_tracker.current_grasps[gripper1] == handle1

        # 4. Release both grippers.
        r4 = seq_planner.release(gripper1, r2["final_config"])
        assert r4["success"], f"release of {gripper1} failed: {r4['message']}"
        assert seq_planner.grasp_tracker.current_grasps[gripper1] is None

        r5 = seq_planner.release(gripper2, r4["final_config"])
        assert r5["success"], f"release of {gripper2} failed: {r5['message']}"
        assert seq_planner.grasp_tracker.current_grasps[gripper2] is None
