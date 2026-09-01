#!/usr/bin/env python3
"""TWIN "Lift Ball" — bimanual grasp-sequence task (YAML-driven).

Two independent UR5 arms (`ur5_left`, `ur5_right`) simultaneously grasp a
ball at two handles, then lift it together — the agimus_spacelab port of
TWIN/PerAct2's "lift ball" bimanual benchmark task (see
research-vault/agimus-spacelab/agimus-spacelab-next-ideas.md idea 2 and
research-vault/papers/twin-benchmark.md). See script/twin/README.md for
the one-time dev-environment setup (hpp_practicals package resolution).

Run:
    python script/twin/task_lift_ball.py --backend pyhpp
    python script/twin/task_lift_ball.py --show-joints
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from agimus_spacelab.config.yaml_loader import YamlTaskLoader
from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner
from agimus_spacelab.visualization import visualize_all_grippers, visualize_all_handles

TASK_NAME = "TWIN: Lift Ball (bimanual)"

_HERE = Path(__file__).parent
_YAML_PATH = _HERE / "config" / "twin_lift_ball_config.yaml"
_BALL_URDF_PATH = _HERE / "assets" / "pokeball_bimanual.urdf"
_BALL_SRDF_PATH = _HERE / "assets" / "pokeball_bimanual.srdf"
_UR5_VISER_URDF_PATH = _HERE / "assets" / "ur5_gripper_viser.urdf"

GRASP_GOALS: list[str] = [
    "ur5_left/gripper grasps ball/handle",
    "ur5_right/gripper grasps ball/handle2",
]

# Both grasps happen as one phase each; nothing releases — the "lift" is a
# separate plan_loop() call once both grasps are active (see run_task()).
GRASP_SEQUENCE: list[tuple[str, str | None]] = [
    ("ur5_left/gripper", "ball/handle"),
    ("ur5_right/gripper", "ball/handle2"),
]

FREEZE_JOINT_SUBSTRINGS: list[str] = []
COLLISION_EXCLUSIONS: list[tuple[str, str]] = []

# How far to lift the ball (meters) once both arms hold it.
LIFT_HEIGHT_M = 0.10

_loader = YamlTaskLoader(_YAML_PATH)
# Override with local, derived asset copies — see the config YAML's header
# comment for why (viser-compatible visual meshes; a second ball handle).
_loader.file_paths["objects"]["ball"]["urdf"] = str(_BALL_URDF_PATH)
_loader.file_paths["objects"]["ball"]["srdf"] = str(_BALL_SRDF_PATH)
_loader.file_paths["robot"]["ur5_left"]["urdf"] = str(_UR5_VISER_URDF_PATH)
_loader.file_paths["robot"]["ur5_right"]["urdf"] = str(_UR5_VISER_URDF_PATH)


class LiftBallTask(ManipulationTask):
    """Bimanual lift-ball grasp-sequence task, driven by YAML configuration."""

    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(self, backend: str = "pyhpp", viewer_type: str = "auto"):
        super().__init__(
            task_name=TASK_NAME,
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
            viewer_type=viewer_type,
        )
        self.task_config = _loader.task_config.with_grasp_goals(GRASP_GOALS)
        self.use_factory = True

    def build_initial_config(self) -> list[float]:
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


_LIFT_JOINT_STEP = -0.04  # rad; see docstring below for why this magnitude
_LIFT_MAX_STEPS = 8


def _lift_the_ball(
    task: LiftBallTask, seq_planner: GraspSequencePlanner, q_after_grasps: list[float]
) -> dict:
    """Raise the ball by up to LIFT_HEIGHT_M while both grasps stay active.

    With BOTH grippers rigidly grasping the ball, the ball's freeflyer pose
    is a *dependent* quantity of the arms' joint angles, not a free
    variable — perturbing the ball's own Z in a project_on_node() guess
    gets silently reprojected right back to zero net change (verified
    empirically: dz == 0.0 exactly). What actually moves the ball is
    perturbing the ARMS' joints and letting the grasp constraints carry the
    ball along with them.

    q_target for plan_loop() must already satisfy both grasp constraints —
    it's planned between, not projected by, plan_loop(). Build it by
    perturbing both arms' shoulder_lift_joint (moved together, matching
    their mirrored mounting) in a copy of the current (valid) configuration,
    then projecting that guess onto the current dual-grasp node's
    constraints via ConfigGenerator.project_on_node(). Empirically this
    projector only accepts sizeable, same-signed steps here — smaller steps
    (+/-0.01 to +/-0.03) fail to converge at all, while -0.04 reliably lands
    on a valid nearby configuration and lifts (dz=+0.017m per step) — so the
    full lift is built up incrementally rather than guessed in one jump.
    """
    model = task.robot.model()
    ball_joint_id = model.getJointId("ball/root_joint")
    idx_q = model.joints[ball_joint_id].idx_q  # start of [x,y,z,qx,qy,qz,qw] in q
    idx_left = model.joints[model.getJointId("ur5_left/shoulder_lift_joint")].idx_q
    idx_right = model.joints[model.getJointId("ur5_right/shoulder_lift_joint")].idx_q

    node_name = seq_planner.grasp_tracker.get_current_state_name()
    # config_gen lives on the planner (lazily constructed there), not on
    # `task` — ManipulationTask.setup(skip_graph=True) never populates its
    # own config_gen since GraspSequencePlanner owns phase-graph building.
    z_start = q_after_grasps[idx_q + 2]
    q_current = list(q_after_grasps)
    for _ in range(_LIFT_MAX_STEPS):
        if q_current[idx_q + 2] - z_start >= LIFT_HEIGHT_M:
            break
        q_guess = list(q_current)
        q_guess[idx_left] += _LIFT_JOINT_STEP
        q_guess[idx_right] += _LIFT_JOINT_STEP
        ok, q_projected = seq_planner.config_gen.project_on_node(
            node_name, q_guess, verbose=False
        )
        if not ok:
            break
        q_current = q_projected

    total_dz = q_current[idx_q + 2] - z_start
    if total_dz <= 0.0:
        return {
            "success": False,
            "message": (
                f"project_on_node('{node_name}') never produced upward "
                f"motion (dz={total_dz:.4f})"
            ),
        }

    return seq_planner.plan_loop(
        gripper="ur5_left/gripper",
        q_current=q_after_grasps,
        q_target=q_current,
        verbose=True,
    )


def _replay_all(task: LiftBallTask, seq_planner: GraspSequencePlanner) -> None:
    """Play every planned path in order, non-interactively.

    Called after the lift step, so `seq_planner.phase_results` already
    holds all three phases: both grasps plus the loop-lift motion
    (`plan_loop` appends its own phase_result to the same list).
    """
    for phase in seq_planner.phase_results:
        for path_obj in phase.get("paths", []):
            if path_obj is None:
                continue
            try:
                if isinstance(path_obj, int):
                    task.planner.play_path(path_obj)
                else:
                    task.planner.play_path_vector(path_obj)
            except Exception as exc:
                print(f"  ⚠ replay failed: {exc}")


def run_task(backend: str = "pyhpp", show_viewer: bool = False, viewer_type: str = "auto") -> bool:
    """Run the full task lifecycle. Returns True on planning success."""
    task = LiftBallTask(backend=backend, viewer_type=viewer_type)

    print("\n" + "=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  Backend   : {backend}")
    print(f"  Goals     : {GRASP_GOALS}")
    print(f"  Sequence  : {GRASP_SEQUENCE}")
    print("=" * 70 + "\n")

    print("Setting up task...")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,
        )
    except Exception as exc:
        import traceback

        print(f"✗ Setup failed: {exc}")
        traceback.print_exc()
        return False
    print("✓ Task set up")

    q_init = task.q_init
    if not q_init:
        print("✗ Failed to get initial configuration")
        return False
    print(f"\n✓ Initial config: {len(q_init)} DOF")

    # Viewer is started AFTER all planning below, not here: the viser
    # server's background thread segfaulted the native RRT/collision-check
    # step deterministically when live at solve time (reproduced twice at
    # the same edge) — planning headless first, then visualizing the
    # already-computed paths, sidesteps that concurrency issue entirely.

    print("\nCreating GraspSequencePlanner...")
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

    print(f"\nPlanning grasp sequence: {GRASP_SEQUENCE}")
    try:
        result = seq_planner.plan_sequence(
            grasp_sequence=GRASP_SEQUENCE, q_init=q_init, verbose=True
        )
    except Exception as exc:
        import traceback

        print(f"\n✗ Planning error: {exc}")
        traceback.print_exc()
        return False

    if not result["success"]:
        print("\n" + "=" * 70)
        print("✗ GRASP SEQUENCE FAILED")
        print("=" * 70)
        print(f"  Reason: {result.get('error', 'Unknown')}")
        return False

    print("\n" + "=" * 70)
    print("✓ BOTH GRASPS SUCCEEDED")
    print("=" * 70)
    print(seq_planner.get_phase_summary())

    print(f"\nLifting the ball {LIFT_HEIGHT_M}m...")
    lift_result = _lift_the_ball(task, seq_planner, result["final_config"])
    if lift_result["success"]:
        print("✓ LIFT SUCCEEDED")
    else:
        print(f"✗ Lift failed: {lift_result.get('message', 'Unknown')}")
        print("  (grasp sequence itself still succeeded)")

    if show_viewer:
        print("\nStarting viewer (all planning is done — nothing solves live)...")
        try:
            task.planner.visualize(q_init)
        except Exception as exc:
            print(f"⚠ Visualization unavailable: {exc}")
            return True
        print("Displaying gripper/handle frames...")
        visualize_all_grippers(
            task.planner.viewer,
            [gripper for gripper, _ in GRASP_SEQUENCE],
            show_approach=False,
        )
        visualize_all_handles(
            task.planner.viewer,
            [handle for _, handle in GRASP_SEQUENCE],
            show_approach=False,
        )
        print("Replaying full sequence in the viewer...")
        _replay_all(task, seq_planner)
        print("✓ Replay done. Viewer stays up — Ctrl+C to exit.")
        import time

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print()

    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=TASK_NAME)
    parser.add_argument("--backend", default="pyhpp", choices=["pyhpp", "corba"])
    parser.add_argument("--show-joints", action="store_true")
    parser.add_argument(
        "--viewer-type",
        default=None,
        choices=["viser", "gepetto", "auto"],
        help="Show the scene in a browser (viser, default) or gepetto-viewer "
        "and replay the planned motion, then keep the viewer alive. Omit to "
        "run headless (no viewer).",
    )
    args = parser.parse_args()

    if args.show_joints:
        task = LiftBallTask(backend=args.backend)
        task.setup(skip_graph=True)
        from agimus_spacelab.visualization import print_joint_info

        print_joint_info(task.robot)
        return 0

    success = run_task(
        backend=args.backend,
        show_viewer=args.viewer_type is not None,
        viewer_type=args.viewer_type or "auto",
    )
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
