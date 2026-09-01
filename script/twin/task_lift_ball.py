#!/usr/bin/env python3
"""TWIN "Lift Ball" — bimanual grasp-sequence task (YAML-driven).

Two independent Franka Panda arms (`panda_left`, `panda_right`) — chosen
for their real 2-DOF articulated parallel gripper, unlike this task's
original UR5 + rigid-tool0 setup — simultaneously grasp a ball at two
handles, then lift it together. The agimus_spacelab port of TWIN/PerAct2's
"lift ball" bimanual benchmark task (see
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
import tempfile
from pathlib import Path

from agimus_spacelab.config.yaml_loader import YamlTaskLoader
from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner

TASK_NAME = "TWIN: Lift Ball (bimanual)"

_HERE = Path(__file__).parent
_YAML_PATH = _HERE / "config" / "twin_lift_ball_config.yaml"
_BALL_URDF_PATH = _HERE / "assets" / "pokeball_bimanual.urdf"
_BALL_SRDF_PATH = _HERE / "assets" / "pokeball_bimanual.srdf"
_GROUND_URDF_PATH = _HERE / "assets" / "ground_bimanual.urdf"
_PANDA_URDF_TEMPLATE_PATH = _HERE / "assets" / "panda_bimanual.urdf.template"
_PANDA_SRDF_PATH = _HERE / "assets" / "panda_bimanual.srdf"
_PANDA_MESH_DIR = _HERE / "assets" / "panda" / "meshes"

GRASP_GOALS: list[str] = [
    "panda_left/gripper grasps ball/handle",
    "panda_right/gripper grasps ball/handle2",
]

# Both grasps happen as one phase each; nothing releases — the "lift" is a
# separate plan_loop() call once both grasps are active (see run_task()).
GRASP_SEQUENCE: list[tuple[str, str | None]] = [
    ("panda_left/gripper", "ball/handle"),
    ("panda_right/gripper", "ball/handle2"),
]

# Finger joints are frozen (fixed at their initial YAML value throughout
# planning) — this task's grasp is a rigid TCP constraint, not simulated
# finger closing, so their only role is to render at a sensible width.
FREEZE_JOINT_SUBSTRINGS: list[str] = ["panda_finger_joint"]
COLLISION_EXCLUSIONS: list[tuple[str, str]] = []

# How far to lift the ball (meters) once both arms hold it.
LIFT_HEIGHT_M = 0.10


def _render_panda_urdf() -> str:
    """Substitute the vendored Panda URDF template's mesh-dir placeholder
    with an absolute path and write it to a temp file.

    Needed because pinocchio's URDF loader resolves non-`package://` mesh
    paths against the process's CWD, not the URDF file's own directory —
    so a plain relative path baked into the checked-in template wouldn't
    reliably find the meshes regardless of where this script is run from.
    """
    text = _PANDA_URDF_TEMPLATE_PATH.read_text()
    text = text.replace("{{PANDA_MESH_DIR}}", str(_PANDA_MESH_DIR))
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", prefix="panda_bimanual_", delete=False
    )
    tmp.write(text)
    tmp.close()
    return tmp.name


_loader = YamlTaskLoader(_YAML_PATH)
# Override with local, derived asset copies — see the config YAML's header
# comment for why (viser-compatible visual meshes; a second ball handle;
# a rendered-from-template Panda URDF with an absolute mesh path).
_loader.file_paths["objects"]["ball"]["urdf"] = str(_BALL_URDF_PATH)
_loader.file_paths["objects"]["ball"]["srdf"] = str(_BALL_SRDF_PATH)
_loader.file_paths["environment"]["ground"] = str(_GROUND_URDF_PATH)
_PANDA_RENDERED_URDF_PATH = _render_panda_urdf()
_loader.file_paths["robot"]["panda_left"]["urdf"] = _PANDA_RENDERED_URDF_PATH
_loader.file_paths["robot"]["panda_left"]["srdf"] = str(_PANDA_SRDF_PATH)
_loader.file_paths["robot"]["panda_right"]["urdf"] = _PANDA_RENDERED_URDF_PATH
_loader.file_paths["robot"]["panda_right"]["srdf"] = str(_PANDA_SRDF_PATH)


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


# rad; see _lift_the_ball()'s docstring for why this joint/magnitude.
_LIFT_JOINT_NAME = "panda_joint2"
_LIFT_JOINT_STEP = 0.10
_LIFT_MAX_STEPS = 8


def _lift_the_ball(
    task: LiftBallTask, seq_planner: GraspSequencePlanner, q_after_grasps: list[float]
) -> dict:
    """Raise the ball by up to LIFT_HEIGHT_M while both grasps stay active.

    With BOTH grippers rigidly grasping the ball, the ball's freeflyer pose
    is a *dependent* quantity of the arms' joint angles, not a free
    variable — perturbing the ball's own Z in a project_on_node() guess
    gets silently reprojected right back to zero net change (verified
    empirically on the UR5 version of this task: dz == 0.0 exactly). What
    actually moves the ball is perturbing the ARMS' joints and letting the
    grasp constraints carry the ball along with them.

    q_target for plan_loop() must already satisfy both grasp constraints —
    it's planned between, not projected by, plan_loop(). Build it by
    perturbing both arms' panda_joint2 (Panda's "shoulder lift" analogue —
    its axis, after joint1's Z-rotation and a -90 degree X offset, tilts
    the upper arm up/down, same role UR5's shoulder_lift_joint played) in
    a copy of the current (valid) configuration, then projecting that guess
    onto the current dual-grasp node's constraints via
    ConfigGenerator.project_on_node(). Small steps can fail to converge, so
    the full lift is built up incrementally rather than guessed in one jump.

    Unlike the UR5 version, a single fixed sign for the step isn't reliable
    here: which sign of panda_joint2 actually raises the ball (rather than
    lowering it) depends on exactly which randomized configuration the
    grasp search landed on for *this* run — elbow-up vs elbow-down variants
    of the "same" grasp pose respond oppositely (observed directly: one run
    lifted fine with a negative step, another produced dz=-0.03 with the
    same negative step). So each step tries both signs and keeps whichever
    both projects successfully and raises the ball; if neither does, the
    loop stops where it is rather than drifting downward.
    """
    model = task.robot.model()
    ball_joint_id = model.getJointId("ball/root_joint")
    idx_q = model.joints[ball_joint_id].idx_q  # start of [x,y,z,qx,qy,qz,qw] in q
    idx_left = model.joints[model.getJointId(f"panda_left/{_LIFT_JOINT_NAME}")].idx_q
    idx_right = model.joints[model.getJointId(f"panda_right/{_LIFT_JOINT_NAME}")].idx_q

    node_name = seq_planner.grasp_tracker.get_current_state_name()
    # config_gen lives on the planner (lazily constructed there), not on
    # `task` — ManipulationTask.setup(skip_graph=True) never populates its
    # own config_gen since GraspSequencePlanner owns phase-graph building.
    z_start = q_after_grasps[idx_q + 2]
    q_current = list(q_after_grasps)
    for _ in range(_LIFT_MAX_STEPS):
        z_now = q_current[idx_q + 2]
        if z_now - z_start >= LIFT_HEIGHT_M:
            break
        best_q, best_dz = None, 0.0
        for sign in (1, -1):
            q_guess = list(q_current)
            q_guess[idx_left] += sign * _LIFT_JOINT_STEP
            q_guess[idx_right] += sign * _LIFT_JOINT_STEP
            ok, q_projected = seq_planner.config_gen.project_on_node(
                node_name, q_guess, verbose=False
            )
            if not ok:
                continue
            dz = q_projected[idx_q + 2] - z_now
            if dz > best_dz:
                best_q, best_dz = q_projected, dz
        if best_q is None:
            break
        q_current = best_q

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
        gripper="panda_left/gripper",
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
