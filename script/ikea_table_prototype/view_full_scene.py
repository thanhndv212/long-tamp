#!/usr/bin/env python3
"""Load the full scene — 2 UR10+Robotiq arms, ground, table + 4 legs — in
viser, for visual layout confirmation before writing the YAML task config.

Local prototype only — see README.md. Run inside the hpp-agimus-arm64
container (after sourcing config.sh); uses the .container.urdf variant of
the combined arm (mesh paths baked to that environment).

Layout (not yet task-tuned, just "does everything fit and reach"):
  - ur10_left at world origin (XY), facing +X. Its 0.3m pedestal is now
    part of ur10_robotiq.urdf itself (see merge_ur10_robotiq.py) — a
    fixed joint onto UR10's own root, not a separately-positioned
    environment object, so arm and pedestal can't visually gap apart.
  - ur10_right at (1.5, 0, 0), rotated 180 deg about Z, facing back
    toward ur10_left — same "face each other" pattern as
    twin_lift_ball_config.yaml's two Pandas, scaled up for UR10's
    ~1.3m reach (vs Panda's ~0.855m).
  - table at (0.75, 0.3, 0.5) — workbench height, off to one side.
  - leg1..leg4 upright in a row at (0.4, 0/0.15/0.3/0.45, 0.13125) —
    standing on the ground, within ur10_left's reach.
  - Each arm's 6 joints get a distinct, non-degenerate "ready"-ish pose
    (elbow bent) instead of the raw all-zero configuration, which for
    UR10 is a fully-extended pose that's a poor default to view or plan
    from.

v2 fixes, per live feedback on v1 (floor-mounted arms, apparently-
unpositioned objects, both arms in the same degenerate zero pose): added
a pedestal (initially as a separate object at 0.5m; feedback wanted it
lower and structurally attached, so v3 folds it into the arm URDF at
0.3m — see merge_ur10_robotiq.py), gave each arm a distinct explicit
joint config, and prints each object's *actual* FK position at the end
so placement is confirmed empirically rather than assumed (all 5 matched
their targets exactly both times — the "unpositioned" read was most
likely the floor-mounted, same-pose layout's camera framing, not a
placement bug).
"""

from pathlib import Path

import numpy as np
import pinocchio as pin

from long_tamp.backends.pyhpp import PyHPPBackend

HERE = Path(__file__).parent
GEN = HERE / "generated"

# ur10_right uses a *separate* URDF (world offset baked in as an outer
# fixed link, see merge_ur10_robotiq.py) rather than the same file loaded
# twice with a pose= offset — PyHPPBackend.load_robot(pose=...) hits a
# real pyhpp_viser bug where a robot's *static* (fixed-to-universe)
# geometry doesn't reflect that post-load repositioning, only geometry
# driven by a real moving joint does. Confirmed live: both arms' pedestals
# were caching to the exact same [0,0,0.15] position before this fix.
ARM_LEFT_URDF = GEN / "ur10_robotiq.container.urdf"
ARM_RIGHT_URDF = GEN / "ur10_robotiq_right.container.urdf"
ARM_SRDF = GEN / "ur10_robotiq.srdf"  # shared — offset is URDF-only
GROUND_URDF = GEN / "ground.urdf"

OBJECTS = {
    "table": (GEN / "table.urdf", GEN / "table.srdf", (0.75, 0.3, 0.5), (0, 0, 0, 1)),
    "leg1": (GEN / "leg1.urdf", GEN / "leg1.srdf", (0.4, 0.00, 0.13125), (0, 0, 0, 1)),
    "leg2": (GEN / "leg2.urdf", GEN / "leg2.srdf", (0.4, 0.15, 0.13125), (0, 0, 0, 1)),
    "leg3": (GEN / "leg3.urdf", GEN / "leg3.srdf", (0.4, 0.30, 0.13125), (0, 0, 0, 1)),
    "leg4": (GEN / "leg4.urdf", GEN / "leg4.srdf", (0.4, 0.45, 0.13125), (0, 0, 0, 1)),
}

UR10_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Two distinct, non-degenerate ("elbow bent", not the all-zero fully
# extended pose) UR10 configs — not task-specific yet, just visually
# sane and clearly different from each other.
LEFT_ARM_Q = [0.0, -1.2, 1.6, -1.97, -1.57, 0.0]
RIGHT_ARM_Q = [0.4, -1.0, 1.3, -1.8, -1.3, 0.5]


def set_freeflyer(q: np.ndarray, model: pin.Model, joint_name: str, xyz, xyzw) -> None:
    jid = model.getJointId(joint_name)
    idx = model.joints[jid].idx_q
    q[idx : idx + 7] = [*xyz, *xyzw]


def set_revolute(q: np.ndarray, model: pin.Model, joint_name: str, value: float) -> None:
    jid = model.getJointId(joint_name)
    idx = model.joints[jid].idx_q
    q[idx] = value


def main() -> None:
    backend = PyHPPBackend(viewer_type="viser")

    backend.load_robot(
        "ur10_left", str(ARM_LEFT_URDF), str(ARM_SRDF), root_joint_type="anchor"
    )
    backend.load_robot(
        "ur10_right", str(ARM_RIGHT_URDF), str(ARM_SRDF), root_joint_type="anchor"
    )
    backend.load_environment("ground", str(GROUND_URDF))
    for name, (urdf_path, srdf_path, _, _) in OBJECTS.items():
        backend.load_object(name, str(urdf_path), str(srdf_path))

    model = backend.device.model()
    q = np.array(backend.device.currentConfiguration())

    for name, (_, _, xyz, xyzw) in OBJECTS.items():
        set_freeflyer(q, model, f"{name}/root_joint", xyz, xyzw)
    for jname, val in zip(UR10_JOINTS, LEFT_ARM_Q):
        set_revolute(q, model, f"ur10_left/{jname}", val)
    for jname, val in zip(UR10_JOINTS, RIGHT_ARM_Q):
        set_revolute(q, model, f"ur10_right/{jname}", val)

    # Verify empirically, not by assumption: FK every object's root and
    # print its actual world position, so a placement bug shows up here
    # rather than only being caught by eyeballing the viewer.
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    print("--- object placement check (FK, not assumed) ---")
    for name, (_, _, xyz, _) in OBJECTS.items():
        jid = model.getJointId(f"{name}/root_joint")
        actual = data.oMi[jid].translation
        print(f"{name:8s} target={xyz}  actual={actual}")

    backend.setup_viewer(viewer_type="viser")
    backend.visualize(q)
    print("nq:", len(q))
    print("Viewer running — open the URL above in your browser. Ctrl-C to stop.")

    import time

    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
