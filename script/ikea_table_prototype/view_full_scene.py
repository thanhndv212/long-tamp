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
  - workbench.urdf: a static support surface, top at z=0.4, centered at
    (0.75, 0) — table + legs rest on top of it (own half-thickness above
    the surface) rather than floating at an unexplained height. The
    table used to sit at z=0.5 with nothing under it at all; legs stood
    correctly on the ground floor, but a floating tabletop above
    floor-standing legs was an incoherent picture regardless.
  - table and leg1..leg4 (upright) all centered at X=0.75 — the true
    midpoint between the two arm bases (x=0 and x=1.5) — legs spread
    along Y at -0.225/-0.075/0.075/0.225.
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
WORKBENCH_URDF = GEN / "workbench.urdf"

# Workbench top surface is at z=0.4 (see workbench.urdf) — table/legs sit
# on top of it (center = top + own half-thickness), not floating at an
# unexplained height. Previously the table sat at z=0.5 with nothing
# under it at all; legs stood correctly on the ground floor (z=0.13125,
# their own half-height) but a floating tabletop above floor-standing
# legs was an incoherent picture regardless — everything now rests on one
# shared surface.
WORKBENCH_TOP_Z = 0.4
TABLE_HALF_THICKNESS = 0.02

# Legs now lie flat on the workbench (long axis horizontal) instead of
# standing upright, and moved into their own row clear of the table's
# footprint (half-extents 0.32 x 0.12 around (0.75, 0), so y in
# [-0.12, 0.12]) rather than overlapping it. A 90deg rotation about local
# Y maps the leg's long local-Z axis to world +X (quaternion for that:
# axis (0,1,0), angle pi/2 -> (x,y,z,w) = (0, sin(pi/4), 0, cos(pi/4))),
# so the "thin" cross-section (originally local X/Y, 0.015m half-extent)
# is what's now vertical/lateral — hence the small z-offset above the
# surface (leg *width*, not its length, is what's resting on it) and the
# 4-row spacing along Y only needing to clear that same thin width.
LEG_LIE_FLAT_QUAT = (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)
LEG_HALF_WIDTH = 0.015
LEG_Y = [-0.20, -0.28, -0.36, -0.44]

OBJECTS = {
    # Centered at X=0.75 — the true midpoint between ur10_left (x=0) and
    # ur10_right (x=1.5) — and Y=0.
    "table": (
        GEN / "table.urdf", GEN / "table.srdf",
        (0.75, 0.0, WORKBENCH_TOP_Z + TABLE_HALF_THICKNESS), (0, 0, 0, 1),
    ),
    "leg1": (
        GEN / "leg1.urdf", GEN / "leg1.srdf",
        (0.75, LEG_Y[0], WORKBENCH_TOP_Z + LEG_HALF_WIDTH), LEG_LIE_FLAT_QUAT,
    ),
    "leg2": (
        GEN / "leg2.urdf", GEN / "leg2.srdf",
        (0.75, LEG_Y[1], WORKBENCH_TOP_Z + LEG_HALF_WIDTH), LEG_LIE_FLAT_QUAT,
    ),
    "leg3": (
        GEN / "leg3.urdf", GEN / "leg3.srdf",
        (0.75, LEG_Y[2], WORKBENCH_TOP_Z + LEG_HALF_WIDTH), LEG_LIE_FLAT_QUAT,
    ),
    "leg4": (
        GEN / "leg4.urdf", GEN / "leg4.srdf",
        (0.75, LEG_Y[3], WORKBENCH_TOP_Z + LEG_HALF_WIDTH), LEG_LIE_FLAT_QUAT,
    ),
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
    backend.load_environment("workbench", str(WORKBENCH_URDF))
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
