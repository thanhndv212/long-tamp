#!/usr/bin/env python3
"""Load the full scene — 2 UR10+Robotiq arms, ground, table + 4 legs — in
viser, for visual layout confirmation before writing the YAML task config.

Local prototype only — see README.md. Run inside the hpp-agimus-arm64
container (after sourcing config.sh); uses the .container.urdf variant of
the combined arm (mesh paths baked to that environment).

Layout (not yet task-tuned, just "does everything fit and reach"):
  - ur10_left at world origin, facing +X.
  - ur10_right at (1.5, 0, 0), rotated 180 deg about Z, facing back
    toward ur10_left — same "face each other" pattern as
    script/twin/config/twin_lift_ball_config.yaml's two Pandas, scaled up
    for UR10's ~1.3m reach (vs Panda's ~0.855m).
  - table at (0.75, 0.3, 0.5) — workbench height, off to one side.
  - leg1..leg4 upright in a row at (0.4, 0/0.15/0.3/0.45, 0.13125) —
    within ur10_left's reach, standing on their own local +Z (matching
    the side-grasp handle authored in generate_urdf_srdf.py).
"""

from pathlib import Path

import numpy as np
import pinocchio as pin

from long_tamp.backends.pyhpp import PyHPPBackend

HERE = Path(__file__).parent
GEN = HERE / "generated"

ARM_URDF = GEN / "ur10_robotiq.container.urdf"
ARM_SRDF = GEN / "ur10_robotiq.srdf"
GROUND_URDF = GEN / "ground.urdf"

OBJECTS = {
    "table": (GEN / "table.urdf", GEN / "table.srdf", (0.75, 0.3, 0.5), (0, 0, 0, 1)),
    "leg1": (GEN / "leg1.urdf", GEN / "leg1.srdf", (0.4, 0.00, 0.13125), (0, 0, 0, 1)),
    "leg2": (GEN / "leg2.urdf", GEN / "leg2.srdf", (0.4, 0.15, 0.13125), (0, 0, 0, 1)),
    "leg3": (GEN / "leg3.urdf", GEN / "leg3.srdf", (0.4, 0.30, 0.13125), (0, 0, 0, 1)),
    "leg4": (GEN / "leg4.urdf", GEN / "leg4.srdf", (0.4, 0.45, 0.13125), (0, 0, 0, 1)),
}

RIGHT_ARM_POSE = pin.SE3(
    pin.utils.rpyToMatrix(0, 0, np.pi), np.array([1.5, 0.0, 0.0])
)


def set_freeflyer(q: np.ndarray, model: pin.Model, joint_name: str, xyz, xyzw) -> None:
    jid = model.getJointId(joint_name)
    idx = model.joints[jid].idx_q
    q[idx : idx + 7] = [*xyz, *xyzw]


def main() -> None:
    backend = PyHPPBackend(viewer_type="viser")

    backend.load_robot("ur10_left", str(ARM_URDF), str(ARM_SRDF), root_joint_type="anchor")
    backend.load_robot(
        "ur10_right",
        str(ARM_URDF),
        str(ARM_SRDF),
        root_joint_type="anchor",
        pose=RIGHT_ARM_POSE,
    )
    backend.load_environment("ground", str(GROUND_URDF))
    for name, (urdf_path, srdf_path, _, _) in OBJECTS.items():
        backend.load_object(name, str(urdf_path), str(srdf_path))

    model = backend.device.model()
    q = np.array(backend.device.currentConfiguration())
    for name, (_, _, xyz, xyzw) in OBJECTS.items():
        set_freeflyer(q, model, f"{name}/root_joint", xyz, xyzw)

    backend.setup_viewer(viewer_type="viser")
    backend.visualize(q)
    print("nq:", len(q))
    print("Viewer running — open the URL above in your browser. Ctrl-C to stop.")

    import time

    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
