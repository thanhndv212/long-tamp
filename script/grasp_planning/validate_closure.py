#!/usr/bin/env python3
"""Check planned finger closures against the real Robotiq 2F-85 meshes.

The grasp planner closes the fingers on an object's collision primitives
with a box model of the hand. This script checks its answer against the
actual gripper geometry, with pinocchio forward kinematics and coal
distance queries:

- the hand is placed so its ``<gripper>`` frame sits on the handle;
- the fingers are set to the planned closure;
- each pad's distance to the object should be ~0 (touching: negative is
  the planned squeeze, positive a gap), and no other gripper link should
  collide with the object.

    python3 validate_closure.py                       # the screw-assembly handles
    python3 validate_closure.py --plan driver --top 5 # also the best planned grasps

Needs pinocchio and coal (both ship with the [hpp] extra).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ARM_URDF = ROOT / "script/ikea_table_prototype/generated/ur10_robotiq.urdf"
TASK_YAML = ROOT / "script/screw_assembly/config/screw_assembly_config.yaml"
# The Robotiq 2F-85 from PickNik's ros2_robotiq_gripper (#71).
GRIPPER_LINKS = (
    "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_knuckle_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_right_finger_tip_link",
)
# Pad frames on the fingertips' inner faces (build_assets.py robotiq-picknik),
# and which way is into the finger from each (the fingertip frame's X).
PADS = {"robotiq_85_left_finger_pad": 1.0, "robotiq_85_right_finger_pad": -1.0}
PAD_THICKNESS = 0.00635
# The pad as a box in its frame: thickness along X (the face normal), 22 mm
# wide (Y), 38 mm long (Z); the fingertip mesh itself is not a pad.
PAD_BOX = (PAD_THICKNESS, 0.022, 0.038)


def coal_shape(prim):
    import coal

    from long_tamp.grasping import Box, Cylinder, Sphere

    if isinstance(prim, Box):
        return coal.Box(*prim.size)
    if isinstance(prim, Cylinder):
        return coal.Cylinder(prim.radius, prim.length)
    if isinstance(prim, Sphere):
        return coal.Sphere(prim.radius)
    raise TypeError(prim)


class HandChecker:
    """The 2F-85 alone (arm at its neutral pose), for distance queries."""

    def __init__(self, urdf: Path = ARM_URDF):
        import pinocchio as pin

        from long_tamp.backends._urdf_paths import resolve_mesh_paths

        path = resolve_mesh_paths(str(urdf))
        self.pin = pin
        self.model = pin.buildModelFromUrdf(path)
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(self.model, path, pin.COLLISION)
        # the <gripper> frame: gripper_tcp turned -90 deg about Y (see the SRDF)
        self.tcp = self.model.getFrameId("gripper_tcp")
        self.tcp_to_gripper = pin.SE3(
            pin.Quaternion(0.7071068, 0.0, -0.7071068, 0.0).normalized().matrix(),
            np.zeros(3),
        )

    def _configure(self, joint_values: dict[str, float]):
        q = self.pin.neutral(self.model)
        for joint, value in joint_values.items():
            jid = self.model.getJointId(joint.split("/", 1)[-1])
            q[self.model.idx_qs[jid]] = value
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return q

    def check(
        self,
        handle_pose: np.ndarray,
        primitives,
        commanded: dict[str, float],
        contact: dict[str, float],
    ):
        """Pad distances with the fingers at the ``commanded`` closure, and
        the gripper links colliding with the object at the ``contact``
        closure (the squeeze past contact is force, not a reachable pose)."""
        import coal

        pin = self.pin
        q = self._configure(commanded)
        o_gripper = self.data.oMf[self.tcp] * self.tcp_to_gripper
        # object placed so that its handle frame coincides with the gripper frame
        o_obj = o_gripper * pin.SE3(handle_pose[:3, :3], handle_pose[:3, 3]).inverse()
        objs = [
            (coal_shape(p), o_obj * pin.SE3(p.pose[:3, :3], p.pose[:3, 3]))
            for p in primitives
        ]

        def distance(shape, placement):
            best = np.inf
            for oshape, opl in objs:
                req, res = coal.DistanceRequest(), coal.DistanceResult()
                d = coal.distance(
                    shape,
                    coal.Transform3s(placement.rotation, placement.translation),
                    oshape,
                    coal.Transform3s(opl.rotation, opl.translation),
                    req,
                    res,
                )
                best = min(best, d)
            return best

        pads = {}
        for pad, into_finger in PADS.items():
            face = self.data.oMf[self.model.getFrameId(pad)]
            # the box behind the face, inside the finger
            pl = face * pin.SE3(
                np.eye(3), np.array([into_finger * PAD_THICKNESS / 2, 0.0, 0.0])
            )
            pads[pad] = distance(coal.Box(*PAD_BOX), pl)
        q = self._configure(contact)
        gdata = self.geom.createData()
        pin.updateGeometryPlacements(self.model, self.data, self.geom, gdata, q)
        colliding = []
        for k, go in enumerate(self.geom.geometryObjects):
            link = self.model.frames[go.parentFrame].name
            if link in GRIPPER_LINKS and distance(go.geometry, gdata.oMg[k]) < 0.0:
                colliding.append(link)
        return pads, colliding


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--plan", metavar="OBJECT", help="also check planned grasps of this object"
    )
    ap.add_argument("--top", type=int, default=5)
    args = ap.parse_args()

    from long_tamp.grasping import ROBOTIQ_2F85, FingerClosureTable, GraspPlanner

    table = FingerClosureTable.from_task_yaml(
        TASK_YAML,
        {"ur10_left/gripper": ROBOTIQ_2F85, "ur10_right/gripper": ROBOTIQ_2F85},
    )
    hand = HandChecker()
    ok = True
    print(
        f"{'grasp':44s} {'q':>6s} {'width':>7s} {'pad L':>8s} {'pad R':>8s}  colliding"
    )
    rows = []
    seen = set()
    for (gripper, handle), ev in sorted(table._cache.items()):
        obj = handle.split("/")[0]
        if handle.split("/")[1] in seen and obj.startswith("part"):
            continue  # every part is the same
        seen.add(handle.split("/")[1])
        o = table.objects[obj]
        rows.append((f"{gripper} > {handle}", o.handle_pose(handle), o.primitives, ev))
    if args.plan:
        o = table.objects[args.plan]
        planner = GraspPlanner(ROBOTIQ_2F85)
        for i, c in enumerate(planner.plan(o.primitives, max_candidates=args.top)):
            rows.append(
                (
                    f"{args.plan} planned #{i + 1} ({c.source})"[:44],
                    c.handle_pose,
                    o.primitives,
                    c.evaluation,
                )
            )
    for label, handle_pose, prims, ev in rows:
        q_contact = ROBOTIQ_2F85.q_for_width(ev.contact_width)
        pads, colliding = hand.check(
            handle_pose,
            prims,
            ROBOTIQ_2F85.joint_values(ev.q),
            ROBOTIQ_2F85.joint_values(q_contact),
        )
        dl, dr = (pads[p] * 1000 for p in PADS)
        # touching within 3 mm (squeeze is 2 mm; the pad arc isn't the box model)
        good = not colliding and min(abs(dl), abs(dr)) < 3.0 and max(dl, dr) < 3.0
        ok &= good
        print(
            f"{label:44s} {ev.q:6.3f} {ev.width * 1000:5.1f}mm {dl:6.2f}mm {dr:6.2f}mm  "
            f"{', '.join(colliding) or '-'}{'' if good else '   <-- CHECK'}"
        )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
