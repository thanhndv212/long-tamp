#!/usr/bin/env python3
"""Generate the screw-assembly cell: URDF/SRDF for every part plus the task YAML.

A generic, parametric long-horizon assembly scene -- two UR10 + Robotiq 2F-85
arms, a fixed jig, a charging dock, a cordless drill/driver, and N plate
parts, each fastened by two screws. The drill's look is a real scan (YCB
035_power_drill, CC BY 4.0, see assets/ycb_035_power_drill/); its collision
geometry, and everything else except the arms, is box/cylinder primitives
authored here. See README.md for the mission it is built for.

Usage (host or container):

    python3 build_scene.py            # 4 parts (default)
    python3 build_scene.py --parts 2  # smaller scene for debugging

The drill's mesh is the only absolute path this writes: like the arms' URDFs
it points into the hpp-agimus-arm64 container's checkout (REPO_ROOT);
``--repo-root`` points it elsewhere.

Writes generated/*.urdf|srdf and config/screw_assembly_config.yaml.

FRAME CONVENTION (every handle and every object-borne gripper)
--------------------------------------------------------------
The Robotiq's <gripper> frame points its +X along the fingers' approach
direction (see ../ikea_table_prototype/generated/ur10_robotiq.srdf). Every
handle and gripper here follows the same rule: local +X points the way the
grasping side *travels* to engage, and ``approaching_direction="1 0 0"``.
For a top-down engagement that is world -Z, i.e. the frame is rotated +90 deg
about Y (``DOWN`` below). Closing/lateral axis is local Y (unchanged by
that rotation), so a Robotiq grasping ``h_grasp`` closes across a part's
0.06 m width. This is the convention that verified empirically on the IKEA
prototype's top-down leg grasp; don't re-derive it from rotation algebra.

``approaching_direction`` is the direction the *gripper* moves relative to
the handle. For a moving gripper engaging a resting object (Robotiq on a
part, driver tip into a hole) that is the frame's +X. For a FIXED gripper
the object is lowered into (jig clamp, charging dock) the gripper moves UP
relative to the object, so those handles use -X: with +X the pregrasp put
the part 3 cm *inside* the jig (found live -- every clamp pregrasp
collided the carrying gripper with the jig).

THE DRILL
---------
A pistol-grip drill held the way a person holds one: the Robotiq closes
across the handle, approaching from behind it along the bit axis. So the
grasp frame and the bit share their +X, and a top-down screw (tip frame +X =
world down) keeps the gripper top-down while it screws.
Drill frame: +X along the bit (forward), +Z from the battery up to the motor
body, origin at the centre of the battery's base. It stands on that base on
the dock, bit horizontal and pointing away from ur10_right, so the pick is a
horizontal reach from behind the handle. The tip gripper is turned 180 deg
about the bit so that, screwing, the handle points toward ur10_right's side
(+X world), away from ur10_left's hand on the part's tab.

CONTACT GAPS
------------
Every seated/resting pair keeps a 3 mm gap (GAP): flush surfaces read as
interpenetration to the collision checker (found live on the IKEA scene).
"""

from __future__ import annotations

import argparse
from pathlib import Path

HERE = Path(__file__).parent
GEN = HERE / "generated"
CFG = HERE / "config"

# The two arms are reused as-is from the IKEA prototype: same UR10 +
# Robotiq merge, pedestal and self-collision exclusions, all verified live
# there. Left arm at the world origin, right arm 1.5 m along X facing it.
ARMS_DIR = "../../ikea_table_prototype/generated"

GAP = 0.003
DOWN = "0 0.7071068 0 0.7071068"  # xyzw: +90 deg about Y, local +X -> world -Z

# Workbench: 1.0 x 1.0 top at z=0.40, centered between the arm bases.
# (Tried 0.30 to give the upper arms more room: grasps got 3x slower and
# block replans went up, so 0.40 stays.)
BENCH_TOP = 0.40
BENCH_X = 0.75

# Parts: plates lying flat, long axis along world X.
PART = (0.36, 0.06, 0.02)
# Each arm works its own end of the part: ur10_left grips the tab near the
# -X end (its side), the driver screws both holes on the +X half (ur10_right's
# side). With the tab at the center and holes at +/-0.12 m the two Robotiqs
# crowded each other at hole 2 -- found live: block A's lookahead rejected
# every clamp candidate on ur10_left/forearm vs ur10_right's fingers/forearm.
TAB_X = -0.13
HOLE_XS = (0.05, 0.15)
# Grip tab: a block standing on the plate's center that the Robotiq holds.
# Gripping the 20 mm plate itself put the fingertips into the jig once the
# part was clamped -- live, every block-A lookahead candidate collided
# ur10_left's fingers with fixtures/base_link, first at a mid-thickness
# grasp, then still at +8 mm. The tab lifts the grasp point clear of it.
TAB = (0.04, 0.03, 0.06)
GRASP_Z = PART[2] / 2 + TAB[2] * 0.6  # tab, upper part
PART_PITCH = 0.10  # Y spacing between parts (staging row and jig slots)
STAGING_X = 0.44

# Jig (on the fixtures robot): a slab the parts are clamped onto.
JIG_X = 0.84
JIG_H = 0.04
JIG_TOP = BENCH_TOP + GAP + JIG_H  # jig floats GAP above the bench

# Charging dock (fixtures robot) the drill stands on, and the drill.
RACK = (1.08, 0.36)  # xy
RACK_SIZE = (0.14, 0.09, 0.03)
RACK_TOP = BENCH_TOP + GAP + RACK_SIZE[2]

# Drill/driver (YCB 035_power_drill scan). Collision proxies, drill frame
# (see THE DRILL), as (name, kind, size, center): measured from the scan's
# vertex slices, each padded a few mm.
REPO_ROOT = "/home/thanhndv212/devel/hpp/src/long_tamp"  # in the container
DRILL_MESH = "script/screw_assembly/assets/ycb_035_power_drill/textured.obj"
# Scan frame -> drill frame: scan +X is backward, +Y up the handle, +Z the
# side; the battery base's centre is at scan (-0.030, -0.0832, 0.0255).
DRILL_MESH_XYZ = "-0.030 -0.0255 0.0832"
DRILL_MESH_RPY = "1.5707963 0 3.1415927"
DRILL_PROXIES = [
    ("battery", "box", (0.110, 0.060, 0.034), (0.0, 0.0, 0.017)),
    ("handle", "box", (0.054, 0.038, 0.097), (-0.022, 0.0, 0.0815)),
    ("trigger", "box", (0.040, 0.038, 0.040), (0.018, 0.0, 0.108)),
    ("body", "box", (0.170, 0.056, 0.062), (0.007, 0.0, 0.159)),
    ("chuck", "cylinder", (0.020, 0.025), (0.1045, 0.0, 0.160)),
    ("bit", "cylinder", (0.0035, 0.050), (0.142, 0.0, 0.160)),
]
DRILL_TIP = (0.167, 0.0, 0.160)  # end of the bit
DRILL_MASS = 0.895  # YCB's measured mass
# Robotiq grasp: centre of the handle, approach along the bit (identity).
DRIVER_GRIP_XYZQUAT = [-0.022, 0.0, 0.085, 0.0, 0.0, 0.0, 1.0]
DRILL_YAW_XYZW = "0 0 1 0"  # on the dock: bit toward -X world
# rack_hold: DOWN, then 180 deg about world Z (the drill's dock yaw).
RACK_HOLD_XYZW = "-0.7071068 0 0.7071068 0"

# Arm start ("home") configurations: "candle" -- upper arm and forearm
# straight up over the pedestal, 1.5 m apart and clear of the bench.
LEFT_HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
RIGHT_HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]


def part_ys(n: int) -> list[float]:
    return [round((i - (n - 1) / 2) * PART_PITCH, 4) for i in range(n)]


def _box_link(name, size, xyz, rgba, mass=None):
    inertial = ""
    if mass is not None:
        inertial = f"""
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>"""
    geom = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
    return f"""  <link name="{name}">{inertial}
    <visual>
      <origin xyz="{xyz}"/>
      <geometry>{geom}</geometry>
      <material name="{name}_mat"><color rgba="{rgba}"/></material>
    </visual>
    <collision>
      <origin xyz="{xyz}"/>
      <geometry>{geom}</geometry>
    </collision>
  </link>
"""


HEADER = "<!-- GENERATED by build_scene.py: edit the generator, not this file. -->\n"


def _handle(name, xyz, clearance, comment, approach="1 0 0"):
    return f"""  <!-- {comment} -->
  <handle name="{name}" clearance="{clearance}" approaching_direction="{approach}">
    <position xyz="{xyz}" xyzw="{DOWN}"/>
    <link name="base_link"/>
  </handle>
"""


def part_urdf(name):
    tab_z = round(PART[2] / 2 + TAB[2] / 2, 4)
    return (
        f'<?xml version="1.0"?>\n{HEADER}<robot name="{name}">\n'
        + _box_link("base_link", PART, "0 0 0", "0.35 0.55 0.80 1", mass=0.2)
        + _box_link("tab", TAB, f"{TAB_X} 0 {tab_z}", "0.25 0.42 0.65 1")
        + """  <joint name="tab_fixed" type="fixed">
    <parent link="base_link"/>
    <child link="tab"/>
  </joint>
</robot>
"""
    )


def part_srdf(name):
    hz = PART[2] / 2
    return (
        f'<?xml version="1.0"?>\n{HEADER}<robot name="{name}">\n'
        + '  <disable_collisions link1="base_link" link2="tab" reason="Adjacent"/>\n'
        + _handle(
            "h_grasp",
            f"{TAB_X} 0 {round(GRASP_Z, 4)}",
            0.05,
            "Robotiq top-down grasp, closing across Y; raised so the "
            "fingertips clear the jig once the part is clamped",
        )
        + _handle(
            "h_seat",
            f"0 0 {-hz}",
            0.03,
            "Bottom face: the jig clamp holds the part here. The clamp is "
            "fixed and the part comes down into it, so relative to the part "
            "the clamp travels UP: approach -X (see FRAME CONVENTION)",
            approach="-1 0 0",
        )
        + _handle(
            "h_hole1",
            f"{HOLE_XS[0]} 0 {round(hz + GAP, 4)}",
            0.03,
            "Screw hole 1 (driver tip docks here; GAP above the face)",
        )
        + _handle(
            "h_hole2", f"{HOLE_XS[1]} 0 {round(hz + GAP, 4)}", 0.03, "Screw hole 2"
        )
        + "</robot>\n"
    )


def driver_urdf(repo_root: str = REPO_ROOT):
    collisions = ""
    for name, kind, size, c in DRILL_PROXIES:
        if kind == "box":
            geom = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
            rpy = "0 0 0"
        else:  # cylinder along the bit (drill +X)
            geom = f'<cylinder radius="{size[0]}" length="{size[1]}"/>'
            rpy = "0 1.5707963 0"
        collisions += f"""    <collision name="{name}">
      <origin xyz="{c[0]} {c[1]} {c[2]}" rpy="{rpy}"/>
      <geometry>{geom}</geometry>
    </collision>
"""
    bit = next(p for p in DRILL_PROXIES if p[0] == "bit")
    return f"""<?xml version="1.0"?>
{HEADER}<!-- Cordless drill/driver: visual = YCB 035_power_drill scan (CC BY 4.0,
     see assets/ycb_035_power_drill/README.md); collision = primitives
     fitted to it, plus a driver bit the scan lacks. -->
<robot name="driver">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.09"/>
      <mass value="{DRILL_MASS}"/>
      <inertia ixx="3e-3" ixy="0" ixz="0" iyy="3e-3" iyz="0" izz="2e-3"/>
    </inertial>
    <visual>
      <origin xyz="{DRILL_MESH_XYZ}" rpy="{DRILL_MESH_RPY}"/>
      <geometry><mesh filename="{repo_root}/{DRILL_MESH}"/></geometry>
    </visual>
    <visual>
      <origin xyz="{bit[3][0]} {bit[3][1]} {bit[3][2]}" rpy="0 1.5707963 0"/>
      <geometry><cylinder radius="{bit[2][0]}" length="{bit[2][1]}"/></geometry>
      <material name="bit_mat"><color rgba="0.75 0.75 0.78 1"/></material>
    </visual>
{collisions}  </link>
</robot>
"""


def driver_srdf():
    g = DRIVER_GRIP_XYZQUAT
    tip = " ".join(str(v) for v in DRILL_TIP)
    return (
        f'<?xml version="1.0"?>\n{HEADER}<robot name="driver">\n'
        + f"""  <!-- Robotiq grasp across the handle, approaching from behind it
       along the bit (+X), fingers closing across its 38 mm width (Y). -->
  <handle name="h_grip" clearance="0.05" approaching_direction="1 0 0">
    <position xyz="{g[0]} {g[1]} {g[2]}" xyzw="{g[3]} {g[4]} {g[5]} {g[6]}"/>
    <link name="base_link"/>
  </handle>
"""
        + _handle(
            "h_rack",
            "0 0 0",
            0.03,
            "Battery base: the dock holds the drill standing here. Fixed "
            "gripper, so like h_seat the approach is -X",
            approach="-1 0 0",
        )
        + f"""  <!-- Tool-mounted gripper: the bit's end. +X along the bit, turned
       180 deg about it (xyzw 1 0 0 0) so that docked in partN/h_holeK
       (+X down, +Z world +X) the handle points to world +X. -->
  <gripper name="tip" clearance="0.01">
    <position xyz="{tip}" xyzw="1 0 0 0"/>
    <link name="base_link"/>
  </gripper>
</robot>
"""
    )


def fixtures_urdf(n):
    jig_len = PART[0] + 0.06
    jig_w = n * PART_PITCH + 0.06
    return (
        f'<?xml version="1.0"?>\n{HEADER}'
        "<!-- Static cell furniture as one fixed-base, zero-joint robot, so\n"
        "     the jig clamps and the charging dock can be <gripper>s that hold\n"
        "     parts and the driver in place (an object handle cannot). -->\n"
        '<robot name="fixtures">\n'
        + _box_link(
            "base_link",
            (jig_len, jig_w, JIG_H),
            f"{JIG_X} 0 {round(BENCH_TOP + GAP + JIG_H / 2, 4)}",
            "0.3 0.3 0.33 1",
        )
        + _box_link(
            "rack",
            RACK_SIZE,
            f"{RACK[0]} {RACK[1]} {round(BENCH_TOP + GAP + RACK_SIZE[2] / 2, 4)}",
            "0.25 0.25 0.28 1",
        )
        + """  <joint name="rack_fixed" type="fixed">
    <parent link="base_link"/>
    <child link="rack"/>
  </joint>
</robot>
"""
    )


def fixtures_srdf(n):
    out = f'<?xml version="1.0"?>\n{HEADER}<robot name="fixtures">\n'
    out += '  <disable_collisions link1="base_link" link2="rack" reason="Static"/>\n'
    for i, y in enumerate(part_ys(n), start=1):
        out += f"""  <!-- Jig clamp {i}: holds part{i}'s h_seat, GAP above the jig top. -->
  <gripper name="clamp{i}" clearance="0.03">
    <position xyz="{JIG_X} {y} {round(JIG_TOP + GAP, 4)}" xyzw="{DOWN}"/>
    <link name="base_link"/>
  </gripper>
"""
    out += f"""  <!-- Charging dock: holds the drill's h_rack (battery base) GAP above
       the dock, bit toward -X world (DOWN turned 180 deg about world Z). -->
  <gripper name="rack_hold" clearance="0.03">
    <position xyz="{RACK[0]} {RACK[1]} {round(RACK_TOP + GAP, 4)}" xyzw="{RACK_HOLD_XYZW}"/>
    <link name="base_link"/>
  </gripper>
</robot>
"""
    return out


def bench_urdf():
    return (
        f'<?xml version="1.0"?>\n{HEADER}<robot name="workbench">\n'
        + _box_link(
            "top",
            (1.0, 1.0, 0.04),
            f"{BENCH_X} 0 {round(BENCH_TOP - 0.02, 4)}",
            "0.45 0.32 0.2 1",
        )
        + "</robot>\n"
    )


def ground_urdf():
    return (
        f'<?xml version="1.0"?>\n{HEADER}<robot name="ground">\n'
        + _box_link("floor", (3.0, 2.0, 0.02), f"{BENCH_X} 0 -0.02", "0.6 0.6 0.6 1")
        + "</robot>\n"
    )


def _joint_group(arm, home):
    names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    pi = "3.14159265359"
    lines = [
        f"    - {{joint: {arm}/{j}, initial: {q}, bounds: [-{pi}, {pi}]}}"
        for j, q in zip(names, home)
    ]
    fingers = [
        ("finger_joint", "0.0, 0.8"),
        ("left_inner_knuckle_joint", "0.0, 0.8757"),
        ("left_inner_finger_joint", "-0.8757, 0.0"),
        ("right_outer_knuckle_joint", "0.0, 0.81"),
        ("right_inner_knuckle_joint", "0.0, 0.8757"),
        ("right_inner_finger_joint", "-0.8757, 0.0"),
    ]
    lines += [
        f"    - {{joint: {arm}/{j}, initial: 0.0, bounds: [{b}]}}" for j, b in fingers
    ]
    return "\n".join(lines)


def config_yaml(n):
    ys = part_ys(n)
    parts = [f"part{i}" for i in range(1, n + 1)]
    rest_z = round(BENCH_TOP + PART[2] / 2 + GAP, 4)
    driver_z = round(RACK_TOP + GAP, 4)
    obj_paths = "\n".join(
        f"    {p}: {{urdf: ../generated/{p}.urdf, srdf: ../generated/{p}.srdf}}"
        for p in parts
    )
    obj_poses = "\n".join(
        f"  {p}:\n    initial_pose_xyzquat: [{STAGING_X}, {y}, {rest_z}, 0, 0, 0, 1]\n"
        f"    handles: [{p}/h_grasp, {p}/h_seat, {p}/h_hole1, {p}/h_hole2]"
        for p, y in zip(parts, ys)
    )
    grasps = [f"{p}/h_grasp" for p in parts]
    holes = [f"{p}/h_hole{k}" for p in parts for k in (1, 2)]
    clamp_pairs = "\n".join(
        f"  fixtures/clamp{i}: [{p}/h_seat]" for i, p in enumerate(parts, start=1)
    )
    return f"""# GENERATED by build_scene.py --parts {n} -- edit the generator, not this file.
# Screw-assembly cell: see ../README.md. Paths are relative to this file.
task: screw_assembly

paths:
  robot:
    ur10_left:
      urdf: {ARMS_DIR}/ur10_robotiq.container.urdf
      srdf: {ARMS_DIR}/ur10_robotiq.srdf
    ur10_right:
      urdf: {ARMS_DIR}/ur10_robotiq_right.container.urdf
      srdf: {ARMS_DIR}/ur10_robotiq.srdf
    fixtures:
      urdf: ../generated/fixtures.urdf
      srdf: ../generated/fixtures.srdf
  environment:
    ground: ../generated/ground.urdf
    workbench: ../generated/workbench.urdf
  objects:
{obj_paths}
    driver: {{urdf: ../generated/driver.urdf, srdf: ../generated/driver.srdf}}

robots: [ur10_left, ur10_right, fixtures]
environments: [ground, workbench]

joint_groups:
  UR10_LEFT:
{_joint_group("ur10_left", LEFT_HOME)}
  UR10_RIGHT:
{_joint_group("ur10_right", RIGHT_HOME)}

# Objects must stay in bounds anywhere an arm can carry them -- including
# the drill held at ur10_right's candle home (x~1.5, z~2): tighter bounds
# made every phase after the first home retreat reject its start config.
freeflyer_bounds:
  translation: [[-0.3, 1.8], [-1.0, 1.0], [0.2, 2.4]]
  quaternion: [[-1.0001, 1.0001], [-1.0001, 1.0001], [-1.0001, 1.0001], [-1.0001, 1.0001]]

# The driver's tip is carried by whichever arm holds the driver (resolved
# from the live grasp state), so it is listed under both arms.
arm_groups:
  ur10_left:
    joint_keyword: ur10_left
    grippers: [ur10_left/gripper, driver/tip]
  ur10_right:
    joint_keyword: ur10_right
    grippers: [ur10_right/gripper, driver/tip]

objects:
{obj_poses}
  driver:
    initial_pose_xyzquat: [{RACK[0]}, {RACK[1]}, {driver_z}, {", ".join(DRILL_YAW_XYZW.split())}]
    handles: [driver/h_grip, driver/h_rack]

grippers:
  - ur10_left/gripper
  - ur10_right/gripper
  - driver/tip
  - fixtures/rack_hold
{chr(10).join(f"  - fixtures/clamp{i}" for i in range(1, n + 1))}

valid_pairs:
  ur10_left/gripper: [{", ".join(grasps)}]
  ur10_right/gripper: [driver/h_grip]
  driver/tip: [{", ".join(holes)}]
  fixtures/rack_hold: [driver/h_rack]
{clamp_pairs}

environment_contacts: {{}}
freeze_joints: []

planning:
  validation_step: 0.01
  projector_step: 0.1
  max_iterations: 1000
  max_random_attempts: 1000

optimization:
  random_shortcut_loops: 50
  spline_zero_derivatives_at_state: false
  time_param_safety: 0.95
  time_param_order: 2
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--parts", type=int, default=4, choices=range(1, 7))
    ap.add_argument(
        "--repo-root",
        default=REPO_ROOT,
        help="long_tamp checkout the drill mesh path points into "
        "(default: the container's)",
    )
    args = ap.parse_args()
    GEN.mkdir(exist_ok=True)
    CFG.mkdir(exist_ok=True)
    files = {
        GEN / "fixtures.urdf": fixtures_urdf(args.parts),
        GEN / "fixtures.srdf": fixtures_srdf(args.parts),
        GEN / "driver.urdf": driver_urdf(args.repo_root),
        GEN / "driver.srdf": driver_srdf(),
        GEN / "workbench.urdf": bench_urdf(),
        GEN / "ground.urdf": ground_urdf(),
        CFG / "screw_assembly_config.yaml": config_yaml(args.parts),
    }
    for i in range(1, args.parts + 1):
        files[GEN / f"part{i}.urdf"] = part_urdf(f"part{i}")
        files[GEN / f"part{i}.srdf"] = part_srdf(f"part{i}")
    for stale in GEN.glob("part*.[us]rdf"):
        if stale not in files:
            stale.unlink()
    for path, text in files.items():
        path.write_text(text)
    print(
        f"wrote {len(files)} files for {args.parts} part(s) -> {GEN.name}/, {CFG.name}/"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
