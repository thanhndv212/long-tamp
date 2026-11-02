"""RoboCerebra open/close/turnon of a fixture's joint, without HPP or env
server (#161)."""

import numpy as np
import pytest

from long_tamp.eval.robocerebra_articulated import (
    BACK_OFF,
    HALF_TURN,
    PAST_LIMIT,
    ArticulatedMotion,
    ArticulationController,
    articulated_motion,
    handle_pose,
    joint_target,
    reached,
)
from long_tamp.eval.robocerebra_plan import PlanningError
from long_tamp.eval.robocerebra_policy import HppPolicy, SegmentGoal
from long_tamp.eval.robocerebra_tracker import POS_SCALE, ROT_SCALE, rotation_vector
from long_tamp.grasping.geometry import matrix_to_quat_xyzw

# The ranges as LIBERO's fixtures have them: a drawer opening along +q, one
# opening along -q (the wooden cabinet), a microwave door, the stove's knob.
DRAWER = {"open": [0.1, 0.3], "close": [0.0, 0.005]}
WOODEN = {"open": [-0.3, -0.1], "close": [-0.005, 0.0]}
DOOR = {"open": [-2.094, -1.3], "close": [-0.005, 0.0]}
KNOB = {"turnon": [0.5, 2.1], "turnoff": [-0.005, 0.5]}


def test_open_goes_into_its_range_from_the_bound_nearest_closed():
    assert joint_target("open", DRAWER, "slide") == pytest.approx(0.14)
    assert joint_target("open", WOODEN, "slide") == pytest.approx(-0.14)
    assert joint_target("open", DOOR, "hinge") == pytest.approx(-1.697)  # half
    assert joint_target("turnon", KNOB, "hinge") == pytest.approx(1.1)
    # A narrow range: half its width.
    assert joint_target("open", {"open": [0.1, 0.12], "close": [0, 0]}, "slide") == (
        pytest.approx(0.11)
    )


def test_close_goes_to_the_far_bound():
    assert joint_target("close", DRAWER, "slide") == 0.0
    assert joint_target("close", WOODEN, "slide") == 0.0
    assert joint_target("turnoff", KNOB, "hinge") == -0.005
    with pytest.raises(ValueError, match="close"):
        joint_target("open", {"open": [0.1, 0.3]}, "slide")


def test_reached_passes_the_bound_nearest_the_opposite_range():
    assert reached("open", DRAWER, 0.11) and not reached("open", DRAWER, 0.09)
    assert reached("open", WOODEN, -0.11) and not reached("open", WOODEN, -0.09)
    assert reached("close", DRAWER, 0.004) and not reached("close", DRAWER, 0.006)
    assert reached("close", WOODEN, -0.004) and not reached("close", WOODEN, -0.006)
    assert reached("turnon", KNOB, 0.6) and not reached("turnon", KNOB, 0.4)
    # The targets pass the checker.
    for predicate, ranges, kind in [
        ("open", DRAWER, "slide"),
        ("open", WOODEN, "slide"),
        ("open", DOOR, "hinge"),
        ("turnon", KNOB, "hinge"),
    ]:
        assert reached(predicate, ranges, joint_target(predicate, ranges, kind))


def test_handle_pose_by_the_joints_suffix():
    top = handle_pose("WoodenCabinet", "wooden_cabinet_1_top_level")
    bottom = handle_pose("WhiteCabinet", "white_cabinet_1_bottom_level")
    assert top[2, 3] > 0.15 and bottom[2, 3] < 0.05
    assert handle_pose("Microwave", "microwave_1_microjoint") is not None
    assert handle_pose("WoodenCabinet", "wooden_cabinet_1_lid") is None
    assert handle_pose("Basket", "basket_1_lid") is None
    for T in (top, bottom):
        assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3))
        assert np.linalg.det(T[:3, :3]) == pytest.approx(1.0)
    # A copy: the caller may turn it.
    top[:3, 3] = 0.0
    assert handle_pose("WoodenCabinet", "wooden_cabinet_1_top_level")[2, 3] > 0.15


def drawer(predicate="open", value=0.0, target=0.14, kind="slide"):
    """A drawer in front of the robot, opening towards it (-x); its handle
    gripped from the robot's side, fingers closing vertically."""
    body = np.eye(4)
    body[:3, 3] = [0.3, 0.0, 0.5]
    handle = np.eye(4)
    handle[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]  # z along +x: into it
    handle[:3, 3] = [-0.1, 0.0, 0.0]
    return ArticulatedMotion(
        predicate,
        "drawer_1_joint",
        kind,
        value,
        target,
        body,
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.3, 0.1, 0.5]),
        handle,
    )


def test_a_slide_translates_along_its_axis_a_hinge_turns_about_its_anchor():
    way = drawer()
    assert np.allclose(way.motion(0.1)[:3, 3], [-0.1, 0.0, 0.0])
    assert np.allclose(way.motion(0.1)[:3, :3], np.eye(3))
    door = drawer(kind="hinge")
    door.axis = np.array([0.0, 0.0, 1.0])
    M = door.motion(np.pi / 2)
    assert np.allclose(M[:3, :3] @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(M[:3, :3] @ door.anchor + M[:3, 3], door.anchor)


def test_close_pushes_past_the_limit():
    assert drawer("open", 0.0, 0.14).travel == pytest.approx(0.14)
    assert drawer("close", 0.14, 0.0).travel == pytest.approx(
        -0.14 - PAST_LIMIT["slide"]
    )
    assert drawer("close", 0.1, 0.0, "hinge").travel == pytest.approx(
        -0.1 - PAST_LIMIT["hinge"]
    )


def obs(hand, wrist=None):
    out = {
        "robot0_eef_pos": hand[:3, 3].copy(),
        "robot0_eef_quat": matrix_to_quat_xyzw(hand[:3, :3]),
    }
    if wrist is not None:
        out["robot0_joint_pos"] = np.array([0, 0, 0, -2, 0, 2, wrist], dtype=float)
    return out


def test_the_grip_approaches_along_the_hands_axis():
    way = drawer()
    grasp = way.grasp()
    stages = way.grip_stages(obs(way.grasp()))
    assert [s.name for s in stages] == ["approach", "reach", "grip"]
    assert np.allclose(stages[0].tcp[-1][:3, 3], grasp[:3, 3] - BACK_OFF * grasp[:3, 2])
    assert np.allclose(stages[1].tcp[-1], grasp)
    assert stages[2].gripper == 1.0 and stages[0].gripper == -1.0
    move = way.move_stage()
    assert np.allclose(move.tcp[0], grasp)
    assert np.allclose(move.tcp[-1][:3, 3], grasp[:3, 3] + [-0.14, 0, 0])


def about_z(T, angle):
    R = np.eye(4)
    c, s = np.cos(angle), np.sin(angle)
    R[:2, :2] = [[c, -s], [s, c]]
    return T @ R


def test_the_grip_takes_the_half_turn_nearer_the_hand():
    way = drawer()
    grasp = way.grasp()
    way.grip_stages(obs(about_z(grasp, 0.3)))
    assert np.allclose(way.grasp(), grasp)
    way.grip_stages(obs(about_z(grasp, np.pi - 0.3)))
    assert np.allclose(way.grasp(), grasp @ HALF_TURN)


def test_the_grip_keeps_the_last_wrist_joint_in_its_limits():
    hand = about_z(drawer().grasp(), -0.5)  # the grasp is 0.5 rad on
    for wrist, turned in [(None, False), (0.0, False), (2.6, True), (-2.6, False)]:
        way = drawer()
        grasp = way.grasp()
        way.grip_stages(obs(hand, wrist))
        expected = grasp @ HALF_TURN if turned else grasp
        assert np.allclose(way.grasp(), expected), wrist
    turn = rotation_vector(hand[:3, :3].T @ drawer().grasp()[:3, :3])
    assert turn[2] == pytest.approx(0.5)


def test_the_grip_leaves_the_wrist_room_for_a_hinge_about_the_hands_axis():
    # A door turning 1 rad about the hand's axis (a handle pinched from
    # above, the hinge vertical): the wrist turns with it.
    hand = about_z(drawer().grasp(), -0.5)
    for direction, turned in [(1.0, True), (-1.0, False)]:
        way = drawer("open", 0.0, 1.0, "hinge")
        way.axis = direction * way.grasp()[:3, 2]
        assert way.swing() == pytest.approx(direction)
        grasp = way.grasp()
        way.grip_stages(obs(hand, 1.5))  # the grasp: 2.0, then 3.0 or 1.0
        expected = grasp @ HALF_TURN if turned else grasp
        assert np.allclose(way.grasp(), expected), direction
    assert drawer().swing() == 0.0  # a slide turns nothing


def test_the_approach_turns_the_wrist_the_long_way_round_when_it_must():
    # Ideal/case9's microwave after the bowl's place: the wrist at 2.58, the
    # grip 2.99 rad on about the hand's axis, the door turning the hand
    # 1.697 more. The short way there, and the half turn's (2.43, then
    # 4.13), pass the limit; turning back 3.29 rad leaves -0.71, then 0.99.
    way = drawer("open", 0.0, 1.697, "hinge")
    way.axis = way.grasp()[:3, 2]
    grasp = way.grasp()
    hand = about_z(grasp, -2.99)
    hand[:3, 3] += [-0.1, 0.0, 0.1]
    approach = way.grip_stages(obs(hand, 2.58))[0].tcp
    assert np.allclose(way.grasp(), grasp)
    start = grasp.copy()
    start[:3, 3] -= BACK_OFF * grasp[:3, 2]
    assert np.allclose(approach[0], hand) and np.allclose(approach[-1], start)
    spins = [
        rotation_vector(a[:3, :3].T @ b[:3, :3])[2]
        for a, b in zip(approach[:-1], approach[1:])
    ]
    assert all(-0.11 < spin < 0.0 for spin in spins)  # back, in small steps
    assert sum(spins) == pytest.approx(2.99 - 2 * np.pi)


def turned(R, rotvec):
    """``R`` turned by the world-frame rotation vector ``rotvec``."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return R
    k = rotvec / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return (np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K) @ R


def run(controller, hand, block=None, steps=3000):
    """The hand follows the actions; ``block`` clamps where it can go."""
    for _ in range(steps):
        action = controller(obs(hand))
        if controller.done:
            return action
        hand = hand.copy()
        hand[:3, 3] += POS_SCALE * action[:3]
        hand[:3, :3] = turned(hand[:3, :3], ROT_SCALE * action[3:6])
        if block is not None:
            hand = block(hand)
    raise AssertionError("never done")


def start():
    hand = np.diag([1.0, -1.0, -1.0, 1.0])
    hand[:3, 3] = [0.0, 0.0, 0.7]
    return hand


def test_controller_grips_moves_then_lets_go():
    controller = ArticulationController(drawer(), info={})
    action = run(controller, start())
    assert controller.done and not controller.stalled
    assert not controller.info["move_stalled"]
    names = [entry["stage"] for entry in controller.info["stages"]]
    assert names == ["approach", "reach", "grip", "move", "release", "retreat"]
    assert action[6] == -1.0  # done, the gripper open
    assert np.allclose(action[:3], 0.0) and np.any(action[3:6])


def test_a_push_stopped_at_the_limit_lets_go_where_it_stopped():
    way = drawer("close", 0.14, 0.0)
    way.body_pose[:3, 3] += [-0.14, 0.0, 0.0]  # open by 14 cm
    stop = way.grasp()[0, 3] + 0.13  # the drawer shuts 1 cm short of the plan

    def block(hand):
        hand[0, 3] = min(hand[0, 3], stop)
        return hand

    controller = ArticulationController(way, info={})
    run(controller, start(), block)
    assert controller.done and not controller.stalled
    assert controller.info["move_stalled"]
    assert controller.info["stages"][-1]["stage"] == "retreat"


def test_a_grip_that_stalls_ends_the_segment():
    way = drawer()
    floor = way.grasp()[2, 3] + 0.2  # the hand cannot get down to the handle

    def block(hand):
        hand[2, 3] = max(hand[2, 3], floor)
        return hand

    controller = ArticulationController(way, info={})
    action = run(controller, start(), block)
    assert controller.stalled
    assert controller.info["error"] == "tracking stalled in approach"
    assert action[6] == -1.0


def scene(value=0.0, joints=("wooden_cabinet_1_top_level",), kind="slide"):
    manifest = {
        "articulated": [
            {
                "joint": joint,
                "type": kind,
                "value": value,
                "axis_world": [0.0, 2.0, 0.0],  # normalised on the way
                "anchor": [0.0, 0.0, 0.0],
                "body_pose": [0.1, -0.2, 0.9, 0.0, 0.0, 0.0, 1.0],
            }
            for joint in joints
        ]
    }
    articulations = {
        "wooden_cabinet_1_top_region": {
            "parent": "wooden_cabinet_1",
            "class": "WoodenCabinet",
            "joints": list(joints),
            "ranges": WOODEN,
        }
    }
    return manifest, articulations


def test_articulated_motion_from_the_export():
    manifest, articulations = scene()
    way = articulated_motion(
        manifest, articulations, "Open", "wooden_cabinet_1_top_region"
    )
    assert way.joint == "wooden_cabinet_1_top_level" and way.kind == "slide"
    assert way.target == pytest.approx(-0.14)
    assert np.allclose(way.axis, [0, 1, 0])
    assert np.allclose(way.body_pose[:3, 3], [0.1, -0.2, 0.9])
    assert way.info["class"] == "WoodenCabinet" and way.info["goal"] == -0.14
    assert np.allclose(way.handle, handle_pose("WoodenCabinet", way.joint))


def test_close_moves_the_one_joint_not_closed():
    joints = ("w_top_level", "w_middle_level")
    manifest, articulations = scene(-0.15, joints)
    manifest["articulated"][0]["value"] = 0.0  # the top drawer is closed
    name = "wooden_cabinet_1_top_region"
    assert articulated_motion(manifest, articulations, "close", name).joint == (
        "w_middle_level"
    )
    # Both open: closing would move two.
    manifest["articulated"][0]["value"] = -0.15
    with pytest.raises(PlanningError, match="moves 2 joints"):
        articulated_motion(manifest, articulations, "close", name)
    # Opening moves the first.
    assert articulated_motion(manifest, articulations, "open", name).joint == (
        "w_top_level"
    )


def test_articulated_motion_rejects_what_it_cannot_move():
    manifest, articulations = scene()
    name = "wooden_cabinet_1_top_region"
    with pytest.raises(PlanningError, match="no articulation"):
        articulated_motion(manifest, articulations, "open", "basket_1")
    with pytest.raises(PlanningError, match="not in the exported scene"):
        articulated_motion({"articulated": []}, articulations, "open", name)
    with pytest.raises(PlanningError, match="ball joint"):
        articulated_motion(scene(kind="ball")[0], articulations, "open", name)
    articulations[name]["class"] = "Basket"
    with pytest.raises(PlanningError, match="no handle for a Basket"):
        articulated_motion(manifest, articulations, "open", name)
    with pytest.raises(ValueError):
        articulated_motion(manifest, articulations, "pick", name)


def test_segment_goal_of_an_articulated_predicate():
    assert SegmentGoal.from_spec(["open", "drawer_1"]) == SegmentGoal(
        "articulated", "drawer_1", predicate="open"
    )
    assert SegmentGoal.from_spec(["TurnOn", "flat_stove_1"]).predicate == "turnon"


def test_policy_runs_an_articulated_goal(monkeypatch, tmp_path):
    import long_tamp.eval.robocerebra_plan as plan_module

    manifest, articulations = scene()

    class Scene:
        root = tmp_path

    Scene.manifest = manifest

    class Env:
        def export_scene(self, out):
            return Scene()

    holding = {"now": None}
    monkeypatch.setattr(plan_module, "held_object", lambda manifest: holding["now"])
    monkeypatch.setattr(plan_module, "robot_files", lambda assets: ("u", "s"))
    name = "wooden_cabinet_1_top_region"
    info = {
        "goal": {name: [["open", name]]},
        "goal_steps": {name: [0]},
        "articulations": articulations,
    }
    controller = HppPolicy(work_dir=tmp_path, isolate=False)(Env(), info, 0, 150)
    assert isinstance(controller, ArticulationController)
    assert controller.info["status"] == "planned"
    assert controller.info["joint"] == "wooden_cabinet_1_top_level"
    # Something in the hand: without recover, it fails; with it, released.
    holding["now"] = "mug_1"
    controller = HppPolicy(work_dir=tmp_path, isolate=False)(Env(), info, 0, 150)
    assert controller.info["status"] == "planning failed"
    assert "mug_1 is in the hand" in controller.info["error"]
    policy = HppPolicy(work_dir=tmp_path, isolate=False, recover=True)
    controller = policy(Env(), info, 0, 150)
    assert controller.info["recovery"] == "release mug_1"
    # With recover and the hand empty, no pick first.
    holding["now"] = None
    controller = policy(Env(), info, 0, 150)
    assert "recovery" not in controller.info
    assert isinstance(controller, ArticulationController)
