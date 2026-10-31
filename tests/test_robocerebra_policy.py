"""RoboCerebra HPP policy pieces that need no HPP or env server (#159)."""

import os

import numpy as np
import pytest

from long_tamp.eval.robocerebra_plan import (
    ONTOP_REACH,
    POUR_ANGLES,
    POUR_REACH,
    SUPPORT_MARGIN,
    PlanningError,
    SegmentPlanner,
    collision_corners,
    collision_samples,
    held_object,
    object_corners,
    object_top_region,
    place_heights,
    place_orientations,
    place_positions,
    pour_positions,
    pour_tilts,
    square,
    support_faces,
    support_margin,
)
from long_tamp.eval.robocerebra_policy import (
    HppPolicy,
    ReleaseController,
    SegmentController,
    SegmentGoal,
    UnsupportedGoal,
    segment_specs,
)
from long_tamp.eval.robocerebra_runner import evaluate_case
from long_tamp.eval.robocerebra_tracker import (
    POS_SCALE,
    OscTracker,
    Stage,
    obs_pose,
    resample,
    rotation_vector,
)


def pose(x=0.0, y=0.0, z=0.0):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def obs_at(T):
    from long_tamp.grasping.geometry import matrix_to_quat_xyzw

    return {
        "robot0_eef_pos": T[:3, 3].copy(),
        "robot0_eef_quat": matrix_to_quat_xyzw(T[:3, :3]),
    }


# --- tracker -----------------------------------------------------------------


def test_obs_pose_round_trips_xyzw():
    T = pose(0.1, -0.2, 0.3)
    T[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert np.allclose(obs_pose(obs_at(T)), T)


def test_rotation_vector_near_pi():
    R = np.diag([1.0, -1.0, -1.0])  # half a turn about x
    assert np.allclose(np.abs(rotation_vector(R)), [np.pi, 0.0, 0.0])


def test_resample_keeps_ends_and_thins():
    poses = np.array([pose(x=0.001 * i) for i in range(101)])
    kept = resample(poses, 0.01, 1.0)
    assert np.allclose(kept[0], poses[0]) and np.allclose(kept[-1], poses[-1])
    assert 10 <= len(kept) <= 12


def simulate(tracker, start, steps):
    """Move the end effector by each action exactly as OSC's goal would."""
    current = start.copy()
    actions = []
    for _ in range(steps):
        action = tracker(obs_at(current))
        actions.append(action)
        current[:3, 3] += POS_SCALE * action[:3]
    return current, actions


def test_tracker_follows_stages_and_gripper():
    stages = [
        Stage("move", np.array([pose(), pose(x=0.1)]), gripper=-1.0),
        Stage("close", pose(x=0.1)[None], gripper=1.0, dwell=3),
    ]
    tracker = OscTracker(stages)
    end, actions = simulate(tracker, pose(), 40)
    assert tracker.done
    assert [entry["stage"] for entry in tracker.log] == ["move", "close"]
    assert np.allclose(end[:3, 3], [0.1, 0, 0], atol=0.005)
    assert actions[0][6] == -1.0 and actions[-1][6] == 1.0
    # A rotation-free target still carries a nonzero rotation (re-anchors OSC).
    assert all(np.any(action[3:6]) for action in actions)


def test_tracker_boosts_through_friction():
    """An action below 0.2 does not move the end effector (static friction)."""

    def end_error(**kwargs):
        stage = Stage(
            "insert",
            np.array([pose(), pose(x=0.012)]),
            gripper=-1.0,
            step=0.025,
            tol=0.002,
        )
        tracker = OscTracker([stage], **kwargs)
        current = pose()
        for _ in range(30):
            action = tracker(obs_at(current))
            if np.abs(action[0]) >= 0.2:
                current[:3, 3] += POS_SCALE * action[:3] * 0.2
        assert tracker.done
        return tracker.log[0]["end_error"][0]

    assert end_error(max_boost=1.0) > 0.009  # stalls once the action drops below 0.2
    assert end_error() < 0.003


def test_tracker_gives_up_on_a_carrot_out_of_reach():
    """A wall at x = 0.03 m: the retreat to 0.3 m never gets past it (at its
    last waypoint, ``patience`` would end it)."""
    retreat = np.array([pose(x=0.01 * i) for i in range(31)])
    stage = Stage("retreat", retreat, gripper=-1.0)
    tracker = OscTracker([stage, Stage("next", pose()[None], gripper=1.0)])
    current = pose()
    for steps in range(1, 200):
        obs = obs_at(current)
        obs["robot0_joint_pos"] = np.zeros(7)
        action = tracker(obs)
        if tracker.done:
            break
        current[:3, 3] += POS_SCALE * action[:3]
        current[0, 3] = min(current[0, 3], 0.03)
    assert tracker.stalled == "retreat" and tracker.stage_name == "retreat"
    assert steps < 50
    entry = tracker.log[-1]
    assert entry["stage"] == "retreat" and entry["stalled"]
    assert entry["joints"] == [0.0] * 7
    assert entry["end_error"][0] > 0.01
    # Stalled, it holds where the hand is, with the stage's gripper.
    held = tracker(obs_at(current))
    assert np.allclose(held[:3], 0.0) and held[6] == -1.0


def test_tracker_does_not_stall_while_it_moves():
    stage = Stage("carry", np.array([pose(), pose(x=0.3)]), gripper=1.0)
    tracker = OscTracker([stage], stall=5)
    current = pose()
    for _ in range(400):
        action = tracker(obs_at(current))
        if tracker.done:
            break
        current[:3, 3] += 0.1 * POS_SCALE * action[:3]  # slow, but moving
    assert tracker.done and tracker.stalled is None


def test_tracker_applies_tcp_offset():
    offset = pose(z=0.1)
    tracker = OscTracker(
        [Stage("move", pose(x=0.05)[None], gripper=-1.0)], tcp_to_eef=offset
    )
    end, _ = simulate(tracker, offset.copy(), 20)
    assert np.allclose(end[:3, 3], [0.05, 0.0, 0.1], atol=0.005)


def test_stage_rejects_empty():
    with pytest.raises(ValueError):
        Stage("none", np.zeros((0, 4, 4)), gripper=1.0)


# --- planner helpers ---------------------------------------------------------


def test_held_object_needs_both_fingers():
    left = {"link": "gripper0_leftfinger", "object": "butter_1"}
    right = {"link": "gripper0_finger_joint2_tip", "object": "butter_1"}
    assert held_object({"robot_contacts": [left, right]}) == "butter_1"
    assert held_object({"robot_contacts": [left]}) is None
    assert held_object({}) is None


def test_object_corners_mirror_bottom():
    corners = object_corners([[0.01, 0.02, -0.03], [-0.01, 0.02, -0.03]])
    assert corners.shape == (4, 3)
    assert np.allclose(corners[2:, 2], 0.03)
    assert object_corners(None) is None


def region(z_range, support=True, height=0.47):
    out = {
        "box": {
            "center": [0.0, 0.0, 0.0],
            "half_extents": [0.05, 0.03, 0.03],
            "z_range": z_range,
        }
    }
    if support:
        out.update(support_geom="floor", height=height)
    return out


def test_place_heights_rest_on_support():
    lo, hi = place_heights(region([0.44, 0.51]), below=0.02, drop=0.01)
    assert lo == pytest.approx(0.50) and hi == pytest.approx(0.56)
    # Resting origin above the box: cannot count.
    assert place_heights(region([0.44, 0.48]), below=0.02, drop=0.01) is None


def test_place_heights_without_support_stay_in_range():
    assert place_heights(
        region([0.44, 0.64], support=False), 0.02, 0.01
    ) == pytest.approx((0.445, 0.635))


def test_place_positions_back_off_along_approach():
    positions = place_positions(
        region([0.0, 0.0]), np.array([0.0, 1.0, 0.0]), (0.5, 0.5)
    )
    xy = np.array([p for p, _ in positions])
    assert np.allclose(xy[0], [0.0, 0.0])
    assert np.all(xy[:, 1] <= 1e-9) and xy[:, 1].min() == pytest.approx(-0.02)
    # A vertical approach tries the center only.
    assert (
        len(
            place_positions(region([0.0, 0.0]), np.array([0.0, 0.0, -1.0]), (0.5, 0.52))
        )
        == 3
    )


def test_place_orientations_turn_approach_horizontal():
    tcp = np.eye(4)
    tcp[:3, :3] = np.diag([1.0, -1.0, -1.0])  # top-down
    options = dict(place_orientations(tcp))
    assert len(options) == 20 and np.allclose(options["held"], np.eye(3))
    assert np.allclose(options["-y/0"] @ tcp[:3, 2], [0.0, -1.0, 0.0])
    # The held turns come right after held, about the vertical only.
    assert [name for name, _ in place_orientations(tcp)[:4]] == [
        "held",
        "held/90",
        "held/180",
        "held/270",
    ]
    assert np.allclose(options["held/180"], np.diag([-1.0, -1.0, 1.0]))
    for R in options.values():
        assert np.allclose(R @ R.T, np.eye(3)) and np.linalg.det(R) == pytest.approx(
            1.0
        )


def test_pour_positions_stay_in_the_checkers_reach():
    center = np.array([0.0, 0.0, 0.5])
    positions = pour_positions(center, np.array([0.3, 0.0, 0.6]), z=0.6)
    assert np.allclose(positions[0], [0.0, 0.0, 0.6])
    # Backed off towards where the object comes from, never past the reach.
    assert {round(p[0], 3) for p in positions} == {0.0, 0.05, 0.1}
    assert all(np.linalg.norm(p - center) < POUR_REACH for p in positions)
    # Too high over the receiver: nothing within reach.
    assert pour_positions(center, center, z=0.8) == []


def test_collision_corners_reach_the_top_of_a_vessel_based_at_its_origin(tmp_path):
    # A frypan: its pan rests at the origin, its handle rises to 6 cm.
    urdf = tmp_path / "pan.urdf"
    urdf.write_text("""<robot name="pan"><link name="base_link">
  <collision><origin xyz="0 0 0.01"/><geometry><box size="0.2 0.2 0.02"/></geometry></collision>
  <collision><origin xyz="0.15 0 0.04"/><geometry><cylinder radius="0.01" length="0.04"/></geometry></collision>
</link></robot>""")
    corners = collision_corners(urdf)
    assert corners[:, 2].min() == pytest.approx(0.0)
    assert corners[:, 2].max() == pytest.approx(0.06)
    assert corners[:, 0].max() == pytest.approx(0.16)


RACK = """<robot name="rack"><link name="base_link">
  <collision><geometry><box size="0.14 0.28 0.035"/></geometry></collision>
  <collision><origin xyz="0.04 0.05 0.06"/><geometry><box size="0.008 0.008 0.09"/></geometry></collision>
  <collision><origin xyz="-0.04 0.05 0.06"/><geometry><box size="0.008 0.008 0.09"/></geometry></collision>
</link></robot>"""


def test_object_top_region_supports_between_and_on_the_pegs(tmp_path):
    urdf = tmp_path / "rack.urdf"
    urdf.write_text(RACK)
    region = object_top_region("rack_1", urdf, pose(0.1, -0.5, 0.44))
    assert region["region"] == region["support_geom"] == "rack_1"
    assert region["box"]["center"][:2] == pytest.approx([0.1, -0.5])
    assert region["box"]["half_extents"][:2] == [ONTOP_REACH, ONTOP_REACH]
    # The checker wants the placed origin no lower than the rack's.
    assert region["box"]["z_range"][0] == pytest.approx(0.44)
    assert region["height"] == pytest.approx(0.44 + 0.0175)  # the plate
    assert region["top"] == pytest.approx(0.44 + 0.105)  # the pegs' tops
    # From resting on the plate up to resting on the pegs, then the rise.
    lo, hi = place_heights(region, below=0.01, drop=0.02)
    assert lo == pytest.approx(0.4875)
    assert hi == pytest.approx(0.545 + 0.01 + 0.02 + 0.06)


def test_object_top_region_turns_with_the_object(tmp_path):
    urdf = tmp_path / "rack.urdf"
    urdf.write_text(RACK)
    upside_down = pose(0.0, 0.0, 0.5)
    upside_down[:3, :3] = np.diag([1.0, -1.0, -1.0])
    region = object_top_region("rack_1", urdf, upside_down)
    # Pegs down: only the plate's face is on top.
    assert region["height"] == region["top"] == pytest.approx(0.5175)


def test_support_faces_are_the_pegs_tips_or_the_plate(tmp_path):
    urdf = tmp_path / "rack.urdf"
    urdf.write_text(RACK)
    at = pose(0.1, -0.5, 0.44)
    tips = support_faces(urdf, at, 0.545)
    assert [list(world[:2, 3]) for world, _ in tips] == [
        pytest.approx([0.14, -0.45]),
        pytest.approx([0.06, -0.45]),
    ]
    plate = support_faces(urdf, at, 0.4575)
    assert len(plate) == 1 and plate[0][1] == pytest.approx([0.07, 0.14, 0.0175])
    assert support_faces(urdf, at, 0.5) == []


def test_collision_samples_fill_the_boxes(tmp_path):
    urdf = tmp_path / "butter.urdf"
    urdf.write_text("""<robot name="butter"><link name="base_link">
  <collision><origin xyz="0 0 0.01"/><geometry><box size="0.06 0.03 0.02"/></geometry></collision>
</link></robot>""")
    points, weights = collision_samples(urdf, n=3)
    assert len(points) == 27
    assert weights.sum() == pytest.approx(0.06 * 0.03 * 0.02)
    assert points.min(axis=0) == pytest.approx([-0.03, -0.015, 0.0])
    assert points.max(axis=0) == pytest.approx([0.03, 0.015, 0.02])


def face(x, y, half_x, half_y):
    return pose(x, y), np.array([half_x, half_y, 0.005])


def block(n=5, half=0.02):
    """A 4 cm cube's sample points, its bottom at z 0, and their weights."""
    grid = np.linspace(-half, half, n)
    points = np.array(np.meshgrid(grid, grid, grid + half)).reshape(3, -1).T
    return points, np.ones(len(points))


def test_support_margin_on_a_plate_is_the_distance_to_its_edge():
    points, weights = block()
    margin = support_margin(points, weights, [face(0.0, 0.0, 0.1, 0.1)])
    assert margin == pytest.approx(0.02)
    # Over the plate's edge by 5 mm: only the part over the plate touches.
    margin = support_margin(points + [0.015, 0, 0], weights, [face(0, 0, 0.03, 0.1)])
    assert margin == pytest.approx(0.01)
    assert margin >= SUPPORT_MARGIN


def test_support_margin_rejects_peg_tips_it_cannot_rest_on():
    points, weights = block()
    # One peg under a corner: a single point of contact, no area.
    assert support_margin(points, weights, [face(0.02, 0.02, 0.004, 0.004)]) == (
        -np.inf
    )
    # One row of pegs along an edge: the center of mass is outside the contacts.
    row = [face(0.02, y, 0.004, 0.004) for y in (-0.02, 0.0, 0.02)]
    assert support_margin(points, weights, row) == -np.inf
    # Two pegs along the edge, one each side: a strip the center is outside of.
    two = [face(x, 0.02, 0.004, 0.012) for x in (-0.02, 0.02)]
    assert support_margin(points, weights, two) < 0
    # Pegs under all four corners: it rests, the center 2 cm inside.
    corners = [face(x, y, 0.004, 0.004) for x in (-0.02, 0.02) for y in (-0.02, 0.02)]
    assert support_margin(points, weights, corners) == pytest.approx(0.02)


def test_resting_tries_the_plate_from_the_center_and_the_tips_widest_first():
    points, weights = block(half=0.01)
    region = {"box": {"center": [0.0, 0.0, 0.5], "half_extents": [0.01, 0.01, 0.1]}}
    heights = (0.5, 0.51)
    plate = [face(0.0, 0.0, 0.1, 0.1)]
    found = SegmentPlanner._resting(
        region, np.eye(4), (points, weights), plate, heights, 0.01, "support"
    )
    # 25 positions on a 5 mm grid, two heights each, the center first.
    assert len(found) == 50
    assert found[0][0] == pytest.approx([0.0, 0.0])
    assert [z for _, z in found[:2]] == pytest.approx([0.5, 0.51])
    # Two rows of pegs at x = ±1 cm: only centered across them does it rest
    # on both, and nowhere along the rows is better than another.
    rows = [face(x, 0.0, 0.002, 0.1) for x in (-0.01, 0.01)]
    found = SegmentPlanner._resting(
        region, np.eye(4), (points, weights), rows, heights, 0.01, "top"
    )
    assert len(found) == 10
    assert all(xy[0] == pytest.approx(0.0, abs=1e-9) for xy, _ in found)
    # Nothing to rest on: no positions.
    assert not SegmentPlanner._resting(
        region, np.eye(4), (points, weights), [], heights, 0.01, "top"
    )


def test_on_an_object_places_on_its_top(tmp_path):
    from long_tamp.eval.robocerebra_policy import _region

    urdf = tmp_path / "rack.urdf"
    urdf.write_text(RACK)

    class Scene:
        def __init__(self):
            self.manifest = {
                "objects": {
                    "pan": {},
                    "rack_1": {"pose": [0.1, -0.5, 0.44, 0, 0, 0, 1]},
                },
                "regions": [{"region": "table", "box": {}}],
            }

        def file_paths(self):
            return {"objects": {"rack_1": {"urdf": str(urdf)}}}

        def object_config(self, name):
            return self.manifest["objects"][name]["pose"]

    scene = Scene()
    assert _region(scene, SegmentGoal("placed", "pan", "table"))["region"] == "table"
    top = _region(scene, SegmentGoal("placed", "pan", "rack_1"))
    assert top["object"] == "rack_1" and top["top"] == pytest.approx(0.545)
    for onto in ("pan", "shelf_1"):
        with pytest.raises(UnsupportedGoal):
            _region(scene, SegmentGoal("placed", "pan", onto))


def test_pour_tilts_turn_about_the_tcp_enough_for_the_checker():
    tcp = pose(0.1, 0.2, 0.3)
    tcp[:3, :3] = np.diag([1.0, -1.0, -1.0])  # top-down
    tilts = pour_tilts(tcp)
    assert [axis for axis, _, _ in tilts[:6]] == ["x"] * 6
    assert len(tilts) == 3 * len(POUR_ANGLES) * 2
    for axis, angle, T in tilts:
        assert np.allclose(T[:3, 3], tcp[:3, 3])
        turn = rotation_vector(tcp[:3, :3].T @ T[:3, :3])
        assert np.linalg.norm(turn) == pytest.approx(abs(angle))
        assert np.allclose(np.abs(turn) / abs(angle), np.eye(3)["xyz".index(axis)])
        # The checker's quaternion change, 2 sin(angle / 4), clears 0.2.
        assert 2 * np.sin(abs(angle) / 4) > 0.2


def test_square_snaps_to_world_axes():
    from long_tamp.eval.robocerebra_plan import _axis_angle

    tilted = _axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2 + 0.2)
    assert np.allclose(square(tilted), [[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert np.linalg.det(square(tilted)) == pytest.approx(1.0)


# --- goal translation --------------------------------------------------------


def test_segment_goal_translation():
    assert SegmentGoal.from_spec(["Pick", "butter_1"]) == SegmentGoal(
        "grasped", "butter_1"
    )
    assert SegmentGoal.from_spec(["in", "butter_1", "box_left"]) == SegmentGoal(
        "placed", "butter_1", "box_left"
    )
    assert SegmentGoal.from_spec(["on", "a", "b"]).kind == "placed"
    assert SegmentGoal.from_spec(["pour", "cookies_1", "plate_1"]) == SegmentGoal(
        "poured", "cookies_1", receiver="plate_1"
    )
    for spec in (["open", "drawer_1"], ["pick", "a", "b"], ["in", "a"], ["pour", "a"]):
        with pytest.raises(UnsupportedGoal):
            SegmentGoal.from_spec(spec)


def test_segment_specs_select_the_segment():
    info = {
        "goal": {"a": [["pick", "a"], ["in", "a", "box"]], "b": [["pick", "b"]]},
        "goal_steps": {"a": [0, 1], "b": [2]},
    }
    assert segment_specs(info, 1) == [["in", "a", "box"]]
    assert segment_specs(info, 2) == [["pick", "b"]]
    assert segment_specs(info, 3) == []


def test_controller_without_stages_holds_closed():
    action = SegmentController(None)(obs_at(pose()))
    assert np.allclose(action[:3], 0.0) and np.any(action[3:6]) and action[6] == 1.0


def test_policy_reports_a_missing_region_as_unsupported(monkeypatch):
    def no_region(env, goal, name, record):
        raise UnsupportedGoal(f"no region {goal.region!r} in the scene")

    policy = HppPolicy()
    monkeypatch.setattr(policy, "plan", no_region)
    info = {"goal": {"pan": [["on", "pan", "rack_1"]]}, "goal_steps": {"pan": [0]}}
    controller = policy(None, info, 0, 150)
    assert controller.info["status"] == "unsupported"
    assert controller.info["error"] == "no region 'rack_1' in the scene"


def test_policy_plans_a_pour_over_the_receiver(monkeypatch, tmp_path):
    import long_tamp.eval.robocerebra_plan as plan_module
    import long_tamp.eval.robocerebra_policy as policy_module

    class Scene:
        def __init__(self):
            self.root = tmp_path
            self.manifest = {"objects": {"cookies_1": {}, "plate_1": {}}}

    class Env:
        def export_scene(self, out):
            return Scene()

    calls = []

    def solve(*args):
        calls.append(args)
        return plan_module.SegmentPlan([], [], np.eye(4), {"pour": "over"}), None

    monkeypatch.setattr(policy_module, "_solve", solve)
    monkeypatch.setattr(plan_module, "held_object", lambda manifest: "cookies_1")
    monkeypatch.setattr(plan_module, "robot_files", lambda assets: ("u", "s"))
    policy = HppPolicy(work_dir=tmp_path, isolate=False)
    info = {
        "goal": {
            "cookies_1": [
                ["pour", "cookies_1", "plate_1"],
                ["pour", "cookies_1", "bowl_9"],
            ]
        },
        "goal_steps": {"cookies_1": [0, 1]},
    }
    controller = policy(Env(), info, 0, 150)
    assert controller.info["status"] == "planned"
    assert controller.info["pour"] == "over"
    assert calls[0][3] == "cookies_1" and calls[0][4] is None
    assert calls[0][-1] == "plate_1"
    # A receiver the scene does not have is unsupported, not a failed plan.
    controller = policy(Env(), info, 1, 150)
    assert controller.info["status"] == "unsupported"
    assert controller.info["error"] == "no object 'bowl_9' in the scene"


def test_recovering_policy_releases_or_picks_first(monkeypatch, tmp_path):
    import long_tamp.eval.robocerebra_plan as plan_module
    import long_tamp.eval.robocerebra_policy as policy_module

    class Scene:
        root = tmp_path
        manifest = {"objects": {}, "regions": [{"region": "box_1"}]}

    class Env:
        def export_scene(self, out):
            return Scene()

    holding = {"now": None}
    solved = []

    def solve(scene, urdf, srdf, target, region, *rest):
        solved.append((target, region))
        return plan_module.SegmentPlan([], [], np.eye(4), {}), None

    monkeypatch.setattr(policy_module, "_solve", solve)
    monkeypatch.setattr(plan_module, "held_object", lambda manifest: holding["now"])
    monkeypatch.setattr(plan_module, "robot_files", lambda assets: ("u", "s"))
    info = {
        "goal": {"cube_1": [["pick", "cube_1"], ["in", "cube_1", "box_1"]]},
        "goal_steps": {"cube_1": [0, 1]},
    }
    policy = HppPolicy(work_dir=tmp_path, isolate=False, recover=True)
    # A pick with another object in the hand releases it.
    holding["now"] = "mug_1"
    controller = policy(Env(), info, 0, 150)
    assert isinstance(controller, ReleaseController)
    assert controller.info["recovery"] == "release mug_1" and not solved
    # A place whose object is not in the hand picks it first.
    holding["now"] = None
    controller = policy(Env(), info, 1, 150)
    assert controller.info["status"] == "planned"
    assert controller.info["recovery"] == "pick cube_1"
    assert solved == [("cube_1", None)]
    # Without recover, the same place fails as before.
    controller = HppPolicy(work_dir=tmp_path, isolate=False)(Env(), info, 1, 150)
    assert controller.info["status"] == "planning failed"


def test_release_controller_opens_then_rises():
    controller = ReleaseController(open_steps=2, lift=0.1)
    T = pose()
    actions = [controller(obs_at(T)) for _ in range(3)]
    assert all(a[6] == -1.0 for a in actions)
    assert np.allclose(actions[0][:3], 0.0) and actions[2][2] == 1.0
    assert not controller.done
    T[2, 3] += 0.095
    controller(obs_at(T))
    assert controller.done


def test_controller_records_the_targets_path():
    controller = SegmentController(None, info={"target": "cube_1"})
    for z in (0.40, 0.45, 0.50):
        controller({**obs_at(pose()), "cube_1_pos": np.array([0.1, 0.2, z])})
    assert controller.info["target_start"] == [0.1, 0.2, 0.4]
    assert controller.info["target_end"] == [0.1, 0.2, 0.5]


# --- runner with a closed-loop controller ------------------------------------


class ObsEnv:
    """Segment 0 is done once the end effector has moved 5 cm in x."""

    def __init__(self):
        self.x = 0.0

    def load(self, case):
        return {
            "case": case,
            "steps": ["move"],
            "spans": [[0, 10]],
            "goal_steps": {"cube_1": [0]},
        }

    def reset(self, step=0, seed=None):
        self.x = 0.0
        return self.check()

    def step(self, action):
        self.x += POS_SCALE * float(action[0])
        return {**self.check(), "monitor": ["moved"]}

    def check(self):
        done = int(self.x >= 0.049)
        return {
            "obs": obs_at(pose(x=self.x)),
            "progress": {"cube_1": done},
            "total_completed": done,
            "total": 1,
            "all_done": bool(done),
        }


def test_evaluate_case_steps_a_controller_from_observations():
    def policy(env, info, segment, limit):
        stages = [Stage("move", pose(x=0.05)[None], gripper=1.0)]
        return SegmentController(stages, pose(), {"goals": "move"})

    result = evaluate_case(ObsEnv(), "Ideal/x", policy, seeds=[0], step_limit=20)
    segment = result["rollouts"][0]["segments"][0]
    assert segment["segment_success"] and 0 < segment["steps_used"] < 20
    assert segment["monitor"][0] == "moved"
    assert segment["policy"]["goals"] == "move" and segment["policy"]["stage"] == "move"


def _fails_to_plan(message):
    raise PlanningError(message, {"handle": "h1"})


def test_planning_in_a_child_returns_the_result_and_errors():
    from long_tamp.eval.robocerebra_policy import _in_child

    assert _in_child(divmod, 7, 2) == (3, 1)
    with pytest.raises(PlanningError, match="no handle") as error:
        _in_child(_fails_to_plan, "no handle plans")
    assert error.value.info == {"handle": "h1", "error": "no handle plans"}
    with pytest.raises(RuntimeError, match="ZeroDivisionError"):
        _in_child(divmod, 1, 0)
    with pytest.raises(RuntimeError, match="exit code 3"):
        _in_child(os._exit, 3)


def test_rim_samples_pinch_a_leaning_wall_both_ways():
    from long_tamp.eval.robocerebra_grasps import _rim_samples
    from long_tamp.grasping import PANDA_HAND, Box, GraspPlanner, GraspPlannerParams

    # A bowl's wall: 3 mm thick along x, leaning 15 degrees out about y.
    tilt = np.radians(15)
    pose = np.eye(4)
    pose[:3, :3] = [
        [np.cos(tilt), 0, np.sin(tilt)],
        [0, 1, 0],
        [-np.sin(tilt), 0, np.cos(tilt)],
    ]
    pose[:3, 3] = [0.05, 0.0, 0.03]
    wall = Box(name="wall", pose=pose, size=(0.003, 0.02, 0.04))
    samples = list(_rim_samples(GraspPlanner(PANDA_HAND, GraspPlannerParams()), [wall]))
    sources = [source for _, source in samples]
    assert sources.count("top-rim") == sources.count("top-rim flipped") >= 1
    for grasp, _ in samples:
        assert np.allclose(grasp[:3, 0], [0, 0, -1])  # straight down
        assert abs(grasp[2, 1]) < 1e-9  # the pads close horizontally


def test_settled_start_takes_hpps_placement_of_the_target():
    planner = object.__new__(SegmentPlanner)
    planner._object_index = list(range(2, 9))
    planner._valid = lambda q: (True, "")
    q = [0.1, 0.2, 0.5, 0.0, 0.42341, 0.0, 0.0, 0.0, 1.0]
    goal = [0.3, 0.4, 0.5, 0.0, 0.42354, 0.0, 0.0, 0.0, 1.0]
    message = f"goal configuration at rank 0: ({','.join(map(str, goal))},) does not satisfy the leaf constraint."
    assert planner._settled(message, q) == [0.1, 0.2, *goal[2:]]
    assert planner._settled("Collision between object panda/hand and table", q) is None
    far = [0.3, 0.4, 0.5, 0.0, 0.45, 0.0, 0.0, 0.0, 1.0]
    message = f"goal configuration at rank 0: ({','.join(map(str, far))},) does not satisfy the leaf constraint."
    assert planner._settled(message, q) is None  # a real move, not settling


def bystander_planner(roots, target="mug_1"):
    from types import SimpleNamespace

    model = SimpleNamespace(
        getJointId=lambda joint: joint.split("/")[0],
        joints={name: SimpleNamespace(idx_q=i) for name, i in roots.items()},
    )
    planner = object.__new__(SegmentPlanner)
    planner.target = target
    planner.scene = SimpleNamespace(objects=list(roots))
    planner.task = SimpleNamespace(planner=SimpleNamespace(model=lambda: model))
    return planner


def test_bystanders_are_raised_out_so_the_targets_collision_shows():
    from contextlib import contextmanager

    # q: [sauce z, butter z, mug z]; the mug is the target, in the rack's pegs.
    planner = bystander_planner({"sauce_1": -2, "butter_1": -1, "mug_1": 0})

    class Problem:
        def isConfigValid(self, q):
            for i, name, sunk in ((0, "sauce_1", 0.4235), (1, "butter_1", 0.4715)):
                if q[i] < sunk:  # 1.5 and 1 mm into the table
                    return (
                        False,
                        f"Collision between object environment/table_0 and {name}/base_link_0",
                    )
            return (
                False,
                "Collision between object rack_1/base_link_1 and mug_1/base_link_0",
            )

    @contextmanager
    def validations():
        yield Problem()

    planner._validations = validations
    planner.q0 = [0.422, 0.4702, 0.48]
    info = {}
    planner._settle_bystanders(info)
    assert info["settled"] == {"sauce_1": 0.002, "butter_1": 0.002}
    assert planner.q0[2] == 0.48  # the target is not moved
    assert "mug_1" in Problem().isConfigValid(planner.q0)[1]


def test_pairs_no_motion_changes_are_dropped():
    from types import SimpleNamespace

    names = ["panda/panda_hand_0", "rack_1/base_link_1", "butter_1/base_link_0"]
    names += ["mug_1/base_link_0", "environment/table_0", "panda/panda_link3_0"]
    pair = lambda a, b: SimpleNamespace(first=a, second=b)
    pairs = [pair(1, 2), pair(0, 2), pair(1, 3), pair(2, 4), pair(2, 1)]
    pairs += [pair(0, 3), pair(5, 3)]

    class Geometry:
        geometryObjects = [SimpleNamespace(name=n) for n in names]

        def __init__(self):
            self.collisionPairs = list(pairs)

        def removeCollisionPair(self, p):
            self.collisionPairs = [
                c
                for c in self.collisionPairs
                if (c.first, c.second) != (p.first, p.second)
            ]

    for held, kept in ((False, [1, 2, 3, 5, 6]), (True, [1, 2, 3, 6])):
        planner = bystander_planner({"rack_1": 0, "butter_1": 7, "mug_1": 14})
        planner.held = held
        geometry = Geometry()
        planner.task.planner.device = SimpleNamespace(geomModel=lambda: geometry)
        # Rack-butter both ways; held, the hand-mug pair too (not the arm's).
        assert planner._drop_static_pairs() == len(pairs) - len(kept)
        assert geometry.collisionPairs == [pairs[i] for i in kept]
