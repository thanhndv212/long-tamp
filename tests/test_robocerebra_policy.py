"""RoboCerebra HPP policy pieces that need no HPP or env server (#159)."""

import os

import numpy as np
import pytest

from long_tamp.eval.robocerebra_plan import (
    PlanningError,
    held_object,
    object_corners,
    place_heights,
    place_orientations,
    place_positions,
    square,
)
from long_tamp.eval.robocerebra_policy import (
    HppPolicy,
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
    assert len(options) == 17 and np.allclose(options["held"], np.eye(3))
    assert np.allclose(options["-y/0"] @ tcp[:3, 2], [0.0, -1.0, 0.0])
    for R in options.values():
        assert np.allclose(R @ R.T, np.eye(3)) and np.linalg.det(R) == pytest.approx(
            1.0
        )


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
    for spec in (["open", "drawer_1"], ["pick", "a", "b"], ["in", "a"]):
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
    def no_region(env, goal, segment, record):
        raise UnsupportedGoal(f"no region {goal.region!r} in the scene")

    policy = HppPolicy()
    monkeypatch.setattr(policy, "_plan", no_region)
    info = {"goal": {"pan": [["on", "pan", "rack_1"]]}, "goal_steps": {"pan": [0]}}
    controller = policy(None, info, 0, 150)
    assert controller.info["status"] == "unsupported"
    assert controller.info["error"] == "no region 'rack_1' in the scene"


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
    pose[:3, :3] = [[np.cos(tilt), 0, np.sin(tilt)], [0, 1, 0], [-np.sin(tilt), 0, np.cos(tilt)]]
    pose[:3, 3] = [0.05, 0.0, 0.03]
    wall = Box(name="wall", pose=pose, size=(0.003, 0.02, 0.04))
    samples = list(_rim_samples(GraspPlanner(PANDA_HAND, GraspPlannerParams()), [wall]))
    sources = [source for _, source in samples]
    assert sources.count("top-rim") == sources.count("top-rim flipped") >= 1
    for grasp, _ in samples:
        assert np.allclose(grasp[:3, 0], [0, 0, -1])  # straight down
        assert abs(grasp[2, 1]) < 1e-9  # the pads close horizontally

