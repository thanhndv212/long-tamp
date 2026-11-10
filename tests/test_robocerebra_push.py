"""RoboCerebra edge pinch of a flat object, without HPP or env server (#188)."""

import numpy as np
import pytest

from long_tamp.eval.robocerebra_plan import PlanningError, SegmentPlan
from long_tamp.eval.robocerebra_policy import HppPolicy
from long_tamp.eval.robocerebra_push import (
    INSET,
    REACHED,
    TIP,
    EdgePinch,
    EdgePinchController,
    edge_pinch,
)
from long_tamp.eval.robocerebra_tracker import POS_SCALE, ROT_SCALE
from long_tamp.grasping.geometry import matrix_to_quat_xyzw

# A plate: a 14 cm square slab, 2 cm thick (radius: its corner, 9.9 cm).
PLATE = """<robot name="plate"><link name="base_link">
  <collision><origin xyz="0 0 0.008"/><geometry><box size="0.14 0.14 0.02"/></geometry></collision>
</link></robot>"""
POT = """<robot name="pot"><link name="base_link">
  <collision><origin xyz="0 0 0.05"/><geometry><box size="0.06 0.06 0.1"/></geometry></collision>
</link></robot>"""
# A handle 10 cm up, reaching out 8 cm: a flat object passes under it.
HANDLE = """<robot name="pot"><link name="base_link">
  <collision><origin xyz="0 0.08 0.1"/><geometry><box size="0.02 0.08 0.02"/></geometry></collision>
</link></robot>"""
TABLE = [[0.35, -0.7, 0.42], [0.35, 0.7, 0.42], [-0.35, 0.7, 0.42], [-0.35, -0.7, 0.42]]


class Scene:
    """An exported scene: the table region, the robot at -x, objects."""

    def __init__(self, tmp_path, objects):
        self.paths, self.poses = {}, {}
        for name, (urdf, xyz) in objects.items():
            path = tmp_path / f"{name}.urdf"
            path.write_text(urdf)
            self.paths[name] = {"urdf": path}
            self.poses[name] = [*xyz, 0.0, 0.0, 0.0, 1.0]
        self.root = tmp_path
        self.manifest = {
            "regions": [{"region": "table_region", "points": TABLE}],
            "robot": {"pose": [-0.6, 0.0, 0.41, 0, 0, 0, 1]},
            "objects": {name: {} for name in objects},
        }

    def file_paths(self):
        return {"objects": self.paths}

    def object_config(self, name):
        return self.poses[name]


def test_edge_pinch_pushes_to_the_edge_nearest_the_robot(tmp_path):
    scene = Scene(tmp_path, {"plate_1": (PLATE, (0.0, 0.1, 0.422))})
    way = edge_pinch(scene, "plate_1")
    assert np.allclose(way.goal, [-0.35 + INSET, 0.1])
    assert np.allclose(way.normal, [-1, 0]) and np.allclose(way.tangent, [0, 1])
    assert np.allclose(way.frame(), np.eye(3))
    assert way.radius == pytest.approx(np.hypot(0.07, 0.07))
    assert way.top == pytest.approx(0.018) and way.table_z == pytest.approx(0.42)
    assert way.info["region"] == "table_region"


def test_edge_pinch_goes_round_what_stands_in_the_way(tmp_path):
    objects = {"plate_1": (PLATE, (0.0, 0.0, 0.422)), "pot_1": (POT, (-0.3, 0.0, 0.42))}
    way = edge_pinch(Scene(tmp_path, objects), "plate_1")
    # Straight on, or 7 cm aside, still hits the pot.
    assert abs(way.goal[1]) >= 0.14 - 1e-9
    # A handle above the plate's height is no obstacle to the push.
    objects["pot_1"] = (HANDLE, (-0.3, -0.08, 0.42))
    way = edge_pinch(Scene(tmp_path, objects), "plate_1")
    assert way.goal[1] == pytest.approx(0.0)


def test_edge_pinch_rejects_what_it_cannot_push(tmp_path):
    with pytest.raises(PlanningError, match="not flat"):
        edge_pinch(Scene(tmp_path, {"pot_1": (POT, (0.0, 0.0, 0.42))}), "pot_1")
    with pytest.raises(PlanningError, match="rests on no region"):
        edge_pinch(Scene(tmp_path, {"plate_1": (PLATE, (0.6, 0.0, 0.422))}), "plate_1")
    # Pots all along the edge: no way through.
    objects = {"plate_1": (PLATE, (0.0, 0.0, 0.422))}
    for i, y in enumerate(np.arange(-0.6, 0.61, 0.08)):
        objects[f"pot_{i}"] = (POT, (-0.3, float(y), 0.42))
    with pytest.raises(PlanningError, match="no clear way"):
        edge_pinch(Scene(tmp_path, objects), "plate_1")


def way_at(goal=(-0.325, 0.0)):
    return EdgePinch(
        "plate_1", np.array(goal), np.array([-1.0, 0.0]), 0.073, 0.016, 0.424
    )


def obs(hand, plate):
    return {
        "robot0_eef_pos": hand[:3, 3].copy(),
        "robot0_eef_quat": matrix_to_quat_xyzw(hand[:3, :3]),
        "plate_1_pos": np.asarray(plate, dtype=float),
    }


def down_at(x, y, z):
    T = np.diag([1.0, -1.0, -1.0, 1.0])
    T[:3, 3] = [x, y, z]
    return T


def test_a_stroke_pushes_from_behind_the_rim_at_fingertip_height():
    way = way_at()
    stages = way.stroke(obs(down_at(0.0, 0.0, 0.6), [0.0, 0.0, 0.426]))
    assert [s.name for s in stages] == ["close", "over", "down", "push", "up"]
    push = stages[3].tcp
    assert np.allclose(push[:, 2, 3], 0.424 + TIP)
    assert push[0, 0, 3] == pytest.approx(0.073 + 0.03)  # behind the far rim
    assert push[-1, 0, 3] == pytest.approx(-0.06 + 0.073 + 0.004)  # 6 cm on
    assert np.allclose(push[0, :3, 2], [0, 0, -1])  # pointing down
    assert all(s.gripper == 1.0 for s in stages)
    # At the goal, no more strokes.
    assert way.stroke(obs(down_at(0, 0, 0.6), [-0.325 + REACHED / 2, 0, 0.426])) is None


def test_the_pinch_closes_vertically_on_the_overhang():
    way = way_at()
    stages = way.pinch(obs(down_at(-0.25, 0.0, 0.5), [-0.325, 0.0, 0.426]))
    assert [s.name for s in stages] == ["open", "turn", "pre", "in", "grip", "lift"]
    grasp = stages[3].tcp[-1]
    assert grasp[0, 3] == pytest.approx(-0.325 - 0.064)  # off the table
    assert grasp[2, 3] == pytest.approx(0.426 + 0.016 - 0.0037)
    assert np.allclose(grasp[:3, 2], [0, -1, 0])  # approach along -y
    assert abs(grasp[2, 1]) > 0.9  # fingers close vertically
    assert stages[1].tcp[-1][1, 3] == pytest.approx(0.15)  # from +y
    assert stages[4].gripper == 1.0 and stages[-1].tcp[-1][2, 3] > grasp[2, 3] + 0.1


def turned(R, rotvec):
    """``R`` turned by the world-frame rotation vector ``rotvec`` (Rodrigues)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return R
    k = rotvec / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return (np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K) @ R


def run(controller, plate, follow, steps=3000):
    """The hand follows the actions; ``follow`` moves the plate."""
    hand = down_at(0.0, 0.0, 0.6)
    for _ in range(steps):
        action = controller(obs(hand, plate))
        if controller.done:
            return action
        hand = hand.copy()
        hand[:3, 3] += POS_SCALE * action[:3]
        hand[:3, :3] = turned(hand[:3, :3], ROT_SCALE * action[3:6])
        plate = follow(plate, hand)
    raise AssertionError("never done")


def test_controller_strokes_until_the_goal_then_pinches():
    def follow(plate, hand):
        # The fingers carry the plate once they reach its far rim.
        low = hand[2, 3] < 0.424 + TIP + 0.005
        if low and plate[0] < hand[0, 3] < plate[0] + 0.073 + 0.005:
            plate = plate.copy()
            plate[0] = hand[0, 3] - 0.073 - 0.004
        return plate

    controller = EdgePinchController(way_at(), info={"target": "plate_1"})
    action = run(controller, np.array([-0.1, 0.0, 0.426]), follow)
    assert not controller.stalled and controller.phase == "done"
    assert 4 <= controller.strokes <= 5  # 22.5 cm in 6 cm strokes
    assert controller.info["target_start"] == [-0.1, 0.0, 0.426]
    assert [e["stage"] for e in controller.info["pinch"]][-1] == "lift"
    assert action[6] == 1.0  # done, it holds the grip


def test_controller_gives_up_when_the_push_moves_nothing():
    controller = EdgePinchController(way_at(), max_strokes=2)
    run(controller, np.array([-0.1, 0.0, 0.426]), lambda plate, hand: plate)
    assert controller.stalled and controller.strokes == 2
    assert "did not reach" in controller.info["error"]


def test_policy_falls_back_on_an_edge_pinch_when_no_handle_plans(monkeypatch, tmp_path):
    import long_tamp.eval.robocerebra_plan as plan_module
    import long_tamp.eval.robocerebra_policy as policy_module

    class Env:
        def export_scene(self, out):
            return Scene(tmp_path, {"plate_1": (PLATE, (0.0, 0.0, 0.422))})

    def solve(scene, urdf, srdf, target, region, *rest):
        if region is None:
            raise PlanningError(f"no handle of {target} plans", {"handles_tried": 6})
        return SegmentPlan([], [], np.eye(4), {}), None

    monkeypatch.setattr(policy_module, "_solve", solve)
    monkeypatch.setattr(plan_module, "held_object", lambda manifest: None)
    monkeypatch.setattr(plan_module, "robot_files", lambda assets: ("u", "s"))
    info = {"goal": {"plate_1": [["pick", "plate_1"]]}, "goal_steps": {"plate_1": [0]}}
    policy = HppPolicy(work_dir=tmp_path, isolate=False)
    controller = policy(Env(), info, 0, 150)
    assert isinstance(controller, EdgePinchController)
    assert controller.info["status"] == "planned"
    assert controller.info["grasp"] == "edge pinch"
    assert controller.info["hpp"]["error"] == "no handle of plate_1 plans"
    # Not flat: the pick fails as HPP left it.
    monkeypatch.setattr(
        policy_module,
        "edge_pinch",
        lambda scene, target: (_ for _ in ()).throw(PlanningError("thick", {})),
    )
    controller = policy(Env(), info, 0, 150)
    assert controller.info["status"] == "planning failed"
    assert controller.info["error"] == "no handle of plate_1 plans"
