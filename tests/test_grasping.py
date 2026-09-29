"""Grasp planner: geometry queries, gripper models, planning, closures.

Pure numpy except the calibration test (pinocchio, a core dependency). The
screw-assembly and IKEA scenes are read from ``script/`` as real objects.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from long_tamp.grasping import (
    PANDA_HAND,
    ROBOTIQ_2F85,
    Box,
    Cylinder,
    FingerClosureTable,
    GraspableObject,
    GraspPlanner,
    GraspPlannerParams,
    ParallelGripperModel,
    Sphere,
    apply_joint_values,
    load_srdf_frames,
    load_urdf_primitives,
)
from long_tamp.grasping.geometry import (
    line_intervals,
    line_intervals_batch,
    matrix_to_quat_xyzw,
    obb_overlap,
    pose,
    quat_xyzw_to_matrix,
    rpy_matrix,
)

ROOT = Path(__file__).resolve().parents[1]
SCREW = ROOT / "script" / "screw_assembly"
IKEA = ROOT / "script" / "ikea_table_prototype" / "generated"
DOWN = quat_xyzw_to_matrix([0, 0.7071068, 0, 0.7071068])  # +X -> world -Z


def top_down(z: float, x: float = 0.0, y: float = 0.0) -> np.ndarray:
    """Canonical grasp pose above the origin, approaching -Z, closing along Y."""
    return pose(DOWN, (x, y, z))


# -- geometry ------------------------------------------------------------------


class TestGeometry:
    def test_box_interval(self):
        box = Box(size=(0.1, 0.04, 0.02))
        assert line_intervals([box], [0, -1, 0], [0, 1, 0]) == [
            pytest.approx((0.98, 1.02))
        ]
        assert line_intervals([box], [0, -1, 0.02], [0, 1, 0]) == []

    def test_cylinder_and_sphere_intervals(self):
        cyl = Cylinder(
            pose=pose(rpy_matrix(0, math.pi / 2, 0)), radius=0.02, length=0.1
        )
        # axis along X after the rotation: a Y line through it crosses the diameter
        (iv,) = line_intervals([cyl], [0.03, -1, 0], [0, 1, 0])
        assert iv == pytest.approx((0.98, 1.02))
        assert line_intervals([cyl], [0.06, -1, 0], [0, 1, 0]) == []
        (iv,) = line_intervals([Sphere(radius=0.025)], [0, -1, 0], [0, 1, 0])
        assert iv == pytest.approx((0.975, 1.025))

    def test_batch_matches_scalar(self):
        rng = np.random.default_rng(0)
        prims = [
            Box(
                pose=pose(rpy_matrix(0.3, -0.2, 0.9), (0.01, 0, 0)),
                size=(0.05, 0.03, 0.08),
            ),
            Cylinder(
                pose=pose(rpy_matrix(1.0, 0.2, 0), (0, 0.02, 0)),
                radius=0.015,
                length=0.06,
            ),
            Sphere(pose=pose(None, (0, 0, 0.03)), radius=0.02),
        ]
        points = rng.uniform(-0.05, 0.05, (200, 3))
        d = np.array([0.2, 1.0, -0.3])
        T0, T1 = line_intervals_batch(prims, points, d)
        for k, prim in enumerate(prims):
            for i, p in enumerate(points):
                iv = line_intervals([prim], p, d)
                if iv:
                    assert (T0[k, i], T1[k, i]) == pytest.approx(iv[0])
                else:
                    assert np.isnan(T0[k, i])

    def test_obb_overlap(self):
        h = np.array([0.05, 0.05, 0.05])
        assert obb_overlap(np.eye(4), h, pose(None, (0.09, 0, 0)), h)
        assert not obb_overlap(np.eye(4), h, pose(None, (0.11, 0, 0)), h)
        # rotated 45 deg: the corner reaches 0.0707
        R = rpy_matrix(0, 0, math.pi / 4)
        assert obb_overlap(np.eye(4), h, pose(R, (0.115, 0, 0)), h)
        assert not obb_overlap(np.eye(4), h, pose(R, (0.125, 0, 0)), h)

    def test_quaternion_roundtrip(self):
        for rpy in [(0, 0, 0), (0.3, -1.2, 2.0), (math.pi, 0, 0), (0, math.pi, 0.1)]:
            R = rpy_matrix(*rpy)
            np.testing.assert_allclose(
                quat_xyzw_to_matrix(matrix_to_quat_xyzw(R)), R, atol=1e-9
            )

    def test_urdf_fixed_joint_is_composed(self):
        # part1's tab hangs off a fixed joint; its box must land on top of the plate
        prims = load_urdf_primitives(SCREW / "generated" / "part1.urdf")
        tab = next(p for p in prims if p.name.startswith("tab"))
        np.testing.assert_allclose(tab.center, [-0.13, 0.0, 0.04], atol=1e-9)

    def test_srdf_both_position_forms(self):
        frames = load_srdf_frames(SCREW / "generated" / "driver.srdf")  # xyz/xyzw
        assert frames["h_grip"].kind == "handle" and frames["tip"].kind == "gripper"
        np.testing.assert_allclose(frames["h_grip"].pose[:3, 3], [-0.022, 0, 0.085])
        legacy = load_srdf_frames(IKEA / "ur10_robotiq.srdf")[
            "gripper"
        ]  # "x y z w x y z"
        np.testing.assert_allclose(legacy.pose[:3, 0], [0, 0, 1], atol=1e-6)


# -- gripper model ---------------------------------------------------------------


class TestGripperModel:
    def test_robotiq_width_roundtrip(self):
        g = ROBOTIQ_2F85
        assert g.max_width == pytest.approx(0.0849, abs=1e-3)
        for w in (0.01, 0.03, 0.038, 0.07):
            assert g.width_at(g.q_for_width(w)) == pytest.approx(w, abs=1e-6)
        assert g.q_for_width(1.0) == pytest.approx(g.q_open)  # clamped

    def test_joint_values_follow_the_mimic_multipliers(self):
        values = ROBOTIQ_2F85.joint_values(0.5, "ur10_right")
        assert values["ur10_right/robotiq_85_left_knuckle_joint"] == 0.5
        assert values["ur10_right/robotiq_85_right_knuckle_joint"] == -0.5
        assert values["ur10_right/robotiq_85_left_finger_tip_joint"] == -0.5
        assert len(values) == 6
        assert PANDA_HAND.joint_values(0.05) == {
            "panda_finger_joint1": 0.025,
            "panda_finger_joint2": 0.025,
        }

    def test_frame_rotations_are_rotations(self):
        for g in (ROBOTIQ_2F85, PANDA_HAND):
            R = g.frame_rotation
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
            assert np.linalg.det(R) == pytest.approx(1.0)

    def test_rejects_bad_stroke(self):
        with pytest.raises(ValueError):
            ParallelGripperModel(
                "x",
                {},
                ((0.0, 0.1, 0.0),),
                0.01,
                0.01,
                0.01,
                0.01,
                -0.05,
                (0.1, 0.1, 0.1),
            )
        with pytest.raises(ValueError):
            ParallelGripperModel(
                "x",
                {},
                ((0.0, 0.1, 0.0), (0.5, 0.2, 0.0), (1.0, 0.0, 0.0)),
                0.01,
                0.01,
                0.01,
                0.01,
                -0.05,
                (0.1, 0.1, 0.1),
            )

    def test_calibration_reproduces_the_preset(self):
        pytest.importorskip("pinocchio")
        from long_tamp.grasping import calibrate_from_urdf

        stroke, _ = calibrate_from_urdf(
            IKEA / "ur10_robotiq.urdf",
            tcp_frame="gripper_tcp",
            pad_frames=("robotiq_85_left_finger_pad", "robotiq_85_right_finger_pad"),
            joints=ROBOTIQ_2F85.joints,
            q_values=[row[0] for row in ROBOTIQ_2F85.stroke],
        )
        np.testing.assert_allclose(stroke, ROBOTIQ_2F85.stroke, atol=2e-5)


# -- evaluation ------------------------------------------------------------------


class TestEvaluate:
    planner = GraspPlanner(ROBOTIQ_2F85)
    block = [Box(size=(0.04, 0.03, 0.06), name="block")]

    def test_closes_to_the_object_width(self):
        ev = self.planner.evaluate(top_down(0.02), self.block)
        assert ev.feasible, ev.reasons
        assert ev.contact_width == pytest.approx(0.03, abs=1e-6)
        assert ev.width == pytest.approx(0.03 - ROBOTIQ_2F85.default_squeeze)
        assert ev.q == pytest.approx(ROBOTIQ_2F85.q_for_width(ev.width))
        assert ev.offset == pytest.approx(0.0, abs=1e-9)

    def test_off_centre_closes_on_the_wider_side(self):
        # TCP 6 mm off the block's centre along the closing axis: the fingers
        # close symmetrically, so the far face sets the width
        ev = self.planner.evaluate(top_down(0.02, y=0.006), self.block)
        assert ev.contact_width == pytest.approx(0.03 + 2 * 0.006, abs=1e-6)
        assert ev.offset == pytest.approx(-0.006, abs=1e-6)
        assert not ev.feasible and any("off-centre" in r for r in ev.reasons)

    def test_too_wide_and_empty(self):
        wide = [Box(size=(0.04, 0.12, 0.06))]
        ev = self.planner.evaluate(top_down(0.02), wide)
        assert not ev.feasible
        ev = self.planner.evaluate(top_down(0.5), self.block)
        assert ev.reasons == ["fingers close on nothing"]

    def test_palm_collision(self):
        # pads centred 1 cm below the top of a tall block: fine; 7 cm deep,
        # the knuckles and palm hit it
        tall = [Box(size=(0.04, 0.03, 0.30), name="tall")]
        pad = ROBOTIQ_2F85.pad_x_for_width(0.03)
        assert self.planner.evaluate(top_down(0.14 + pad), tall).feasible
        ev = self.planner.evaluate(top_down(0.08 + pad), tall)
        assert not ev.feasible and ev.collisions

    def test_obstacle_is_checked_but_not_closed_on(self):
        # the block stands on a table top at z = -0.03; pads pushed down to
        # the block's bottom put the fingertips into the table
        table = [Box(pose=pose(None, (0, 0, -0.04)), size=(1, 1, 0.02), name="table")]
        pad = ROBOTIQ_2F85.pad_x_for_width(0.03)
        high = self.planner.evaluate(top_down(0.02 + pad), self.block, obstacles=table)
        assert high.feasible and high.contact_width == pytest.approx(0.03, abs=1e-6)
        low = self.planner.evaluate(top_down(-0.02 + pad), self.block, obstacles=table)
        assert any("table" in c for c in low.collisions)


# -- planning --------------------------------------------------------------------


class TestPlan:
    def test_box_candidates_are_feasible_and_ranked(self):
        cands = GraspPlanner(ROBOTIQ_2F85).plan(
            [Box(size=(0.03, 0.03, 0.2), name="leg")], max_candidates=None
        )
        assert cands and all(c.evaluation.feasible for c in cands)
        assert [c.score for c in cands] == sorted(
            (c.score for c in cands), reverse=True
        )
        for c in cands:
            assert c.evaluation.contact_width == pytest.approx(0.03, abs=1e-6)
            # closing axis across a 30 mm side, never along the 200 mm one
            assert abs(c.closing[2]) < 1e-9

    def test_preferred_approach_wins(self):
        params = GraspPlannerParams(preferred_approach=(0, 0, -1))
        best = GraspPlanner(ROBOTIQ_2F85, params).plan([Box(size=(0.04, 0.03, 0.06))])[
            0
        ]
        np.testing.assert_allclose(best.approach, [0, 0, -1], atol=1e-9)

    def test_cylinder_and_sphere(self):
        cands = GraspPlanner(ROBOTIQ_2F85).plan(
            [Cylinder(radius=0.02, length=0.12, name="c")], max_candidates=None
        )
        assert any(c.source.startswith("c: side") for c in cands)
        assert all(
            c.evaluation.contact_width == pytest.approx(0.04, abs=1e-3) for c in cands
        )
        cands = GraspPlanner(PANDA_HAND).plan([Sphere(radius=0.025)])
        assert cands and cands[0].evaluation.contact_width == pytest.approx(
            0.05, abs=1e-6
        )

    def test_nothing_fits(self):
        big = [Box(size=(0.2, 0.2, 0.2))]
        assert GraspPlanner(ROBOTIQ_2F85).plan(big) == []
        rejected = GraspPlanner(ROBOTIQ_2F85).plan(big, feasible_only=False)
        assert rejected == []  # no face pair fits the stroke, so nothing is sampled

    def test_srdf_handle_roundtrip(self, tmp_path):
        planner = GraspPlanner(PANDA_HAND)
        cand = planner.plan([Box(size=(0.04, 0.03, 0.06))])[0]
        srdf = tmp_path / "o.srdf"
        srdf.write_text(f'<robot name="o">\n{cand.srdf_handle("h")}</robot>\n')
        frame = load_srdf_frames(srdf)["h"]
        np.testing.assert_allclose(frame.pose, cand.handle_pose, atol=1e-6)
        # the handle is a gripper-frame pose: it maps back to the same grasp
        np.testing.assert_allclose(
            planner.canonical_from_handle(frame.pose), cand.pose, atol=1e-6
        )
        again = planner.evaluate_handle(frame.pose, [Box(size=(0.04, 0.03, 0.06))])
        assert again.feasible and again.contact_width == pytest.approx(
            cand.evaluation.contact_width
        )


# -- real objects ----------------------------------------------------------------


@pytest.fixture(scope="module")
def closures():
    return FingerClosureTable.from_task_yaml(
        SCREW / "config" / "screw_assembly_config.yaml",
        {"ur10_left/gripper": ROBOTIQ_2F85, "ur10_right/gripper": ROBOTIQ_2F85},
    )


class TestRealScenes:
    def test_screw_assembly_handles(self, closures):
        drill = closures.evaluation("ur10_right/gripper", "driver/h_grip")
        assert drill.feasible, drill.reasons
        assert drill.contact_width == pytest.approx(0.038, abs=1e-6)  # the handle proxy
        assert drill.q == pytest.approx(0.484, abs=2e-3)
        tab = closures.evaluation("ur10_left/gripper", "part1/h_grasp")
        assert tab.feasible and tab.contact_width == pytest.approx(0.03, abs=1e-6)
        # the table covers exactly the arm grippers' valid pairs
        assert {g for g, _ in closures._cache} == {
            "ur10_left/gripper",
            "ur10_right/gripper",
        }
        assert not closures.has("driver/tip") and not closures.has("fixtures/clamp1")

    def test_closed_values_apply_to_a_configuration(self, closures):
        values = closures.closed_values("ur10_right/gripper", "driver/h_grip")
        rank = {name: i for i, name in enumerate(values)}
        q = apply_joint_values(np.zeros(len(values)), rank, values)
        assert q[rank["ur10_right/robotiq_85_left_knuckle_joint"]] == pytest.approx(
            0.484, abs=2e-3
        )
        assert set(closures.open_values("ur10_right/gripper").values()) == {0.0}

    def test_hand_written_handles_are_among_the_planned(self):
        # the planner rediscovers the IKEA leg's handle and the drill's grip
        leg = GraspableObject.from_urdf("leg1", IKEA / "leg1.urdf", IKEA / "leg1.srdf")
        cands = GraspPlanner(ROBOTIQ_2F85).plan(leg.primitives)
        assert cands[0].evaluation.contact_width == pytest.approx(0.03, abs=1e-6)
        drill = GraspableObject.from_urdf(
            "driver",
            SCREW / "generated" / "driver.urdf",
            SCREW / "generated" / "driver.srdf",
        )
        best = GraspPlanner(ROBOTIQ_2F85).plan(drill.primitives)[0]
        h_grip = GraspPlanner(ROBOTIQ_2F85).canonical_from_handle(
            drill.handle_pose("h_grip")
        )
        assert best.source.startswith("handle")
        np.testing.assert_allclose(best.approach, h_grip[:3, 0], atol=1e-9)
        assert abs(best.closing @ h_grip[:3, 1]) == pytest.approx(1.0)


# -- behaviour-tree capabilities ---------------------------------------------------


class TestCapabilities:
    def _registry(self, closures, actuated):
        from long_tamp.tasks.task_planning import CapabilityRegistry
        from long_tamp.tasks.task_planning.grasp_capability import (
            register_grasp_capabilities,
        )

        registry = CapabilityRegistry()
        register_grasp_capabilities(
            registry, closures, actuate=lambda g, v: actuated.append((g, v))
        )
        return registry

    def test_capabilities(self, closures):
        actuated = []
        registry = self._registry(closures, actuated)
        plan = registry.implementation("plan_grasp")(
            {"gripper": "ur10_right/gripper", "object": "driver"}
        )
        assert plan["count"] >= 1 and plan["candidates"][0]["feasible"]
        assert registry.implementation("grasp_feasible")(
            {"gripper": "ur10_right/gripper", "handle": "driver/h_grip"}
        )
        closed = registry.implementation("close_gripper")(
            {"gripper": "ur10_right/gripper", "handle": "driver/h_grip"}
        )
        assert closed["joint_values"][
            "ur10_right/robotiq_85_left_knuckle_joint"
        ] == pytest.approx(0.484, abs=2e-3)
        registry.implementation("open_gripper")({"gripper": "ur10_right/gripper"})
        assert [g for g, _ in actuated] == ["ur10_right/gripper"] * 2
        assert actuated[1][1]["ur10_right/robotiq_85_left_knuckle_joint"] == 0.0

    def test_close_on_an_unusable_handle_fails(self, closures):
        registry = self._registry(closures, [])
        # the dock handle is not a Robotiq grasp: the capability refuses
        with pytest.raises(RuntimeError):
            registry.implementation("close_gripper")(
                {"gripper": "ur10_right/gripper", "handle": "driver/h_rack"}
            )
        assert not registry.implementation("grasp_feasible")(
            {"gripper": "ur10_right/gripper", "handle": "driver/h_rack"}
        )

    def test_runs_in_a_task_plan(self, closures):
        import json

        from long_tamp.tasks.task_planning import TaskPlan, TaskPlanningSession

        actuated = []
        registry = self._registry(closures, actuated)
        document = {
            "schema_version": "1.0",
            "mission_id": "grasp-demo",
            "scene": {"id": "screw_assembly", "config_size": 1},
            "provenance": {"kind": "human", "generator": "test"},
            "root": {
                "type": "sequence",
                "id": "root",
                "children": [
                    {
                        "type": "operation",
                        "id": "plan",
                        "capability": "plan_grasp",
                        "parameters": {
                            "gripper": "ur10_right/gripper",
                            "object": "driver",
                        },
                    },
                    {
                        "type": "operation",
                        "id": "close",
                        "capability": "close_gripper",
                        "parameters": {
                            "gripper": "ur10_right/gripper",
                            "handle": "driver/h_grip",
                        },
                    },
                ],
            },
        }
        session = TaskPlanningSession(TaskPlan.from_dict(document, registry), registry)
        for step in ("plan", "close"):
            assert json.loads(session.execute_step(step))["status"] == "success"
        assert len(actuated) == 1


def test_planned_grasps_hold_on_the_real_robotiq_meshes():
    """The box hull is conservative: every accepted grasp, checked with the
    2F-85's actual collision meshes (pinocchio + coal), touches with both
    pads and collides nowhere else."""
    pytest.importorskip("coal")
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "validate_closure", ROOT / "script" / "grasp_planning" / "validate_closure.py"
    )
    validate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validate)
    hand = validate.HandChecker()
    g = ROBOTIQ_2F85
    for urdf in (
        SCREW / "generated" / "driver.urdf",
        SCREW / "generated" / "part1.urdf",
        IKEA / "leg1.urdf",
    ):
        prims = load_urdf_primitives(urdf)
        for cand in GraspPlanner(g).plan(prims, max_candidates=12):
            ev = cand.evaluation
            pads, colliding = hand.check(
                cand.handle_pose,
                prims,
                g.joint_values(ev.q),
                g.joint_values(g.q_for_width(ev.contact_width)),
            )
            assert colliding == [], (urdf.name, cand.source, colliding)
            for d in pads.values():  # commanded squeeze: each pad 1 mm in
                assert d == pytest.approx(-g.default_squeeze / 2, abs=5e-4)
