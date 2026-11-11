"""The planning helpers that used to do nothing (#229).

``freeze_joints_by_substrings`` needs pinocchio only. The scene helpers are
checked on mocks for their arguments and, where HPP is installed, on the TWIN
lift-ball scene for what collision checking then sees.
"""

import logging
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from long_tamp.planning import freeze_joints_by_substrings
from long_tamp.planning.scene import SceneBuilder

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

_TWIN_SCRIPT_DIR = Path(__file__).resolve().parent.parent / "script" / "twin"
_LOGGER = "long_tamp.planning.scene"


# --- freeze_joints_by_substrings -------------------------------------------


@pytest.fixture(scope="module")
def two_arms():
    """Two one-joint arms and a free-flying ball: nq = 1 + 1 + 7."""
    pin = pytest.importorskip("pinocchio")
    model = pin.Model()
    here = pin.SE3.Identity()
    model.addJoint(0, pin.JointModelRX(), here, "left/joint1")
    model.addJoint(0, pin.JointModelRY(), here, "right/joint1")
    model.addJoint(0, pin.JointModelFreeFlyer(), here, "ball/root_joint")
    return model


def test_matching_joints_take_the_reference_value(two_arms):
    q = [0.1, 0.2, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    q_ref = [9.0, 8.0, 7.0, 7.0, 7.0, 0.0, 1.0, 0.0, 0.0]
    frozen = freeze_joints_by_substrings(two_arms, q, q_ref, ["right/"])
    assert frozen == [0.1, 8.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    assert q[1] == 0.2  # the input is left alone


def test_a_free_flyer_is_frozen_whole(two_arms):
    q = [0.1, 0.2, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    q_ref = [9.0, 8.0, 7.0, 7.0, 7.0, 0.0, 1.0, 0.0, 0.0]
    frozen = freeze_joints_by_substrings(two_arms, q, q_ref, ["ball"])
    assert frozen == [0.1, 0.2, 7.0, 7.0, 7.0, 0.0, 1.0, 0.0, 0.0]


def test_no_match_and_no_substring_change_nothing(two_arms):
    q, q_ref = list(range(9)), [0.0] * 9
    assert freeze_joints_by_substrings(two_arms, q, q_ref, ["elbow"]) == q
    assert freeze_joints_by_substrings(two_arms, q, q_ref, []) == q


def test_the_device_is_accepted_as_well_as_its_model(two_arms):
    device = MagicMock()
    device.model.return_value = two_arms
    q, q_ref = [0.0] * 9, [1.0] * 9
    frozen = freeze_joints_by_substrings(device, q, q_ref, ["left"])
    assert frozen == [1.0] + [0.0] * 8


def test_a_configuration_of_the_wrong_length_is_refused(two_arms):
    with pytest.raises(ValueError, match="9"):
        freeze_joints_by_substrings(two_arms, [0.0] * 8, [0.0] * 9, ["left"])


# --- the collision-pair flags ----------------------------------------------


def _scene_builder():
    builder = object.__new__(SceneBuilder)  # no planner behind it
    builder.backend = "pyhpp"
    builder.planner = MagicMock()
    return builder


@pytest.mark.parametrize(
    "call",
    [
        lambda b, **kw: b.disable_collision_pair("ground/base", "arm/joint1", **kw),
        lambda b, **kw: b.disable_collisions_between_subtrees(
            "arm/joint1", "ground/root", **kw
        ),
    ],
    ids=["pair", "subtrees"],
)
def test_remove_collision_false_removes_nothing(call, caplog):
    builder = _scene_builder()
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        assert call(builder, remove_collision=False) is builder
        assert not caplog.records
        # Asking for the distance check alone cannot be honoured: say so.
        call(builder, remove_collision=False, remove_distance=True)
    assert "remove_distance" in caplog.records[0].getMessage()
    builder.planner.device.geomModel.assert_not_called()


# --- move_obstacle ----------------------------------------------------------


def test_move_obstacle_hands_hpp_the_root_pose():
    builder = _scene_builder()
    device = builder.planner.device
    device.contactSurfaceNames.return_value = ["ball/bottom"]
    assert builder.move_obstacle("ground", [0.1, 0.2, 0.3], [0, 0, 0, 1]) is builder
    (name, pose), _ = device.setRobotRootPosition.call_args
    assert name == "ground"
    np.testing.assert_allclose(pose.translation, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(pose.rotation, np.eye(3))


def test_move_obstacle_refuses_an_unknown_name():
    builder = _scene_builder()
    builder.planner.device.setRobotRootPosition.side_effect = ValueError("no frame")
    with pytest.raises(ValueError, match="'shelf'"):
        builder.move_obstacle("shelf", [0, 0, 0], [0, 0, 0, 1])


def test_move_obstacle_says_which_contact_surfaces_stay_behind(caplog):
    builder = _scene_builder()
    builder.planner.device.contactSurfaceNames.return_value = [
        "table/top",
        "ball/bottom",
    ]
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        builder.move_obstacle("table", [0, 0, 0.1], [0, 0, 0, 1])
    (record,) = caplog.records
    assert "table/top" in record.getMessage()
    assert "ball/bottom" not in record.getMessage()


# --- on a real scene ---------------------------------------------------------


@pytest.fixture(scope="module")
def twin():
    """The TWIN scene with its graph, and collision checking switched on."""
    if not HAS_PYHPP:
        pytest.skip("PyHPP backend not available")
    if str(_TWIN_SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(_TWIN_SCRIPT_DIR))
    import task_lift_ball  # noqa: PLC0415 -- path-dependent import

    task = task_lift_ball.LiftBallTask(backend="pyhpp")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
        )
    except Exception as exc:
        pytest.skip(f"TWIN scene could not be set up in this environment: {exc}")
    # A problem checks nothing about a configuration until it is told to.
    task.planner.problem.addConfigValidation("CollisionValidation")
    return task


_HOME = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
_RAISED = ([0.0, 0.0, 0.6], [0.0, 0.0, 0.0, 1.0])  # the ground, through the arms


def _valid(task, q):
    return bool(task.planner.problem.isConfigValid(np.asarray(q, dtype=float))[0])


def test_a_moved_ground_is_seen_for_the_same_configuration(twin):
    # The same configuration each time: the case a placement written by
    # hand gets wrong, HPP reusing what it computed for that configuration.
    seen = []
    try:
        for position, orientation in (_RAISED, _HOME, _RAISED, _HOME):
            twin.scene_builder.move_obstacle("ground", position, orientation)
            seen.append(_valid(twin, twin.q_init))
    finally:
        twin.scene_builder.move_obstacle("ground", *_HOME)
    assert seen == [False, True, False, True]


def test_a_moved_ground_is_seen_by_the_paths_of_a_graph_already_built(twin):
    backend = twin.planner
    model = backend.device.model()
    edge = next(
        name
        for name in backend.graph.getTransitionNames()
        if name.startswith("Loop") and name.endswith("| f")
    )
    joint = next(
        i
        for i, name in enumerate(model.names)
        if "panda_left" in name and name.endswith("joint2")
    )
    q1 = np.asarray(twin.q_init, dtype=float)
    q2 = q1.copy()
    q2[model.joints[joint].idx_q] += 0.6
    planner = backend.ensure_transition_planner()
    backend._configure_transition_planner_for_edge(
        planner, backend._resolve_transition(edge)
    )
    seen = []
    try:
        for position, orientation in (_HOME, _RAISED, _HOME):
            twin.scene_builder.move_obstacle("ground", position, orientation)
            seen.append(bool(planner.directPath(q1, q2, True)[0]))
    finally:
        twin.scene_builder.move_obstacle("ground", *_HOME)
    assert seen == [True, False, True]


def test_an_unknown_obstacle_is_refused_by_hpp(twin):
    with pytest.raises(ValueError, match="'shelf'"):
        twin.scene_builder.move_obstacle("shelf", *_HOME)


def test_frozen_joints_follow_the_reference_on_the_real_robot(twin):
    device = twin.planner.device
    model = device.model()
    q_ref = list(twin.q_init)
    q = [value + 0.05 for value in q_ref]
    frozen = freeze_joints_by_substrings(device, q, q_ref, ["panda_right"])
    for joint_id in range(1, model.njoints):
        joint = model.joints[joint_id]
        values = slice(joint.idx_q, joint.idx_q + joint.nq)
        expected = q_ref if "panda_right" in model.names[joint_id] else q
        assert frozen[values] == expected[values], model.names[joint_id]


def _ground_pairs(task, joint_name):
    device = task.planner.device
    geometry = device.geomModel()
    joint = device.model().getJointId(joint_name)
    ground = {
        i
        for i, g in enumerate(geometry.geometryObjects)
        if g.name.startswith("ground/")
    }
    link = {i for i, g in enumerate(geometry.geometryObjects) if g.parentJoint == joint}
    return [
        pair
        for pair in geometry.collisionPairs
        if (pair.first in ground and pair.second in link)
        or (pair.first in link and pair.second in ground)
    ]


def test_remove_collision_decides_whether_pairs_leave_the_real_scene(twin):
    joint = "panda_left/panda_joint3"
    obstacle = "ground/ground_base"
    before = _ground_pairs(twin, joint)
    assert before  # the scene checks this link against the ground
    twin.scene_builder.disable_collision_pair(obstacle, joint, remove_collision=False)
    assert len(_ground_pairs(twin, joint)) == len(before)
    geometry = twin.planner.device.geomModel()
    try:
        twin.scene_builder.disable_collision_pair(obstacle, joint)
        assert not _ground_pairs(twin, joint)
    finally:
        for pair in before:
            if not geometry.existCollisionPair(pair):
                geometry.addCollisionPair(pair)


def test_build_graph_for_task_builds_the_task_s_graph(twin):
    # After the tests above: this replaces the graph the fixture was set up with.
    builder = twin.graph_builder
    previous = builder.graph
    builder.build_graph_for_task(twin.task_config)
    assert builder.graph is not previous
    states = set(builder.graph.getStateNames())
    assert "free" in states
    assert {s for s in states if " grasps " in s and " : " not in s} == {
        "panda_left/gripper grasps ball/handle",
        "panda_right/gripper grasps ball/handle2",
    }
    with pytest.raises(ValueError, match="mode"):
        builder.build_graph_for_task(twin.task_config, mode="neither")
