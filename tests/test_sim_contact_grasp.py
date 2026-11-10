"""Contact grasps in the MuJoCo backend (#71): the Robotiq 2F-85 holds a part
by friction through a carry move, on the screw-assembly scene. Grasp
verification and the gripper actuation interface (#127, ADR-0009).

No planner: ``ur10_left`` starts at its zero configuration with part 1 in
its open fingers, placed so the part's handle sits on the gripper frame (the
planner's grasp). Configurations are MuJoCo ``qpos`` (``to_qpos`` is the
identity). The carry path moves the arm with the part following the gripper,
as the planner's path would.
"""

import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from long_tamp.execution import (
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionStatus,
    run_command,
)
from long_tamp.grasping import ROBOTIQ_2F85, FingerClosureTable, GripperBinding
from long_tamp.sim import MuJoCoBackend, export_mjcf

SCREW_DIR = Path(__file__).resolve().parents[1] / "script" / "screw_assembly"
CONFIG = SCREW_DIR / "config" / "screw_assembly_config.yaml"
GRIPPER, HANDLE = "ur10_left/gripper", "part1/h_grasp"
FINGER = "ur10_left/robotiq_85_left_knuckle_joint"
PART = "part1/base_link"  # objects go by their root body
# The HPP gripper frame: gripper_tcp turned -90 deg about Y (see the SRDF).
TCP_TO_GRIPPER = np.array([math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0])
# Arm joints moved by the carry (rad): lift the shoulder, swing the base.
CARRY = {
    "ur10_left/shoulder_pan_joint": 0.6,
    "ur10_left/shoulder_lift_joint": -0.5,
    "ur10_left/wrist_1_joint": 0.8,
}


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def _matrix_quat(rot):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rot, float).reshape(9))
    return quat


class Scene:
    def __init__(self, tmp_path):
        self.export = export_mjcf(CONFIG, tmp_path / "mjcf")
        self.model = self.export.load()
        self.data = mujoco.MjData(self.model)
        table = FingerClosureTable.from_task_yaml(CONFIG, {GRIPPER: ROBOTIQ_2F85})
        self.closure = table.closed_values(GRIPPER, HANDLE)
        self.expected = table.expected_grasp(GRIPPER, HANDLE)
        handle = table.objects["part1"].handle_pose(HANDLE)
        # the part in the gripper frame: the inverse of the handle's pose
        self.part_pos = -handle[:3, :3].T @ handle[:3, 3]
        self.part_quat = _matrix_quat(handle[:3, :3].T)
        self.tcp = self.model.body("ur10_left/gripper_tcp").id
        self.part = self.model.jnt_qposadr[self.model.joint("part1/root_joint").id]

    def joint(self, name):
        return self.model.jnt_qposadr[self.model.joint(name).id]

    def with_part_in_hand(self, qpos):
        """``qpos`` with part 1 where the planner's grasp puts it."""
        qpos = np.array(qpos, float)
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)
        grip_quat = _quat_mul(self.data.xquat[self.tcp], TCP_TO_GRIPPER)
        grip_rot = np.zeros(9)
        mujoco.mju_quat2Mat(grip_rot, grip_quat)
        a = self.part
        qpos[a : a + 3] = (
            self.data.xpos[self.tcp] + grip_rot.reshape(3, 3) @ self.part_pos
        )
        qpos[a + 3 : a + 7] = _quat_mul(grip_quat, self.part_quat)
        return qpos

    def start(self):
        return self.with_part_in_hand(self.model.qpos0)

    def carried(self, start):
        end = start.copy()
        for name, delta in CARRY.items():
            end[self.joint(name)] += delta
        return self.with_part_in_hand(end)


class Carry:
    """The arm from ``a`` to ``b`` over ``T`` seconds (smooth ends), with the
    part following the gripper."""

    def __init__(self, scene, a, b, T):
        self.scene, self.a, self.b, self.T = scene, np.array(a), np.array(b), T

    def length(self):
        return self.T

    def eval(self, t):
        u = min(max(t / self.T, 0.0), 1.0)
        u = 3 * u * u - 2 * u**3
        return self.scene.with_part_in_hand(self.a + (self.b - self.a) * u), True


class Retreat:
    """The arm backs off while the part stays where it is (a release)."""

    def __init__(self, scene, a, T):
        self.scene, self.a, self.T = scene, np.array(a), T

    def length(self):
        return self.T

    def eval(self, t):
        u = min(max(t / self.T, 0.0), 1.0)
        q = self.a.copy()
        q[self.scene.joint("ur10_left/shoulder_lift_joint")] -= 0.3 * u
        return q, True


def run(backend, path):
    command = ExecutionCommand("step", duration=path.length(), payload=path)
    return run_command(backend, command, ExecutionPolicy(poll_interval=0))


def pads(arm):
    """Box pads on the 2F-85's pad frames (the fingertips' inner faces): 1 mm
    thick behind the face, 22 x 38 mm (ROBOTIQ_2F85's pad)."""
    half = (0.001, ROBOTIQ_2F85.pad_width / 2, ROBOTIQ_2F85.pad_length / 2)
    return {
        f"{arm}/robotiq_85_{side}_finger_pad": ((sign * 0.001, 0.0, 0.0), half)
        for side, sign in (("left", 1.0), ("right", -1.0))
    }


def make_backend(scene, grasp="contact", verify=False):
    verified = {}
    if verify:
        verified = {
            "expect": lambda obj, carrier: scene.expected if obj == PART else None,
            "hands": (GripperBinding(GRIPPER, ROBOTIQ_2F85),),
        }
    return MuJoCoBackend(
        scene.export,
        lambda q: np.asarray(q, float),
        speed=math.inf,
        grasp=grasp,
        fingers=(FINGER, "ur10_right/robotiq_85_left_knuckle_joint"),
        grip=lambda obj, carrier: scene.closure if obj == PART else None,
        pads={**pads("ur10_left"), **pads("ur10_right")},
        **verified,
    )


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    return Scene(tmp_path_factory.mktemp("contact"))


def test_a_contact_grasp_holds_a_part_through_a_carry_move(scene):
    backend = make_backend(scene)
    start = scene.start()
    backend.reset(start)
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.SUCCESS, result.message
    assert backend.carriers[PART].startswith("ur10_left/")
    # held by friction, not welded
    assert not backend.data.eq_active[backend._weld(PART)]
    m = result.metrics
    assert m["slip"] < 0.002, m
    assert m["object_drift"] < 0.003, m
    # the fingers closed on the part: at the planned closure (2 mm past
    # contact), give or take the pads' compliance under the squeeze
    finger = backend.data.qpos[scene.joint(FINGER)]
    assert finger == pytest.approx(scene.closure[FINGER], abs=0.02)


def test_a_released_part_stays_where_it_was_let_go(scene):
    backend = make_backend(scene)
    start = scene.start()
    backend.reset(start)
    end = scene.carried(start)
    assert run(backend, Carry(scene, start, end, 2.0)).status is ExecutionStatus.SUCCESS
    here = backend.data.xpos[backend.model.body(PART).id].copy()
    result = run(backend, Retreat(scene, end, 1.0))
    assert result.status is ExecutionStatus.SUCCESS, result.message
    assert backend.carriers[PART] == "world"
    assert "slip" not in result.metrics  # nothing gripped any more
    # welded to the world where the fingers let go, and the fingers opened
    moved = backend.data.xpos[backend.model.body(PART).id] - here
    assert np.linalg.norm(moved) < 0.005
    assert backend.data.qpos[scene.joint(FINGER)] < 0.05


def test_welds_remain_the_default(scene):
    backend = make_backend(scene, grasp="weld")
    start = scene.start()
    backend.reset(start)
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.SUCCESS
    assert backend.data.eq_active[backend._weld(PART)]
    assert "slip" not in result.metrics
    assert backend.data.qpos[scene.joint(FINGER)] < 0.05  # fingers stay open


def test_contact_grasps_need_fingers_and_grip(scene):
    with pytest.raises(ValueError):
        MuJoCoBackend(scene.export, lambda q: q, grasp="contact")


# ------------------------------------------------- grasp verification (#127)

HELD = "holds(ur10_left/gripper, part1)"
MISSED = "grasp_missed(ur10_left/gripper, part1)"


def removed(scene, qpos):
    """``qpos`` with part 1 moved 30 cm away (it was taken before the grasp)."""
    qpos = np.array(qpos, float)
    qpos[scene.part + 1] += 0.3
    return qpos


def test_a_contact_grasp_that_holds_reports_it(scene):
    backend = make_backend(scene, verify=True)
    start = scene.start()
    backend.reset(start)
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.SUCCESS, result.message
    assert result.facts == (HELD,)
    assert scene.expected.held(backend.gripper_state(GRIPPER))


def test_a_grasp_of_a_removed_part_misses_and_fails_the_step(scene):
    backend = make_backend(scene, verify=True)
    start = scene.start()
    backend.reset(removed(scene, start))
    away = backend.data.xpos[backend.model.body(PART).id].copy()
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.FAILURE
    assert result.facts == (MISSED,)
    assert "grasp missed: part1 not in ur10_left/gripper" in result.message
    # the fingers closed past where the part would have stopped them
    state = backend.gripper_state(GRIPPER)
    assert state.width < scene.expected.threshold
    # and the part stayed where it was
    here = backend.data.xpos[backend.model.body(PART).id]
    assert np.linalg.norm(here - away) < 1e-3


def test_a_missed_weld_grasp_reports_the_fact(scene):
    backend = make_backend(scene, grasp="weld", verify=True)
    start = scene.start()
    backend.reset(removed(scene, start))
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.FAILURE
    assert result.facts == (MISSED,)
    assert "from its planned grasp" in result.message


def test_without_expectations_nothing_is_verified(scene):
    backend = make_backend(scene)
    start = scene.start()
    backend.reset(start)
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.SUCCESS
    assert result.facts == ()


def test_the_gripper_opens_and_closes_between_commands(scene):
    backend = make_backend(scene, verify=True)
    with pytest.raises(RuntimeError, match="no state"):
        backend.close_gripper(GRIPPER)
    backend.reset(removed(scene, scene.start()))
    assert backend.gripper_state(GRIPPER).width == pytest.approx(
        ROBOTIQ_2F85.max_width, abs=0.002
    )
    backend.close_gripper(GRIPPER, width=0.04)
    assert backend.gripper_state(GRIPPER).width == pytest.approx(0.04, abs=0.002)
    backend.close_gripper(GRIPPER)
    closed = backend.gripper_state(GRIPPER)
    assert closed.width < 0.005 and closed.effort is not None
    backend.open_gripper(GRIPPER)
    assert backend.gripper_state(GRIPPER).width == pytest.approx(
        ROBOTIQ_2F85.max_width, abs=0.002
    )
    with pytest.raises(KeyError):
        backend.gripper_state("ur10_right/gripper")  # no hand bound


def test_opening_a_holding_gripper_lets_the_part_go_where_it_is(scene):
    backend = make_backend(scene, verify=True)
    start = scene.start()
    backend.reset(start)
    assert run(backend, Carry(scene, start, scene.carried(start), 2.0)).facts == (HELD,)
    with pytest.raises(RuntimeError, match="holds an object"):
        backend.close_gripper(GRIPPER)
    here = backend.data.xpos[backend.model.body(PART).id].copy()
    backend.open_gripper(GRIPPER)
    assert backend.data.eq_active[backend._weld(PART)]  # welded where it was
    moved = backend.data.xpos[backend.model.body(PART).id] - here
    assert np.linalg.norm(moved) < 0.005
    assert backend.gripper_state(GRIPPER).width > 0.07


# ------------------------------------------- repair from grasp_missed (#128)


def test_a_displaced_part_misses_and_is_picked_again_where_it_is(scene):
    """The injection behind ``--inject-miss``: a part moved after it was seen
    misses; once the hand is open, a grasp planned where the part now is
    (here: the part moved back under the hand) holds."""
    backend = make_backend(scene, verify=True)
    start = scene.start()
    backend.reset(start)
    seen = backend.data.xpos[backend.model.body(PART).id].copy()
    backend.displace("part1", (0.0, 0.3, 0.0))
    moved = backend.data.xpos[backend.model.body(PART).id] - seen
    assert moved == pytest.approx([0.0, 0.3, 0.0], abs=1e-9)
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.FAILURE
    assert result.facts == (MISSED,)
    # the part stayed where it was moved to
    there = backend.data.xpos[backend.model.body(PART).id] - seen
    assert np.linalg.norm(there - moved) < 1e-3
    backend.open_gripper(GRIPPER)
    assert backend.gripper_state(GRIPPER).width > 0.08
    backend.displace("part1/base_link", (0.0, -0.3, 0.0))
    result = run(backend, Carry(scene, start, scene.carried(start), 2.0))
    assert result.status is ExecutionStatus.SUCCESS, result.message
    assert result.facts == (HELD,)
    with pytest.raises(KeyError):
        backend.displace("part9", (0.0, 0.0, 0.0))
