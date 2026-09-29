"""The MuJoCo execution backend (#18), on a tiny scene: no HPP needed.

A two-joint arm (a vertical pan joint, then a horizontal link with a
"gripper" at its tip) and a free box. Configurations are MuJoCo ``qpos``
directly (``to_qpos`` is the identity): pan, lift, then the box's
x y z qw qx qy qz.
"""

import math

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from long_tamp.execution import (
    ExecutionCommand,
    ExecutionPolicy,
    ExecutionStatus,
    run_command,
)
from long_tamp.sim import MuJoCoBackend, export_mjcf

ARM = """<robot name="arm">
  <link name="base"><inertial><mass value="5"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="upper"><inertial><origin xyz="0 0 0.1"/><mass value="2"/><inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="forearm"><inertial><origin xyz="0.25 0 0"/><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="tip"/>
  <joint name="pan" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="200" velocity="1.0"/></joint>
  <joint name="lift" type="revolute"><parent link="upper"/><child link="forearm"/><origin xyz="0 0 0.2"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="200" velocity="1.0"/></joint>
  <joint name="tip_fixed" type="fixed"><parent link="forearm"/><child link="tip"/><origin xyz="0.5 0 0"/></joint>
</robot>
"""

BOX = """<robot name="box">
  <link name="base_link">
    <inertial><mass value="0.3"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision>
  </link>
</robot>
"""

CONFIG = """task: tiny
paths:
  robot:
    arm: {urdf: arm.urdf, srdf: ""}
  objects:
    box: {urdf: box.urdf, srdf: ""}
objects:
  box:
    initial_pose_xyzquat: [0.5, 0.0, 0.7, 0, 0, 0, 1]
"""

# The tip is at (0.5, 0, 0.7) with the arm at rest: the box starts in the
# "gripper".
REST = np.array([0.0, 0.0, 0.5, 0.0, 0.7, 1.0, 0.0, 0.0, 0.0])


class Line:
    """A path from ``a`` to ``b`` (qpos) over ``T`` seconds, smooth ends;
    the box follows the tip when ``carry``."""

    def __init__(self, a, b, T, carry=False):
        self.a, self.b, self.T, self.carry = np.array(a), np.array(b), T, carry

    def length(self):
        return self.T

    def eval(self, t):
        u = min(max(t / self.T, 0.0), 1.0)
        u = 3 * u * u - 2 * u**3
        q = self.a + (self.b - self.a) * u
        if self.carry:
            pan, lift = q[0], q[1]
            reach = 0.5 * math.cos(lift)
            q[2:5] = [
                reach * math.cos(pan),
                reach * math.sin(pan),
                0.7 - 0.5 * math.sin(lift),
            ]
            # Rz(pan) * Ry(lift), as (w, x, y, z)
            cz, sz = math.cos(pan / 2), math.sin(pan / 2)
            cy, sy = math.cos(lift / 2), math.sin(lift / 2)
            q[5:9] = [cz * cy, -sz * sy, cz * sy, sz * cy]
        return q, True


@pytest.fixture
def backend(tmp_path):
    (tmp_path / "arm.urdf").write_text(ARM)
    (tmp_path / "box.urdf").write_text(BOX)
    (tmp_path / "tiny.yaml").write_text(CONFIG)
    export = export_mjcf(tmp_path / "tiny.yaml", tmp_path / "mjcf")
    return MuJoCoBackend(export, lambda q: np.asarray(q, float), speed=math.inf)


def run(backend, path):
    command = ExecutionCommand("step", duration=path.length(), payload=path)
    return run_command(backend, command, ExecutionPolicy(poll_interval=0))


def test_a_path_is_tracked_and_metrics_are_reported(backend):
    target = REST.copy()
    target[0] = 0.8
    result = run(backend, Line(REST, target, 2.0))
    assert result.status is ExecutionStatus.SUCCESS
    m = result.metrics
    assert m["tracking_error"] < 0.01
    assert m["drift"] < 0.005
    assert m["start_drift"] == 0.0
    assert m["time_scale"] == pytest.approx(1.0, abs=0.02)  # already smooth
    assert m["object_drift"] < 0.001  # the box stays put: welded to the world
    assert backend.joint_names == ["arm/pan", "arm/lift"]


def test_a_too_fast_path_is_retimed_within_the_limits(backend):
    target = REST.copy()
    target[0] = 1.5  # 1.5 rad in 0.5 s: peak 4.5 rad/s, limit 1 rad/s
    result = run(backend, Line(REST, target, 0.5))
    assert result.status is ExecutionStatus.SUCCESS
    # At least 1.5 s at 1 rad/s, plus accelerating from and to rest.
    assert result.metrics["time_scale"] > 3.0
    assert result.metrics["tracking_error"] < 0.01


def test_a_path_that_starts_at_full_speed_is_tracked(backend):
    class Ramp:  # constant velocity from t=0: an instant start, no easing
        def length(self):
            return 1.0

        def eval(self, t):
            q = REST.copy()
            q[0] = 0.9 * min(max(t, 0.0), 1.0)
            return q, True

    result = run(backend, Ramp())
    assert result.status is ExecutionStatus.SUCCESS
    assert result.metrics["tracking_error"] < 0.01
    assert result.metrics["drift"] < 0.005


def test_a_carried_object_follows_the_arm(backend):
    target = REST.copy()
    target[0] = 1.0
    result = run(backend, Line(REST, target, 2.0, carry=True))
    assert result.status is ExecutionStatus.SUCCESS
    tip = backend.data.xpos[backend.model.body("arm/tip").id]
    box = backend.data.xpos[backend.model.body("box/base_link").id]
    assert np.linalg.norm(tip - box) < 0.005
    assert result.metrics["object_drift"] < 0.01


def test_a_grasp_far_from_the_object_misses(backend):
    # In simulation the box is 30 cm from the tip; the plan carries it from
    # the tip.
    away = REST.copy()
    away[2] += 0.3
    backend.reset(away)
    target = REST.copy()
    target[0] = 0.5
    result = run(backend, Line(REST, target, 1.0, carry=True))
    assert result.status is ExecutionStatus.FAILURE
    assert "grasp missed" in result.message
    assert result.metrics["grasp_error"] == pytest.approx(0.3, abs=0.01)


# ------------------------------------------------------------------- skills

from long_tamp.sim import SCREW, ScrewDriving  # noqa: E402
from long_tamp.tasks.task_planning.skills import SkillCommand  # noqa: E402

PARAMETERS = {"tool": "box", "part": "part1", "hole": "part1/h_hole1"}


def _screw_backend(tmp_path, **params):
    (tmp_path / "arm.urdf").write_text(ARM)
    (tmp_path / "box.urdf").write_text(BOX)
    (tmp_path / "tiny.yaml").write_text(CONFIG)
    export = export_mjcf(tmp_path / "tiny.yaml", tmp_path / "mjcf")
    return MuJoCoBackend(
        export,
        lambda q: np.asarray(q, float),
        speed=math.inf,
        skills={"screw": ScrewDriving(**params)},
    )


def _screw(backend):
    down = REST.copy()
    down[1] = 0.06  # the tip (carrying the "driver") goes ~3 cm down
    command = SkillCommand(SCREW, dict(PARAMETERS), Line(REST, down, 1.0, carry=True))
    return run(backend, command)


def test_the_screw_skill_drives_a_screw_and_reports_its_postcondition(tmp_path):
    result = _screw(_screw_backend(tmp_path))
    assert result.status is ExecutionStatus.SUCCESS, result.message
    assert result.facts == ("screwed(part1, part1/h_hole1)",)
    m = result.metrics
    assert m["screw_torque"] == pytest.approx(2.0, rel=0.01)
    assert m["screw_depth"] >= 0.0079
    assert m["lateral_error"] < 0.001
    assert m["contact_force"] == 15.0


def test_a_hole_off_the_planned_axis_fails_as_misaligned(tmp_path):
    result = _screw(_screw_backend(tmp_path, hole_error=(0.0, 0.005, 0.0)))
    assert result.status is ExecutionStatus.FAILURE
    assert result.facts == ("screw_misaligned(box, part1/h_hole1)",)
    assert result.metrics["lateral_error"] == pytest.approx(0.005, abs=0.001)


def test_a_hole_out_of_reach_fails_as_no_contact(tmp_path):
    # The real hole is 5 cm further along the approach than planned.
    down = REST.copy()
    down[1] = 0.06
    line = Line(REST, down, 1.0, carry=True)
    start = np.array(line.eval(0.0)[0][2:5])
    end = np.array(line.eval(1.0)[0][2:5])
    axis = (end - start) / np.linalg.norm(end - start)
    backend = _screw_backend(tmp_path, hole_error=tuple(0.05 * axis))
    result = _screw(backend)
    assert result.status is ExecutionStatus.FAILURE
    assert result.facts == ("screw_no_contact(box, part1/h_hole1)",)


def test_a_backend_without_the_skill_plays_its_approach(backend):
    down = REST.copy()
    down[1] = 0.06
    command = SkillCommand(SCREW, dict(PARAMETERS), Line(REST, down, 1.0, carry=True))
    result = run(backend, command)  # no "screw" controller registered
    assert result.status is ExecutionStatus.SUCCESS
    assert result.facts == ()
    assert result.metrics["drift"] < 0.005
