"""Gripper actuation and grasp verification (issue #127, ADR-0009): the
thresholds come from the hand's model, and the interface crosses the wire."""

import pytest

from long_tamp.execution import (
    ExpectedGrasp,
    GripperActuation,
    GripperState,
    MockBackend,
    RemoteBackend,
    remote,
)
from long_tamp.execution.gripper import FULLY_CLOSED, MISS_MARGIN, driving_value
from long_tamp.grasping import ROBOTIQ_2F85, SO101_JAW
from tests.test_execution_remote import _roundtrip, served


def _expected(model, contact_width):
    width = contact_width - model.default_squeeze
    return ExpectedGrasp("arm/gripper", "cube", model, contact_width, width, "arm")


def test_the_threshold_is_a_margin_of_squeezes_past_the_command():
    expected = _expected(ROBOTIQ_2F85, 0.030)
    squeeze = ROBOTIQ_2F85.default_squeeze
    assert expected.threshold == pytest.approx(0.030 - (1 + MISS_MARGIN) * squeeze)
    assert expected.verify(GripperState(0.0269)) == "holds(arm/gripper, cube)"
    assert expected.verify(GripperState(0.014)) == "grasp_missed(arm/gripper, cube)"


def test_a_thin_object_misses_only_when_the_hand_closed_fully():
    # SO-101: 5 mm squeeze; a 12 mm object leaves no room for the margin
    expected = _expected(SO101_JAW, 0.012)
    floor = SO101_JAW.min_width + FULLY_CLOSED
    assert expected.threshold == pytest.approx(floor)
    assert expected.held(GripperState(floor + 1e-4))
    assert not expected.held(GripperState(SO101_JAW.min_width))


def test_the_driving_value_is_read_from_any_measured_joint():
    q = 0.4
    values = ROBOTIQ_2F85.joint_values(q, "arm")
    assert driving_value(ROBOTIQ_2F85, values, "arm") == pytest.approx(q)
    one = {
        "arm/robotiq_85_right_knuckle_joint": values[
            "arm/robotiq_85_right_knuckle_joint"
        ]
    }  # a -1 multiplier
    assert driving_value(ROBOTIQ_2F85, one, "arm") == pytest.approx(q)
    assert driving_value(ROBOTIQ_2F85, {"other": 1.0}, "arm") is None


class _Hand(MockBackend):
    """A mock backend with a hand: an opening that ``close_gripper`` sets."""

    def __init__(self):
        super().__init__()
        self.width = 0.085

    def open_gripper(self, gripper):
        self.width = 0.085

    def close_gripper(self, gripper, width=None, effort=None):
        self.width = 0.0 if width is None else width

    def gripper_state(self, gripper):
        return GripperState(self.width, 1.5)


def test_a_hand_implements_the_protocol():
    assert isinstance(_Hand(), GripperActuation)
    assert not isinstance(MockBackend(), GripperActuation)


def test_the_gripper_state_crosses_the_wire():
    assert _roundtrip(GripperState(0.03, None)) == GripperState(0.03, None)
    assert _roundtrip(GripperState(0.03, 2.0)) == GripperState(0.03, 2.0)
    with pytest.raises(TypeError):
        remote.encode({"$gripper": 1})


def test_a_remote_hand_is_actuated_through_the_client():
    hand = _Hand()
    with served(hand) as port, RemoteBackend(port=port) as client:
        # The client forwards the hand's methods through __getattr__, which
        # isinstance() on a runtime protocol doesn't see from Python 3.12 on.
        assert all(
            hasattr(client, name)
            for name in ("open_gripper", "close_gripper", "gripper_state")
        )
        client.close_gripper("arm/gripper", 0.03)
        assert client.gripper_state("arm/gripper") == GripperState(0.03, 1.5)
        client.open_gripper("arm/gripper")
        assert hand.width == 0.085
    with served(MockBackend()) as port, RemoteBackend(port=port) as client:
        assert not hasattr(client, "close_gripper")
