"""Gripper actuation and grasp verification (ADR-0009).

A backend that drives a real hand implements :class:`GripperActuation`
(``open_gripper``, ``close_gripper``, ``gripper_state``) on top of the
execution contract. After it closes the fingers on a planned grasp, it checks
the measured opening against what the hand's :class:`GripperModel` expects
(:class:`ExpectedGrasp`) and reports the outcome as a fact in
``Feedback.facts``:

- ``holds(<gripper>, <object>)``: the pads stopped on the object;
- ``grasp_missed(<gripper>, <object>)``: they closed past where the object
  would have stopped them. The command then fails with that fact.

``<gripper>`` is the HPP gripper (``"ur10_left/gripper"``) and ``<object>``
the object (``"part1"``).

The threshold comes from the model: the fingers are commanded to the contact
opening minus ``default_squeeze``, and a hand on an object stops near there
(its pads give a little). One more ``MISS_MARGIN`` squeezes past the command
counts as closing on nothing. A model whose stroke can't tell the two apart
(the threshold is at or below ``min_width``: an object thinner than the
squeeze) reports a miss only when the hand closed fully.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from long_tamp.grasping.gripper import GripperModel

#: How many ``default_squeeze`` past the commanded opening the fingers may
#: close and still be holding the object.
MISS_MARGIN = 2.0
#: Below this opening above ``min_width`` [m], the hand closed fully.
FULLY_CLOSED = 5e-4


@dataclass(frozen=True)
class GripperState:
    """A hand's measured state.

    ``width`` is the pad-to-pad opening (m); ``effort`` the driving joint's
    effort (N or N m), ``None`` when the hand doesn't measure it.
    """

    width: float
    effort: float | None = None


@runtime_checkable
class GripperActuation(Protocol):
    """Opening and closing a hand outside planned motion (ADR-0009).

    ``gripper`` is the HPP gripper name. ``close_gripper`` closes to
    ``width`` (m; default: fully) or until the fingers push with ``effort``,
    whichever comes first, and returns when the fingers have settled.
    """

    def open_gripper(self, gripper: str) -> None: ...

    def close_gripper(
        self, gripper: str, width: float | None = None, effort: float | None = None
    ) -> None: ...

    def gripper_state(self, gripper: str) -> GripperState: ...


def driving_value(
    model: GripperModel, values: Mapping[str, float], prefix: str = ""
) -> float | None:
    """The model's driving value from measured finger joint ``values``
    (prefixed names), averaged over the joints measured; ``None`` if none is."""
    zero = model.joint_values(0.0, prefix)
    one = model.joint_values(1.0, prefix)
    qs = [
        (values[joint] - zero[joint]) / (one[joint] - zero[joint])
        for joint in one
        if joint in values and one[joint] != zero[joint]
    ]
    return sum(qs) / len(qs) if qs else None


@dataclass(frozen=True)
class ExpectedGrasp:
    """What closing ``gripper`` on ``obj`` should measure.

    ``contact_width`` is where the pads touch the object and ``width`` the
    commanded opening (``GraspEvaluation``); ``prefix`` the robot name the
    finger joints carry. :meth:`FingerClosureTable.expected_grasp
    <long_tamp.grasping.FingerClosureTable.expected_grasp>` builds one.
    """

    gripper: str
    obj: str
    model: GripperModel
    contact_width: float
    width: float
    prefix: str = ""

    @property
    def threshold(self) -> float:
        """Openings below this mean the hand closed on nothing."""
        missed = self.width - MISS_MARGIN * self.model.default_squeeze
        return max(missed, self.model.min_width + FULLY_CLOSED)

    def held(self, state: GripperState) -> bool:
        """Whether a hand measured at ``state`` holds the object."""
        return state.width >= self.threshold

    def fact(self, held: bool) -> str:
        name = "holds" if held else "grasp_missed"
        return f"{name}({self.gripper}, {self.obj})"

    def verify(self, state: GripperState) -> str:
        """``holds(...)`` or ``grasp_missed(...)`` for ``state``."""
        return self.fact(self.held(state))
