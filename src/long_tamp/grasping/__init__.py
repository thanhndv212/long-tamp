"""Grasp planning: where to grasp an object, and how far the fingers close.

Independent of HPP and of the constraint graph, so an orchestrator (the BT
host, a task script) can run it as its own step next to motion planning:

- :class:`GraspPlanner` samples, evaluates and ranks parallel-jaw grasps on
  an object's collision primitives, and closes the fingers at any grasp pose
  (including hand-written SRDF handles);
- a :class:`GripperModel` describes a hand: :class:`ParallelGripperModel`
  (presets :data:`ROBOTIQ_2F85`, :data:`PANDA_HAND`) or
  :class:`RotaryJawGripperModel` (:data:`SO101_JAW`); :data:`PRESETS` holds
  them by name and :func:`register_gripper` adds one;
- :class:`FingerClosureTable` gives, per ``(gripper, handle)`` pair of a
  task, the finger joint values to command when the grasp closes.
"""

from .geometry import (
    Box,
    Cylinder,
    Primitive,
    Sphere,
    load_srdf_frames,
    load_urdf_primitives,
    pose_to_xyzquat,
    xyzquat_to_pose,
)
from .gripper import (
    PANDA_HAND,
    PRESETS,
    ROBOTIQ_2F85,
    ROBOTIQ_2F85_ROS_INDUSTRIAL,
    SO101_JAW,
    GripperModel,
    ParallelGripperModel,
    RotaryJawGripperModel,
    calibrate_from_urdf,
    register_gripper,
)
from .objects import (
    FingerClosureTable,
    GraspableObject,
    GripperBinding,
    apply_joint_values,
)
from .planner import GraspCandidate, GraspEvaluation, GraspPlanner, GraspPlannerParams

__all__ = [
    "PANDA_HAND",
    "PRESETS",
    "ROBOTIQ_2F85",
    "ROBOTIQ_2F85_ROS_INDUSTRIAL",
    "SO101_JAW",
    "Box",
    "Cylinder",
    "FingerClosureTable",
    "GraspCandidate",
    "GraspEvaluation",
    "GraspPlanner",
    "GraspPlannerParams",
    "GraspableObject",
    "GripperBinding",
    "GripperModel",
    "ParallelGripperModel",
    "Primitive",
    "RotaryJawGripperModel",
    "Sphere",
    "apply_joint_values",
    "calibrate_from_urdf",
    "load_srdf_frames",
    "load_urdf_primitives",
    "pose_to_xyzquat",
    "register_gripper",
    "xyzquat_to_pose",
]
