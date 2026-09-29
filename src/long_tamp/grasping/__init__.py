"""Grasp planning: where to grasp an object, and how far the fingers close.

Independent of HPP and of the constraint graph, so an orchestrator (the BT
host, a task script) can run it as its own step next to motion planning:

- :class:`GraspPlanner` samples, evaluates and ranks parallel-jaw grasps on
  an object's collision primitives, and closes the fingers at any grasp pose
  (including hand-written SRDF handles);
- :class:`ParallelGripperModel` describes a hand (presets:
  :data:`ROBOTIQ_2F85`, :data:`PANDA_HAND`);
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
    ParallelGripperModel,
    calibrate_from_urdf,
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
    "Box",
    "Cylinder",
    "FingerClosureTable",
    "GraspCandidate",
    "GraspEvaluation",
    "GraspPlanner",
    "GraspPlannerParams",
    "GraspableObject",
    "GripperBinding",
    "ParallelGripperModel",
    "Primitive",
    "Sphere",
    "apply_joint_values",
    "calibrate_from_urdf",
    "load_srdf_frames",
    "load_urdf_primitives",
    "pose_to_xyzquat",
    "xyzquat_to_pose",
]
