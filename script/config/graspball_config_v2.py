"""
Configuration for grasp ball manipulation task.

This configuration defines the UR5 robot with gripper and pokeball
manipulation scenario for both CORBA and PyHPP backends.
"""

from typing import Dict, List

from agimus_spacelab.config.base_config import (
    BaseTaskConfig,
    Defaults,
    ConstraintDef,
    StateDef,
    EdgeDef,
)


# =============================================================================
# Model Paths
# =============================================================================

GRASPBALL_PATHS = {
    "robot": {
        "ur5": {
            "urdf": "package://hpp_practicals/urdf/ur5_gripper.urdf",
            "srdf": "package://hpp_practicals/srdf/ur5_gripper.srdf",
        }
    },
    "objects": {
        "pokeball": {
            "urdf": "package://hpp_practicals/urdf/ur_benchmark/pokeball.urdf",
            "srdf": "package://hpp_practicals/srdf/ur_benchmark/pokeball.srdf",
        }
    },
    "environment": {
        "ground": "package://hpp_practicals/urdf/ur_benchmark/ground.urdf",
        "box": "package://hpp_practicals/urdf/ur_benchmark/box.urdf",
    },
}


# =============================================================================
# Joint Configuration
# =============================================================================

class UR5Joints:
    """Joint names for UR5 robot with gripper."""
    
    JOINTS = [
        "ur5/shoulder_pan_joint",
        "ur5/shoulder_lift_joint",
        "ur5/elbow_joint",
        "ur5/wrist_1_joint",
        "ur5/wrist_2_joint",
        "ur5/wrist_3_joint",
    ]
    
    GRIPPER = "ur5/wrist_3_joint"
    POKEBALL_ROOT = "pokeball/root_joint"
    
    @classmethod
    def all_joints(cls) -> List[str]:
        return cls.JOINTS


class UR5Bounds:
    """Joint bounds for UR5 robot."""
    
    # UR5 joint limits (in radians)
    JOINT_LIMITS = [Defaults.REVOLUTE_BOUNDS] * 6
    
    # Pokeball freeflyer bounds
    POKEBALL_BOUNDS = [
        -0.4, 0.4,    # x
        -0.4, 0.4,    # y
        -0.1, 1.0,    # z
        -1.0001, 1.0001,  # qx
        -1.0001, 1.0001,  # qy
        -1.0001, 1.0001,  # qz
        -1.0001, 1.0001,  # qw
    ]
    
    @classmethod
    def get_joint_bounds(cls) -> Dict[str, List[float]]:
        bounds = {}
        for i, joint in enumerate(UR5Joints.JOINTS):
            bounds[joint] = list(cls.JOINT_LIMITS[i])
        return bounds


# =============================================================================
# Task Configuration
# =============================================================================

class GraspBallConfig(BaseTaskConfig):
    """
    Configuration for grasp ball manipulation task.
    
    This task involves:
    1. UR5 robot with gripper
    2. Pokeball object on ground
    3. Pick and place operations
    """
    
    # =========================================================================
    # Scene Configuration
    # =========================================================================
    
    PATHS = GRASPBALL_PATHS
    
    ROBOT_NAMES = ["ur5"]
    ENVIRONMENT_NAMES = ["ground", "box"]
    OBJECTS = ["pokeball"]
    
    # =========================================================================
    # Manipulation Configuration
    # =========================================================================
    
    GRIPPERS = ["ur5/gripper"]
    
    HANDLES_PER_OBJECT = [
        ["pokeball/handle"],  # pokeball handles
    ]
    
    CONTACT_SURFACES_PER_OBJECT = [
        ["pokeball/bottom"],  # pokeball contact surfaces
    ]
    
    ENVIRONMENT_CONTACTS = ["ground/surface"]
    
    VALID_PAIRS = {
        "ur5/gripper": ["pokeball/handle"],
    }
    
    # =========================================================================
    # Initial Configuration
    # =========================================================================
    
    # UR5 initial joint configuration (radians)
    INITIAL_ROBOT_CONFIG = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
    
    # Object initial poses (XYZQUAT format)
    INITIAL_OBJECT_CONFIGS = {
        "pokeball": [0.3, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0],
    }
    
    # =========================================================================
    # Transformation Definitions
    # =========================================================================
    
    # Ball properties
    BALL_RADIUS = 0.025
    BOX_X = 0.3
    BOX_OFFSET = 0.04
    
    # Key transforms (XYZQUAT format)
    BALL_IN_GRIPPER = [0.0, 0.137, 0.0, 0.5, 0.5, -0.5, 0.5]
    BALL_ON_GROUND = [0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0]
    GRIPPER_ABOVE_BALL = [0.0, 0.2, 0.0, 0.5, 0.5, -0.5, 0.5]
    BALL_NEAR_TABLE = [BOX_X, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
    
    # =========================================================================
    # Constraint Masks
    # =========================================================================
    
    GRASP_MASK = Defaults.MASK_ALL
    PLACEMENT_MASK = Defaults.MASK_PLACEMENT
    PLACEMENT_COMPLEMENT_MASK = Defaults.MASK_PLACEMENT_COMPLEMENT
    
    # =========================================================================
    # Graph Definition
    # =========================================================================
    
    STATES = {
        "placement": StateDef(
            name="placement",
            constraints=["placement"],
        ),
        "gripper-above-ball": StateDef(
            name="gripper-above-ball",
            constraints=["placement", "gripper_ball_aligned"],
        ),
        "grasp-placement": StateDef(
            name="grasp-placement",
            constraints=["grasp", "placement"],
        ),
        "ball-above-ground": StateDef(
            name="ball-above-ground",
            constraints=["grasp", "ball_near_table"],
        ),
        "grasp": StateDef(
            name="grasp",
            constraints=["grasp"],
        ),
    }
    
    EDGES = {
        # Self-loops
        "transit": EdgeDef(
            name="transit",
            from_state="placement",
            to_state="placement",
            containing_state="placement",
            path_constraints=["placement/complement"],
        ),
        "transfer": EdgeDef(
            name="transfer",
            from_state="grasp",
            to_state="grasp",
            containing_state="grasp",
        ),
        # From placement
        "approach-ball": EdgeDef(
            name="approach-ball",
            from_state="placement",
            to_state="gripper-above-ball",
            containing_state="placement",
            path_constraints=["placement/complement"],
        ),
        # From gripper-above-ball
        "move-gripper-away": EdgeDef(
            name="move-gripper-away",
            from_state="gripper-above-ball",
            to_state="placement",
            containing_state="placement",
            path_constraints=["placement/complement"],
        ),
        "grasp-ball": EdgeDef(
            name="grasp-ball",
            from_state="gripper-above-ball",
            to_state="grasp-placement",
            containing_state="placement",
            path_constraints=["placement/complement"],
        ),
        # From grasp-placement
        "move-gripper-up": EdgeDef(
            name="move-gripper-up",
            from_state="grasp-placement",
            to_state="gripper-above-ball",
            containing_state="placement",
            path_constraints=["placement/complement"],
        ),
        "take-ball-up": EdgeDef(
            name="take-ball-up",
            from_state="grasp-placement",
            to_state="ball-above-ground",
            containing_state="grasp",
            path_constraints=["ball_near_table/complement"],
        ),
        # From ball-above-ground
        "put-ball-down": EdgeDef(
            name="put-ball-down",
            from_state="ball-above-ground",
            to_state="grasp-placement",
            containing_state="grasp",
            path_constraints=["ball_near_table/complement"],
        ),
        "take-ball-away": EdgeDef(
            name="take-ball-away",
            from_state="ball-above-ground",
            to_state="grasp",
            containing_state="grasp",
        ),
        # From grasp
        "approach-ground": EdgeDef(
            name="approach-ground",
            from_state="grasp",
            to_state="ball-above-ground",
            containing_state="grasp",
        ),
    }
    
    # =========================================================================
    # Constraint Definitions
    # =========================================================================
    
    @classmethod
    def get_constraint_defs(cls) -> List[ConstraintDef]:
        """Return constraint definitions for grasp ball task."""
        gripper = UR5Joints.GRIPPER
        ball = UR5Joints.POKEBALL_ROOT
        
        return [
            ConstraintDef(
                type="grasp",
                name="grasp",
                gripper=gripper,
                obj=ball,
                transform=cls.BALL_IN_GRIPPER,
                mask=cls.GRASP_MASK,
            ),
            ConstraintDef(
                type="placement",
                name="placement",
                obj=ball,
                transform=cls.BALL_ON_GROUND,
                mask=cls.PLACEMENT_MASK,
            ),
            ConstraintDef(
                type="complement",
                name="placement",
                obj=ball,
                transform=cls.BALL_ON_GROUND,
                mask=cls.PLACEMENT_COMPLEMENT_MASK,
            ),
            ConstraintDef(
                type="grasp",
                name="gripper_ball_aligned",
                gripper=gripper,
                obj=ball,
                transform=cls.GRIPPER_ABOVE_BALL,
                mask=cls.GRASP_MASK,
            ),
            ConstraintDef(
                type="placement",
                name="ball_near_table",
                obj=ball,
                transform=cls.BALL_NEAR_TABLE,
                mask=cls.PLACEMENT_MASK,
            ),
            ConstraintDef(
                type="complement",
                name="ball_near_table",
                obj=ball,
                transform=[cls.BOX_X, 0.2, 0.1, 0, 0, 0, 1],
                mask=cls.PLACEMENT_COMPLEMENT_MASK,
            ),
        ]


# =============================================================================
# Convenience Aliases (Backward Compatibility)
# =============================================================================

# For backward compatibility with old code
RobotJoints = UR5Joints
JointBounds = UR5Bounds
InitialConfigurations = type(
    'InitialConfigurations', (), {
        'UR5': GraspBallConfig.INITIAL_ROBOT_CONFIG,
        'POKEBALL': GraspBallConfig.INITIAL_OBJECT_CONFIGS['pokeball'],
        'FULL_INIT': GraspBallConfig.get_full_initial_config(),
    }
)
ManipulationConfig = GraspBallConfig
PATHS = GRASPBALL_PATHS


__all__ = [
    "GRASPBALL_PATHS",
    "PATHS",
    "UR5Joints",
    "UR5Bounds",
    "GraspBallConfig",
    # Backward compatibility
    "RobotJoints",
    "JointBounds",
    "InitialConfigurations",
    "ManipulationConfig",
]
