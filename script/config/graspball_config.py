"""
Configuration for grasp ball manipulation task.

This configuration defines the UR5 robot with gripper and pokeball
manipulation scenario for both CORBA and PyHPP backends.
"""

from typing import Dict, List
import numpy as np

PATHS = {
    "robot": {"ur5": {"urdf": "package://hpp_practicals/urdf/ur5_gripper.urdf",
        "srdf": ""}},
    "objects": {
        "pokeball": "package://hpp_practicals/urdf/ur_benchmark/pokeball.urdf",
    },
    "environment": {"ground": "package://hpp_practicals/urdf/ur_benchmark/ground.urdf",
                    "box": "package://hpp_practicals/urdf/ur_benchmark/box.urdf"},
}
class RobotJoints:
    """Joint names for UR5 robot with gripper."""
    
    # UR5 joints (6 DOF)
    UR5 = [
        "ur5/shoulder_pan_joint",
        "ur5/shoulder_lift_joint",
        "ur5/elbow_joint",
        "ur5/wrist_1_joint",
        "ur5/wrist_2_joint",
        "ur5/wrist_3_joint",
    ]
    
    # Pokeball root joint (freeflyer: 7 DOF)
    POKEBALL_ROOT = "pokeball/root_joint"
    
    @classmethod
    def all_robot_joints(cls) -> List[str]:
        """Get all robot joint names."""
        return cls.UR5
    
    @classmethod
    def all_object_roots(cls) -> List[str]:
        """Get all object root joint names."""
        return [cls.POKEBALL_ROOT]


class InitialConfigurations:
    """Initial joint configurations for robot and objects."""
    
    # Robot joint configuration (in radians)
    UR5 = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # 6 DOF
    
    # Pokeball pose in XYZQUAT format [x, y, z, qx, qy, qz, qw]
    POKEBALL = [0.3, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0]  # 7 DOF (freeflyer)
    
    # Full initial configuration (UR5 + Pokeball)
    FULL_INIT = UR5 + POKEBALL
    
    @classmethod
    def build_configuration(cls, ur5_config=None, pokeball_config=None):
        """Build full configuration from components."""
        ur5 = ur5_config if ur5_config is not None else cls.UR5
        ball = pokeball_config if pokeball_config is not None else cls.POKEBALL
        return ur5 + ball


class JointBounds:
    """Joint bounds for robot and free-flying objects."""
    
    # UR5 joint limits (in radians)
    UR5_LIMITS = [(-2 * np.pi, 2 * np.pi)] * 6
    
    # Freeflyer bounds for pokeball
    TRANSLATION_BOUNDS = [
        (-0.4, 0.4),  # x
        (-0.4, 0.4),  # y
        (-0.1, 1.0),  # z
    ]
    
    # Quaternion bounds (must contain unit quaternion)
    QUATERNION_BOUNDS = [
        (-1.0001, 1.0001),  # qx
        (-1.0001, 1.0001),  # qy
        (-1.0001, 1.0001),  # qz
        (-1.0001, 1.0001),  # qw
    ]
    
    @classmethod
    def freeflyer_bounds(cls) -> List[float]:
        """Get combined translation + quaternion bounds as flat list."""
        bounds = []
        for t_bound in cls.TRANSLATION_BOUNDS:
            bounds.extend(t_bound)
        for q_bound in cls.QUATERNION_BOUNDS:
            bounds.extend(q_bound)
        return bounds
    
    @classmethod
    def all_robot_bounds(cls) -> Dict[str, List[float]]:
        """Get all robot joint bounds."""
        bounds = {}
        
        # UR5 bounds
        for i, joint in enumerate(RobotJoints.UR5):
            bounds[joint] = list(cls.UR5_LIMITS[i])
            
        return bounds


class ManipulationConfig:
    """Configuration for grasp ball manipulation task."""
    
    # Gripper name
    GRIPPER_NAME = "ur5/wrist_3_joint"
    BALL_NAME = "pokeball/root_joint"
    
    # Ball properties
    BALL_RADIUS = 0.025
    
    # Transformation constraints (XYZQUAT format)
    # Ball in gripper when grasped
    BALL_IN_GRIPPER = [0.0, 0.137, 0.0, 0.5, 0.5, -0.5, 0.5]
    
    # Ball on ground (placement constraint)
    BALL_ON_GROUND = [0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0]
    
    # Gripper above ball (for approach)
    GRIPPER_ABOVE_BALL = [0.0, 0.2, 0.0, 0.5, 0.5, -0.5, 0.5]
    
    # Box configuration
    BOX_X = 0.3
    BOX_OFFSET = 0.04
    BALL_NEAR_TABLE = [BOX_X, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]
    
    # Constraint masks (6 DOF: x, y, z, roll, pitch, yaw)
    # Grasp mask: all 6 DOF constrained
    GRASP_MASK = [True, True, True, True, True, True]
    
    # Placement mask: z, roll, pitch constrained
    PLACEMENT_MASK = [False, False, True, True, True, False]
    
    # Placement complement mask: x, y, yaw free
    PLACEMENT_COMPLEMENT_MASK = [True, True, False, False, False, True]
    
    # URDF/SRDF paths
    ROBOT_URDF = "package://hpp_practicals/urdf/ur5_gripper.urdf"
    ROBOT_SRDF = ""
    
    BALL_URDF = "package://hpp_practicals/urdf/ur_benchmark/pokeball.urdf"
    BALL_SRDF = "package://hpp_practicals/srdf/ur_benchmark/pokeball.srdf"
    
    GROUND_URDF = "package://hpp_practicals/urdf/ur_benchmark/ground.urdf"
    BOX_URDF = "package://hpp_practicals/urdf/ur_benchmark/box.urdf"
    
    # Graph nodes for manipulation (order matters for solver performance!)
    GRAPH_NODES = [
        "grasp",              # Ball grasped, no placement constraint
        "ball-above-ground",   # Ball grasped and lifted
        "grasp-placement",     # Ball grasped and on ground
        "gripper-above-ball",  # Gripper aligned above ball
        "placement",           # Ball on ground, gripper free
    ]
    
    # Graph edges (from, to, name)
    GRAPH_EDGES = [
        # Free motion with ball on ground
        ("placement", "placement", "transit"),
        
        # Approach and retreat ball
        ("placement", "gripper-above-ball", "approach-ball"),
        ("gripper-above-ball", "placement", "move-gripper-away"),
        
        # Grasp and release
        ("gripper-above-ball", "grasp-placement", "grasp-ball"),
        ("grasp-placement", "gripper-above-ball", "move-gripper-up"),
        
        # Lift and place
        ("grasp-placement", "ball-above-ground", "take-ball-up"),
        ("ball-above-ground", "grasp-placement", "put-ball-down"),
        
        # Transfer with ball grasped
        ("ball-above-ground", "grasp", "take-ball-away"),
        ("grasp", "ball-above-ground", "approach-ground"),
        ("grasp", "grasp", "transfer"),
    ]
    # ============================================================================
    # Graph Definition (Declarative)
    # ============================================================================

    GRASPBALL_GRAPH = {
        "name": "graspball_graph",

        # States with their constraints
        "states": {
            "placement": {"constraints": ["placement"]},
            "gripper-above-ball": {
                "constraints": ["placement", "gripper_ball_aligned"]
            },
            "grasp-placement": {"constraints": ["grasp", "placement"]},
            "ball-above-ground": {"constraints": ["grasp", "ball_near_table"]},
            "grasp": {"constraints": ["grasp"]},
        },

        # Edges: from -> to via containing state
        "edges": {
            # Self-loops
            "transit": {
                "from": "placement", "to": "placement", "in": "placement"
            },
            "transfer": {
                "from": "grasp", "to": "grasp", "in": "grasp"
            },
            # From placement
            "approach-ball": {
                "from": "placement", "to": "gripper-above-ball", "in": "placement"
            },
            # From gripper-above-ball
            "move-gripper-away": {
                "from": "gripper-above-ball", "to": "placement", "in": "placement"
            },
            "grasp-ball": {
                "from": "gripper-above-ball", "to": "grasp-placement",
                "in": "placement"
            },
            # From grasp-placement
            "move-gripper-up": {
                "from": "grasp-placement", "to": "gripper-above-ball",
                "in": "placement"
            },
            "take-ball-up": {
                "from": "grasp-placement", "to": "ball-above-ground", "in": "grasp"
            },
            # From ball-above-ground
            "put-ball-down": {
                "from": "ball-above-ground", "to": "grasp-placement", "in": "grasp"
            },
            "take-ball-away": {
                "from": "ball-above-ground", "to": "grasp", "in": "grasp"
            },
            # From grasp
            "approach-ground": {
                "from": "grasp", "to": "ball-above-ground", "in": "grasp"
            },
        },

        # Edge path constraints (grouped by constraint)
        # Edges not listed are free motion (no path constraints)
        "edge_constraints": {
            "placement/complement": [
                "transit", "approach-ball", "move-gripper-away",
                "grasp-ball", "move-gripper-up"
            ],
            "ball_near_table/complement": ["take-ball-up", "put-ball-down"],
        },

        # Free moton edges (no path constraints)
        "free_motion_edges": ["transfer", "take-ball-away", "approach-ground"],

        # Constant RHS settings (CORBA only)
        "constant_rhs": {
            "placement": True,
            "placement/complement": False,
            "ball_near_table/complement": False,
        },
    }

    # ============================================================================
    # Constraint Definitions (Data-driven)
    # ============================================================================
    # Each tuple: (type, name, args_dict)
    # - type: "grasp", "placement", or "complement"
    # - name: constraint name
    # - args: dict with keys depending on type
    #   - grasp: gripper, obj, transform, mask
    #   - placement/complement: obj, transform, mask

    @classmethod
    def get_constraint_defs(cls):
        """Return constraint definitions for this task."""
        return [
            ("grasp", "grasp", {
                "gripper": cls.GRIPPER_NAME,
                "obj": cls.BALL_NAME,
                "transform": cls.BALL_IN_GRIPPER,
                "mask": cls.GRASP_MASK,
            }),
            ("placement", "placement", {
                "obj": cls.BALL_NAME,
                "transform": cls.BALL_ON_GROUND,
                "mask": cls.PLACEMENT_MASK,
            }),
            ("complement", "placement", {
                "obj": cls.BALL_NAME,
                "transform": cls.BALL_ON_GROUND,
                "mask": cls.PLACEMENT_COMPLEMENT_MASK,
            }),
            ("grasp", "gripper_ball_aligned", {
                "gripper": cls.GRIPPER_NAME,
                "obj": cls.BALL_NAME,
                "transform": cls.GRIPPER_ABOVE_BALL,
                "mask": cls.GRASP_MASK,
            }),
            ("placement", "ball_near_table", {
                "obj": cls.BALL_NAME,
                "transform": cls.BALL_NEAR_TABLE,
                "mask": cls.PLACEMENT_MASK,
            }),
            ("complement", "ball_near_table", {
                "obj": cls.BALL_NAME,
                "transform": [cls.BOX_X, 0.2, 0.1, 0, 0, 0, 1],
                "mask": cls.PLACEMENT_COMPLEMENT_MASK,
            }),
        ]

    # Path planning parameters
    PATH_VALIDATION_STEP = 0.01
    PATH_PROJECTOR_STEP = 0.1
    MAX_RANDOM_ATTEMPTS = 1000  # Increased for better success rate


__all__ = [
    "RobotJoints",
    "InitialConfigurations",
    "JointBounds",
    "ManipulationConfig",
]
