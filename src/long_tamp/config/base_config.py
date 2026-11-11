"""
Base configuration classes for manipulation tasks.

Provides abstract base classes and default values that can be overridden
by specific task configurations.
"""

from abc import ABC
from dataclasses import dataclass, field

import numpy as np

from long_tamp.utils.transforms import xyzquat_to_xyzrpy, xyzrpy_to_xyzquat

# =============================================================================
# Default Values
# =============================================================================


class Defaults:
    """Default values for manipulation planning."""

    # Planning parameters
    PATH_VALIDATION_STEP = 0.01
    PATH_PROJECTOR_STEP = 0.1
    MAX_ITERATIONS = 1000
    ERROR_THRESHOLD = 1e-4
    MAX_RANDOM_ATTEMPTS = 1000

    # Optimizer parameters
    # Number of shortcut attempts per RandomShortcut pass (HPP default is 5).
    RANDOM_SHORTCUT_LOOPS: int = 50
    # When False the SplineGradientBased optimizer is free to carry velocity
    # through state junctions, producing smoother, shorter splines.
    SPLINE_ZERO_DERIVATIVES_AT_STATE: bool = False
    # SimpleTimeParameterization: fraction of velocity limits used during
    # time parameterization.  Lower = slower execution.  Range [0, 1].
    TIME_PARAM_SAFETY: float = 0.95
    # Polynomial order: 1=linear, 2=cubic (zero-vel ends), 3=quintic (zero-vel+accel)
    TIME_PARAM_ORDER: int = 2
    # Time parameterization method: "stp", "trapezoidal", or "toppra"
    TIME_PARAM_METHOD: str = "stp"
    # TOPPRA-specific (ignored unless TIME_PARAM_METHOD == "toppra")
    TOPPRA_VELOCITY_SCALE: float = 1.0
    TOPPRA_EFFORT_SCALE: float = -1  # -1 = disable torque constraints
    TOPPRA_SOLVER: int = 0  # 0=Seidel, 1=GLPK, 2=qpOASES
    TOPPRA_N: int = 100  # grid points
    TOPPRA_INTERPOLATION: str = "hermite"  # or "constant_acceleration"
    TOPPRA_GRIDPOINT_METHOD: str = "param_space"  # or "time_space"
    TOPPRA_ACTIVE_JOINTS: list = []  # empty = all joints

    # Constraint masks (6 DOF: x, y, z, roll, pitch, yaw)
    MASK_ALL = [True, True, True, True, True, True]
    MASK_NONE = [False, False, False, False, False, False]
    MASK_POSITION = [True, True, True, False, False, False]
    MASK_ORIENTATION = [False, False, False, True, True, True]
    MASK_PLACEMENT = [False, False, True, True, True, False]  # z, roll, pitch
    # x, y, yaw free
    MASK_PLACEMENT_COMPLEMENT = [True, True, False, False, False, True]

    # Joint bounds
    REVOLUTE_BOUNDS = (-2 * np.pi, 2 * np.pi)
    TRANSLATION_BOUNDS = (-5.0, 5.0)
    QUATERNION_BOUNDS = (-1.0001, 1.0001)

    @classmethod
    def freeflyer_bounds(
        cls,
        x: tuple[float, float] = (-2.0, 2.0),
        y: tuple[float, float] = (-3.0, 3.0),
        z: tuple[float, float] = (-2.0, 2.0),
    ) -> list[float]:
        """Get freeflyer bounds as flat list [xmin, xmax, ymin, ymax, ...]."""
        qb = cls.QUATERNION_BOUNDS
        return [x[0], x[1], y[0], y[1], z[0], z[1], qb, qb, qb, qb, qb, qb, qb, qb]


# =============================================================================
# Data Classes for Configuration
# =============================================================================


@dataclass
class ModelPaths:
    """URDF/SRDF paths for a model."""

    urdf: str
    srdf: str = ""

    def __post_init__(self):
        # Auto-generate SRDF path if not provided
        if not self.srdf and self.urdf:
            self.srdf = self.urdf.replace("/urdf/", "/srdf/")
            self.srdf = self.srdf.replace(".urdf", ".srdf")


@dataclass
class TransformConfig:
    """Transformation configuration (position + quaternion)."""

    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])

    def as_xyzquat(self) -> list[float]:
        """Return as [x, y, z, qx, qy, qz, qw]."""
        return self.position + self.quaternion

    @classmethod
    def from_xyzquat(cls, xyzquat: list[float]) -> "TransformConfig":
        """Create from [x, y, z, qx, qy, qz, qw] list."""
        return cls(position=xyzquat[:3], quaternion=xyzquat[3:])

    @classmethod
    def identity(cls) -> "TransformConfig":
        """Return identity transform."""
        return cls([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])


@dataclass
class ConstraintDef:
    """Definition of a constraint."""

    type: str  # "grasp", "placement", "complement"
    name: str
    gripper: str | None = None  # For grasp constraints
    obj: str | None = None  # Object joint name
    transform: list[float] | None = None  # XYZQUAT
    mask: list[bool] = field(default_factory=lambda: Defaults.MASK_ALL)


@dataclass
class EdgeDef:
    """Definition of a graph edge."""

    name: str
    from_state: str
    to_state: str
    containing_state: str
    weight: float = 1.0
    path_constraints: list[str] = field(default_factory=list)


@dataclass
class StateDef:
    """Definition of a graph state."""

    name: str
    constraints: list[str] = field(default_factory=list)
    is_waypoint: bool = False
    priority: int = 0


# =============================================================================
# Base Configuration Classes
# =============================================================================


class BaseTaskConfig(ABC):
    """
    Abstract base class for task configurations.

    Subclasses should override class attributes to customize the task.
    """

    # ==========================================================================
    # Scene Configuration (Override in subclass)
    # ==========================================================================

    # Model paths
    PATHS: dict[str, dict] = {}

    # Robot configuration
    ROBOT_NAMES: list[str] = []
    ENVIRONMENT_NAMES: list[str] = []
    OBJECTS: list[str] = []

    # ==========================================================================
    # Manipulation Configuration (Override in subclass)
    # ==========================================================================

    # Grippers and handles for factory mode
    GRIPPERS: list[str] = []
    HANDLES_PER_OBJECT: list[list[str]] = []
    CONTACT_SURFACES_PER_OBJECT: list[list[str]] = []
    ENVIRONMENT_CONTACTS: list[str] = []

    # Valid gripper-handle pairs (for setPossibleGrasps)
    VALID_PAIRS: dict[str, list[str]] = {}

    # Factory rules (for setRules); when set, they replace VALID_PAIRS
    RULES: list | None = None

    # Arm groups for auto-freeze (populated from YAML arm_groups section)
    # ARM_GROUPS: raw arm group data {arm_name: {joint_keyword, grippers}}
    # GRIPPER_TO_ARM_KEYWORD: {full_gripper_name: joint_keyword | [joint_keyword, ...]}
    # ALL_ARM_KEYWORDS: ordered list of joint keywords
    ARM_GROUPS: dict = {}
    GRIPPER_TO_ARM_KEYWORD: dict[str, str | list[str]] = {}
    ALL_ARM_KEYWORDS: list[str] = []
    # JOINT_GROUPS (optional): {joint_keyword: [joint names]}, used to
    # restrict TOPPRA time parameterization to the unfrozen arms' joints
    JOINT_GROUPS: dict[str, list[str]] = {}

    # ==========================================================================
    # Initial Configuration (Override in subclass)
    # ==========================================================================

    INITIAL_ROBOT_CONFIG: list[float] = []
    INITIAL_OBJECT_CONFIGS: dict[str, list[float]] = {}

    # ==========================================================================
    # Planning Parameters (Use defaults or override)
    # ==========================================================================

    PATH_VALIDATION_STEP: float = Defaults.PATH_VALIDATION_STEP
    PATH_PROJECTOR_STEP: float = Defaults.PATH_PROJECTOR_STEP
    MAX_ITERATIONS: int = Defaults.MAX_ITERATIONS
    ERROR_THRESHOLD: float = Defaults.ERROR_THRESHOLD
    MAX_RANDOM_ATTEMPTS: int = Defaults.MAX_RANDOM_ATTEMPTS
    RANDOM_SHORTCUT_LOOPS: int = Defaults.RANDOM_SHORTCUT_LOOPS
    SPLINE_ZERO_DERIVATIVES_AT_STATE: bool = Defaults.SPLINE_ZERO_DERIVATIVES_AT_STATE
    TIME_PARAM_SAFETY: float = Defaults.TIME_PARAM_SAFETY
    TIME_PARAM_ORDER: int = Defaults.TIME_PARAM_ORDER
    TIME_PARAM_METHOD: str = Defaults.TIME_PARAM_METHOD
    TOPPRA_VELOCITY_SCALE: float = Defaults.TOPPRA_VELOCITY_SCALE
    TOPPRA_EFFORT_SCALE: float = Defaults.TOPPRA_EFFORT_SCALE
    TOPPRA_SOLVER: int = Defaults.TOPPRA_SOLVER
    TOPPRA_N: int = Defaults.TOPPRA_N
    TOPPRA_INTERPOLATION: str = Defaults.TOPPRA_INTERPOLATION
    TOPPRA_GRIDPOINT_METHOD: str = Defaults.TOPPRA_GRIDPOINT_METHOD
    TOPPRA_ACTIVE_JOINTS: list = Defaults.TOPPRA_ACTIVE_JOINTS

    # ==========================================================================
    # Graph Definition (Override in subclass)
    # ==========================================================================

    STATES: dict[str, StateDef] = {}
    EDGES: dict[str, EdgeDef] = {}
    CONSTRAINTS: list[ConstraintDef] = []

    # ==========================================================================
    # Helper Methods
    # ==========================================================================

    @classmethod
    def get_full_initial_config(cls) -> list[float]:
        """Build full initial configuration from robot and object configs."""
        config = list(cls.INITIAL_ROBOT_CONFIG)
        for obj_name in cls.OBJECTS:
            if obj_name in cls.INITIAL_OBJECT_CONFIGS:
                config.extend(cls.INITIAL_OBJECT_CONFIGS[obj_name])
        return config

    @classmethod
    def get_handles_per_object(cls) -> list[list[str]]:
        """
        Get handles per object.

        Uses HANDLES_PER_OBJECT if defined, otherwise infers from VALID_PAIRS.
        """
        if cls.HANDLES_PER_OBJECT:
            return cls.HANDLES_PER_OBJECT

        # Infer from VALID_PAIRS
        handles_by_object: dict[str, list[str]] = {}
        for gripper, handles in cls.VALID_PAIRS.items():
            for handle in handles:
                obj_name = handle.split("/")[0]
                if obj_name not in handles_by_object:
                    handles_by_object[obj_name] = []
                if handle not in handles_by_object[obj_name]:
                    handles_by_object[obj_name].append(handle)

        return [handles_by_object.get(obj, []) for obj in cls.OBJECTS]

    @classmethod
    def get_constraint_defs(cls) -> list[ConstraintDef]:
        """Return constraint definitions for this task."""
        return cls.CONSTRAINTS

    @classmethod
    def get_state_names(cls) -> list[str]:
        """Get list of state names."""
        return list(cls.STATES.keys())

    @classmethod
    def get_edge_names(cls) -> list[str]:
        """Get list of edge names."""
        return list(cls.EDGES.keys())

    @classmethod
    def validate(cls) -> list[str]:
        """Validate configuration, return list of warnings/errors."""
        errors = []

        if not cls.ROBOT_NAMES:
            errors.append("No ROBOT_NAMES defined")

        if not cls.GRIPPERS and not cls.VALID_PAIRS:
            errors.append("No GRIPPERS or VALID_PAIRS defined")

        # Check that all handles in VALID_PAIRS belong to objects in OBJECTS
        for gripper, handles in cls.VALID_PAIRS.items():
            for handle in handles:
                obj_name = handle.split("/")[0]
                if obj_name not in cls.OBJECTS:
                    errors.append(
                        f"Handle '{handle}' belongs to object '{obj_name}' "
                        f"which is not in OBJECTS"
                    )

        return errors


# =============================================================================
# Utility Functions
# =============================================================================


def merge_configs(base: dict, override: dict) -> dict:
    """Deep merge two configuration dictionaries."""
    result = base.copy()
    for key, value in override.items():
        if key in result:
            if isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = merge_configs(result[key], value)
                continue
        result[key] = value
    return result


__all__ = [
    "BaseTaskConfig",
    "ConstraintDef",
    "Defaults",
    "EdgeDef",
    "ModelPaths",
    "StateDef",
    "TransformConfig",
    "merge_configs",
    "xyzquat_to_xyzrpy",
    "xyzrpy_to_xyzquat",
]
