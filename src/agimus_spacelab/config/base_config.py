"""
Base configuration classes for manipulation tasks.

Provides abstract base classes and default values that can be overridden
by specific task configurations.
"""

from abc import ABC
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np
from agimus_spacelab.utils.transforms import xyzrpy_to_xyzquat, xyzquat_to_xyzrpy

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
        x: Tuple[float, float] = (-2.0, 2.0),
        y: Tuple[float, float] = (-3.0, 3.0),
        z: Tuple[float, float] = (-2.0, 2.0),
    ) -> List[float]:
        """Get freeflyer bounds as flat list [xmin, xmax, ymin, ymax, ...]."""
        qb = cls.QUATERNION_BOUNDS
        return [
            x[0], x[1], y[0], y[1], z[0], z[1],
            qb, qb, qb, qb, qb, qb, qb, qb
        ]


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
    position: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    quaternion: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0]
    )
    
    def as_xyzquat(self) -> List[float]:
        """Return as [x, y, z, qx, qy, qz, qw]."""
        return self.position + self.quaternion
    
    @classmethod
    def from_xyzquat(cls, xyzquat: List[float]) -> "TransformConfig":
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
    gripper: Optional[str] = None  # For grasp constraints
    obj: Optional[str] = None  # Object joint name
    transform: Optional[List[float]] = None  # XYZQUAT
    mask: List[bool] = field(default_factory=lambda: Defaults.MASK_ALL)


@dataclass
class EdgeDef:
    """Definition of a graph edge."""
    name: str
    from_state: str
    to_state: str
    containing_state: str
    weight: float = 1.0
    path_constraints: List[str] = field(default_factory=list)


@dataclass
class StateDef:
    """Definition of a graph state."""
    name: str
    constraints: List[str] = field(default_factory=list)
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
    PATHS: Dict[str, Dict] = {}
    
    # Robot configuration
    ROBOT_NAMES: List[str] = []
    ENVIRONMENT_NAMES: List[str] = []
    OBJECTS: List[str] = []
    
    # ==========================================================================
    # Manipulation Configuration (Override in subclass)
    # ==========================================================================
    
    # Grippers and handles for factory mode
    GRIPPERS: List[str] = []
    HANDLES_PER_OBJECT: List[List[str]] = []
    CONTACT_SURFACES_PER_OBJECT: List[List[str]] = []
    ENVIRONMENT_CONTACTS: List[str] = []
    
    # Valid gripper-handle pairs (for setPossibleGrasps)
    VALID_PAIRS: Dict[str, List[str]] = {}
    
    # ==========================================================================
    # Initial Configuration (Override in subclass)
    # ==========================================================================
    
    INITIAL_ROBOT_CONFIG: List[float] = []
    INITIAL_OBJECT_CONFIGS: Dict[str, List[float]] = {}
    
    # ==========================================================================
    # Planning Parameters (Use defaults or override)
    # ==========================================================================
    
    PATH_VALIDATION_STEP: float = Defaults.PATH_VALIDATION_STEP
    PATH_PROJECTOR_STEP: float = Defaults.PATH_PROJECTOR_STEP
    MAX_ITERATIONS: int = Defaults.MAX_ITERATIONS
    ERROR_THRESHOLD: float = Defaults.ERROR_THRESHOLD
    MAX_RANDOM_ATTEMPTS: int = Defaults.MAX_RANDOM_ATTEMPTS
    
    # ==========================================================================
    # Graph Definition (Override in subclass)
    # ==========================================================================
    
    STATES: Dict[str, StateDef] = {}
    EDGES: Dict[str, EdgeDef] = {}
    CONSTRAINTS: List[ConstraintDef] = []
    
    # ==========================================================================
    # Helper Methods
    # ==========================================================================
    
    @classmethod
    def get_full_initial_config(cls) -> List[float]:
        """Build full initial configuration from robot and object configs."""
        config = list(cls.INITIAL_ROBOT_CONFIG)
        for obj_name in cls.OBJECTS:
            if obj_name in cls.INITIAL_OBJECT_CONFIGS:
                config.extend(cls.INITIAL_OBJECT_CONFIGS[obj_name])
        return config
    
    @classmethod
    def get_handles_per_object(cls) -> List[List[str]]:
        """
        Get handles per object.
        
        Uses HANDLES_PER_OBJECT if defined, otherwise infers from VALID_PAIRS.
        """
        if cls.HANDLES_PER_OBJECT:
            return cls.HANDLES_PER_OBJECT
        
        # Infer from VALID_PAIRS
        handles_by_object: Dict[str, List[str]] = {}
        for gripper, handles in cls.VALID_PAIRS.items():
            for handle in handles:
                obj_name = handle.split("/")[0]
                if obj_name not in handles_by_object:
                    handles_by_object[obj_name] = []
                if handle not in handles_by_object[obj_name]:
                    handles_by_object[obj_name].append(handle)
        
        return [handles_by_object.get(obj, []) for obj in cls.OBJECTS]
    
    @classmethod
    def get_constraint_defs(cls) -> List[ConstraintDef]:
        """Return constraint definitions for this task."""
        return cls.CONSTRAINTS
    
    @classmethod
    def get_state_names(cls) -> List[str]:
        """Get list of state names."""
        return list(cls.STATES.keys())
    
    @classmethod
    def get_edge_names(cls) -> List[str]:
        """Get list of edge names."""
        return list(cls.EDGES.keys())
    
    @classmethod
    def validate(cls) -> List[str]:
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

def merge_configs(base: Dict, override: Dict) -> Dict:
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
    "Defaults",
    "ModelPaths",
    "TransformConfig",
    "ConstraintDef",
    "EdgeDef",
    "StateDef",
    "BaseTaskConfig",
    "merge_configs",
    "xyzrpy_to_xyzquat",
    "xyzquat_to_xyzrpy",
]
