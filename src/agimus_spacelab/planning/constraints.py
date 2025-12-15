#!/usr/bin/env python3
"""agimus_spacelab.planning.constraints

Constraint creation utilities for manipulation tasks.

This module contains two layers:
- `ConstraintBuilder`: creates backend-specific numerical constraints.
- `FactoryConstraintLibrary`: provides ConstraintGraphFactory-compatible
    naming helpers and optional enumeration of expected factory names.
"""

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence

# Import PyHPP constraint types (optional)
try:
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, StdVec_Bool as Mask
    HAS_PYHPP_CONSTRAINTS = True
except ImportError:
    HAS_PYHPP_CONSTRAINTS = False
    RelativeTransformation = None
    Transformation = None
    ComparisonTypes = None
    ComparisonType = None
    Implicit = None
    SE3 = None
    Mask = None

# Import transformation utilities
try:
    from agimus_spacelab.utils import xyzquat_to_se3
except ImportError:
    xyzquat_to_se3 = None


class ConstraintBuilder:
    """
    Helper class for creating transformation constraints.
    
    Supports dual backend (CORBA and PyHPP).
    """
    
    @staticmethod
    def create_grasp_constraint(
        ps, name: str, gripper: str, tool: str,
        transform: List[float],
        mask: List[bool] = None,
        robot=None, backend: str = "corba"
    ) -> Any:
        """
        Create a grasp constraint (rigid attachment).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            name: Constraint name
            gripper: Gripper joint/frame name
            tool: Tool joint/frame name
            transform: [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for DOF constraints (default: all True)
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        if mask is None:
            mask = [True] * 6
        
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint IDs
            joint_gripper = robot.model().getJointId(gripper)
            joint_tool = robot.model().getJointId(tool)
            
            # Convert transform to SE3
            grasp_tf = xyzquat_to_se3(transform)
            
            # Create mask
            mask_vec = Mask()
            mask_vec[:] = tuple(mask)
            
            # Create relative transformation constraint
            pc = RelativeTransformation.create(
                name, robot.asPinDevice(),
                joint_gripper, joint_tool,
                grasp_tf, SE3.Identity(), mask_vec
            )
            
            # Create comparison types (all EqualToZero for grasp)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * sum(mask))
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, mask_vec)
            print(f"    ✓ {name}: {gripper} -> {tool} (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                name, gripper, tool, transform, mask
            )
            print(f"    ✓ {name}: {gripper} -> {tool}")
            return None
        
    @staticmethod
    def create_placement_constraint(
        ps, name: str, tool: str,
        world_pose: List[float],
        mask: List[bool],
        robot=None, backend: str = "corba"
    ) -> Any:
        """
        Create a placement constraint (object on surface).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            name: Constraint name
            tool: Tool joint name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            mask: Boolean mask for DOF constraints
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint ID
            joint_tool = robot.model().getJointId(tool)
            
            # Convert pose to SE3
            placement_tf = xyzquat_to_se3(world_pose)
            
            # Create transformation constraint
            pc = Transformation.create(
                name, robot.asPinDevice(),
                joint_tool, SE3.Identity(), placement_tf, mask
            )
            
            # Create comparison types (EqualToZero for placement)
            num_constrained = sum(mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * num_constrained)
            implicit_mask = [True] * num_constrained
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, implicit_mask)
            print(f"    ✓ {name}: tool at {world_pose[:3]} (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                name, "", tool, world_pose, mask
            )
            print(f"    ✓ {name}: tool at {world_pose[:3]}")
            return None
        
    @staticmethod
    def create_complement_constraint(
        ps, base_name: str, tool: str,
        world_pose: List[float],
        complement_mask: List[bool],
        robot=None, backend: str = "corba"
    ) -> Any:
        """
        Create complement constraint (free DOFs).
        
        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            base_name: Base constraint name
            tool: Tool joint name
            world_pose: World pose [x, y, z, qx, qy, qz, qw]
            complement_mask: Boolean mask for complement DOFs
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
            
        Returns:
            Implicit constraint for PyHPP, None for CORBA
        """
        constraint_name = f"{base_name}/complement"
        
        if backend == "pyhpp":
            if robot is None:
                raise ValueError("robot parameter required for PyHPP backend")
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("PyHPP constraints not available")
            
            # Get joint ID
            joint_tool = robot.model().getJointId(tool)
            
            # Convert pose to SE3
            placement_tf = xyzquat_to_se3(world_pose)
            
            # Create transformation constraint
            pc = Transformation.create(
                constraint_name, robot.asPinDevice(),
                joint_tool, SE3.Identity(), placement_tf, complement_mask
            )
            
            # Complement uses Equality comparison type
            num_constrained = sum(complement_mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.Equality] * num_constrained)
            implicit_mask = [True] * num_constrained
            
            # Create implicit constraint
            constraint = Implicit.create(pc, cts, implicit_mask)
            print(f"    ✓ {constraint_name}: free DOFs (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                constraint_name, "", tool, world_pose, complement_mask
            )
            print(f"    ✓ {constraint_name}: free DOFs")
            return None


@dataclass(frozen=True)
class FactoryConstraintLibrary:
    """Helper for ConstraintGraphFactory-compatible constraint names.

    The factory (both CORBA and PyHPP) uses fixed naming conventions:
    - Grasp:         "{gripper} grasps {handle}"
    - Pregrasp:      "{gripper} pregrasps {handle}"
    - Placement:     "place_{object}"
    - Preplacement:  "preplace_{object}"
    - Hold:          add suffix "/hold" (e.g. grasp/placement constraints)
    - Complements:   add suffix "/complement"

    This class centralizes those conventions so the rest of the framework can
    reliably reference factory-created constraints.
    """

    grippers: Sequence[str]
    objects: Sequence[str]
    handles_per_object: Sequence[Sequence[str]]

    @property
    def handles(self) -> List[str]:
        return [h for hs in self.handles_per_object for h in hs]

    # --- Naming helpers -------------------------------------------------

    @staticmethod
    def grasp(gripper: str, handle: str) -> str:
        return f"{gripper} grasps {handle}"

    @staticmethod
    def pregrasp(gripper: str, handle: str) -> str:
        return f"{gripper} pregrasps {handle}"

    @staticmethod
    def place(obj: str) -> str:
        return f"place_{obj}"

    @staticmethod
    def preplace(obj: str) -> str:
        return f"preplace_{obj}"

    @staticmethod
    def complement(name: str) -> str:
        return f"{name}/complement"

    @staticmethod
    def hold(name: str) -> str:
        return f"{name}/hold"

    @classmethod
    def grasp_complement(cls, gripper: str, handle: str) -> str:
        return cls.complement(cls.grasp(gripper, handle))

    @classmethod
    def grasp_hold(cls, gripper: str, handle: str) -> str:
        return cls.hold(cls.grasp(gripper, handle))

    @classmethod
    def place_complement(cls, obj: str) -> str:
        return cls.complement(cls.place(obj))

    @classmethod
    def place_hold(cls, obj: str) -> str:
        return cls.hold(cls.place(obj))

    # --- Enumeration (optional) ----------------------------------------

    def iter_expected_placement_names(self) -> Iterable[str]:
        for obj in self.objects:
            yield self.place(obj)
            yield self.place_complement(obj)
            yield self.place_hold(obj)
            yield self.preplace(obj)

    def iter_expected_grasp_names(
        self,
        valid_pairs: Optional[Dict[str, List[str]]] = None,
    ) -> Iterable[str]:
        """Yield grasp-related names for all (gripper, handle) pairs.

        If `valid_pairs` is provided, only yields names for those allowed
        (gripper -> handles) associations.
        """

        if valid_pairs is None:
            for gripper in self.grippers:
                for handle in self.handles:
                    g = self.grasp(gripper, handle)
                    yield g
                    yield self.complement(g)
                    yield self.hold(g)
                    yield self.pregrasp(gripper, handle)
        else:
            for gripper, handles in valid_pairs.items():
                for handle in handles:
                    g = self.grasp(gripper, handle)
                    yield g
                    yield self.complement(g)
                    yield self.hold(g)
                    yield self.pregrasp(gripper, handle)

    def iter_all_expected_factory_names(
        self,
        valid_pairs: Optional[Dict[str, List[str]]] = None,
    ) -> Iterable[str]:
        yield from self.iter_expected_placement_names()
        yield from self.iter_expected_grasp_names(valid_pairs=valid_pairs)

    @staticmethod
    def is_factory_reserved_name(name: str) -> bool:
        # Keep this intentionally simple: it protects the known factory
        # conventions and helps avoid accidental collisions.
        return any(
            predicate
            for predicate in (
                " grasps " in name,
                " pregrasps " in name,
                name.startswith("place_"),
                name.startswith("preplace_"),
                name.endswith("/hold"),
            )
        )


__all__ = [
    "ConstraintBuilder",
    "FactoryConstraintLibrary",
]
