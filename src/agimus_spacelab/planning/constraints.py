#!/usr/bin/env python3
"""
Constraint creation utilities for manipulation tasks.

Provides ConstraintBuilder for creating transformation constraints.
"""

from typing import List, Any, Optional

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


__all__ = ["ConstraintBuilder"]
