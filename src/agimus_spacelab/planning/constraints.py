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
        LockedJoint,
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
    LockedJoint = None
    SE3 = None
    Mask = None

# Import transformation utilities
try:
    from agimus_spacelab.utils import xyzquat_to_se3
except ImportError:
    xyzquat_to_se3 = None


def get_joint_id_for_name(robot, name: str) -> tuple:
    """Get joint ID and frame placement for a joint or frame name.

    If `name` is a joint name, returns (joint_id, SE3.Identity()).
    If `name` is a frame name, returns (parent_joint_id, frame_placement)
    where frame_placement is the SE3 transform from parent joint to frame.

    Args:
        robot: Robot/Device instance with model() method
        name: Joint name or frame name

    Returns:
        Tuple of (joint_id: int, placement: SE3)
        - joint_id: The joint ID (or parent joint ID for frames)
        - placement: SE3.Identity() for joints, or frame.placement for frames

    Raises:
        ValueError: If name is neither a valid joint nor frame name
    """
    model = robot.model()

    # Check if it's a joint name
    if model.existJointName(name):
        return model.getJointId(name), SE3.Identity()

    # Check if it's a frame name
    if model.existFrame(name):
        frame_id = model.getFrameId(name)
        frame = model.frames[frame_id]
        return frame.parentJoint, frame.placement

    raise ValueError(
        f"'{name}' is neither a joint name nor a frame name in the robot model"
    )


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

            # Get joint IDs and frame placements (handles both joint and frame names)
            joint_gripper, frame1_placement = get_joint_id_for_name(robot, gripper)
            joint_tool, frame2_placement = get_joint_id_for_name(robot, tool)

            # Convert user transform to SE3
            grasp_tf = xyzquat_to_se3(transform)

            # Compose transforms: frame1_placement * grasp_tf for joint1 frame
            # and frame2_placement for joint2 frame
            # RelativeTransformation expects transforms from joint to constraint frame
            frame1_tf = frame1_placement * grasp_tf
            frame2_tf = frame2_placement

            # Create mask
            mask_vec = Mask()
            mask_vec[:] = tuple(mask)

            # Create relative transformation constraint
            pc = RelativeTransformation(
                name,
                robot,
                joint_gripper,
                joint_tool,
                frame1_tf,
                frame2_tf,
                mask_vec,
            )

            # Create comparison types (all EqualToZero for grasp)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * sum(mask))

            # Create implicit constraint
            constraint = Implicit(pc, cts, mask_vec)
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

            # Get joint ID and frame placement (handles both joint and frame names)
            joint_tool, frame_placement = get_joint_id_for_name(robot, tool)

            # Convert world pose to SE3
            world_tf = xyzquat_to_se3(world_pose)

            # Transformation constraint: joint frame should be at world_tf
            # If tool is a frame, we need frame_placement as the local transform
            # The constraint is: world_tf = joint_tf * frame_placement
            # So joint_tf should satisfy this relationship

            # Create transformation constraint
            pc = Transformation(
                name,
                robot,
                joint_tool,
                frame_placement,
                world_tf,
                mask,
            )

            # Create comparison types (EqualToZero for placement)
            num_constrained = sum(mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.EqualToZero] * num_constrained)
            implicit_mask = [True] * num_constrained

            # Create implicit constraint
            constraint = Implicit(pc, cts, implicit_mask)
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

            # Get joint ID and frame placement (handles both joint and frame names)
            joint_tool, frame_placement = get_joint_id_for_name(robot, tool)

            # Convert world pose to SE3
            world_tf = xyzquat_to_se3(world_pose)

            # Create transformation constraint
            pc = Transformation(
                constraint_name,
                robot,
                joint_tool,
                frame_placement,
                world_tf,
                complement_mask,
            )

            # Complement uses Equality comparison type
            num_constrained = sum(complement_mask)
            cts = ComparisonTypes()
            cts[:] = tuple([ComparisonType.Equality] * num_constrained)
            implicit_mask = [True] * num_constrained

            # Create implicit constraint
            constraint = Implicit(pc, cts, implicit_mask)
            print(f"    ✓ {constraint_name}: free DOFs (PyHPP)")
            return constraint
        else:
            # CORBA backend
            ps.createTransformationConstraint(
                constraint_name, "", tool, world_pose, complement_mask
            )
            print(f"    ✓ {constraint_name}: free DOFs")
            return None

    @staticmethod
    def create_constraints_from_defs(
        ps,
        constraint_defs: Iterable[tuple],
        robot=None,
        backend: str = "corba",
    ) -> Dict[str, Any]:
        """Create constraints from a list of (type, name, args) definitions.

        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            constraint_defs: Iterable of (ctype, name, args) tuples where:
                - ctype: "grasp", "placement", or "complement"
                - name: Constraint name
                - args: Dict with keys depending on ctype:
                    - grasp: gripper, obj, transform, mask
                    - placement: obj, transform, mask
                    - complement: obj, transform, mask
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"

        Returns:
            Dict mapping constraint names to constraint objects (PyHPP) or
            empty dict (CORBA, constraints stored in problem solver).
        """
        constraints: Dict[str, Any] = {}

        for ctype, name, args in constraint_defs:
            if ctype == "grasp":
                result = ConstraintBuilder.create_grasp_constraint(
                    ps, name,
                    args["gripper"], args["obj"],
                    args["transform"], args.get("mask"),
                    robot=robot, backend=backend
                )
            elif ctype == "placement":
                result = ConstraintBuilder.create_placement_constraint(
                    ps, name,
                    args["obj"], args["transform"], args["mask"],
                    robot=robot, backend=backend
                )
            elif ctype == "complement":
                result = ConstraintBuilder.create_complement_constraint(
                    ps, name,
                    args["obj"], args["transform"], args["mask"],
                    robot=robot, backend=backend
                )
            else:
                continue

            # Store constraint for PyHPP
            if backend == "pyhpp" and result is not None:
                key = f"{name}/complement" if ctype == "complement" else name
                constraints[key] = result

        return constraints

    @staticmethod
    def create_locked_joint_constraints(
        ps,
        robot,
        q_ref: List[float],
        patterns: List[str],
        backend: str = "corba",
    ) -> tuple:
        """Create locked joint constraints for freezing joints.

        Locks joints whose names contain any of the given pattern substrings.

        Args:
            ps: Problem solver instance (CORBA) or Problem (PyHPP)
            robot: Robot/Device instance
            q_ref: Reference configuration to extract joint values from
            patterns: List of substrings to match against joint names
                (case-insensitive)
            backend: "corba" or "pyhpp"

        Returns:
            Tuple of (locked, joint_names):
            - CORBA: (List[str] constraint names, List[str] joint names)
            - PyHPP:  (List[LockedJoint] objects,  List[str] joint names)

        Example:
            # Lock all gripper joints
            names, joints = ConstraintBuilder.create_locked_joint_constraints(
                ps, robot, q_init, ["gripper"]
            )
        """
        if not patterns or robot is None or ps is None:
            return [], []

        patterns_l = [p.lower() for p in patterns]

        if backend == "pyhpp":
            if not HAS_PYHPP_CONSTRAINTS:
                raise ImportError("pyhpp.constraints not available")

            model = robot.model()
            locked_constraints = []
            frozen_names: List[str] = []

            for jn in robot.getJointNames():
                if not any(p in jn.lower() for p in patterns_l):
                    continue
                if not model.existJointName(jn):
                    continue
                try:
                    jid = model.getJointId(jn)
                    rank = model.joints[jid].idx_q
                    size = model.joints[jid].nq
                except Exception:
                    continue
                if size <= 0 or rank < 0 or rank + size > len(q_ref):
                    continue

                import numpy as np
                # Boost.Python requires an Eigen vector (numpy array), not list
                values = np.array(q_ref[rank:rank + size], dtype=float)

                # Use EqualToZero: the locked value is baked into the
                # ConstantFunction at creation time.  Equality would make
                # this a foliation constraint whose RHS is updated from the
                # start config on every generateTargetConfig call, triggering
                # the use-after-free in ExplicitConstraintSet::rightHandSideFromInput.
                # comparisonType must have size nv (velocity DOF), not nq.
                nv = model.joints[jid].nv
                comp = ComparisonTypes()
                comp[:] = tuple([ComparisonType.EqualToZero] * nv)

                try:
                    locked = LockedJoint(robot, jn, values, comp)
                    locked_constraints.append(locked)
                    frozen_names.append(jn)
                    print(f"    ✓ Locked joint (PyHPP): {jn} (nq={size}, nv={nv})")
                except Exception as e:
                    print(f"   ⚠ Failed to lock {jn}: {e}")

            return locked_constraints, frozen_names

        else:
            # CORBA backend
            get_joint_names = getattr(robot, "getJointNames", None)
            get_size = getattr(robot, "getJointConfigSize", None)
            rank_map = getattr(robot, "rankInConfiguration", None)
            create_locked = getattr(ps, "createLockedJoint", None)
            set_rhs = getattr(ps, "setConstantRightHandSide", None)

            all_required = [get_joint_names, get_size, create_locked]
            if not all(callable(f) for f in all_required):
                return [], []
            if rank_map is None:
                return [], []

            constraint_names: List[str] = []
            frozen_names = []

            for jn in get_joint_names():
                if not any(p in jn.lower() for p in patterns_l):
                    continue
                try:
                    size = int(get_size(jn))
                    rank = int(rank_map[jn])
                except Exception:
                    continue
                if size <= 0 or rank < 0 or rank + size > len(q_ref):
                    continue

                values = [float(q_ref[rank + k]) for k in range(size)]
                constraint_name = f"locked_{jn}"
                try:
                    create_locked(constraint_name, jn, values)
                    # Make RHS constant (True = EqualToZero semantics).
                    # False would call comparisonType(name, Equality) inside
                    # the CORBA server, making the joint a foliation constraint
                    # whose RHS is updated per leaf — triggering the
                    # use-after-free in ExplicitConstraintSet::rightHandSideFromInput.
                    if callable(set_rhs):
                        set_rhs(constraint_name, True)
                    constraint_names.append(constraint_name)
                    frozen_names.append(jn)
                    print(f"    ✓ Locked joint: {jn} (size={size})")
                except Exception as e:
                    print(f"   ⚠ Failed to lock {jn}: {e}")

            return constraint_names, frozen_names


class FactoryConstraintRegistry:
    """Register constraints for use with ConstraintGraphFactory.

    The ConstraintGraphFactory (both CORBA and PyHPP) uses fixed naming:
    - Grasp:         "{gripper} grasps {handle}"
    - Pregrasp:      "{gripper} pregrasps {handle}"
    - Placement:     "place_{object}"
    - Preplacement:  "preplace_{object}"
    - Hold:          suffix "/hold"
    - Complements:   suffix "/complement"

    This class creates constraints with factory-compatible names and stores
    them for passing to the factory:

    **CORBA backend:**
    - Constraints are created via `ps.createTransformationConstraint(name, ...)`
    - The factory checks `problem.getAvailable("numericalconstraint")` to see
      if a constraint exists; if so, it skips creation in `buildGrasp`/`buildPlacement`
    - Pass nothing extra to factory; constraints are already registered

    **PyHPP backend:**
    - Constraints are Implicit objects stored in `self.constraints` dict
    - Pass `self.constraints` to `ConstraintGraphFactory(graph, constraints=...)`
    - The factory's `ConstraintFactory.registerConstraints()` makes them available
    - Factory's `buildGrasp`/`buildPlacement` checks `n in available_constraints`

    Usage:
        # Create registry
        registry = FactoryConstraintRegistry(ps, robot, backend)

        # Register constraints with factory naming
        registry.register_grasp(gripper, handle, transform, mask)
        registry.register_placement(obj, pose, mask)
        registry.register_placement_complement(obj, pose, complement_mask)

        # For PyHPP: pass constraints to factory
        if backend == "pyhpp":
            factory = ConstraintGraphFactory(graph, constraints=registry.constraints)
        else:
            factory = ConstraintGraphFactory(graph)

        # Factory will skip creating constraints that already exist
        factory.generate()
    """

    def __init__(
        self,
        ps,
        robot=None,
        backend: str = "corba",
    ):
        """
        Initialize the registry.

        Args:
            ps: Problem solver (CORBA) or Problem (PyHPP)
            robot: Robot/Device instance (required for PyHPP)
            backend: "corba" or "pyhpp"
        """
        self.ps = ps
        self.robot = robot
        self.backend = backend.lower()
        # Store constraint objects (PyHPP) or just names (CORBA)
        # For PyHPP: pass this dict to ConstraintGraphFactory(graph, constraints=...)
        self.constraints: Dict[str, Any] = {}

    # --- Factory naming conventions ------------------------------------

    @staticmethod
    def grasp_name(gripper: str, handle: str) -> str:
        """Factory name for grasp constraint: '{gripper} grasps {handle}'"""
        return f"{gripper} grasps {handle}"

    @staticmethod
    def pregrasp_name(gripper: str, handle: str) -> str:
        """Factory name for pregrasp constraint: '{gripper} pregrasps {handle}'"""
        return f"{gripper} pregrasps {handle}"

    @staticmethod
    def place_name(obj: str) -> str:
        """Factory name for placement constraint: 'place_{obj}'"""
        return f"place_{obj}"

    @staticmethod
    def preplace_name(obj: str) -> str:
        """Factory name for preplacement constraint: 'preplace_{obj}'"""
        return f"preplace_{obj}"

    @staticmethod
    def complement_name(base: str) -> str:
        """Factory name for complement constraint: '{base}/complement'"""
        return f"{base}/complement"

    @staticmethod
    def hold_name(base: str) -> str:
        """Factory name for hold constraint: '{base}/hold'"""
        return f"{base}/hold"

    # --- Registration methods ------------------------------------------

    def register_grasp(
        self,
        gripper: str,
        handle: str,
        transform: List[float],
        mask: List[bool] = None,
    ) -> str:
        """Register a grasp constraint with factory naming.

        Creates constraint named "{gripper} grasps {handle}".

        Args:
            gripper: Gripper frame name
            handle: Handle frame name
            transform: [x, y, z, qx, qy, qz, qw]
            mask: DOF mask (default: all True)

        Returns:
            The factory constraint name
        """
        name = self.grasp_name(gripper, handle)
        result = ConstraintBuilder.create_grasp_constraint(
            self.ps, name, gripper, handle, transform, mask,
            robot=self.robot, backend=self.backend
        )
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_pregrasp(
        self,
        gripper: str,
        handle: str,
        transform: List[float],
        mask: List[bool] = None,
    ) -> str:
        """Register a pregrasp constraint with factory naming.

        Creates constraint named "{gripper} pregrasps {handle}".

        Args:
            gripper: Gripper frame name
            handle: Handle frame name
            transform: [x, y, z, qx, qy, qz, qw] (approach offset)
            mask: DOF mask (default: all True)

        Returns:
            The factory constraint name
        """
        name = self.pregrasp_name(gripper, handle)
        result = ConstraintBuilder.create_grasp_constraint(
            self.ps, name, gripper, handle, transform, mask,
            robot=self.robot, backend=self.backend
        )
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_placement(
        self,
        obj: str,
        world_pose: List[float],
        mask: List[bool],
    ) -> str:
        """Register a placement constraint with factory naming.

        Creates constraint named "place_{obj}".

        Args:
            obj: Object joint/frame name
            world_pose: [x, y, z, qx, qy, qz, qw]
            mask: DOF mask

        Returns:
            The factory constraint name
        """
        name = self.place_name(obj)
        result = ConstraintBuilder.create_placement_constraint(
            self.ps, name, obj, world_pose, mask,
            robot=self.robot, backend=self.backend
        )
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_placement_complement(
        self,
        obj: str,
        world_pose: List[float],
        complement_mask: List[bool],
    ) -> str:
        """Register a placement complement constraint with factory naming.

        Creates constraint named "place_{obj}/complement".

        Args:
            obj: Object joint/frame name
            world_pose: [x, y, z, qx, qy, qz, qw]
            complement_mask: DOF mask for free directions

        Returns:
            The factory constraint name
        """
        base_name = self.place_name(obj)
        result = ConstraintBuilder.create_complement_constraint(
            self.ps, base_name, obj, world_pose, complement_mask,
            robot=self.robot, backend=self.backend
        )
        name = self.complement_name(base_name)
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_grasp_complement(
        self,
        gripper: str,
        handle: str,
        transform: List[float],
        complement_mask: List[bool],
    ) -> str:
        """Register a grasp complement constraint with factory naming.

        Creates constraint named "{gripper} grasps {handle}/complement".

        Args:
            gripper: Gripper frame name
            handle: Handle frame name
            transform: [x, y, z, qx, qy, qz, qw]
            complement_mask: DOF mask for free directions

        Returns:
            The factory constraint name
        """
        base_name = self.grasp_name(gripper, handle)
        result = ConstraintBuilder.create_complement_constraint(
            self.ps, base_name, handle, transform, complement_mask,
            robot=self.robot, backend=self.backend
        )
        name = self.complement_name(base_name)
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_grasp_hold(
        self,
        gripper: str,
        handle: str,
        transform: List[float],
        mask: List[bool] = None,
    ) -> str:
        """Register a grasp hold constraint with factory naming.

        Creates constraint named "{gripper} grasps {handle}/hold".
        Hold constraints are used during transitions.

        Args:
            gripper: Gripper frame name
            handle: Handle frame name
            transform: [x, y, z, qx, qy, qz, qw]
            mask: DOF mask (default: all True)

        Returns:
            The factory constraint name
        """
        base_name = self.grasp_name(gripper, handle)
        name = self.hold_name(base_name)
        result = ConstraintBuilder.create_grasp_constraint(
            self.ps, name, gripper, handle, transform, mask,
            robot=self.robot, backend=self.backend
        )
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    def register_placement_hold(
        self,
        obj: str,
        world_pose: List[float],
        mask: List[bool],
    ) -> str:
        """Register a placement hold constraint with factory naming.

        Creates constraint named "place_{obj}/hold".
        Hold constraints are used during transitions.

        Args:
            obj: Object joint/frame name
            world_pose: [x, y, z, qx, qy, qz, qw]
            mask: DOF mask

        Returns:
            The factory constraint name
        """
        base_name = self.place_name(obj)
        name = self.hold_name(base_name)
        result = ConstraintBuilder.create_placement_constraint(
            self.ps, name, obj, world_pose, mask,
            robot=self.robot, backend=self.backend
        )
        if self.backend == "pyhpp" and result is not None:
            self.constraints[name] = result
        return name

    # --- Query methods -------------------------------------------------

    def get_constraint(self, name: str) -> Any:
        """Get constraint object by factory name (PyHPP only)."""
        return self.constraints.get(name)

    def get_all_constraints(self) -> Dict[str, Any]:
        """Get all registered constraint objects.

        For PyHPP: pass this to ConstraintGraphFactory(graph, constraints=...).
        For CORBA: this dict is empty (constraints registered in problem solver).
        """
        return dict(self.constraints)

    def get_factory_constraints_arg(self) -> Dict[str, Any]:
        """Get the constraints argument for ConstraintGraphFactory.

        Usage:
            factory = ConstraintGraphFactory(graph, **registry.get_factory_kwargs())
        or:
            if backend == "pyhpp":
                factory = ConstraintGraphFactory(
                    graph, constraints=registry.get_factory_constraints_arg()
                )
        """
        if self.backend == "pyhpp":
            return dict(self.constraints)
        return {}

    # --- Bulk registration from constraint defs ------------------------

    def register_from_defs(
        self,
        constraint_defs: List[tuple],
        obj_name: str,
        skip_placement: bool = False,
    ) -> Dict[str, str]:
        """Register constraints from get_constraint_defs() with factory naming.

        Maps constraint definitions to factory naming conventions:
        - "grasp" type -> "{gripper} grasps {handle}"
        - "placement" type -> "place_{obj_name}"
        - "complement" type -> "{base}/complement"

        Args:
            constraint_defs: List of (type, name, args) tuples from
                config.get_constraint_defs()
            obj_name: Object name for placement constraints (e.g., "frame_gripper")
            skip_placement: If True, skip registration of placement and
                complement constraints.  Use this with the PyHPP backend when
                the object has no contact surfaces: the
                ConstraintGraphFactory.buildPlacement no-contacts path
                internally creates LockedJoint constraints (which correctly
                parameterise the free-state foliation).  Pre-registering a
                RelativeTransformation placement constraint causes
                buildPlacement to bypass that path, resulting in the wrong
                constraint type on sub-edges and projection failures.

        Returns:
            Dict mapping user constraint names to factory constraint names

        Example:
            constraint_defs = [
                ("grasp", "grasp", {"gripper": "g", "obj": "h", ...}),
                ("placement", "placement", {"obj": "joint", ...}),
                ("complement", "placement", {"obj": "joint", ...}),
            ]
            name_map = registry.register_from_defs(constraint_defs, "frame_gripper")
            # name_map = {
            #     "grasp": "g grasps h",
            #     "placement": "place_frame_gripper",
            #     "placement/complement": "place_frame_gripper/complement",
            # }
        """
        name_map: Dict[str, str] = {}

        for ctype, user_name, args in constraint_defs:
            if ctype == "grasp":
                # Grasp constraint: "{gripper} grasps {handle}"
                gripper = args.get("gripper")
                handle = args.get("obj")
                transform = args.get("transform")
                mask = args.get("mask")
                factory_name = self.register_grasp(
                    gripper, handle, transform, mask
                )
                name_map[user_name] = factory_name

            elif ctype == "placement":
                if skip_placement:
                    continue
                # Placement constraint: "place_{obj_name}"
                transform = args.get("transform")
                mask = args.get("mask")
                # Use obj_name parameter, not the joint name from args
                factory_name = self.register_placement(
                    obj_name, transform, mask
                )
                name_map[user_name] = factory_name

            elif ctype == "complement":
                if skip_placement:
                    continue
                # Complement constraint: "{base}/complement"
                transform = args.get("transform")
                mask = args.get("mask")
                # Determine base constraint from user_name
                # e.g., user_name="placement" -> base is placement
                factory_name = self.register_placement_complement(
                    obj_name, transform, mask
                )
                name_map[f"{user_name}/complement"] = factory_name

        return name_map

    @staticmethod
    def is_factory_name(name: str) -> bool:
        """Check if a name follows factory naming conventions."""
        return any([
            " grasps " in name,
            " pregrasps " in name,
            name.startswith("place_"),
            name.startswith("preplace_"),
            name.endswith("/hold"),
            name.endswith("/complement"),
        ])


__all__ = [
    "ConstraintBuilder",
    "FactoryConstraintRegistry",
    "get_joint_id_for_name",
]
