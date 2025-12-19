#!/usr/bin/env python3
"""
Configuration generation and management for manipulation tasks.

Provides ConfigGenerator for generating and validating configurations.
"""

from typing import List, Tuple, Optional
import numpy as np

# Import from package config
from agimus_spacelab.config.spacelab_config import InitialConfigurations

# Import transformation utilities
try:
    from agimus_spacelab.utils import xyzrpy_to_xyzquat
except ImportError:
    xyzrpy_to_xyzquat = None


class ConfigGenerator:
    """
    Generate and validate configurations for manipulation tasks.
    
    Handles projection, random sampling, and waypoint generation.
    """

    def __init__(self, robot, graph, planner, ps, backend: str = "corba",
                 max_attempts: int = 1000):
        """
        Initialize configuration generator.
        
        Args:
            robot: Robot instance
            graph: ConstraintGraph or Graph instance
            planner: Unified Planner instance
            ps: ProblemSolver or Problem instance
            backend: "corba" or "pyhpp"
            max_attempts: Maximum random sampling attempts
        """
        self.robot = robot
        self.graph = graph
        self.planner = planner
        self.ps = ps
        self.backend = backend.lower()
        self.max_attempts = max_attempts
        self.configs = {}
        # PyHPP shooter for random configurations
        self._shooter = None
        if self.backend == "pyhpp":
            self._shooter = self.ps.configurationShooter()

    def is_config_valid(
        self, q: List[float], verbose: bool = False
    ) -> Tuple[bool, str]:
        """
        Check if a configuration is valid (collision-free and within bounds).
        
        Args:
            q: Configuration to validate
            verbose: If True, print validation result
            
        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if configuration is valid
            - error_message: Empty string if valid, otherwise describes the issue
        """
        if self.backend == "corba":
            is_valid, error_msg = self.robot.isConfigValid(list(q))
        else:  # pyhpp
            q_arr = np.array(q) if not isinstance(q, np.ndarray) else q
            is_valid, error_msg = self.ps.isConfigValid(q_arr)

        if verbose:
            if is_valid:
                print("       ✓ Configuration is valid")
            else:
                print(f"       ⚠ Configuration invalid: {error_msg}")

        return is_valid, error_msg

    def project_on_node(
        self, node_name: str, q: List[float],
        config_label: Optional[str] = None
    ) -> Tuple[bool, List[float]]:
        """
        Project configuration onto node constraints.
        
        Args:
            node_name: Name of the graph node
            q: Configuration to project
            config_label: Optional label to store result
            
        Returns:
            Tuple of (success, projected_config)
        """
        for attempt in range(self.max_attempts):
            if self.backend == "corba":
                res, q_proj, err = self.graph.applyNodeConstraints(
                    node_name, list(q)
                )
                success = res
                config = q_proj
            else:  # pyhpp
                # PyHPP bindings expect a State object (not a state name).
                q_arr = np.array(q) if not isinstance(q, np.ndarray) else q

                state_obj = node_name
                if isinstance(node_name, str):
                    # Try common APIs to fetch state object by name.
                    get_state = getattr(self.graph, "getState", None)
                    if callable(get_state):
                        try:
                            state_obj = get_state(node_name)
                        except Exception:
                            # getState may be overloaded for (config)->State
                            # and reject string; fall through to other names.
                            pass

                    if isinstance(state_obj, str):
                        for attr in ("getStateByName", "state", "getNode"):
                            fn = getattr(self.graph, attr, None)
                            if callable(fn):
                                try:
                                    state_obj = fn(node_name)
                                    break
                                except Exception:
                                    continue

                res, q_proj, err = self.graph.applyStateConstraints(
                    state_obj, q_arr
                )
                success = res
                config = q_proj.tolist() if success else None
            if success:
                # Validate the projected configuration
                is_valid, err_msg = self.is_config_valid(config)
                if is_valid:
                    if config_label:
                        self.configs[config_label] = config
                        print(f"       ✓ {config_label} projected onto "
                              f"'{node_name}' after {attempt + 1} attempts")
                    return True, config
                else:
                    if (attempt + 1) % 200 == 0:
                        print(f"       Projected config invalid: {err_msg}, "
                              f"attempt {attempt + 1}/{self.max_attempts}...")
                    continue
            else:
                if (attempt + 1) % 200 == 0:
                    print(f"       Projection failed, "
                          f"attempt {attempt + 1}/{self.max_attempts}...")
                continue

        # All attempts failed
        if config_label:
            self.configs[config_label] = list(q)
            print(f"       ⚠ Projection failed after {self.max_attempts} attempts")

        return False, list(q)

    def generate_via_edge(
        self, edge_name: str, q_from: List[float],
        config_label: Optional[str] = None,
        verbose: bool = True
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        Generate target configuration by shooting random configs along edge.
        
        Args:
            edge_name: Name of the edge
            q_from: Source configuration
            config_label: Optional label to store result
            verbose: Print progress every 200 attempts
            
        Returns:
            Tuple of (success, generated_config or None)
        """
        for i in range(self.max_attempts):
            # Generate random config
            if self.backend == "corba":
                q_rand = self.planner.random_config()
                res, q_target, err = self.graph.generateTargetConfig(
                    edge_name, q_from, q_rand
                )
                success = res
                config = q_target
            else:  # pyhpp
                q_rand = self.planner.random_config()
                q_from_arr = np.array(q_from) if not isinstance(
                    q_from, np.ndarray) else q_from

                # PyHPP bindings often expect a Transition object, not a name.
                transition = edge_name
                if isinstance(edge_name, str):
                    get_transition = getattr(self.graph, "getTransition", None)
                    if callable(get_transition):
                        try:
                            transition = get_transition(edge_name)
                        except Exception:
                            transition = edge_name

                try:
                    res, q_target, err = self.graph.generateTargetConfig(
                        transition, q_from_arr, q_rand
                    )
                except Exception:
                    # Fallback for bindings that accept the edge name.
                    res, q_target, err = self.graph.generateTargetConfig(
                        edge_name, q_from_arr, q_rand
                    )
                success = res
                config = q_target.tolist() if success else None
            if success:
                # Validate the generated configuration
                is_valid, err_msg = self.is_config_valid(config)
                if not is_valid:
                    # Config generated but invalid, continue trying
                    if verbose and (i + 1) % 200 == 0:
                        print(f"       Config invalid: {err_msg}, retrying...")
                    continue

                if config_label:
                    self.configs[config_label] = config
                    print(f"       ✓ {config_label} generated via "
                          f"'{edge_name}' after {i + 1} attempts")
                return True, config

            if verbose and (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}/{self.max_attempts}...")

        if config_label:
            print(f"       ⚠ Failed after {self.max_attempts} attempts")
        return False, None

    def build_robot_config(
        self, joint_groups: Optional[List[str]] = None
    ) -> List[float]:
        """
        Build robot configuration from joint group names.
        
        Args:
            joint_groups: List of joint group names to include.
                          Keys should match InitialConfigurations attributes
                          (e.g., ["UR10", "VISPA_BASE", "VISPA_ARM"]).
                          If None, uses default robot joint groups.
            
        Returns:
            Combined robot configuration
            
        Examples:
            # Use all defaults
            config = gen.build_robot_config()
            
            # Use specific groups
            config = gen.build_robot_config(["UR10", "VISPA_ARM"])
        """
        if joint_groups is None:
            joint_groups = ["UR10", "VISPA_BASE", "VISPA_ARM"]

        q_robot = []
        for group in joint_groups:
            if hasattr(InitialConfigurations, group):
                q_robot.extend(list(getattr(InitialConfigurations, group)))
            else:
                print(f"       ⚠ No initial config for joint group '{group}'")
                # Add zero configuration as fallback
        return q_robot

    def build_object_configs(self, object_names: List[str]) -> List[float]:
        """
        Build object configurations from initial poses.
        
        Args:
            object_names: List of object names
            
        Returns:
            Concatenated object configurations in XYZQUAT format
        """
        q_objects = []

        for obj_name in object_names:
            # Get initial pose in XYZRPY
            obj_attr = obj_name.replace("-", "_").replace(" ", "_").upper()
            if hasattr(InitialConfigurations, obj_attr):
                pose_xyzrpy = getattr(InitialConfigurations, obj_attr)
                pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
                q_objects.extend(pose_xyzquat.tolist())
            else:
                print(f"       ⚠ No initial config for {obj_name}")
                # Add zero configuration as fallback
                q_objects.extend([0.0] * 7)

        return q_objects

    def get_robot_dof(self, joint_groups: Optional[List[str]] = None) -> int:
        """
        Get total robot DOF for specified joint groups.
        
        Args:
            joint_groups: List of joint group names to include.
                          If None, uses default ["UR10", "VISPA_BASE", "VISPA_ARM"].
                          
        Returns:
            Total degrees of freedom
        """
        if joint_groups is None:
            joint_groups = ["UR10", "VISPA_BASE", "VISPA_ARM"]
        total_dof = 0
        for group in joint_groups:
            if hasattr(InitialConfigurations, group):
                total_dof += len(getattr(InitialConfigurations, group))
        return total_dof

    def modify_object_pose(
        self, q: List[float], object_index: int,
        translation_delta: Optional[List[float]] = None,
        quaternion: Optional[List[float]] = None
    ) -> List[float]:
        """
        Modify object pose in configuration.
        
        Args:
            q: Full configuration
            object_index: Index of object (0 for first object after robot)
            translation_delta: [dx, dy, dz] to add to position
            quaternion: New quaternion [qx, qy, qz, qw] (replaces existing)
            
        Returns:
            Modified configuration
        """
        q_new = list(q)
        robot_dof = self.get_robot_dof()
        obj_start = robot_dof + object_index * 7

        if translation_delta:
            for i in range(3):
                q_new[obj_start + i] += translation_delta[i]

        if quaternion:
            for i in range(4):
                q_new[obj_start + 3 + i] = quaternion[i]

        return q_new


# Alias for backward compatibility
ConfigurationGenerator = ConfigGenerator


__all__ = [
    "ConfigGenerator",
    "ConfigurationGenerator",
]
