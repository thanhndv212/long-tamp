#!/usr/bin/env python3
"""
Configuration generation and management for manipulation tasks.

Provides ConfigGenerator for generating and validating configurations.
"""

from collections import deque
from typing import Dict, List, Optional, Tuple
import numpy as np

# Import transformation utilities
# NOTE: InitialConfigurations is imported lazily inside build_robot_config()
# and build_object_configs() so that importing this module does not
# unconditionally pull in SpaceLab-specific configuration.
try:
    from agimus_spacelab.utils import xyzrpy_to_xyzquat
except ImportError:
    xyzrpy_to_xyzquat = None


def bfs_edge_path(
    start_state: str,
    goal_state: str,
    edge_topology: Dict[str, Tuple[str, str]],
) -> List[str]:
    """Find a directed edge-name path from start_state to goal_state.

    Args:
        start_state: Source state name.
        goal_state: Target state name.
        edge_topology: Mapping edge_name -> (src_state, dst_state)

    Returns:
        List of edge names forming a path. Empty if no path exists.
    """

    adjacency: Dict[str, List[Tuple[str, str]]] = {}
    for edge_name, (src, dst) in edge_topology.items():
        adjacency.setdefault(src, []).append((dst, edge_name))

    queue: deque[str] = deque([start_state])
    prev_state: Dict[str, str] = {}
    prev_edge: Dict[str, str] = {}
    visited = {start_state}

    while queue:
        state = queue.popleft()
        if state == goal_state:
            break
        for nxt, edge_name in adjacency.get(state, []):
            if nxt in visited:
                continue
            visited.add(nxt)
            prev_state[nxt] = state
            prev_edge[nxt] = edge_name
            queue.append(nxt)

    if goal_state not in visited:
        return []

    edges: List[str] = []
    cur = goal_state
    while cur != start_state:
        edges.append(prev_edge[cur])
        cur = prev_state[cur]
    edges.reverse()
    return edges


def freeze_joints_by_substrings(
    robot,
    q: List[float],
    q_ref: List[float],
    joint_substrings: List[str],
    *,
    backend: str = "corba",
) -> List[float]:
    """Keep joint values constant for joints whose names match substrings.

    Intended use: tasks that do not use some robot groups (e.g. VISPA arms)
    can keep them fixed during projection/shooting to avoid drift.

    Notes:
    - For PyHPP, pinocchio model joint iteration would be needed to look up
      joint configuration ranks (model.names, joints[i].idx_q, joints[i].nq).
      This is not yet implemented; the function returns ``q`` unchanged.
      PYHPP-GAP: implement via device.model() when needed.
    - The CORBA robot API is expected to provide `getJointNames()`,
      `rankInConfiguration[...]`, `getJointConfigSize(name)`.
    """

    if backend.lower() != "corba":
        # PYHPP-GAP: pyhpp Device exposes pinocchio model via device.model().
        # Joint ranks are available as model.joints[i].idx_q and .nq.
        # Implementation pending; returning q unchanged.
        return q
    if robot is None:
        return q
    if not joint_substrings:
        return list(q)

    q_out = list(q)

    try:
        joint_names = robot.getJointNames()
    except Exception:
        return q_out

    for joint in joint_names:
        if not any(s in joint for s in joint_substrings):
            continue
        try:
            rank = robot.rankInConfiguration[joint]
            size = robot.getJointConfigSize(joint)
        except Exception:
            continue
        if size <= 0:
            continue
        if rank + size > len(q_out) or rank + size > len(q_ref):
            continue
        q_out[rank : rank + size] = q_ref[rank : rank + size]

    return q_out


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

    def update_graph(self, new_graph) -> None:
        """Update the constraint graph reference.

        Used by GraspSequencePlanner when rebuilding phase graphs to keep
        ConfigGenerator synchronized with the current graph.

        Args:
            new_graph: New ConstraintGraph or Graph instance
        """
        self.graph = new_graph

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
        config_label: Optional[str] = None,
        verbose: bool = True
    ) -> Tuple[bool, List[float]]:
        """
        Project configuration onto node constraints.
        
        Args:
            node_name: Name of the graph node
            q: Configuration to project
            config_label: Optional label to store result
            verbose: Print progress messages
            
        Returns:
            Tuple of (success, projected_config)
        """
        last_err = None
        last_valid_err = None
        for attempt in range(self.max_attempts):
            if self.backend == "corba":
                res, q_proj, err = self.graph.applyNodeConstraints(
                    node_name, list(q)
                )
                success = res
                config = q_proj
                last_err = err
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
                last_err = err
            if success:
                is_valid, valid_err = self.is_config_valid(config)
                if is_valid:
                    if config_label:
                        self.configs[config_label] = config
                        print(f"       ✓ SUCCESS   {config_label} projected onto node "
                              f"'{node_name}' after {attempt + 1} attempts")
                    return True, config
                last_valid_err = valid_err

        # All attempts failed
        if config_label:
            self.configs[config_label] = list(q)
            if verbose:
                print(f"       ⚠ Projection onto node FAILED: {node_name} "
                      f"({last_valid_err or last_err or 'unknown'})")

        return False, list(q)

    def generate_via_edge(
        self, edge_name: str, q_from: List[float],
        config_label: Optional[str] = None,
        verbose: bool = True,
        q_hint: Optional[List[float]] = None,
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        Generate target configuration by shooting random configs along edge.

        Args:
            edge_name: Name of the edge
            q_from: Source configuration (sets fold RHS via rightHandSideFromConfig)
            config_label: Optional label to store result
            verbose: Print progress every 200 attempts
            q_hint: Optional warm-start for the IK initial guess.  When set,
                the first attempt uses q_hint instead of a random config.
                Useful when a nearby solution is known (e.g. for release
                pregrasp generation: starting from q_grasped, the IK only
                needs to pull the arm back by the clearance distance).

        Returns:
            Tuple of (success, generated_config or None)
        """
        last_err = None
        last_valid_err = None
        for i in range(self.max_attempts):
            use_hint = (i == 0 and q_hint is not None)
            # Generate random config (or use hint on first attempt)
            if self.backend == "corba":
                q_rand = (list(q_hint) if use_hint
                          else self.planner.random_config())

                # omniORB stubs expect plain Python sequences (list/tuple),
                # not numpy arrays.
                q_from_seq = (
                    q_from.tolist()
                    if isinstance(q_from, np.ndarray)
                    else list(q_from)
                )
                q_rand_seq = (
                    q_rand.tolist()
                    if isinstance(q_rand, np.ndarray)
                    else list(q_rand)
                )

                res, q_target, err = self.graph.generateTargetConfig(
                    edge_name, q_from_seq, q_rand_seq
                )
                success = res
                config = q_target
                last_err = err
            else:  # pyhpp
                q_rand = (np.array(q_hint, dtype=float) if use_hint
                          else self.planner.random_config())
                q_from_arr = np.array(q_from) if not isinstance(
                    q_from, np.ndarray) else q_from

                # Keep all object freeflyer DOF from q_from in q_rand.
                # Objects not constrained by this edge (e.g. RS1 during a
                # Phase 1 frame_gripper-only edge) have no constraint and
                # their DOF pass through q_rand unchanged.  If we use a
                # fully random q_rand, those objects end up at random
                # positions in the generated config, corrupting q_init for
                # the next phase.  Objects that ARE constrained (e.g. a held
                # object via LockedJoint foliation) will have their DOF
                # projected to the correct value by the solver regardless of
                # what q_rand contains, so overriding them here is harmless.
                # Skip when using a hint — the hint already contains the
                # right joint values and mustn't be overwritten.
                if not use_hint:
                    try:
                        rank_map = dict(self.robot.rankInConfiguration)
                        q_rand_arr = np.array(q_rand, dtype=float)
                        for joint_name, rank in rank_map.items():
                            if joint_name.endswith("/root_joint"):
                                if rank + 7 <= len(q_rand_arr) and rank + 7 <= len(
                                    q_from_arr
                                ):
                                    q_rand_arr[rank : rank + 7] = q_from_arr[
                                        rank : rank + 7
                                    ]
                        q_rand = q_rand_arr
                    except Exception:
                        pass  # rankInConfiguration not available — fall back to fully random

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
                last_err = err

            if success:
                if verbose:
                    # Debug: print config and check for invalid values
                    print(f"       [DEBUG] Generated config via edge '{edge_name}' (attempt {i+1}):")
                    print(f"         config = {config}")
                    if config is None:
                        print(f"         [ERROR] Config is None!")
                        continue
                    config_arr = np.array(config)
                    if not np.all(np.isfinite(config_arr)):
                        print(f"         [ERROR] Config contains non-finite values: {config_arr}")
                        # Print offending indices and values
                        for idx, val in enumerate(config_arr):
                            if not np.isfinite(val):
                                print(f"           [BAD] idx={idx}, value={val}")
                        continue

                is_valid, valid_err = self.is_config_valid(config)
                if not is_valid:
                    last_valid_err = valid_err
                    continue

                if config_label:
                    self.configs[config_label] = config
                    print(f"       ✓ SUCCESS {config_label} generated via edge "
                          f"'{edge_name}' after {i + 1} attempts")
                return True, config

        if config_label and verbose:
            print(f"       ⚠ Generation via edge FAILED: {edge_name} "
                  f"({last_valid_err or last_err or 'unknown'})")
            # Diagnostic: print solver details on failure
            self._print_edge_failure_diagnostics(
                edge_name, q_from, q_hint, last_err, last_valid_err
            )
        return False, None

    def _print_edge_failure_diagnostics(
        self, edge_name, q_from, q_hint, last_err, last_valid_err,
    ):
        """Print diagnostic info when generate_via_edge exhausts all attempts."""
        print(f"       --- Edge failure diagnostics for '{edge_name}' ---")
        print(f"       Last solver residual: {last_err}")
        print(f"       Last validity error: {last_valid_err}")
        if q_hint is not None:
            q_from_arr = np.array(q_from, dtype=float)
            q_hint_arr = np.array(q_hint, dtype=float)
            diff = np.abs(q_hint_arr - q_from_arr)
            sig = np.where(diff > 1e-6)[0]
            if len(sig) > 0:
                print(f"       q_hint vs q_from differ at {len(sig)} DOFs:")
                for idx in sig[:20]:  # cap output
                    print(f"         [{idx}] q_from={q_from_arr[idx]:.6f}  "
                          f"q_hint={q_hint_arr[idx]:.6f}  "
                          f"delta={diff[idx]:.6f}")
            else:
                print("       q_hint == q_from (identical — cached pregrasp not available)")
        # Try applying targetConstraint of the edge to diagnose residuals
        # Use targetConstraint (end-state constraints) rather than
        # pathConstraint (fold), since generateTargetConfig projects to
        # the target state.
        try:
            if self.backend != "corba":
                transition = edge_name
                if isinstance(edge_name, str):
                    get_t = getattr(self.graph, "getTransition", None)
                    if callable(get_t):
                        transition = get_t(edge_name)
                # targetConstraint() is the end-state constraint set
                get_tc = getattr(transition, "targetConstraint", None)
                if callable(get_tc):
                    tc = get_tc()
                    get_proj = getattr(tc, "configProjector", None)
                    if callable(get_proj):
                        proj = get_proj()
                        if proj is not None:
                            q_from_arr = np.array(q_from, dtype=float)
                            # Test 1: apply to q_from (grasped)
                            try:
                                q_test_from = q_from_arr.copy()
                                proj.rightHandSideFromConfig(q_from_arr)
                                ok_from = proj.apply(q_test_from)
                                res_from = proj.residualError()
                                print(f"       targetConstraint.apply(q_from):  "
                                      f"residual={res_from:.6e}, success={ok_from}")
                            except Exception as de:
                                print(f"       targetConstraint.apply(q_from) failed: {de}")
                            # Test 2: apply to q_hint if different from q_from
                            if q_hint is not None:
                                q_hint_arr = np.array(q_hint, dtype=float)
                                if np.any(np.abs(q_hint_arr - q_from_arr) > 1e-6):
                                    try:
                                        q_test_hint = q_hint_arr.copy()
                                        proj.rightHandSideFromConfig(q_from_arr)
                                        ok_hint = proj.apply(q_test_hint)
                                        res_hint = proj.residualError()
                                        print(f"       targetConstraint.apply(q_hint): "
                                              f"residual={res_hint:.6e}, success={ok_hint}")
                                    except Exception as de2:
                                        print(f"       targetConstraint.apply(q_hint) failed: {de2}")
        except Exception as e:
            print(f"       Could not retrieve edge constraints: {e}")
        print("       --- End diagnostics ---")


# Alias for backward compatibility
ConfigurationGenerator = ConfigGenerator


__all__ = [
    "ConfigGenerator",
    "ConfigurationGenerator",
    "bfs_edge_path",
    "freeze_joints_by_substrings",
]
