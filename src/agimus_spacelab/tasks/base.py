#!/usr/bin/env python3
"""
Base class for manipulation tasks.

Provides ManipulationTask base class with common structure.
"""

from typing import List, Dict, Any, Optional, Tuple

import re

from agimus_spacelab.planning import (
    SceneBuilder,
    ConfigGenerator,
    GraphBuilder,
    ConstraintBuilder,
    FactoryConstraintRegistry
)


class ManipulationTask:
    """
    Base class for manipulation tasks.
    
    Provides common structure for task definition, constraint creation,
    graph building, and configuration management.
    """

    def __init__(
        self,
        joint_bounds=None,
        FILE_PATHS: Optional[Dict[str, Any]] = None,
        task_name: str = "Manipulation Task",
        backend: str = "corba",
    ):
        """
        Initialize manipulation task.
        
        Args:
            task_name: Descriptive name for the task
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.task_name = task_name
        self.backend = backend.lower()
        self.scene_builder = SceneBuilder(
            joint_bounds=joint_bounds,
            FILE_PATHS=FILE_PATHS,
            backend=backend,
        )
        self.planner = None
        self.robot = None
        self.ps = None
        self.graph = None
        self.config_gen = None

    def get_robot_names(self) -> List[str]:
        return self.config.ROBOT_NAMES

    def get_composite_names(self) -> List[str]:
        return self.config.ROBOT_NAMES

    def get_object_names(self) -> List[str]:
        return self.config.OBJECTS

    def get_environment_names(self) -> List[str]:
        return self.config.ENVIRONMENT_NAMES

    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return self.config.ROBOTS

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return self.config.OBJECTS

    def create_constraints(self) -> None:
        """Create all transformation constraints for both backends.

        In factory mode, uses FactoryConstraintRegistry to create constraints
        with factory naming conventions:
        - Grasp: "{gripper} grasps {handle}"
        - Placement: "place_{object}"
        - Complement: "{base}/complement"

        For CORBA, they're stored in the problem solver.
        For PyHPP, they're stored in self.pyhpp_constraints and passed to
        the factory constructor.
        """
        cfg = self.config

        # Get robot reference (differs by backend)
        robot = (
            self.planner.get_robot() if self.backend == "pyhpp" else self.robot
        )

        if self.use_factory:
            print("    Registering constraints for factory mode...")
            self._create_factory_constraints(robot)
        else:
            print("    Creating constraints manually...")
            self._create_manual_constraints(robot)

        print("   ✓ Created transformation constraints")

    def _create_factory_constraints(self, robot) -> None:
        """Create constraints with factory naming using FactoryConstraintRegistry.

        Uses constraint definitions from config but registers them with
        factory naming conventions.
        """
        cfg = self.config

        # Use FactoryConstraintRegistry for proper factory naming
        registry = FactoryConstraintRegistry(
            self.ps, robot=robot, backend=self.backend
        )

        # Get constraint definitions from config
        constraint_defs = cfg.get_constraint_defs()

        # Object name for placement constraints (e.g., "frame_gripper")
        obj_name = cfg.TOOL_NAME

        # Register all constraints with factory naming
        # Maps user names -> factory names
        self._constraint_name_map = registry.register_from_defs(
            constraint_defs, obj_name
        )

        # Store for PyHPP graph building
        self.pyhpp_constraints = registry.get_factory_constraints_arg()

    def _create_manual_constraints(self, robot) -> None:
        """Create constraints with custom naming (manual mode)."""
        cfg = self.config
        cb = ConstraintBuilder

        # Get constraint definitions from config
        constraint_defs = cfg.get_constraint_defs()

        # Create all constraints from definitions
        constraints = cb.create_constraints_from_defs(
            self.ps, constraint_defs, robot=robot, backend=self.backend
        )

        # Store for PyHPP graph building
        self.pyhpp_constraints = constraints

    def create_graph(self):
        """Create and configure constraint graph."""
        # Backend-specific objects
        if self.backend == "pyhpp":
            robot = self.planner.get_robot()
            problem = self.planner.get_problem()
        else:
            robot = self.robot
            problem = self.ps

        # Initialize GraphBuilder
        self.graph_builder = GraphBuilder(
            self.planner, robot, problem, backend=self.backend
        )

        if self.use_factory:
            # Pass pre-registered constraints to factory (PyHPP uses them
            # directly; CORBA already has them in the problem solver)
            return self.graph_builder.create_factory_graph(
                self.config,
                pyhpp_constraints=self.pyhpp_constraints,
            )
        else:
            return self.graph_builder.create_manual_graph(
                self.config,
                pyhpp_constraints=self.pyhpp_constraints,
            )

    def setup_collision_management(self) -> None:
        """
        Configure collision checking (disable expected contacts, etc.).
        Override in subclass if needed.
        """
        pass

    def build_initial_config(self) -> List[float]:
        """
        Build the initial configuration.
        Override in subclass if custom logic needed.
        """
        objects = self.get_objects()
        joint_groups = self.get_joint_groups()
        q_robot = self.config_gen.build_robot_config(joint_groups)
        q_objects = self.config_gen.build_object_configs(objects)
        return q_robot + q_objects

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """
        Generate all intermediate configurations.
        Override in subclass.
        """
        raise NotImplementedError(
            "Subclass must implement generate_configurations()"
        )

    def setup(self, validation_step: float = 0.01,
              projector_step: float = 0.1):
        """
        Complete task setup: scene, constraints, graph.
        
        Args:
            validation_step: Path validation discretization
            projector_step: Path projector step
        """
        print("=" * 70)
        print(f"{self.task_name}")
        print("=" * 70)

        # 1. Scene setup
        self.planner, self.robot, self.ps = self.scene_builder.build(
            robot_names=self.get_robot_names(),
            environment_names=self.get_environment_names(),
            composite_names=self.get_composite_names(),
            object_names=self.get_object_names(),
            validation_step=validation_step,
            projector_step=projector_step
        )

        # 2. Custom collision management
        self.setup_collision_management()

        # 3. Create constraints
        print("\n2. Creating constraints...")
        self.create_constraints()

        # 4. Create graph
        print("\n3. Creating constraint graph...")
        self.graph = self.create_graph()

        # 5. Initialize configuration generator
        self.config_gen = ConfigGenerator(
            self.robot, self.graph, self.planner, self.ps, backend=self.backend
        )

        print("\n   ✓ Task setup complete")

    def run(self, visualize: bool = True,
            solve: bool = False,
            preferred_configs: List[str] = [],
            max_iterations: int = 1000,
            freeze_joint_substrings: Optional[List[str]] = None,
            freeze_joint_eps: float = 0.0) -> Dict[str, Any]:
        """
        Run the complete task workflow.
        
        Args:
            visualize: Whether to visualize configurations
            solve: Whether to solve the planning problem
            
        Returns:
            Dictionary with configs, paths, and other results
        """
        if not self.graph:
            raise RuntimeError("Must call setup() before run()")

        # 4. Generate configurations
        print("\n4. Generating configurations...")
        q_init = self.build_initial_config()
        configs = self.generate_configurations(q_init)

        # 5. Visualize
        if visualize:
            print("\n5. Starting visualization...")
            try:
                self.planner.visualize(configs.get("q_init", q_init))
                print("   ✓ Initial configuration displayed")
            except Exception as e:
                print(f"   ⚠ Visualization failed: {e}")

        # 6. Solve
        if solve and "q_goal" in configs:
            print("\n6. Solving planning problem...")

            def _freeze_joints_for_planning(q_ref: List[float]) -> Tuple[List[str], List[Tuple[str, List[float]]]]:
                """Freeze joints by clamping bounds; returns (frozen_joint_names, restore_list)."""
                patterns = freeze_joint_substrings
                if patterns is None:
                    patterns = getattr(self, "FREEZE_JOINT_SUBSTRINGS", None)
                if not patterns:
                    return [], []

                robot = self.robot
                if robot is None or self.planner is None:
                    return [], []

                get_joint_names = getattr(robot, "getJointNames", None)
                get_size = getattr(robot, "getJointConfigSize", None)
                rank_map = getattr(robot, "rankInConfiguration", None)
                get_bounds = getattr(robot, "getJointBounds", None)

                if not callable(get_joint_names) or not callable(get_size) or rank_map is None:
                    return [], []

                patterns_l = [p.lower() for p in patterns]
                restore: List[Tuple[str, List[float]]] = []
                frozen_names: List[str] = []

                for jn in get_joint_names():
                    jn_l = jn.lower()
                    if not any(p in jn_l for p in patterns_l):
                        continue
                    try:
                        size = int(get_size(jn))
                        rank = int(rank_map[jn])
                    except Exception:
                        continue
                    if size <= 0 or rank < 0 or rank + size > len(q_ref):
                        continue

                    old = None
                    if callable(get_bounds):
                        try:
                            old = list(get_bounds(jn))
                        except Exception:
                            old = None
                    if old is not None:
                        restore.append((jn, old))

                    bounds: List[float] = []
                    for k in range(size):
                        v = float(q_ref[rank + k])
                        bounds.extend([v - freeze_joint_eps, v + freeze_joint_eps])
                    try:
                        self.planner.set_joint_bounds(jn, bounds)
                        frozen_names.append(jn)
                    except Exception:
                        # Best effort: if we can't set bounds, just skip.
                        continue

                return frozen_names, restore

            def _restore_joint_bounds(restore: List[Tuple[str, List[float]]]) -> None:
                if not restore or self.planner is None:
                    return
                for jn, bounds in restore:
                    try:
                        self.planner.set_joint_bounds(jn, bounds)
                    except Exception:
                        pass

            def _reset_goals_if_possible() -> None:
                if self.ps is None:
                    return
                reset = getattr(self.ps, "resetGoalConfigs", None)
                if callable(reset):
                    reset()

            def _ordered_config_keys(cfgs: Dict[str, Any]) -> List[str]:
                # Always plan from q_init to q_goal.
                if "q_init" not in cfgs or "q_goal" not in cfgs:
                    return []

                # Factory mode: q_wp_<i>_<edge>
                wp = []
                for k in cfgs.keys():
                    m = re.match(r"^q_wp_(\d+)_", k)
                    if m:
                        wp.append((int(m.group(1)), k))
                if wp:
                    wp_sorted = [k for _, k in sorted(wp, key=lambda t: t[0])]
                    return ["q_init", *wp_sorted, "q_goal"]

                # Manual mode (common naming convention)
                preferred = preferred_configs
                mids = [k for k in preferred if k in cfgs]
                if mids:
                    return ["q_init", *mids, "q_goal"]

                # Fallback: any q_* keys in insertion order.
                mids = [
                    # k
                    # for k in cfgs.keys()
                    # if k.startswith("q_") and k not in ("q_init", "q_goal")
                ]
                return ["q_init", *mids, "q_goal"]

            seq = _ordered_config_keys(configs)
            frozen_names, restore = _freeze_joints_for_planning(configs.get("q_init", q_init))
            if frozen_names:
                frozen_names_sorted = sorted(frozen_names)
                eps = float(freeze_joint_eps)
                print(f"   ✓ Frozen joints during planning (eps={eps:g}): {', '.join(frozen_names_sorted)}")
            try:
                if not seq or len(seq) < 2:
                    print("   ⚠ Planning skipped: missing q_init/q_goal")
                elif len(seq) == 2:
                    _reset_goals_if_possible()
                    self.planner.set_initial_config(configs["q_init"])
                    self.planner.add_goal_config(configs["q_goal"])
                    success = self.planner.solve(max_iterations=max_iterations)
                    if success:
                        print("   ✓ Planning successful")
                    else:
                        print("   ⚠ Planning failed")

                    if success and visualize:
                        print("\n7. Playing solution path...")
                        try:
                            self.planner.play_path(0)
                            print("   ✓ Path playback complete")
                        except Exception as e:
                            print(f"   ⚠ Path playback failed: {e}")
                else:
                    path_ids: List[int] = []
                    for i in range(len(seq) - 1):
                        a, b = seq[i], seq[i + 1]
                        print(f"\n   Segment {i+1}/{len(seq)-1}: {a} -> {b}")
                        _reset_goals_if_possible()
                        self.planner.set_initial_config(configs[a])
                        self.planner.add_goal_config(configs[b])
                        success = self.planner.solve(max_iterations=max_iterations)
                        if success:
                            print("   ✓ Planning successful")
                        else:
                            print("   ⚠ Planning failed")
                            break
                        # Record the latest path id when available (CORBA).
                        if self.ps is not None:
                            num_paths = getattr(self.ps, "numberPaths", None)
                            if callable(num_paths):
                                try:
                                    path_ids.append(int(num_paths()) - 1)
                                except Exception:
                                    pass

                    # Concatenate path segments when available (CORBA).
                    if len(path_ids) > 1 and self.ps is not None:
                        concat = getattr(self.ps, "concatenatePath", None)
                        if callable(concat):
                            try:
                                for j in range(1, len(path_ids)):
                                    concat(path_ids[0], path_ids[j])
                            except Exception:
                                pass

                    if visualize:
                        print("\n7. Playing solution path...")
                        try:
                            # Use concatenated path id when known, otherwise fall back to 0.
                            pid = path_ids[0] if path_ids else 0
                            self.planner.play_path(pid)
                            print("   ✓ Path playback complete")
                        except Exception as e:
                            print(f"   ⚠ Path playback failed: {e}")
            finally:
                _restore_joint_bounds(restore)

        return {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
            "viewer": self.planner.viewer if self.planner else None
        }


__all__ = ["ManipulationTask"]
