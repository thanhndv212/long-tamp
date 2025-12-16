#!/usr/bin/env python3
"""
Base class for manipulation tasks.

Provides ManipulationTask base class with common structure.
"""

from typing import List, Dict, Any, Optional

import re

from agimus_spacelab.planning import SceneBuilder, ConfigGenerator

class ManipulationTask:
    """
    Base class for manipulation tasks.
    
    Provides common structure for task definition, constraint creation,
    graph building, and configuration management.
    """
    
    def __init__(self, joint_bounds=None, FILE_PATHS: Optional[Dict[str, Any]] = None, task_name: str = "Manipulation Task", backend: str = "corba"):
        """
        Initialize manipulation task.
        
        Args:
            task_name: Descriptive name for the task
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.task_name = task_name
        self.backend = backend.lower()
        self.scene_builder = SceneBuilder(joint_bounds=joint_bounds,
                                          FILE_PATHS=FILE_PATHS,
                                          backend=backend)
        self.planner = None
        self.robot = None
        self.ps = None
        self.graph = None
        self.config_gen = None
    
    def get_joint_groups(self) -> List[str]:
        """
        Define which joint groups to use for the robot.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement get_joint_groups()")

    def get_objects(self) -> List[str]:
        """
        Define which objects are needed for this task.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement get_objects()")
    
    def get_robot_names(self) -> List[str]:
        """
        Define robot names to load.
        Override in subclass. Default: empty list.
        """
        return []
    
    def get_composite_names(self) -> List[str]:
        """
        Define composite robot names.
        Override in subclass. Default: empty list.
        """
        return []
    
    def get_object_names(self) -> List[str]:
        """
        Define object names to load.
        Override in subclass. Default: uses get_objects().
        """
        return self.get_objects()
    
    def get_environment_names(self) -> List[str]:
        """
        Define environment names to load.
        Override in subclass. Default: empty list.
        """
        return []
        
    def create_constraints(self) -> None:
        """
        Create all transformation constraints for the task.
        Override in subclass.
        """
        raise NotImplementedError(
            "Subclass must implement create_constraints()"
        )
        
    def create_graph(self):
        """
        Create and configure the constraint graph.
        Override in subclass.
        """
        raise NotImplementedError("Subclass must implement create_graph()")
        
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
            preferred_configs: List[str] = None,
            max_iterations: int = 1000) -> Dict[str, Any]:
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
                    k
                    for k in cfgs.keys()
                    if k.startswith("q_") and k not in ("q_init", "q_goal")
                ]
                return ["q_init", *mids, "q_goal"]

            seq = _ordered_config_keys(configs)
            if not seq or len(seq) < 2:
                print("   ⚠ Planning skipped: missing q_init/q_goal")
            elif len(seq) == 2:
                _reset_goals_if_possible()
                self.planner.set_initial_config(configs["q_init"])
                self.planner.add_goal_config(configs["q_goal"])
                ok = self.planner.solve()
                if not ok:
                    print("   ⚠ Planning failed")
                elif visualize:
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
                    ok = self.planner.solve(max_iterations=max_iterations)
                    if not ok:
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
                
        return {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
            "viewer": self.planner.viewer if self.planner else None
        }


__all__ = ["ManipulationTask"]
