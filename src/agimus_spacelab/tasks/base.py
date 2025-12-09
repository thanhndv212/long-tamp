#!/usr/bin/env python3
"""
Base class for manipulation tasks.

Provides ManipulationTask base class with common structure.
"""

from typing import List, Dict, Any, Optional

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
            objects=self.get_objects(),
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
            solve: bool = False) -> Dict[str, Any]:
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
            self.planner.set_initial_config(configs["q_init"])
            self.planner.add_goal_config(configs["q_goal"])
            
            try:
                self.planner.solve()
                # print("   ✓ Solution found!")
                
                if visualize:
                    print("\n7. Playing solution path...")
                    try:
                        self.planner.play_path(0)
                        print("   ✓ Path playback complete")
                    except Exception as e:
                        print(f"   ⚠ Path playback failed: {e}")
            except Exception as e:
                print(f"   ⚠ Planning failed: {e}")
                
        return {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
            "viewer": self.planner.viewer if self.planner else None
        }


__all__ = ["ManipulationTask"]
