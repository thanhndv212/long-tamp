#!/usr/bin/env python3
"""
Task: UR5 grasps pokeball from box.

This task demonstrates a complete pick-and-place sequence:
1. Ball rests on ground (placement)
2. Gripper approaches ball (gripper-above-ball)
3. Gripper grasps ball on ground (grasp-placement)
4. Ball is lifted from ground (ball-above-ground)
5. Free motion with grasped ball (grasp)

Task sequence:
1. placement -> approach-ball -> gripper-above-ball
2. gripper-above-ball -> grasp-ball -> grasp-placement
3. grasp-placement -> take-ball-up -> ball-above-ground
4. ball-above-ground -> take-ball-away -> grasp
5. grasp -> approach-ground -> ball-above-ground
6. ball-above-ground -> put-ball-down -> grasp-placement
7. grasp-placement -> move-gripper-up -> gripper-above-ball
8. gripper-above-ball -> move-gripper-away -> placement

Supports both CORBA and PyHPP backends.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Dict
import numpy as np
from pinocchio import SE3

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import ConstraintBuilder, GraphBuilder
from agimus_spacelab.visualization import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_all_handles,
    visualize_all_grippers,
)
# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from graspball_config import (
    PATHS,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

# Import backend availability flags for validation
try:
    from agimus_spacelab.backends import get_available_backends
    _available = get_available_backends()
    HAS_CORBA = 'corba' in _available
    HAS_PYHPP = 'pyhpp' in _available
except ImportError:
    HAS_CORBA = False
    HAS_PYHPP = False

# ============================================================================
# Task Implementation
# ============================================================================

class GraspBallTask(ManipulationTask):
    """UR5 grasps pokeball from box."""
    
    def __init__(self, backend: str = "corba"):
        """
        Initialize task.
        
        Args:
            backend: "corba" or "pyhpp" - which backend to use
        """
        super().__init__(
            joint_bounds=JointBounds,
            FILE_PATHS=PATHS,
            task_name="Grasp Ball Manipulation",
            backend=backend
        )
        self.config = ManipulationConfig
        self.pyhpp_constraints = {}
        self.pyhpp_states = None
        self.pyhpp_edges = None
    
        self.robot_names = ["ur5"]
        self.composite_names = ["ur5-pokeball"]
        self.object_names = ["pokeball"]
        self.environment_names = ["ground", "box"]

    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return ["UR5"]  # Only UR5 robot joints

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return ["pokeball"]
    
    def build_initial_config(self) -> List[float]:
        """Build the initial configuration for UR5 + pokeball."""
        return InitialConfigurations.FULL_INIT
    
    def setup_collision_management(self) -> None:
        """No special collision management needed for this task."""
        pass
    
    def create_constraints(self) -> None:
        """Create all transformation constraints for both backends."""
        cfg = self.config
        cb = ConstraintBuilder
        
        # Get robot reference (differs by backend)
        robot = (self.planner.get_robot() if self.backend == "pyhpp"
                 else self.robot)
        
        # Get constraint definitions from config
        constraint_defs = cfg.get_constraint_defs()
        
        constraints = {}
        for ctype, name, args in constraint_defs:
            if ctype == "grasp":
                result = cb.create_grasp_constraint(
                    self.ps, name,
                    args["gripper"], args["obj"],
                    args["transform"], args["mask"],
                    robot=robot, backend=self.backend
                )
            elif ctype == "placement":
                result = cb.create_placement_constraint(
                    self.ps, name,
                    args["obj"], args["transform"], args["mask"],
                    robot=robot, backend=self.backend
                )
            elif ctype == "complement":
                result = cb.create_complement_constraint(
                    self.ps, name,
                    args["obj"], args["transform"], args["mask"],
                    robot=robot, backend=self.backend
                )
            
            # Store constraint for PyHPP (returns object), CORBA returns None
            if self.backend == "pyhpp":
                key = f"{name}/complement" if ctype == "complement" else name
                constraints[key] = result
        
        # Store for PyHPP graph building
        if self.backend == "pyhpp":
            self.pyhpp_constraints = constraints
        
        print("   ✓ Created transformation constraints")

    def create_graph(self):
        """Create and configure constraint graph for both backends."""
        print("    Building constraint graph...")
        cfg = self.config

        # Get backend-specific references
        if self.backend == "pyhpp":
            robot = self.planner.get_robot()
            problem = self.planner.get_problem()
            constraints = self.pyhpp_constraints
        else:
            robot = self.robot
            problem = self.ps
            constraints = None

        # Initialize GraphBuilder
        self.graph_builder = GraphBuilder(
            self.planner, robot, problem, backend=self.backend
        )

        # Create empty graph
        graph_name = "manipulation_graph" if self.backend == "pyhpp" else "graph"
        self.graph_builder.create_manual_graph(graph_name)

        # Create states (order matters for solver performance)
        if self.backend == "corba":
            self.graph_builder.add_states(cfg.GRAPH_NODES)
        else:
            for state_name in cfg.GRAPH_NODES:
                self.graph_builder.add_state(state_name)

        # Create edges from declarative definition
        edges_def = cfg.GRASPBALL_GRAPH['edges']
        for edge_name, edge_info in edges_def.items():
            self.graph_builder.add_edge(
                edge_info["from"],
                edge_info["to"],
                edge_name,
                edge_info.get("weight", 1),
                edge_info["in"]
            )

        # Add constraints to states from declarative definition
        states_def = cfg.GRASPBALL_GRAPH['states']
        for state_name, state_info in states_def.items():
            constraint_names = state_info.get("constraints", [])
            if constraint_names:
                if self.backend == "corba":
                    self.graph_builder.add_state_constraints(
                        state_name, [],
                        constraint_names=constraint_names
                    )
                else:
                    constraint_objs = [constraints[n] for n in constraint_names]
                    self.graph_builder.add_state_constraints(
                        state_name, constraint_objs
                    )

        # Add constraints to edges from declarative definition
        edge_constraints_def = cfg.GRASPBALL_GRAPH['edge_constraints']
        for constraint_name, edge_list in edge_constraints_def.items():
            for edge_name in edge_list:
                if self.backend == "corba":
                    self.graph_builder.add_edge_constraints(
                        edge_name, [],
                        constraint_names=[constraint_name]
                    )
                else:
                    self.graph_builder.add_edge_constraints(
                        edge_name, [constraints[constraint_name]]
                    )

        # Free motion edges (no path constraints)
        free_edges = cfg.GRASPBALL_GRAPH["free_motion_edges"]
        for edge_name in free_edges:
            if self.backend == "corba":
                self.graph_builder.add_edge_constraints(
                    edge_name, [], constraint_names=[]
                )
            else:
                self.graph_builder.add_edge_constraints(edge_name, [])

        # Set constant RHS (CORBA only)
        if self.backend == "corba":
            constant_rhs = cfg.GRASPBALL_GRAPH["constant_rhs"]
            for constraint_name, is_constant in constant_rhs.items():
                self.ps.setConstantRightHandSide(constraint_name, is_constant)

        # Finalize graph
        self.graph_builder.finalize_manual_graph()

        # Store graph and metadata
        graph = self.graph_builder.get_graph()

        return graph
        
    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations for both backends."""
        cg = self.config_gen
        gb = self.graph_builder
        q1 = list(q_init)

        # Helper to get node/edge identifier based on backend
        def node(name):
            return gb.states[name] if self.backend == "pyhpp" else name

        def edge(name):
            return gb.edges[name] if self.backend == "pyhpp" else name

        # 1. Project initial config on placement
        print("    1. Projecting onto 'placement' state...")
        cg.project_on_node(node("placement"), q1, "q_init")

        # 2. Generate approach-ball config
        print("    2. Generating 'approach-ball' config...")
        success, _ = cg.generate_via_edge(
            edge("approach-ball"), cg.configs["q_init"], "q_ab"
        )
        if not success:
            return cg.configs

        # 3. Project onto gripper-above-ball
        print("    3. Projecting onto 'gripper-above-ball' state...")
        cg.project_on_node(
            node("gripper-above-ball"), cg.configs["q_ab"], "q_above"
        )

        # 4. Generate grasp-ball config
        print("    4. Generating 'grasp-ball' config...")
        success, _ = cg.generate_via_edge(
            edge("grasp-ball"), cg.configs["q_above"], "q_gb"
        )
        if not success:
            return cg.configs

        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        cg.project_on_node(
            node("grasp-placement"), cg.configs["q_gb"], "q_grasp_place"
        )

        # 6. Generate take-ball-up config
        print("    6. Generating 'take-ball-up' config...")
        success, _ = cg.generate_via_edge(
            edge("take-ball-up"), cg.configs["q_grasp_place"], "q_tbu"
        )
        if not success:
            return cg.configs

        # 7. Project onto ball-above-ground
        print("    7. Projecting onto 'ball-above-ground' state...")
        cg.project_on_node(
            node("ball-above-ground"), cg.configs["q_tbu"], "q_ball_up"
        )

        # 8. Generate take-ball-away config
        print("    8. Generating 'take-ball-away' config...")
        success, _ = cg.generate_via_edge(
            edge("take-ball-away"), cg.configs["q_ball_up"], "q_tba"
        )
        if not success:
            return cg.configs

        # 9. Project onto grasp
        print("    9. Projecting onto 'grasp' state...")
        cg.project_on_node(node("grasp"), cg.configs["q_tba"], "q_grasp")

        # 10. Generate goal (move ball to x=0.2)
        print("    10. Generating goal configuration...")
        q2 = list(q1)
        q2[6] = 0.2  # Ball x position (index 6 = first object DOF)
        cg.project_on_node(node("placement"), q2, "q_goal")

        return cg.configs
    
    def setup(self, validation_step: float = 0.01,
              projector_step: float = 0.1):
        """
        Complete task setup: scene, constraints, graph.
        
        Overrides base class to use custom scene setup.
        """
        cfg = self.config
        if self.backend == "pyhpp":
            box_pose = SE3(
                rotation=np.eye(3),
                translation=np.array([cfg.BOX_X, 0, 0.04])
            )
        # 1. Custom scene setup (not using SceneBuilder)
        self.scene_builder.load_robot(
            composite_names=self.composite_names,
            robot_names=self.robot_names
        )
        self.scene_builder.load_environment(
            environment_names=self.environment_names,
            pose=[None,box_pose] if self.backend == "pyhpp" else None
        )
        self.scene_builder.load_objects(
            object_names=self.object_names
        )
        self.scene_builder.set_joint_bounds()
        self.scene_builder.configure_path_validation(validation_step,
                                                     projector_step)
        self.planner, self.robot, self.ps = self.scene_builder.get_instances()
        
        # Position box walls
        if self.backend == "corba":
            box_x = cfg.BOX_X
            box_off = cfg.BOX_OFFSET
            self.scene_builder.move_obstacle(
                'box/base_link_0', [box_x + box_off, 0, 0.04], [0, 0, 0, 1]
            )
            self.scene_builder.move_obstacle(
                'box/base_link_1', [box_x - box_off, 0, 0.04], [0, 0, 0, 1]
            )
            self.scene_builder.move_obstacle(
                'box/base_link_2', [box_x, box_off, 0.04], [0, 0, 0, 1]
            )
            self.scene_builder.move_obstacle(
                'box/base_link_3', [box_x, -box_off, 0.04], [0, 0, 0, 1]
            )
        # 2. Create constraints
        print("\n2. Creating constraints...")
        self.create_constraints()
        
        # 3. Create graph
        print("\n3. Creating constraint graph...")
        self.graph = self.create_graph()
        
        # 4. Initialize config generator (simplified for this task)
        from agimus_spacelab.planning import ConfigGenerator
        self.config_gen = ConfigGenerator(
            self.robot, self.graph, self.planner, self.ps, backend=self.backend
        )
        
        print("\n   ✓ Task setup complete")


# ============================================================================
# Main Execution
# ============================================================================

def main(
    visualize: bool = True,
    solve: bool = False,
    backend: str = "corba"
):
    """
    Run the grasp ball task.
    
    Args:
        visualize: Show configurations in viewer
        solve: Attempt to solve planning problem
        backend: "corba" or "pyhpp" - which backend to use
    """
    # Validate backend availability
    if backend == "corba" and not HAS_CORBA:
        print("Error: CORBA backend not available")
        return None, None
    elif backend == "pyhpp" and not HAS_PYHPP:
        print("Error: PyHPP backend not available. Install pyhpp.")
        return None, None
    
    # Create and setup task
    task = GraspBallTask(backend=backend)
    task.setup(
        validation_step=ManipulationConfig.PATH_VALIDATION_STEP,
        projector_step=ManipulationConfig.PATH_PROJECTOR_STEP
    )
    
    # Run task
    result = task.run(visualize=visualize, solve=solve)
    
    # Visualize handles and grippers if viewer available
    if visualize and result.get('viewer'):
        print("\n" + "=" * 70)
        print("Visualizing Handles and Grippers")
        print("=" * 70)
        viewer = result['viewer']
        
        # Collect handle names (pokeball handle)
        handle_names = ["pokeball/handle"]
        
        # Visualize handles with approach arrows
        print("\n  Handles (green frames, cyan approach arrows):")
        try:
            visualize_all_handles(
                viewer, handle_names,
                show_approach=True,
                frame_color=[0, 0.8, 0, 1],   # Green
                arrow_color=[0, 1, 1, 1],      # Cyan
                axis_length=0.03,
                arrow_length=0.08
            )
            for handle_name in handle_names:
                print(f"    ✓ {handle_name}")
        except Exception as e:
            print(f"    ⚠ Handle visualization failed: {e}")
        
        # Visualize grippers with approach arrows
        gripper_names = ["ur5/gripper"]
        print("\n  Grippers (red frames, orange approach arrows):")
        try:
            visualize_all_grippers(
                viewer, gripper_names,
                show_approach=True,
                frame_color=[1, 0, 0, 1],      # Red
                arrow_color=[1, 0.5, 0, 1],    # Orange
                axis_length=0.03,
                arrow_length=0.08
            )
            for gripper_name in gripper_names:
                print(f"    ✓ {gripper_name}")
        except Exception as e:
            print(f"    ⚠ Gripper visualization failed: {e}")
        
        try:
            viewer.client.gui.refresh()
        except Exception:
            pass
    
    # Print summary
    print("\n" + "=" * 70)
    print("Task Complete!")
    print("=" * 70)
    
    if result:
        configs = result.get("configs", {})
        print(f"\nGenerated {len(configs)} configurations:")
        for name, q in configs.items():
            print(f"  {name}: {len(q)} DOF")
        
        print("\nTo visualize specific configs:")
        print("  task.planner.visualize(result['configs']['q_init'])")
        print("  task.planner.visualize(result['configs']['q_above'])")
        print("  task.planner.visualize(result['configs']['q_goal'])")
        
        if solve:
            print("\nTo replay path:")
            print("  task.planner.play_path(0)")
    
        # Visualize constraint graph
    print("\n📊 Generating constraint graph visualization...")
    # Pass state/edge dicts for PyHPP backend
    states_dict = getattr(task, 'pyhpp_states', None)
    edges_dict = getattr(task, 'pyhpp_edges', None)
    edge_topology = getattr(task, 'pyhpp_edge_topology', None)
    viz_path = visualize_constraint_graph(
        task.graph_builder,
        output_path="constraint_graph",
        states_dict=states_dict,
        edges_dict=edges_dict,
        edge_topology=edge_topology
    )
    if viz_path:
        print(f"✓ Graph visualization saved to: {viz_path}")
    

    return task, result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="UR5 grasps pokeball from box"
    )
    parser.add_argument(
        "--no-viz", action="store_true", help="Disable visualization"
    )
    parser.add_argument(
        "--solve", action="store_true", help="Solve planning problem"
    )
    parser.add_argument(
        "--backend", type=str, default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use: corba (default) or pyhpp"
    )
    
    args = parser.parse_args()
    
    task, result = main(
        visualize=not args.no_viz,
        solve=args.solve,
        backend=args.backend
    )
    
    if task and result:
        print("\n" + "=" * 70)
        print("Available objects:")
        print("=" * 70)
        print("  task   - GraspBallTask instance")
        print("  result - Dictionary with configs, planner, robot, ps, graph")
