#!/usr/bin/env python3
"""
Grasp ball manipulation example using CORBA backend.

This example demonstrates manipulation planning with CORBA to:
1. Approach the ball
2. Grasp the ball  
3. Lift and move the ball
4. Place the ball at a new location

Task sequence:
1. placement -> approach-ball -> gripper-above-ball
2. gripper-above-ball -> grasp-ball -> grasp-placement
3. grasp-placement -> take-ball-up -> ball-above-ground
4. ball-above-ground -> take-ball-away -> grasp
5. grasp -> approach-ground -> ball-above-ground
6. ball-above-ground -> put-ball-down -> grasp-placement
7. grasp-placement -> move-gripper-up -> gripper-above-ball
8. gripper-above-ball -> move-gripper-away -> placement
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab.corba import CorbaManipulationPlanner
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from graspball_config import (
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

# CORBA-specific imports
try:
    from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
    from hpp.corbaserver import Client
    HAS_CORBA = True
except ImportError:
    HAS_CORBA = False
    print("Warning: CORBA backend not available")


def setup_environment(planner):
    """
    Load environment objects (ground and box).
    
    Args:
        planner: CorbaManipulationPlanner instance
    """
    print("  Loading ground...")
    planner.load_environment("ground", ManipulationConfig.GROUND_URDF)
    
    print("  Loading box...")
    planner.load_environment("box", ManipulationConfig.BOX_URDF)


def create_transformation_constraints(ps):
    """
    Define all transformation constraints for the task.
    
    Args:
        ps: ProblemSolver instance
    """
    # Grasp constraint: gripper holds ball
    ps.createTransformationConstraint(
        "grasp",
        ManipulationConfig.GRIPPER_NAME,
        ManipulationConfig.BALL_NAME,
        ManipulationConfig.BALL_IN_GRIPPER,
        ManipulationConfig.GRASP_MASK
    )
    
    # Placement constraint: ball on ground
    ps.createTransformationConstraint(
        "placement",
        "",
        ManipulationConfig.BALL_NAME,
        ManipulationConfig.BALL_ON_GROUND,
        ManipulationConfig.PLACEMENT_MASK
    )
    
    # Placement complement: ball can move in x-y, rotate in yaw
    ps.createTransformationConstraint(
        "placement/complement",
        "",
        ManipulationConfig.BALL_NAME,
        ManipulationConfig.BALL_ON_GROUND,
        ManipulationConfig.PLACEMENT_COMPLEMENT_MASK
    )
    
    # Gripper-ball alignment: gripper above ball
    ps.createTransformationConstraint(
        "gripper_ball_aligned",
        ManipulationConfig.GRIPPER_NAME,
        ManipulationConfig.BALL_NAME,
        ManipulationConfig.GRIPPER_ABOVE_BALL,
        [True, True, True, True, True, True]
    )
    
    # Ball near table constraint
    ps.createTransformationConstraint(
        "ball_near_table",
        "",
        ManipulationConfig.BALL_NAME,
        ManipulationConfig.BALL_NEAR_TABLE,
        ManipulationConfig.PLACEMENT_MASK
    )
    
    # Ball near table complement
    ps.createTransformationConstraint(
        "ball_near_table/complement",
        "",
        ManipulationConfig.BALL_NAME,
        [ManipulationConfig.BOX_X, 0.2, 0.1, 0, 0, 0, 1],
        ManipulationConfig.PLACEMENT_COMPLEMENT_MASK
    )


def create_constraint_graph(robot, ps):
    """
    Create and configure the constraint graph.
    
    Args:
        robot: Robot instance
        ps: ProblemSolver instance
        
    Returns:
        ConstraintGraph object
    """
    graph = ConstraintGraph(robot, "graph")
    
    # Create all nodes
    graph.createNode(ManipulationConfig.GRAPH_NODES)
    
    # Create all edges
    for from_node, to_node, edge_name in ManipulationConfig.GRAPH_EDGES:
        graph.createEdge(from_node, to_node, edge_name, 1, from_node)
    
    # Assign constraints to nodes
    graph.addConstraints(node="placement", constraints=Constraints(
        numConstraints=["placement"]
    ))
    
    graph.addConstraints(node="gripper-above-ball", constraints=Constraints(
        numConstraints=["placement", "gripper_ball_aligned"]
    ))
    
    graph.addConstraints(node="grasp-placement", constraints=Constraints(
        numConstraints=["grasp", "placement"]
    ))
    
    graph.addConstraints(node="ball-above-ground", constraints=Constraints(
        numConstraints=["grasp", "ball_near_table"]
    ))
    
    graph.addConstraints(node="grasp", constraints=Constraints(
        numConstraints=["grasp"]
    ))
    
    # Assign constraints to edges
    # Transit, approach-ball, move-gripper-away use placement/complement
    for edge_name in ["transit", "approach-ball", "move-gripper-away"]:
        graph.addConstraints(edge=edge_name, constraints=Constraints(
            numConstraints=["placement/complement"]
        ))
    
    # Grasp-ball and move-gripper-up use placement/complement
    for edge_name in ["grasp-ball", "move-gripper-up"]:
        graph.addConstraints(edge=edge_name, constraints=Constraints(
            numConstraints=["placement/complement"]
        ))
    
    # Take-ball-up and put-ball-down use ball_near_table/complement
    for edge_name in ["take-ball-up", "put-ball-down"]:
        graph.addConstraints(edge=edge_name, constraints=Constraints(
            numConstraints=["ball_near_table/complement"]
        ))
    
    # Set constant right-hand side
    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("placement/complement", False)
    ps.setConstantRightHandSide("ball_near_table/complement", False)
    
    # Initialize graph
    graph.initialize()
    
    return graph


def generate_configurations(graph, ps, q_init):
    """
    Generate all intermediate configurations for the task.
    
    Args:
        graph: ConstraintGraph instance
        ps: ProblemSolver instance
        q_init: Initial configuration
        
    Returns:
        dict: Dictionary of generated configurations
    """
    configs = {}
    
    # Project initial config on placement node
    res, configs["q_init"], err = graph.applyNodeConstraints(
        "placement", q_init
    )
    if not res:
        print(f"Warning: Failed to project initial config (error: {err})")
        configs["q_init"] = q_init
    
    # Generate configuration for gripper above ball
    print("  Generating gripper-above-ball configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = ps.shoot()
        res, q_target, err = graph.generateTargetConfig(
            "approach-ball", configs["q_init"], q_rand
        )
        if res and ps.isConfigValid(q_target)[0]:
            configs["q_above"] = q_target
            break
    else:
        raise RuntimeError("Failed to generate gripper-above-ball config")
    
    # Generate configuration for grasp-placement
    print("  Generating grasp-placement configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = ps.shoot()
        res, q_target, err = graph.generateTargetConfig(
            "grasp-ball", configs["q_above"], q_rand
        )
        if res and ps.isConfigValid(q_target)[0]:
            configs["q_grasp_place"] = q_target
            break
    else:
        raise RuntimeError("Failed to generate grasp-placement config")
    
    # Generate configuration for ball above ground
    print("  Generating ball-above-ground configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = ps.shoot()
        res, q_target, err = graph.generateTargetConfig(
            "take-ball-up", configs["q_grasp_place"], q_rand
        )
        if res and ps.isConfigValid(q_target)[0]:
            configs["q_ball_up"] = q_target
            break
    else:
        raise RuntimeError("Failed to generate ball-above-ground config")
    
    # Generate final grasp configuration
    print("  Generating final grasp configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = ps.shoot()
        res, q_target, err = graph.generateTargetConfig(
            "take-ball-away", configs["q_ball_up"], q_rand
        )
        if res and ps.isConfigValid(q_target)[0]:
            configs["q_final_grasp"] = q_target
            break
    else:
        raise RuntimeError("Failed to generate final grasp config")
    
    # Goal: approach ground with ball at different location
    q_goal = configs["q_init"].copy()
    q_goal[6] = 0.2  # Move ball in x direction
    
    res, configs["q_goal"], err = graph.applyNodeConstraints(
        "placement", q_goal
    )
    if not res:
        print(f"Warning: Failed to project goal config (error: {err})")
        configs["q_goal"] = q_goal
    
    return configs


def solve_manipulation_problem(graph, ps, configs):
    """
    Solve the manipulation planning problem.
    
    Args:
        graph: ConstraintGraph instance
        ps: ProblemSolver instance
        configs: Dictionary of configurations
        
    Returns:
        bool: True if successful, False otherwise
    """
    # Define waypoints for the task
    waypoints = [
        ("q_init", "q_above", "approach-ball"),
        ("q_above", "q_grasp_place", "grasp-ball"),
        ("q_grasp_place", "q_ball_up", "take-ball-up"),
        ("q_ball_up", "q_final_grasp", "take-ball-away"),
        ("q_final_grasp", "q_goal", "transfer"),
    ]
    
    path_ids = []
    
    for i, (q_start_key, q_end_key, edge_name) in enumerate(waypoints):
        q_start = configs[q_start_key]
        q_end = configs[q_end_key]
        
        print(f"\n  Step {i+1}/{len(waypoints)}: {edge_name}")
        print(f"    Planning from {q_start_key} to {q_end_key}...")
        
        ps.resetGoalConfigs()
        ps.setInitialConfig(q_start)
        ps.addGoalConfig(q_end)
        
        try:
            ps.solve()
            path_ids.append(ps.numberPaths() - 1)
            print(f"    ✓ Path found (ID: {path_ids[-1]})")
        except Exception as e:
            print(f"    ✗ Planning failed: {e}")
            return False
    
    # Concatenate paths
    if len(path_ids) > 1:
        print(f"\n  Concatenating {len(path_ids)} path segments...")
        for i in range(1, len(path_ids)):
            ps.concatenatePath(path_ids[0], path_ids[i])
        print(f"  ✓ Final path ID: {path_ids[0]}")
    
    return True


def main(visualize=True, solve=True):
    """
    Run grasp ball manipulation example.
    
    Args:
        visualize: Whether to display visualization
        solve: Whether to solve the planning problem
    """
    if not HAS_CORBA:
        print("Error: CORBA backend not available")
        return False
    
    print("=" * 70)
    print("Grasp Ball Manipulation - CORBA Backend")
    print("=" * 70)
    
    # Reset problem before starting
    Client().problem.resetProblem()
    
    # Create planner
    planner = CorbaManipulationPlanner()
    ps = planner.get_problem_solver()
    
    # Load robot
    print("\n1. Loading robot and objects...")
    robot = planner.load_robot(
        name="ur5-pokeball",
        urdf_path=ManipulationConfig.ROBOT_URDF,
        srdf_path=ManipulationConfig.ROBOT_SRDF
    )
    
    # Load ball
    planner.load_object(
        name="pokeball",
        urdf_path=ManipulationConfig.BALL_URDF,
        root_joint_type="freeflyer"
    )
    
    # Set joint bounds
    bounds = JointBounds.freeflyer_bounds()
    planner.set_joint_bounds(ManipulationConfig.BALL_NAME, bounds)
    
    # Setup environment
    setup_environment(planner)
    
    # Create constraints
    print("\n2. Creating transformation constraints...")
    create_transformation_constraints(ps)
    
    # Configure path planning
    ps.selectPathValidation(
        "Discretized", ManipulationConfig.PATH_VALIDATION_STEP
    )
    ps.selectPathProjector(
        "Progressive", ManipulationConfig.PATH_PROJECTOR_STEP
    )
    
    # Create constraint graph
    print("\n3. Creating constraint graph...")
    graph = create_constraint_graph(robot, ps)
    
    # Generate configurations
    print("\n4. Generating intermediate configurations...")
    q_init = InitialConfigurations.FULL_INIT
    configs = generate_configurations(graph, ps, q_init)
    
    print("\n  Configuration summary:")
    print(f"    Initial: {configs['q_init'][:6]}")
    print(f"    Goal: {configs['q_goal'][:6]}")
    
    # Visualize
    if visualize:
        print("\n5. Visualizing initial configuration...")
        try:
            planner.visualize(configs["q_init"])
            print("  ✓ Visualization ready")
        except Exception as e:
            print(f"  ✗ Visualization failed: {e}")
    
    # Solve
    if solve:
        print("\n6. Solving manipulation problem...")
        print("  This may take a while...")
        
        success = solve_manipulation_problem(graph, ps, configs)
        
        if success:
            print("\n  ✓ Solution found!")
            
            if visualize:
                print("\n7. Playing solution path...")
                try:
                    planner.play_path(0)
                    print("  ✓ Path playback complete")
                except Exception as e:
                    print(f"  ✗ Path playback failed: {e}")
        else:
            print("\n  ✗ No solution found")
            return False
    
    print("\n" + "=" * 70)
    print("Example completed successfully!")
    print("=" * 70)
    
    return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Grasp ball manipulation example with CORBA"
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization"
    )
    parser.add_argument(
        "--no-solve",
        action="store_true",
        help="Skip solving (just setup and visualize)"
    )
    
    args = parser.parse_args()
    
    success = main(
        visualize=not args.no_viz,
        solve=not args.no_solve
    )
    
    sys.exit(0 if success else 1)
