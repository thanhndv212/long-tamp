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
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from graspball_config import (
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

from agimus_spacelab.corba import CorbaManipulationPlanner

# CORBA-specific imports
try:
    from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
    from hpp.corbaserver import Client
    # Reset problem before starting
    Client().problem.resetProblem()
    HAS_CORBA = True
except ImportError:
    HAS_CORBA = False
    print("Warning: CORBA backend not available")


def setup_environment(planner):
    """
    Load environment objects (ground only).
    
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
    
    # Create edges with appropriate waypoint states
    # The last parameter is the waypoint state for path planning
    graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
    graph.createEdge('placement', 'gripper-above-ball', 'approach-ball', 1, 'placement')
    graph.createEdge('gripper-above-ball', 'placement', 'move-gripper-away', 1, 'placement')
    graph.createEdge('gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'placement')
    graph.createEdge('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'placement')
    graph.createEdge('grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp')
    graph.createEdge('ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'grasp')
    graph.createEdge('ball-above-ground', 'grasp', 'take-ball-away', 1, 'grasp')
    graph.createEdge('grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp')
    graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
    
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
    
    # Transfer, take-ball-away, approach-ground use no constraints
    for edge_name in ["transfer", "take-ball-away", "approach-ground"]:
        graph.addConstraints(edge=edge_name, constraints=Constraints())
    
    # Set constant right-hand side
    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("placement/complement", False)
    ps.setConstantRightHandSide("ball_near_table/complement", False)
    ps.setConstantRightHandSide
    # Initialize graph
    graph.initialize()
    
    return graph


def generate_configurations(robot, graph, ps, q_init):
    """
    Generate all intermediate configurations for the task.
    
    Args:
        robot: Robot instance
        graph: ConstraintGraph instance
        ps: ProblemSolver instance
        q_init: Initial configuration
        
    Returns:
        dict: Dictionary of generated configurations
    """
    configs = {}
    q1 = list(q_init)  # Keep original for reference
    
    # Project initial config on placement node
    print("  Projecting initial configuration...")
    res, configs["q_init"], err = graph.applyNodeConstraints(
        "placement", q1
    )
    if not res:
        print(f"Warning: Failed to project initial config (error: {err})")
        configs["q_init"] = q1
    
    # Generate configuration after approach-ball
    print("  Generating approach-ball configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_ab, err = graph.generateTargetConfig(
            "approach-ball", q1, q_rand
        )
        if res:
            valid, _ = robot.isConfigValid(q_ab)
            if valid:
                configs["q_ab"] = q_ab
                break
        if (i + 1) % 100 == 0:
            print(f"    Attempt {i + 1}...")
    else:
        raise RuntimeError("Failed to generate approach-ball config")
    
    # Project onto gripper-above-ball node
    res, configs["q_above"], err = graph.applyNodeConstraints(
        "gripper-above-ball", configs["q_ab"]
    )
    if not res:
        print(f"Warning: Failed to project onto gripper-above-ball")
        configs["q_above"] = configs["q_ab"]
    
    # Generate configuration after grasp-ball
    print("  Generating grasp-ball configuration...")
    success = False
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_gb, err = graph.generateTargetConfig(
            "grasp-ball", configs["q_above"], q_rand
        )
        if res:
            # Check for collisions
            valid, report = robot.isConfigValid(q_gb)
            if valid:
                configs["q_gb"] = q_gb
                success = True
                break
        if (i + 1) % 100 == 0:
            print(f"    Attempt {i + 1}... (last err: {err})")
    
    if not success:
        print(f"  Warning: Failed after {ManipulationConfig.MAX_RANDOM_ATTEMPTS} attempts")
        print(f"  Last error: {err}")
        print("  Edge 'grasp-ball' may have constraints that are hard to satisfy")
        raise RuntimeError("Failed to generate grasp-ball config")
    
    # Project onto grasp-placement node
    res, configs["q_grasp_place"], err = graph.applyNodeConstraints(
        "grasp-placement", configs["q_gb"]
    )
    if not res:
        print(f"Warning: Failed to project onto grasp-placement")
        configs["q_grasp_place"] = configs["q_gb"]
    
    # Generate configuration after take-ball-up
    print("  Generating take-ball-up configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_tbu, err = graph.generateTargetConfig(
            "take-ball-up", configs["q_grasp_place"], q_rand
        )
        if res:
            valid, _ = robot.isConfigValid(q_tbu)
            if valid:
                configs["q_tbu"] = q_tbu
                break
        if (i + 1) % 100 == 0:
            print(f"    Attempt {i + 1}...")
    else:
        raise RuntimeError("Failed to generate take-ball-up config")
    
    # Project onto ball-above-ground node
    res, configs["q_ball_up"], err = graph.applyNodeConstraints(
        "ball-above-ground", configs["q_tbu"]
    )
    if not res:
        print(f"Warning: Failed to project onto ball-above-ground")
        configs["q_ball_up"] = configs["q_tbu"]
    
    # Generate configuration after take-ball-away
    print("  Generating take-ball-away configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_tba, err = graph.generateTargetConfig(
            "take-ball-away", configs["q_ball_up"], q_rand
        )
        if res:
            valid, _ = robot.isConfigValid(q_tba)
            if valid:
                configs["q_tba"] = q_tba
                break
        if (i + 1) % 100 == 0:
            print(f"    Attempt {i + 1}...")
    else:
        raise RuntimeError("Failed to generate take-ball-away config")
    
    # Project onto grasp node
    res, configs["q_grasp"], err = graph.applyNodeConstraints(
        "grasp", configs["q_tba"]
    )
    if not res:
        print(f"Warning: Failed to project onto grasp")
        configs["q_grasp"] = configs["q_tba"]
    
    # Goal configuration: move ball to x=0.2
    q2 = q1[:]
    q2[7] = 0.2
    
    # Generate configuration after approach-ground
    print("  Generating approach-ground configuration...")
    res, q_ag, err = graph.generateTargetConfig(
        "approach-ground", configs["q_grasp"], q2
    )
    if not res:
        print(f"Warning: Failed to generate approach-ground, using q2")
        q_ag = q2
    configs["q_ag"] = q_ag
    
    # Project onto ball-above-ground node
    res, configs["q_ball_above_2"], err = graph.applyNodeConstraints(
        "ball-above-ground", configs["q_ag"]
    )
    if not res:
        print(f"Warning: Failed to project onto ball-above-ground")
        configs["q_ball_above_2"] = configs["q_ag"]
    
    # Generate configuration after put-ball-down
    print("  Generating put-ball-down configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_pbd, err = graph.generateTargetConfig(
            "put-ball-down", configs["q_ball_up"], q_rand
        )
        if res:
            configs["q_pbd"] = q_pbd
            break
    else:
        raise RuntimeError("Failed to generate put-ball-down config")
    
    # Project onto grasp-placement node
    res, configs["q_grasp_place_2"], err = graph.applyNodeConstraints(
        "grasp-placement", configs["q_pbd"]
    )
    if not res:
        print(f"Warning: Failed to project onto grasp-placement")
        configs["q_grasp_place_2"] = configs["q_pbd"]
    
    # Generate configuration after move-gripper-up
    print("  Generating move-gripper-up configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_mgu, err = graph.generateTargetConfig(
            "move-gripper-up", configs["q_grasp_place"], q_rand
        )
        if res:
            configs["q_mgu"] = q_mgu
            break
    else:
        raise RuntimeError("Failed to generate move-gripper-up config")
    
    # Project onto gripper-above-ball node
    res, configs["q_above_2"], err = graph.applyNodeConstraints(
        "gripper-above-ball", configs["q_mgu"]
    )
    if not res:
        print(f"Warning: Failed to project onto gripper-above-ball")
        configs["q_above_2"] = configs["q_mgu"]
    
    # Generate configuration after move-gripper-away
    print("  Generating move-gripper-away configuration...")
    for i in range(ManipulationConfig.MAX_RANDOM_ATTEMPTS):
        q_rand = robot.shootRandomConfig()
        res, q_mga, err = graph.generateTargetConfig(
            "move-gripper-away", configs["q_above"], q_rand
        )
        if res:
            configs["q_mga"] = q_mga
            break
    else:
        raise RuntimeError("Failed to generate move-gripper-away config")
    
    q2 = q1[::]
    q2[7] = .2

    # Final goal: project onto placement with ball at new location
    res, configs["q_goal"], err = graph.applyNodeConstraints(
        "placement", q2
    )
    if not res:
        print(f"Warning: Failed to project goal config")
        configs["q_goal"] = q2
    
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
    # These correspond to the manipulation sequence:
    # 1. Approach ball (placement -> gripper-above-ball)
    # 2. Grasp ball (gripper-above-ball -> grasp-placement)
    # 3. Lift ball (grasp-placement -> ball-above-ground)
    # 4. Move ball away (ball-above-ground -> grasp)
    # 5. Transfer to goal (grasp -> placement)
    # waypoints = [
    #     ("q_init", "q_above", "approach-ball"),
    #     ("q_above", "q_grasp_place", "grasp-ball"),
    #     ("q_grasp_place", "q_ball_up", "take-ball-up"),
    #     ("q_ball_up", "q_grasp", "take-ball-away"),
    #     ("q_grasp", "q_goal", "transfer"),
    # ]
    
    # path_ids = []
    
    # for i, (q_start_key, q_end_key, edge_name) in enumerate(waypoints):
    #     q_start = configs[q_start_key]
    #     q_end = configs[q_end_key]
        
    #     print(f"\n  Step {i+1}/{len(waypoints)}: {edge_name}")
    #     print(f"    Planning from {q_start_key} to {q_end_key}...")
        
    #     ps.resetGoalConfigs()
    #     ps.setInitialConfig(q_start)
    #     ps.addGoalConfig(q_end)
        
    #     try:
    #         ps.solve()
    #         path_ids.append(ps.numberPaths() - 1)
    #         print(f"    ✓ Path found (ID: {path_ids[-1]})")
    #     except Exception as e:
    #         print(f"    ✗ Planning failed: {e}")
    #         return False
    
    # # Concatenate paths
    # if len(path_ids) > 1:
    #     print(f"\n  Concatenating {len(path_ids)} path segments...")
    #     for i in range(1, len(path_ids)):
    #         ps.concatenatePath(path_ids[0], path_ids[i])
    #     print(f"  ✓ Final path ID: {path_ids[0]}")
    # path planning solver
    ps.setInitialConfig (configs["q_init"])
    ps.addGoalConfig (configs["q_goal"])
    ps.solve()
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

    
    # Create planner
    planner = CorbaManipulationPlanner()
    
    # Load robot
    print("\n1. Loading robot and objects...")
    robot = planner.load_robot(
        composite_name="ur5-pokeball",
        robot_name="ur5",
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
    
    # Get problem solver
    ps = planner.get_problem_solver()
    
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
    configs = generate_configurations(robot, graph, ps, q_init)
    
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
