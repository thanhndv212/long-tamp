#!/usr/bin/env python3
"""
Grasp ball manipulation example using PyHPP backend.

This example demonstrates manipulation planning with PyHPP to:
1. Approach the ball
2. Grasp the ball
3. Lift and move the ball
4. Place the ball at a new location
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab.pyhpp import PyHPPManipulationPlanner
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from graspball_config import (
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

# PyHPP-specific imports
try:
    from pyhpp.manipulation import Graph
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, StdVec_Bool as Mask, Quaternion
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False
    print("Warning: PyHPP not available")


def create_constraint_graph(planner):
    """
    Create constraint graph with states and transitions.
    
    States:
    - placement: Ball on ground, gripper free
    - grasp: Ball grasped by gripper
    
    Args:
        planner: PyHPPManipulationPlanner instance
        
    Returns:
        Graph object
    """
    if not HAS_PYHPP:
        raise ImportError("PyHPP is required for this example")
    
    robot = planner.get_robot()
    problem = planner.get_problem()
    
    graph = Graph("graph", robot, problem)
    
    # Create states
    state_placement = graph.createState("placement", False, 0)
    state_grasp = graph.createState("grasp", False, 0)
    
    # Create transitions
    transition_transit = graph.createTransition(
        state_placement, state_placement, "transit", 1, state_placement
    )
    transition_transfer = graph.createTransition(
        state_grasp, state_grasp, "transfer", 1, state_grasp
    )
    transition_grasp_ball = graph.createTransition(
        state_placement, state_grasp, "grasp-ball", 1, state_placement
    )
    transition_release_ball = graph.createTransition(
        state_grasp, state_placement, "release-ball", 1, state_grasp
    )
    
    # Get joint IDs
    joint_gripper = robot.model().getJointId(ManipulationConfig.GRIPPER_NAME)
    joint_ball = robot.model().getJointId(ManipulationConfig.BALL_NAME)
    Id = SE3.Identity()
    
    # Create placement constraint (ball on ground)
    mask_placement = [False, False, True, True, True, False]
    q_ground = Quaternion(0, 0, 0, 1)
    ball_ground = SE3(q_ground, np.array([0, 0, ManipulationConfig.BALL_RADIUS]))
    
    pc_placement = Transformation.create(
        "placement_constraint",
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_ground,
        mask_placement
    )
    cts = ComparisonTypes()
    cts[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    implicit_mask = [True, True, True]
    placement_constraint = Implicit.create(pc_placement, cts, implicit_mask)
    
    # Create placement complement constraint
    mask_complement = [True, False, False, False, False, True]
    pc_complement = Transformation.create(
        "placement_complement_constraint",
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_ground,
        mask_complement
    )
    cts_complement = ComparisonTypes()
    cts_complement[:] = (
        ComparisonType.Equality,
        ComparisonType.Equality,
        ComparisonType.Equality,
    )
    placement_complement_constraint = Implicit.create(
        pc_complement, cts_complement, implicit_mask
    )
    
    # Create grasp constraint (ball in gripper)
    q_grasp = Quaternion(0.5, 0.5, -0.5, 0.5)
    ball_in_gripper = SE3(q_grasp, np.array([0, 0.137, 0]))
    mask_grasp = Mask()
    mask_grasp[:] = (True,) * 6
    
    pc_grasp = RelativeTransformation.create(
        "grasp",
        robot.asPinDevice(),
        joint_gripper,
        joint_ball,
        ball_in_gripper,
        Id,
        mask_grasp
    )
    cts_grasp = ComparisonTypes()
    cts_grasp[:] = (
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
        ComparisonType.EqualToZero,
    )
    grasp_constraint = Implicit.create(pc_grasp, cts_grasp, mask_grasp)
    
    # Set constant right-hand side
    problem.setConstantRightHandSide(placement_constraint, True)
    problem.setConstantRightHandSide(placement_complement_constraint, False)
    
    # Add constraints to states
    graph.addNumericalConstraintsToState(state_placement, [placement_constraint])
    graph.addNumericalConstraintsToState(state_grasp, [grasp_constraint])
    
    # Add constraints to transitions
    graph.addNumericalConstraintsToTransition(
        transition_transit, [placement_complement_constraint]
    )
    graph.addNumericalConstraintsToTransition(
        transition_grasp_ball, [placement_complement_constraint]
    )
    
    # Initialize graph
    graph.maxIterations(100)
    graph.errorThreshold(1e-5)
    graph.initialize()
    
    return graph


def main(visualize=True, solve=True):
    """
    Run grasp ball manipulation example.
    
    Args:
        visualize: Whether to display visualization
        solve: Whether to solve the planning problem
    """
    if not HAS_PYHPP:
        print("Error: PyHPP backend not available")
        return False
    
    print("=" * 70)
    print("Grasp Ball Manipulation - PyHPP Backend")
    print("=" * 70)
    
    # Create planner
    planner = PyHPPManipulationPlanner()
    
    # Load robot
    print("\n1. Loading robot and objects...")
    planner.load_robot(
        name="ur5",
        urdf_path=ManipulationConfig.ROBOT_URDF,
        srdf_path=ManipulationConfig.ROBOT_SRDF,
        root_joint_type="anchor"
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
    
    # Create constraint graph
    print("\n2. Creating constraint graph...")
    graph = create_constraint_graph(planner)
    
    # Set configurations
    print("\n3. Setting initial and goal configurations...")
    q_init = np.array(InitialConfigurations.FULL_INIT)
    
    # Project initial config onto placement state
    res, q_init_proj, error = graph.applyStateConstraints(
        graph.createState("placement", False, 0), q_init
    )
    if not res:
        print(f"Warning: Failed to project initial config (error: {error})")
        q_init_proj = q_init
    
    # Goal: move ball to x=0.2
    q_goal = q_init_proj.copy()
    q_goal[6] = 0.2  # Change ball x position
    
    # Project goal config onto placement state
    res, q_goal_proj, error = graph.applyStateConstraints(
        graph.createState("placement", False, 0), q_goal
    )
    if not res:
        print(f"Warning: Failed to project goal config (error: {error})")
        q_goal_proj = q_goal
    
    planner.set_initial_config(q_init_proj)
    planner.add_goal_config(q_goal_proj)
    
    # Set graph in problem
    problem = planner.get_problem()
    problem.constraintGraph(graph)
    
    print("\nInitial configuration:")
    print(f"  Robot: {q_init_proj[:6]}")
    print(f"  Ball: {q_init_proj[6:]}")
    
    print("\nGoal configuration:")
    print(f"  Robot: {q_goal_proj[:6]}")
    print(f"  Ball: {q_goal_proj[6:]}")
    
    # Visualize
    if visualize:
        print("\n4. Visualizing initial configuration...")
        try:
            planner.visualize(q_init_proj)
            print("  ✓ Visualization ready")
        except Exception as e:
            print(f"  ✗ Visualization failed: {e}")
    
    # Solve
    if solve:
        print("\n5. Solving manipulation problem...")
        print("  This may take a while...")
        
        success = planner.solve(max_iterations=5000)
        
        if success:
            print("  ✓ Solution found!")
            
            if visualize:
                print("\n6. Playing solution path...")
                try:
                    planner.play_path()
                    print("  ✓ Path playback complete")
                except Exception as e:
                    print(f"  ✗ Path playback failed: {e}")
        else:
            print("  ✗ No solution found")
            return False
    
    print("\n" + "=" * 70)
    print("Example completed successfully!")
    print("=" * 70)
    
    return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Grasp ball manipulation example with PyHPP"
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
