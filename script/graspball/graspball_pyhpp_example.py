#!/usr/bin/env python3
"""
Grasp ball manipulation example using PyHPP backend.

This example demonstrates manipulation planning with PyHPP to:
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

Based on: grasp_ball_in_box_refactor.py from hpp-practicals
Uses: agimus_spacelab.pyhpp.PyHPPManipulationPlanner
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab.backends import PyHPPBackend as PyHPPManipulationPlanner  # noqa: E402
from graspball_config import (  # noqa: E402
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

# PyHPP-specific imports
try:
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, Quaternion, StdVec_Bool as Mask
    HAS_PYHPP = True
except ImportError:
    HAS_PYHPP = False
    print("Warning: PyHPP not available")


# ============================================================================
# Constraint Graph Setup
# ============================================================================

def create_constraints(planner):
    """
    Create all transformation constraints using planner's robot.
    
    Returns dictionary of constraint name -> Implicit constraint object
    """
    constraints = {}
    
    robot = planner.get_robot()
    
    # Get joint IDs
    joint_gripper = robot.model().getJointId(
        ManipulationConfig.GRIPPER_JOINT
    )
    joint_ball = robot.model().getJointId(ManipulationConfig.BALL_JOINT)
    Id = SE3.Identity()
    
    # ========================================================================
    # 1. GRASP: gripper/ball fixed (all 6 DOF)
    # ========================================================================
    q_grasp = Quaternion(0.5, 0.5, -0.5, 0.5)
    ball_in_gripper = SE3(q_grasp, np.array([0, 0.137, 0]))
    mask_full = Mask()
    mask_full[:] = (True,) * 6
    
    pc_grasp = RelativeTransformation.create(
        'grasp',
        robot.asPinDevice(),
        joint_gripper,
        joint_ball,
        ball_in_gripper,
        Id,
        mask_full
    )
    
    cts_grasp = ComparisonTypes()
    cts_grasp[:] = tuple([ComparisonType.EqualToZero] * 6)
    constraints['grasp'] = Implicit.create(pc_grasp, cts_grasp, mask_full)
    
    # ========================================================================
    # 2. PLACEMENT: world/ball - fixed z, fixed r, p
    # ========================================================================
    q_placement = Quaternion(0, 0, 0, 1)
    ball_on_ground = SE3(
        q_placement, np.array([0, 0, ManipulationConfig.BALL_RADIUS])
    )
    mask_placement = [False, False, True, True, True, False]
    
    pc_placement = Transformation.create(
        'placement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_on_ground,
        mask_placement
    )
    
    cts_placement = ComparisonTypes()
    cts_placement[:] = tuple([ComparisonType.EqualToZero] * 3)
    implicit_mask_placement = [True, True, True]
    constraints['placement'] = Implicit.create(
        pc_placement, cts_placement, implicit_mask_placement
    )
    
    # ========================================================================
    # 3. PLACEMENT/COMPLEMENT: world/ball - fixed x, y, yaw
    # ========================================================================
    mask_placement_comp = [True, True, False, False, False, True]
    
    pc_placement_comp = Transformation.create(
        'placement/complement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_on_ground,
        mask_placement_comp
    )
    
    # NOTE: Complement constraints use ComparisonType.Equality (not EqualToZero)
    cts_placement_comp = ComparisonTypes()
    cts_placement_comp[:] = tuple([ComparisonType.Equality] * 3)
    implicit_mask_comp = [True, True, True]
    constraints['placement/complement'] = Implicit.create(
        pc_placement_comp, cts_placement_comp, implicit_mask_comp
    )
    
    # ========================================================================
    # 4. GRIPPER_BALL_ALIGNED: gripper/ball - all fixed
    # ========================================================================
    q_aligned = Quaternion(0.5, 0.5, -0.5, 0.5)
    gripper_above_ball = SE3(q_aligned, np.array([0, 0.2, 0]))
    mask_aligned = Mask()
    mask_aligned[:] = (True,) * 6
    
    pc_aligned = RelativeTransformation.create(
        'gripper_ball_aligned',
        robot.asPinDevice(),
        joint_gripper,
        joint_ball,
        gripper_above_ball,
        Id,
        mask_aligned
    )
    
    cts_aligned = ComparisonTypes()
    cts_aligned[:] = tuple([ComparisonType.EqualToZero] * 6)
    constraints['gripper_ball_aligned'] = Implicit.create(
        pc_aligned, cts_aligned, mask_aligned
    )
    
    # ========================================================================
    # 5. BALL_NEAR_TABLE: world/ball - fixed z, fixed r, p
    # ========================================================================
    q_table = Quaternion(0, 0, 0, 1)
    ball_near_table = SE3(q_table, np.array([0.3, 0, 0.1]))
    mask_table = [False, False, True, True, True, False]
    
    pc_table = Transformation.create(
        'ball_near_table',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_near_table,
        mask_table
    )
    
    cts_table = ComparisonTypes()
    cts_table[:] = tuple([ComparisonType.EqualToZero] * 3)
    implicit_mask_table = [True, True, True]
    constraints['ball_near_table'] = Implicit.create(
        pc_table, cts_table, implicit_mask_table
    )
    
    # ========================================================================
    # 6. BALL_NEAR_TABLE/COMPLEMENT: world/ball - fixed x, y, yaw
    # ========================================================================
    ball_near_table_comp = SE3(q_table, np.array([0.3, 0.2, 0.1]))
    mask_table_comp = [True, True, False, False, False, True]
    
    pc_table_comp = Transformation.create(
        'ball_near_table/complement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_near_table_comp,
        mask_table_comp
    )
    
    # NOTE: Complement constraints use ComparisonType.Equality (not EqualToZero)
    cts_table_comp = ComparisonTypes()
    cts_table_comp[:] = tuple([ComparisonType.Equality] * 3)
    implicit_mask_table_comp = [True, True, True]
    constraints['ball_near_table/complement'] = Implicit.create(
        pc_table_comp, cts_table_comp, implicit_mask_table_comp
    )
    
    return constraints


def create_constraint_graph(planner):
    """
    Create constraint graph for manipulation with 5 states.
    
    Uses planner's advanced features: create_state, create_edge, etc.
    
    States:
    - placement: Ball on ground
    - gripper-above-ball: Gripper aligned above ball, ball on ground
    - grasp-placement: Gripper grasping ball, ball on ground
    - ball-above-ground: Gripper grasping ball, ball near table
    - grasp: Gripper grasping ball (free motion)
    
    Returns:
        tuple: (states_dict, edges_dict, constraints_dict)
    """
    
    if not HAS_PYHPP:
        raise ImportError("PyHPP is required for this example")
    
    # Create constraints first
    constraints = create_constraints(planner)
    print("✓ Created transformation constraints")
    
    # Initialize graph through planner
    # Note: PyHPPManipulationPlanner doesn't have direct graph creation
    # We need to use the raw pyhpp Graph
    from pyhpp.manipulation import Graph
    
    robot = planner.get_robot()
    problem = planner.get_problem()
    
    graph = Graph("manipulation_graph", robot, problem)
    print("✓ Created constraint graph")
    
    # ========================================================================
    # Create States (Nodes)
    # Order matters for solver performance! Must match task flow:
    # grasp -> ball-above-ground -> grasp-placement -> gripper-above-ball -> placement
    # ========================================================================
    
    states = {}
    states['grasp'] = graph.createState("grasp", False, 0)
    states['ball-above-ground'] = graph.createState(
        "ball-above-ground", False, 0
    )
    states['grasp-placement'] = graph.createState(
        "grasp-placement", False, 0
    )
    states['gripper-above-ball'] = graph.createState(
        "gripper-above-ball", False, 0
    )
    states['placement'] = graph.createState("placement", False, 0)
    
    print("✓ Created 5 states (order: grasp -> ... -> placement)")
    
    # ========================================================================
    # Create Edges (Transitions)
    # ========================================================================
    
    edges = {}
    
    # Self-loops
    edges['transit'] = graph.createTransition(
        states['placement'], states['placement'],
        "transit", 1, states['placement']
    )
    edges['transfer'] = graph.createTransition(
        states['grasp'], states['grasp'],
        "transfer", 1, states['grasp']
    )
    
    # From placement
    edges['approach-ball'] = graph.createTransition(
        states['placement'], states['gripper-above-ball'],
        "approach-ball", 1, states['placement']
    )
    
    # From gripper-above-ball
    edges['move-gripper-away'] = graph.createTransition(
        states['gripper-above-ball'], states['placement'],
        "move-gripper-away", 1, states['placement']
    )
    edges['grasp-ball'] = graph.createTransition(
        states['gripper-above-ball'], states['grasp-placement'],
        "grasp-ball", 1, states['placement']
    )
    
    # From grasp-placement
    edges['move-gripper-up'] = graph.createTransition(
        states['grasp-placement'], states['gripper-above-ball'],
        "move-gripper-up", 1, states['placement']
    )
    edges['take-ball-up'] = graph.createTransition(
        states['grasp-placement'], states['ball-above-ground'],
        "take-ball-up", 1, states['grasp']
    )
    
    # From ball-above-ground
    edges['put-ball-down'] = graph.createTransition(
        states['ball-above-ground'], states['grasp-placement'],
        "put-ball-down", 1, states['grasp']
    )
    edges['take-ball-away'] = graph.createTransition(
        states['ball-above-ground'], states['grasp'],
        "take-ball-away", 1, states['grasp']
    )
    
    # From grasp
    edges['approach-ground'] = graph.createTransition(
        states['grasp'], states['ball-above-ground'],
        "approach-ground", 1, states['grasp']
    )
    
    print("✓ Created transitions")
    
    # ========================================================================
    # Add Constraints to States
    # ========================================================================
    
    # State: placement (ball on ground)
    graph.addNumericalConstraint(
        states['placement'],
        constraints['placement']
    )
    
    # State: gripper-above-ball (ball on ground, gripper aligned)
    graph.addNumericalConstraint(
        states['gripper-above-ball'], constraints['placement']
    )
    graph.addNumericalConstraint(
        states['gripper-above-ball'], constraints['gripper_ball_aligned']
    )
    
    # State: grasp-placement (grasping ball on ground)
    graph.addNumericalConstraint(
        states['grasp-placement'], constraints['grasp']
    )
    graph.addNumericalConstraint(
        states['grasp-placement'], constraints['placement']
    )
    
    # State: ball-above-ground (grasping ball near table)
    graph.addNumericalConstraint(
        states['ball-above-ground'], constraints['grasp']
    )
    graph.addNumericalConstraint(
        states['ball-above-ground'], constraints['ball_near_table']
    )
    
    # State: grasp (grasping ball, free motion)
    graph.addNumericalConstraint(states['grasp'], constraints['grasp'])
    
    print("✓ Added constraints to states")
    
    # ========================================================================
    # Add Constraints to Edges (CRITICAL for solver performance!)
    # ========================================================================
    
    # Ball on table: transit, approach-ball, move-gripper-away
    graph.addNumericalConstraintsToTransition(
        edges['transit'], [constraints['placement/complement']]
    )
    graph.addNumericalConstraintsToTransition(
        edges['approach-ball'], [constraints['placement/complement']]
    )
    graph.addNumericalConstraintsToTransition(
        edges['move-gripper-away'], [constraints['placement/complement']]
    )
    
    # Gripper above ball, ball on table: grasp-ball, move-gripper-up
    graph.addNumericalConstraintsToTransition(
        edges['grasp-ball'], [constraints['placement/complement']]
    )
    graph.addNumericalConstraintsToTransition(
        edges['move-gripper-up'], [constraints['placement/complement']]
    )
    
    # Gripper grasping ball, lifting: take-ball-up, put-ball-down
    graph.addNumericalConstraintsToTransition(
        edges['take-ball-up'], [constraints['ball_near_table/complement']]
    )
    graph.addNumericalConstraintsToTransition(
        edges['put-ball-down'], [constraints['ball_near_table/complement']]
    )
    
    # Note: take-ball-away, approach-ground, transfer have no path constraints
    # (free motion with grasp constraint from state)
    
    print("✓ Added constraints to edges")
    
    # ========================================================================
    # Note: setConstantRightHandSide is NOT available in PyHPP bindings
    # It's a CORBA-only feature. The solver should still work without it.
    # In CORBA: ps.setConstantRightHandSide('placement', True)
    # ========================================================================
    
    # ========================================================================
    # Configure Graph Parameters
    # ========================================================================
    
    graph.maxIterations(100)
    graph.errorThreshold(0.00001)
    
    # ========================================================================
    # Initialize Graph (REQUIRED)
    # ========================================================================
    
    graph.initialize()
    print("✓ Graph initialized")
    
    # Store graph in problem
    problem.constraintGraph(graph)
    
    return graph, states, edges, constraints
# ============================================================================
# Configuration Testing
# ============================================================================

def generate_configurations(graph, states, edges, planner):
    """
    Test the constraint graph by generating configurations along the path.
    
    This mimics the CORBA version's configuration testing.
    """
    
    print("\n" + "=" * 70)
    print("Testing Graph Traversal")
    print("=" * 70)
    
    Q = []  # Store all configurations
    
    # Get configuration shooter
    problem = planner.get_problem()
    shooter = problem.configurationShooter()
    
    # Initial configuration
    q1 = np.array(
        InitialConfigurations.UR5 +
        InitialConfigurations.POKEBALL
    )
    
    # 1. Apply placement state constraints
    print("\n1. Projecting initial config onto 'placement' state...")
    result = graph.applyStateConstraints(states['placement'], q1)
    if not result.success:
        print(f"   ⚠ Projection failed with error {result.error}")
    q_init = result.configuration
    print(f"   ✓ Initial configuration: {np.around(q_init, 4)}")
    Q.append(q_init)
    
    # 2. Generate target for 'approach-ball'
    print("\n2. Generating target for 'approach-ball'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(
            edges['approach-ball'], q_init, q_rand
        )
        if result.success:
            q_ab = result.configuration
            break
    if result.success:
        print(f"   ✓ After approach-ball: {np.around(q_ab, 4)}")
        Q.append(q_ab)
    else:
        print("   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 3. Apply 'gripper-above-ball' state
    print("\n3. Projecting onto 'gripper-above-ball' state...")
    result = graph.applyStateConstraints(
        states['gripper-above-ball'], q_ab
    )
    q_gab = result.configuration
    print(f"   ✓ At gripper-above-ball: {np.around(q_gab, 4)}")
    Q.append(q_gab)
    
    # 4. Generate target for 'grasp-ball'
    print("\n4. Generating target for 'grasp-ball'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(
            edges['grasp-ball'], q_gab, q_rand
        )
        if result.success:
            q_gb = result.configuration
            break
    if result.success:
        print(f"   ✓ After grasp-ball: {np.around(q_gb, 4)}")
        Q.append(q_gb)
    else:
        print("   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 5. Apply 'grasp-placement' state
    print("\n5. Projecting onto 'grasp-placement' state...")
    result = graph.applyStateConstraints(states['grasp-placement'], q_gb)
    q_gp = result.configuration
    print(f"   ✓ At grasp-placement: {np.around(q_gp, 4)}")
    Q.append(q_gp)
    
    # 6. Generate target for 'take-ball-up'
    print("\n6. Generating target for 'take-ball-up'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(
            edges['take-ball-up'], q_gp, q_rand
        )
        if result.success:
            q_tbu = result.configuration
            break
    if result.success:
        print(f"   ✓ After take-ball-up: {np.around(q_tbu, 4)}")
        Q.append(q_tbu)
    else:
        print("   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 7. Apply 'ball-above-ground' state
    print("\n7. Projecting onto 'ball-above-ground' state...")
    result = graph.applyStateConstraints(
        states['ball-above-ground'], q_tbu
    )
    q_bag = result.configuration
    print(f"   ✓ At ball-above-ground: {np.around(q_bag, 4)}")
    Q.append(q_bag)
    
    # 8. Generate target for 'take-ball-away'
    print("\n8. Generating target for 'take-ball-away'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(
            edges['take-ball-away'], q_bag, q_rand
        )
        if result.success:
            q_tba = result.configuration
            break
    if result.success:
        print(f"   ✓ After take-ball-away: {np.around(q_tba, 4)}")
        Q.append(q_tba)
    else:
        print("   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 9. Apply 'grasp' state
    print("\n9. Projecting onto 'grasp' state...")
    result = graph.applyStateConstraints(states['grasp'], q_tba)
    q_g = result.configuration
    print(f"   ✓ At grasp: {np.around(q_g, 4)}")
    Q.append(q_g)
    
    # 10. Generate goal configuration
    print("\n10. Generating goal configuration...")
    q2 = q1.copy()
    q2[6] = 0.2  # Move ball x position
    result = graph.applyStateConstraints(states['placement'], q2)
    q_goal = result.configuration
    print(f"   ✓ Goal configuration: {np.around(q_goal, 4)}")
    
    print("\n✓ Graph traversal test complete!")
    print(f"  Generated {len(Q)} configurations")
    
    return Q, q_init, q_goal


# ============================================================================
# Utility Functions
# ============================================================================

def animate_configurations(viewer, configs, dt=0.5):
    """
    Animate a sequence of configurations.
    
    Args:
        viewer: Gepetto Viewer instance
        configs: List of configurations
        dt: Time delay between configurations
    """
    import time
    
    print(f"\nAnimating {len(configs)} configurations...")
    for i, q in enumerate(configs):
        print(f"  Config {i+1}/{len(configs)}")
        viewer(q)
        time.sleep(dt)
    print("✓ Animation complete!")


def animate_path(viewer, path, dt=0.01):
    """
    Animate a path in the viewer.
    
    Args:
        viewer: Gepetto Viewer instance
        path: Path object
        dt: Time step
    """
    import time
    
    t = 0.0
    path_length = path.length()
    
    print(f"Animating path (length={path_length:.3f})...")
    
    while t <= path_length:
        q = path(t)
        viewer(q)
        time.sleep(dt)
        t += dt
    
    print("✓ Animation complete!")


# ============================================================================
# Main Function
# ============================================================================

def main(visualize=True, solve=True):
    """
    Run grasp ball manipulation example.
    
    Args:
        visualize: Whether to display visualization
        solve: Whether to solve the planning problem
    """
    if not HAS_PYHPP:
        print("Error: PyHPP backend not available")
        return False, None, None, None, None, None, None, None, None
    
    print("=" * 70)
    print("Grasp Ball Manipulation - PyHPP Backend")
    print("=" * 70)
    
    # 1. Create planner and load robot
    print("\n1. Loading robot and objects...")
    planner = PyHPPManipulationPlanner()
    
    # Load robot
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
    planner.set_joint_bounds(ManipulationConfig.BALL_JOINT, bounds)
    
    # Load ground (visualization only)
    planner.load_environment(
        name="ground",
        urdf_path=ManipulationConfig.GROUND_URDF
    )
    
    # Load box at correct position (0.3, 0, 0.04)
    # In CORBA, this is done via ps.moveObstacle() for each wall
    # In PyHPP, we position the entire box at load time
    from pinocchio import SE3
    box_pose = SE3(
        rotation=np.eye(3),
        translation=np.array([ManipulationConfig.BOX_X, 0, 0.04])
    )
    planner.load_environment(
        name="box",
        urdf_path=ManipulationConfig.BOX_URDF,
        pose=box_pose
    )
    
    print("✓ Robot and objects loaded")
    
    # 2. Configure path validation and projection BEFORE graph creation
    print("\n2. Configuring path validation and projection...")
    from pyhpp.manipulation import createProgressiveProjector
    from pyhpp.core import (
        createDiscretizedCollisionAndJointBound,
        createDiscretizedCollision,
        createDiscretizedJointBound,
        createDiscretized,
    )
    
    
    problem = planner.get_problem()
    robot = planner.get_robot()
    
    # Path projector (Progressive) - for constraint projection along paths
    problem.pathProjector = createProgressiveProjector(
        problem.distance(), problem.steeringMethod(),
        ManipulationConfig.PATH_PROJECTOR_STEP  # 0.1
    )
    
    # Path validation (Discretized) - for collision checking
    problem.pathValidation = createDiscretized(
        robot.asPinDevice(),
        ManipulationConfig.PATH_VALIDATION_STEP  # 0.01
    )
    
    # 3. Create constraint graph
    print("\n3. Creating constraint graph...")
    graph, states, edges, constraints = create_constraint_graph(planner)
    
    # 4. Test graph traversal
    Q, q_init, q_goal = generate_configurations(
        graph, states, edges, planner
    )
    
    # 5. Setup planning problem
    print("\n" + "=" * 70)
    print("Setting up planning problem")
    print("=" * 70)
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    print("✓ Problem setup complete")
    
    # 6. Visualization
    viewer = None
    if visualize:
        print("\n6. Starting visualization...")
        try:
            from pyhpp.gepetto.viewer import Viewer
            robot = planner.get_robot()
            viewer = Viewer(robot)
            viewer(q_init)
            print("✓ Displaying initial configuration")
        except Exception as e:
            print(f"⚠ Visualization failed: {e}")
    
    # 7. Solve
    path = None
    solve_success = False
    if solve:
        print("\n7. Solving manipulation problem...")
        print("  This may take a while...")
        
        from pyhpp.manipulation import ManipulationPlanner as HPPPlanner
        
        problem = planner.get_problem()
        
        hpp_planner = HPPPlanner(problem)
        hpp_planner.maxIterations(10000)
        
        print("  Max iterations: 10000")
        
        try:
            solve_success = hpp_planner.solve()
            
            if solve_success:
                print("  ✓ Solution found!")
                path = hpp_planner.path()
                
                if visualize and viewer:
                    print("\n8. Playing solution path...")
                    try:
                        animate_path(viewer, path)
                        print("  ✓ Path playback complete")
                    except Exception as e:
                        print(f"  ✗ Path playback failed: {e}")
            else:
                print("  ✗ No solution found")
                solve_success = False
        except Exception as e:
            print(f"  ✗ Planning failed: {e}")
            solve_success = False
    
    # Final summary
    print("\n" + "=" * 70)
    if solve and solve_success:
        print("Example Completed Successfully!")
    else:
        print("Setup Complete!")
    print("=" * 70)
    
    if not solve or not solve_success:
        print("\nNext steps:")
        if not solve:
            print("1. Access returned objects to solve manually")
            print("2. Create ManipulationPlanner: ")
            print("   from pyhpp.manipulation import ManipulationPlanner")
            print("   hpp_planner = ManipulationPlanner(problem)")
            print("3. Call hpp_planner.solve()")
        else:
            print("1. Review graph configuration and constraints")
            print("2. Check Q configurations for validity")
            print("3. Try adjusting path validation parameters")
        print("4. Use viewer(q) to display configurations")
        print("5. Animate configs: animate_configurations(viewer, Q, dt=1.0)")
    
    print("\nReturned objects:")
    print("  planner     - PyHPPManipulationPlanner instance")
    print("  graph       - Constraint graph")
    print("  states      - Dictionary of state objects")
    print("  edges       - Dictionary of edge objects")
    print("  constraints - Dictionary of constraint objects")
    print("  viewer      - Gepetto viewer (if visualization enabled)")
    print("  path        - Solution path (if solved successfully)")
    print("  Q           - List of test configurations")
    
    return (
        solve_success if solve else True,
        planner, graph, states, edges,
        constraints, viewer, path, Q
    )


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
    
    result = main(
        visualize=not args.no_viz,
        solve=not args.no_solve
    )
    
    if result[0]:  # success
        (
            success, planner, graph, states, edges,
            constraints, viewer, path, Q
        ) = result
        sys.exit(0)
    else:
        sys.exit(1)
