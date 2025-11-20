#!/usr/bin/env python3
"""
Pick and place manipulation example.

This example demonstrates a simple pick-and-place task using agimus_spacelab
with a single robot and object.
"""

import numpy as np
from agimus_spacelab import ManipulationPlanner
from agimus_spacelab.utils import xyzrpy_to_xyzquat


def main(backend="corba", visualize=True):
    """
    Simple pick and place example.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
        visualize: Whether to visualize the scene
    """
    print("=" * 70)
    print("Pick and Place Example")
    print("=" * 70)
    
    # Create planner
    print(f"\n1. Creating planner (backend: {backend})...")
    planner = ManipulationPlanner(backend=backend)
    
    # Load robot
    print("2. Loading robot...")
    pkg = "package://my_robot/description"
    planner.load_robot(
        name="robot",
        urdf_path=f"{pkg}/urdf/robot.urdf",
        srdf_path=f"{pkg}/srdf/robot.srdf"
    )
    
    # Load environment
    print("3. Loading environment...")
    planner.load_environment(
        name="table",
        urdf_path="package://environment/urdf/table.urdf"
    )
    
    # Load object to manipulate
    print("4. Loading object...")
    planner.load_object(
        name="box",
        urdf_path="package://objects/urdf/box.urdf",
        root_joint_type="freeflyer"
    )
    
    # Define configurations
    print("5. Defining configurations...")
    
    # Robot configuration (6 DOF arm)
    q_robot_home = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
    
    # Object initial pose (on table)
    box_start_xyzrpy = [0.5, 0.0, 0.8, 0.0, 0.0, 0.0]
    box_start = xyzrpy_to_xyzquat(box_start_xyzrpy)
    
    # Object goal pose (different location on table)
    box_goal_xyzrpy = [0.5, 0.5, 0.8, 0.0, 0.0, 0.0]
    box_goal = xyzrpy_to_xyzquat(box_goal_xyzrpy)
    
    # Initial configuration: robot at home, box at start
    q_init = np.concatenate([q_robot_home, box_start])
    
    # Goal configuration: robot at home, box at goal
    q_goal = np.concatenate([q_robot_home, box_goal])
    
    print(f"   Initial config: {len(q_init)} DOF")
    print(f"   Goal config: {len(q_goal)} DOF")
    
    # Set configurations
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    # Create constraint graph
    print("6. Creating constraint graph...")
    graph = planner.create_constraint_graph(
        name="pick_place_graph",
        grippers=["robot/gripper"],
        objects={
            "box": {
                "handles": ["box/handle"],
                "contact_surfaces": ["box/top"],
            }
        },
        rules="all"  # Allow all grasp combinations
    )
    
    # Visualize
    if visualize:
        print("\n7. Visualizing scene...")
        print("   Displaying initial configuration...")
        planner.visualize(q_init)
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    
    print("\nTo solve:")
    print("  success = planner.solve()")
    print("  if success:")
    print("      planner.play_path(0)")
    
    return planner


if __name__ == "__main__":
    import sys
    
    backend = "corba" if "--corba" in sys.argv else "pyhpp"
    no_viz = "--no-viz" in sys.argv
    
    try:
        planner = main(backend=backend, visualize=not no_viz)
        print("\nPlanner object available for interactive use.")
    except Exception as e:
        print(f"\n⚠ Error: {e}")
        print("  (This is expected if robot models are not available)")
