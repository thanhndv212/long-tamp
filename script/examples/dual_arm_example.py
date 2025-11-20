#!/usr/bin/env python3
"""
Dual-arm coordination example.

This example demonstrates coordinated manipulation with two robot arms
working together on a shared object.
"""

import sys
from pathlib import Path
import numpy as np

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab import ManipulationPlanner
from agimus_spacelab.config import RuleGenerator
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from spacelab_config import ManipulationConfig


def main(backend="pyhpp", visualize=True):
    """
    Dual-arm coordination example.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
        visualize: Whether to visualize the scene
    """
    print("=" * 70)
    print("Dual-Arm Coordination Example")
    print("=" * 70)
    
    # Create planner
    print(f"\n1. Creating planner (backend: {backend})...")
    planner = ManipulationPlanner(backend=backend)
    
    # Load robots
    print("2. Loading robots...")
    pkg_robot1 = "package://robot1/description"
    pkg_robot2 = "package://robot2/description"
    
    # Load first robot
    planner.load_robot(
        name="robot1",
        urdf_path=f"{pkg_robot1}/urdf/robot.urdf",
        srdf_path=f"{pkg_robot1}/srdf/robot.srdf"
    )
    
    # Load second robot
    planner.load_robot(
        name="robot2",
        urdf_path=f"{pkg_robot2}/urdf/robot.urdf",
        srdf_path=f"{pkg_robot2}/srdf/robot.srdf"
    )
    
    # Load environment
    print("3. Loading environment...")
    planner.load_environment(
        name="workspace",
        urdf_path="package://environment/urdf/workspace.urdf"
    )
    
    # Load large object (requires two hands)
    print("4. Loading object...")
    planner.load_object(
        name="large_panel",
        urdf_path="package://objects/urdf/panel.urdf",
        root_joint_type="freeflyer"
    )
    
    # Define configurations
    print("5. Defining configurations...")
    
    # Robot 1 configuration (6 DOF)
    q_robot1 = np.array([0.0, -1.0, 1.5, 0.0, 1.0, 0.0])
    
    # Robot 2 configuration (6 DOF)
    q_robot2 = np.array([0.0, 1.0, -1.5, 0.0, -1.0, 0.0])
    
    # Panel initial pose (between robots)
    panel_start_xyzrpy = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    panel_start = xyzrpy_to_xyzquat(panel_start_xyzrpy)
    
    # Panel goal pose (rotated 90 degrees)
    panel_goal_xyzrpy = [0.0, 0.0, 1.2, 0.0, 0.0, 1.57]
    panel_goal = xyzrpy_to_xyzquat(panel_goal_xyzrpy)
    
    # Initial configuration
    q_init = np.concatenate([q_robot1, q_robot2, panel_start])
    
    # Goal configuration
    q_goal = np.concatenate([q_robot1, q_robot2, panel_goal])
    
    print(f"   Total DOF: {len(q_init)}")
    print(f"   - Robot 1: {len(q_robot1)} DOF")
    print(f"   - Robot 2: {len(q_robot2)} DOF")
    print(f"   - Panel: {len(panel_start)} DOF")
    
    # Set configurations
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    # Create constraint graph with sequential rules
    print("6. Creating constraint graph...")
    
    # Define task sequence: both robots grasp panel
    task_sequence = [
        ("robot1/gripper", "large_panel/handle_left"),
        ("robot2/gripper", "large_panel/handle_right"),
    ]
    
    # Generate sequential rules
    rules = RuleGenerator.generate_sequential_rules(
        ManipulationConfig,
        task_sequence
    )
    
    graph = planner.create_constraint_graph(
        name="dual_arm_graph",
        grippers=["robot1/gripper", "robot2/gripper"],
        objects={
            "large_panel": {
                "handles": [
                    "large_panel/handle_left",
                    "large_panel/handle_right"
                ],
                "contact_surfaces": [],
            }
        },
        rules=rules
    )
    
    # Visualize
    if visualize:
        print("\n7. Visualizing scene...")
        planner.visualize(q_init)
    
    print("\n" + "=" * 70)
    print("Dual-Arm Setup Complete!")
    print("=" * 70)
    
    print("\nTask: Rotate large panel using two robot arms")
    print("\nConstraint graph enforces:")
    print("  - Robot 1 grasps left handle")
    print("  - Robot 2 grasps right handle")
    print("  - Both maintain grasp during motion")
    
    print("\nTo solve:")
    print("  success = planner.solve(max_iterations=10000)")
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
