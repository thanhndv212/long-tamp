#!/usr/bin/env python3
"""
Visualization example for agimus_spacelab.

This example demonstrates how to use visualization features with both
CORBA and PyHPP backends.
"""

import sys
from pathlib import Path
import numpy as np
import time

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab import ManipulationPlanner, get_available_backends
from agimus_spacelab.utils import xyzrpy_to_xyzquat
from spacelab_config import InitialConfigurations


def interpolate_configs(q_start, q_goal, num_steps=10):
    """
    Create interpolated configurations between start and goal.
    
    Args:
        q_start: Start configuration
        q_goal: Goal configuration
        num_steps: Number of interpolation steps
        
    Returns:
        List of interpolated configurations
    """
    configs = []
    for i in range(num_steps + 1):
        alpha = i / num_steps
        q = (1 - alpha) * q_start + alpha * q_goal
        configs.append(q)
    return configs


def demo_static_visualization(backend="corba"):
    """
    Demonstrate static configuration visualization.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
    """
    print(f"\n{'=' * 70}")
    print(f"Static Visualization Demo ({backend.upper()} backend)")
    print('=' * 70)
    
    # Create planner
    planner = ManipulationPlanner(backend=backend)
    
    # Load models (simplified paths for example)
    pkg = "package://spacelab_mock_hardware/description"
    try:
        planner.load_robot(
            "spacelab-robots",
            f"{pkg}/urdf/allRobots_spacelab_robot.urdf",
            f"{pkg}/srdf/allRobots_spacelab_robot.srdf"
        )
        
        planner.load_environment("ground_demo", f"{pkg}/urdf/ground_demo.urdf")
        planner.load_object("RS1", f"{pkg}/urdf/RS.urdf", "freeflyer")
        
    except Exception as e:
        print(f"⚠ Model loading failed: {e}")
        print("  (This is expected if URDFs are not available)")
        return None
    
    # Build configuration
    q_robot = np.array(
        InitialConfigurations.UR10 + 
        InitialConfigurations.VISPA2 + 
        InitialConfigurations.VISPA
    )
    
    q_rs1 = xyzrpy_to_xyzquat(InitialConfigurations.RS1)
    q = np.concatenate([q_robot, q_rs1])
    
    # Visualize different poses
    print("\n1. Displaying initial configuration...")
    planner.visualize(q)
    time.sleep(2)
    
    print("2. Moving robot joints...")
    q_modified = q.copy()
    q_modified[0] += 0.5  # Move first joint
    q_modified[1] -= 0.3  # Move second joint
    planner.visualize(q_modified)
    time.sleep(2)
    
    print("3. Moving object...")
    q_modified2 = q.copy()
    robot_dof = len(q_robot)
    q_modified2[robot_dof] += 0.3  # Move object in x
    q_modified2[robot_dof + 1] += 0.2  # Move object in y
    planner.visualize(q_modified2)
    time.sleep(2)
    
    print("✓ Static visualization demo complete!")
    
    return planner


def demo_animated_visualization(backend="corba"):
    """
    Demonstrate animated configuration sequence.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
    """
    print(f"\n{'=' * 70}")
    print(f"Animated Visualization Demo ({backend.upper()} backend)")
    print('=' * 70)
    
    # Create planner
    planner = ManipulationPlanner(backend=backend)
    
    # Load models
    pkg = "package://spacelab_mock_hardware/description"
    try:
        planner.load_robot(
            "spacelab-robots",
            f"{pkg}/urdf/allRobots_spacelab_robot.urdf",
            f"{pkg}/srdf/allRobots_spacelab_robot.srdf"
        )
        
        planner.load_environment("ground_demo", f"{pkg}/urdf/ground_demo.urdf")
        
    except Exception as e:
        print(f"⚠ Model loading failed: {e}")
        return None
    
    # Create start and goal configurations
    q_robot = np.array(
        InitialConfigurations.UR10 + 
        InitialConfigurations.VISPA2 + 
        InitialConfigurations.VISPA
    )
    
    q_start = q_robot.copy()
    q_goal = q_robot.copy()
    
    # Modify goal: rotate several joints
    q_goal[0] = 1.0
    q_goal[2] = -0.8
    q_goal[4] = 1.2
    
    # Interpolate between start and goal
    print("\n1. Interpolating between configurations...")
    configs = interpolate_configs(q_start, q_goal, num_steps=20)
    
    print("2. Animating motion...")
    for i, q in enumerate(configs):
        print(f"   Frame {i+1}/{len(configs)}", end='\r')
        planner.visualize(q)
        time.sleep(0.1)
    
    print("\n✓ Animated visualization demo complete!")
    
    return planner


def demo_path_visualization(backend="corba"):
    """
    Demonstrate path visualization after planning.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
    """
    print(f"\n{'=' * 70}")
    print(f"Path Visualization Demo ({backend.upper()} backend)")
    print('=' * 70)
    
    # Create planner
    planner = ManipulationPlanner(backend=backend)
    
    # Load models
    pkg = "package://spacelab_mock_hardware/description"
    try:
        planner.load_robot(
            "spacelab-robots",
            f"{pkg}/urdf/allRobots_spacelab_robot.urdf",
            f"{pkg}/srdf/allRobots_spacelab_robot.srdf"
        )
        
        planner.load_environment("ground_demo", f"{pkg}/urdf/ground_demo.urdf")
        planner.load_object("RS1", f"{pkg}/urdf/RS.urdf", "freeflyer")
        
    except Exception as e:
        print(f"⚠ Model loading failed: {e}")
        return None
    
    # Build configurations
    q_robot = np.array(
        InitialConfigurations.UR10 + 
        InitialConfigurations.VISPA2 + 
        InitialConfigurations.VISPA
    )
    q_rs1 = xyzrpy_to_xyzquat(InitialConfigurations.RS1)
    q_init = np.concatenate([q_robot, q_rs1])
    
    # Create goal (move object)
    q_goal = q_init.copy()
    robot_dof = len(q_robot)
    q_goal[robot_dof] += 0.5  # Move object 0.5m in x
    
    # Set configurations
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    print("\n1. Displaying initial configuration...")
    planner.visualize(q_init)
    time.sleep(2)
    
    print("2. Displaying goal configuration...")
    planner.visualize(q_goal)
    time.sleep(2)
    
    # Note: Actual planning would require constraint graph setup
    print("\n3. Planning would happen here...")
    print("   (Requires constraint graph setup)")
    
    # If planning was successful, we could play the path:
    # success = planner.solve()
    # if success:
    #     print("4. Playing solution path...")
    #     planner.play_path(0)
    
    print("\n✓ Path visualization demo complete!")
    
    return planner


def demo_multiple_views(backend="corba"):
    """
    Demonstrate viewing the same scene from different angles.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
    """
    print(f"\n{'=' * 70}")
    print(f"Multiple Views Demo ({backend.upper()} backend)")
    print('=' * 70)
    
    # Create planner
    planner = ManipulationPlanner(backend=backend)
    
    # Load models
    pkg = "package://spacelab_mock_hardware/description"
    try:
        planner.load_robot(
            "spacelab-robots",
            f"{pkg}/urdf/allRobots_spacelab_robot.urdf",
            f"{pkg}/srdf/allRobots_spacelab_robot.srdf"
        )
        
        planner.load_environment("ground_demo", f"{pkg}/urdf/ground_demo.urdf")
        planner.load_object("RS1", f"{pkg}/urdf/RS.urdf", "freeflyer")
        
    except Exception as e:
        print(f"⚠ Model loading failed: {e}")
        return None
    
    # Build configuration
    q_robot = np.array(
        InitialConfigurations.UR10 + 
        InitialConfigurations.VISPA2 + 
        InitialConfigurations.VISPA
    )
    q_rs1 = xyzrpy_to_xyzquat(InitialConfigurations.RS1)
    q = np.concatenate([q_robot, q_rs1])
    
    print("\n1. Front view...")
    planner.visualize(q)
    time.sleep(2)
    
    print("2. Side view...")
    # Note: Changing camera position requires backend-specific calls
    # This is a simplified demonstration
    planner.visualize(q)
    time.sleep(2)
    
    print("3. Top view...")
    planner.visualize(q)
    time.sleep(2)
    
    print("\n✓ Multiple views demo complete!")
    print("   (Camera control requires backend-specific API)")
    
    return planner


def main():
    """Main execution function."""
    print("=" * 70)
    print("Agimus Spacelab - Visualization Examples")
    print("=" * 70)
    
    # Check available backends
    backends = get_available_backends()
    print(f"\nAvailable backends: {backends}")
    
    if not backends:
        print("\n⚠ No backends available!")
        print("  Please install hpp-manipulation-corba or hpp-python")
        return
    
    # Choose backend
    backend = backends[0]
    print(f"\nUsing backend: {backend}")
    
    # Run demos
    demos = [
        ("Static Visualization", demo_static_visualization),
        ("Animated Visualization", demo_animated_visualization),
        ("Path Visualization", demo_path_visualization),
        ("Multiple Views", demo_multiple_views),
    ]
    
    print("\nAvailable demos:")
    for i, (name, _) in enumerate(demos, 1):
        print(f"  {i}. {name}")
    
    print("\nRunning all demos...")
    print("(Press Ctrl+C to stop)\n")
    
    try:
        for name, demo_func in demos:
            planner = demo_func(backend)
            if planner is None:
                print(f"⚠ Skipping {name} due to errors")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    
    print("\n" + "=" * 70)
    print("Visualization examples completed!")
    print("=" * 70)
    
    print("\nTips:")
    print("  - Use planner.visualize(q) to display any configuration")
    print("  - Use planner.play_path(i) to animate a planned path")
    print("  - Interpolate between configs for smooth animations")
    print("  - Check backend-specific docs for camera control")


if __name__ == "__main__":
    import sys
    
    # Parse arguments
    if "--help" in sys.argv or "-h" in sys.argv:
        print("Usage: python3 visualization_example.py [options]")
        print("\nOptions:")
        print("  --backend <name>  Use specific backend (corba|pyhpp)")
        print("  --demo <number>   Run specific demo only (1-4)")
        print("  --help, -h        Show this help message")
        sys.exit(0)
    
    main()
