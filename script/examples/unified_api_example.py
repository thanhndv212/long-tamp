#!/usr/bin/env python3
"""
Example: Unified API demonstration.

This script shows how to use the unified API which works
with both CORBA and PyHPP backends.
"""

import numpy as np
import sys
from pathlib import Path

# Add config directory to path for task-specific configs
config_dir = Path(__file__).parent.parent.parent / "config"
sys.path.insert(0, str(config_dir))

from agimus_spacelab import ManipulationPlanner, get_available_backends
from agimus_spacelab.utils import xyzrpy_to_xyzquat, merge_configurations
from spacelab_config import (
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)


def main(backend="pyhpp"):
    """
    Run manipulation planning example.
    
    Args:
        backend: Backend to use ('corba' or 'pyhpp')
    """
    print("=" * 70)
    print(f"Agimus Spacelab - Unified API Example")
    print(f"Backend: {backend}")
    print("=" * 70)
    
    # Check available backends
    available = get_available_backends()
    print(f"\nAvailable backends: {available}")
    
    if backend not in available:
        print(f"\nError: Backend '{backend}' not available")
        print("Please install the required dependencies:")
        if backend == "corba":
            print("  - hpp-manipulation-corba")
            print("  - hpp-gepetto-viewer")
        elif backend == "pyhpp":
            print("  - hpp-python")
        return
    
    # Create planner
    print(f"\n1. Creating planner with {backend} backend...")
    planner = ManipulationPlanner(backend=backend)
    
    # Load robot
    print("2. Loading robot...")
    # Note: Paths should be adjusted to your installation
    robot_urdf = "package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf"  # noqa: E501
    robot_srdf = "package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf"  # noqa: E501
    
    try:
        planner.load_robot("spacelab", robot_urdf, robot_srdf, "anchor")
        print("   ✓ Robot loaded")
    except Exception as e:
        print(f"   ✗ Failed to load robot: {e}")
        print("\n   Note: Make sure URDF/SRDF files are accessible")
        return
    
    # Load environment
    print("3. Loading environment...")
    env_urdf = "package://spacelab_mock_hardware/description/urdf/ground_demo.urdf"  # noqa: E501
    
    try:
        planner.load_environment("ground_demo", env_urdf)
        print("   ✓ Environment loaded")
    except Exception as e:
        print(f"   ✗ Failed to load environment: {e}")
    
    # Load objects
    print("4. Loading objects...")
    objects = [
        ("RS1", "package://spacelab_mock_hardware/description/urdf/RS.urdf"),
        ("screw_driver", "package://spacelab_mock_hardware/description/urdf/screw_driver.urdf"),  # noqa: E501
        ("frame_gripper", "package://spacelab_mock_hardware/description/urdf/frame_gripper.urdf"),  # noqa: E501
        ("cleat_gripper", "package://spacelab_mock_hardware/description/urdf/cleat_gripper.urdf"),  # noqa: E501
    ]
    
    for obj_name, obj_urdf in objects:
        try:
            planner.load_object(obj_name, obj_urdf, "freeflyer")
            print(f"   ✓ {obj_name} loaded")
        except Exception as e:
            print(f"   ✗ Failed to load {obj_name}: {e}")
    
    # Set joint bounds
    print("5. Setting joint bounds...")
    bounds = JointBounds.freeflyer_bounds()
    for obj_name, _ in objects:
        try:
            planner.set_joint_bounds(f"{obj_name}/root_joint", bounds)
        except Exception as e:
            print(f"   ✗ Failed to set bounds for {obj_name}: {e}")
    
    # Build initial configuration
    print("6. Building initial configuration...")
    config = InitialConfigurations
    
    # Robot config
    q_robot = np.array(config.UR10 + config.VISPA_BASE + config.VISPA_ARM)
    
    # Object configs (convert XYZRPY to XYZQUAT)
    q_objects = []
    for pose_xyzrpy in [
        config.RS1,
        config.SCREW_DRIVER,
        config.FRAME_GRIPPER,
        config.CLEAT_GRIPPER
    ]:
        q_objects.append(xyzrpy_to_xyzquat(pose_xyzrpy))
    
    q_init = merge_configurations(q_robot, *q_objects)
    print(f"   Configuration size: {len(q_init)}")
    
    # Set initial config
    try:
        planner.set_initial_config(q_init)
        print("   ✓ Initial configuration set")
    except Exception as e:
        print(f"   ✗ Failed to set initial config: {e}")
    
    print("\n" + "=" * 70)
    print("Setup Complete!")
    print("=" * 70)
    print("\nNext steps:")
    print("- Define goal configuration")
    print("- Create constraint graph")
    print("- Call planner.solve()")
    print("- Visualize with planner.visualize() and planner.play_path()")
    print("\nExample:")
    print("  q_goal = q_init.copy()")
    print("  q_goal[7] += 0.5  # Move first object")
    print("  planner.add_goal_config(q_goal)")
    print("  success = planner.solve()")
    

if __name__ == "__main__":
    import sys
    
    # Get backend from command line or use default
    backend = sys.argv[1] if len(sys.argv) > 1 else "pyhpp"
    
    main(backend)
