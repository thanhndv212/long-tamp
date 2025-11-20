#!/usr/bin/env python3
"""
Assembly task example.

This example demonstrates a complex assembly task with multiple objects
and sequential manipulation steps.
"""

import numpy as np
from agimus_spacelab import ManipulationPlanner
from agimus_spacelab.config import RuleGenerator
from agimus_spacelab.utils import xyzrpy_to_xyzquat


class AssemblyConfig:
    """Configuration for assembly task."""
    
    GRIPPERS = {
        "robot_gripper": "assembly_robot/gripper",
    }
    
    OBJECTS = {
        "base_plate": {
            "handles": ["base_plate/handle_top"],
            "contact_surfaces": ["base_plate/surface"],
        },
        "component_a": {
            "handles": ["component_a/handle"],
            "contact_surfaces": ["component_a/bottom"],
        },
        "component_b": {
            "handles": ["component_b/handle"],
            "contact_surfaces": ["component_b/bottom"],
        },
        "fastener": {
            "handles": ["fastener/handle"],
            "contact_surfaces": [],
        },
    }
    
    VALID_PAIRS = {
        "robot_gripper": [
            "base_plate/handle_top",
            "component_a/handle",
            "component_b/handle",
            "fastener/handle",
        ],
    }


def main(backend="corba", visualize=True):
    """
    Assembly task example.
    
    Args:
        backend: Backend to use ("corba" or "pyhpp")
        visualize: Whether to visualize the scene
    """
    print("=" * 70)
    print("Assembly Task Example")
    print("=" * 70)
    
    # Create planner
    print(f"\n1. Creating planner (backend: {backend})...")
    planner = ManipulationPlanner(backend=backend)
    
    # Load robot
    print("2. Loading assembly robot...")
    pkg = "package://assembly_robot/description"
    planner.load_robot(
        name="assembly_robot",
        urdf_path=f"{pkg}/urdf/robot.urdf",
        srdf_path=f"{pkg}/srdf/robot.srdf"
    )
    
    # Load environment
    print("3. Loading assembly station...")
    planner.load_environment(
        name="station",
        urdf_path="package://station/urdf/assembly_station.urdf"
    )
    
    # Load parts
    print("4. Loading assembly parts...")
    parts = [
        "base_plate",
        "component_a",
        "component_b",
        "fastener",
    ]
    
    for part in parts:
        planner.load_object(
            name=part,
            urdf_path=f"package://parts/urdf/{part}.urdf",
            root_joint_type="freeflyer"
        )
    
    # Define configurations
    print("5. Defining configurations...")
    
    # Robot configuration
    q_robot = np.array([0.0, -1.2, 1.5, 0.0, 1.2, 0.0])
    
    # Part poses (all on assembly table initially)
    base_plate_pose = xyzrpy_to_xyzquat([0.3, 0.0, 0.8, 0.0, 0.0, 0.0])
    component_a_pose = xyzrpy_to_xyzquat([0.5, 0.2, 0.8, 0.0, 0.0, 0.0])
    component_b_pose = xyzrpy_to_xyzquat([0.5, -0.2, 0.8, 0.0, 0.0, 0.0])
    fastener_pose = xyzrpy_to_xyzquat([0.7, 0.0, 0.8, 0.0, 0.0, 0.0])
    
    # Initial configuration: all parts separate
    q_init = np.concatenate([
        q_robot,
        base_plate_pose,
        component_a_pose,
        component_b_pose,
        fastener_pose,
    ])
    
    # Goal configuration: parts assembled
    base_plate_goal = xyzrpy_to_xyzquat([0.3, 0.0, 0.8, 0.0, 0.0, 0.0])
    component_a_goal = xyzrpy_to_xyzquat([0.3, 0.0, 0.85, 0.0, 0.0, 0.0])  # On base
    component_b_goal = xyzrpy_to_xyzquat([0.3, 0.0, 0.90, 0.0, 0.0, 0.0])  # On A
    fastener_goal = xyzrpy_to_xyzquat([0.3, 0.0, 0.95, 0.0, 0.0, 0.0])  # On top
    
    q_goal = np.concatenate([
        q_robot,
        base_plate_goal,
        component_a_goal,
        component_b_goal,
        fastener_goal,
    ])
    
    print(f"   Total DOF: {len(q_init)}")
    print(f"   - Robot: {len(q_robot)} DOF")
    print(f"   - Parts: {len(q_init) - len(q_robot)} DOF")
    
    # Set configurations
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    # Create constraint graph with sequential assembly rules
    print("6. Creating constraint graph for assembly...")
    
    # Define assembly sequence
    assembly_sequence = [
        ("robot_gripper", "base_plate/handle_top"),     # Pick base
        ("robot_gripper", "component_a/handle"),        # Pick component A
        ("robot_gripper", "component_b/handle"),        # Pick component B
        ("robot_gripper", "fastener/handle"),           # Pick fastener
    ]
    
    rules = RuleGenerator.generate_sequential_rules(
        AssemblyConfig,
        assembly_sequence
    )
    
    graph = planner.create_constraint_graph(
        name="assembly_graph",
        grippers=list(AssemblyConfig.GRIPPERS.values()),
        objects=AssemblyConfig.OBJECTS,
        rules=rules
    )
    
    # Visualize
    if visualize:
        print("\n7. Visualizing scene...")
        planner.visualize(q_init)
    
    print("\n" + "=" * 70)
    print("Assembly Task Setup Complete!")
    print("=" * 70)
    
    print("\nTask: Assemble 4 parts in sequence")
    print("  1. Position base plate")
    print("  2. Place component A on base")
    print("  3. Place component B on A")
    print("  4. Add fastener on top")
    
    print("\nConstraint graph enforces sequential assembly order")
    
    print("\nTo solve:")
    print("  success = planner.solve(max_iterations=15000)")
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
