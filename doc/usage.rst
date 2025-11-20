Usage Guide
===========

This guide covers common usage patterns and workflows for agimus_spacelab.

Getting Started
---------------

Backend Selection
~~~~~~~~~~~~~~~~~

The agimus_spacelab package supports two backends: CORBA and PyHPP.
You can check which backends are available:

.. code-block:: python

    from agimus_spacelab import get_available_backends
    
    backends = get_available_backends()
    print(f"Available backends: {backends}")

Creating a Planner
~~~~~~~~~~~~~~~~~~

Use the unified API to create a planner with your preferred backend:

.. code-block:: python

    from agimus_spacelab import ManipulationPlanner
    
    # Use CORBA backend
    planner = ManipulationPlanner(backend="corba")
    
    # Or use PyHPP backend
    planner = ManipulationPlanner(backend="pyhpp")
    
    # Auto-detect (uses first available)
    planner = ManipulationPlanner()


Loading Models
--------------

Loading a Robot
~~~~~~~~~~~~~~~

.. code-block:: python

    planner.load_robot(
        name="my_robot",
        urdf_path="package://my_robot/urdf/robot.urdf",
        srdf_path="package://my_robot/srdf/robot.srdf"
    )

Loading Environment
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    planner.load_environment(
        name="scene",
        urdf_path="package://my_scene/urdf/scene.urdf"
    )

Loading Objects
~~~~~~~~~~~~~~~

.. code-block:: python

    planner.load_object(
        name="box",
        urdf_path="package://objects/urdf/box.urdf",
        root_joint_type="freeflyer"
    )


Constraint Graphs
-----------------

Creating a Constraint Graph
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab.config import ManipulationConfig
    
    graph = planner.create_constraint_graph(
        name="manipulation_graph",
        grippers=list(ManipulationConfig.GRIPPERS.values()),
        objects=ManipulationConfig.OBJECTS,
        rules="auto"
    )

Custom Rules
~~~~~~~~~~~~

You can define custom rules for the constraint graph:

.. code-block:: python

    from agimus_spacelab.config import RuleGenerator, ManipulationConfig
    
    # Auto-generate from valid pairs
    rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
    
    # Sequential rules
    task_sequence = [
        ("gripper1", "object1/handle1"),
        ("gripper2", "object2/handle2"),
    ]
    rules = RuleGenerator.generate_sequential_rules(
        ManipulationConfig,
        task_sequence
    )
    
    # Priority-based rules
    priority_map = {
        ("gripper1", "object1/handle1"): 10,
        ("gripper2", "object2/handle2"): 5,
    }
    rules = RuleGenerator.generate_priority_rules(
        ManipulationConfig,
        priority_map
    )


Motion Planning
---------------

Setting Configurations
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import numpy as np
    
    # Set initial configuration
    q_init = np.array([...])
    planner.set_initial_config(q_init)
    
    # Add goal configuration
    q_goal = np.array([...])
    planner.add_goal_config(q_goal)

Solving
~~~~~~~

.. code-block:: python

    # Solve the planning problem
    success = planner.solve(max_iterations=10000)
    
    if success:
        print("Solution found!")
        path = planner.get_path()
    else:
        print("No solution found")


Visualization
-------------

Displaying Configurations
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # Visualize initial configuration
    planner.visualize(q_init)
    
    # Visualize goal configuration
    planner.visualize(q_goal)

Playing Paths
~~~~~~~~~~~~~

.. code-block:: python

    # Play the computed path
    if success:
        planner.play_path(0)


Configuration Management
------------------------

Using Built-in Configurations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The package provides pre-defined configurations for the Spacelab project:

.. code-block:: python

    from agimus_spacelab.config import (
        InitialConfigurations,
        RobotJoints,
        JointBounds
    )
    
    # Access robot configurations
    ur10_config = InitialConfigurations.UR10
    vispa_config = InitialConfigurations.VISPA
    
    # Access joint names
    ur10_joints = RobotJoints.UR10
    vispa_joints = RobotJoints.VISPA_ARM
    
    # Get bounds
    freeflyer_bounds = JointBounds.freeflyer()

Building Custom Configurations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab.utils.config_utils import ConfigBuilder
    
    builder = ConfigBuilder()
    
    # Build robot configuration
    q_robot = builder.build_robot_config(
        ur10_joints=ur10_config,
        vispa_base=vispa_base_config,
        vispa_arm=vispa_arm_config
    )


Transformation Utilities
------------------------

The package provides utilities for transformation conversions:

.. code-block:: python

    from agimus_spacelab.utils import (
        xyzrpy_to_xyzquat,
        xyzrpy_to_se3,
        se3_to_xyzquat,
        xyzquat_to_se3
    )
    
    # Convert XYZRPY to XYZQUAT
    xyzrpy = [0.5, 0.3, 0.2, 0.0, 0.0, 1.57]
    xyzquat = xyzrpy_to_xyzquat(xyzrpy)
    
    # Convert to SE3
    se3 = xyzrpy_to_se3(xyzrpy)
    
    # Convert SE3 to XYZQUAT
    xyzquat = se3_to_xyzquat(se3)


Advanced Usage
--------------

Direct Backend Access
~~~~~~~~~~~~~~~~~~~~~

For advanced use cases, you can access the backend directly:

.. code-block:: python

    # CORBA backend
    planner = ManipulationPlanner(backend="corba")
    robot = planner.get_robot()
    ps = planner.get_problem_solver()
    
    # PyHPP backend
    planner = ManipulationPlanner(backend="pyhpp")
    device = planner.get_robot()
    problem = planner.get_problem()
    graph = planner.get_graph()

Custom Backend Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can extend the backends for custom needs:

.. code-block:: python

    from agimus_spacelab.corba import CorbaManipulationPlanner
    
    class MyCustomPlanner(CorbaManipulationPlanner):
        def custom_method(self):
            # Your custom implementation
            pass


Error Handling
--------------

The package raises clear errors when backends are unavailable:

.. code-block:: python

    try:
        planner = ManipulationPlanner(backend="corba")
    except ImportError as e:
        print(f"CORBA backend not available: {e}")
        # Fall back to PyHPP
        planner = ManipulationPlanner(backend="pyhpp")


Best Practices
--------------

1. **Backend Selection**: Choose CORBA for ROS integration, PyHPP for Python-native workflows
2. **Configuration Management**: Use the provided configuration classes for consistency
3. **Rule Generation**: Use RuleGenerator for automatic constraint graph rules
4. **Error Handling**: Always check for backend availability
5. **Visualization**: Visualize initial and goal configs before solving
6. **Testing**: Use the provided test infrastructure to validate your setup
