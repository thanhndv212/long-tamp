Examples
========

This section provides complete examples demonstrating different use cases
of the agimus_spacelab package.

Spacelab Examples
-----------------

CORBA Backend Example
~~~~~~~~~~~~~~~~~~~~~

Complete example using the CORBA backend for Spacelab manipulation:

.. literalinclude:: ../script/examples/spacelab_corba_example.py
   :language: python
   :linenos:

To run:

.. code-block:: bash

    cd script/examples
    python3 spacelab_corba_example.py

PyHPP Backend Example
~~~~~~~~~~~~~~~~~~~~~

Complete example using the PyHPP backend for Spacelab manipulation:

.. literalinclude:: ../script/examples/spacelab_pyhpp_example.py
   :language: python
   :linenos:

To run:

.. code-block:: bash

    cd script/examples
    python3 spacelab_pyhpp_example.py

Unified API Example
~~~~~~~~~~~~~~~~~~~

Backend-agnostic example using the unified API:

.. literalinclude:: ../script/examples/unified_api_example.py
   :language: python
   :linenos:

Basic Usage Examples
--------------------

Simple Pick and Place
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab import ManipulationPlanner
    from agimus_spacelab.config import ManipulationConfig
    import numpy as np
    
    # Create planner
    planner = ManipulationPlanner(backend="corba")
    
    # Load robot
    planner.load_robot(
        name="robot",
        urdf_path="package://robot/urdf/robot.urdf"
    )
    
    # Load object
    planner.load_object(
        name="box",
        urdf_path="package://objects/urdf/box.urdf",
        root_joint_type="freeflyer"
    )
    
    # Set configurations
    q_init = np.array([...])  # Initial configuration
    q_goal = np.array([...])  # Goal configuration
    
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    
    # Create constraint graph
    graph = planner.create_constraint_graph(
        name="pick_place",
        grippers=["robot/gripper"],
        objects={"box": {"handles": ["box/handle"], "contact_surfaces": []}},
        rules="all"
    )
    
    # Solve
    success = planner.solve()
    
    if success:
        planner.visualize()
        planner.play_path(0)

Multi-Robot Coordination
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab import ManipulationPlanner
    from agimus_spacelab.config import RuleGenerator, ManipulationConfig
    
    planner = ManipulationPlanner(backend="pyhpp")
    
    # Load robots
    planner.load_robot("robot1", "package://robot1/urdf/robot.urdf")
    planner.load_robot("robot2", "package://robot2/urdf/robot.urdf")
    
    # Load shared object
    planner.load_object("large_object", "package://obj/urdf/large.urdf")
    
    # Define sequential task
    task_sequence = [
        ("robot1/gripper", "large_object/handle1"),
        ("robot2/gripper", "large_object/handle2"),
    ]
    
    rules = RuleGenerator.generate_sequential_rules(
        ManipulationConfig,
        task_sequence
    )
    
    # Create graph with sequential rules
    graph = planner.create_constraint_graph(
        name="dual_arm",
        grippers=["robot1/gripper", "robot2/gripper"],
        objects={"large_object": {
            "handles": ["large_object/handle1", "large_object/handle2"],
            "contact_surfaces": []
        }},
        rules=rules
    )

Custom Configuration
~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab.utils import xyzrpy_to_xyzquat
    import numpy as np
    
    # Define custom robot configuration
    robot_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    
    # Define object poses
    object_poses = {
        "box1": [0.5, 0.3, 0.0, 0.0, 0.0, 0.0],  # XYZRPY
        "box2": [0.8, 0.2, 0.0, 0.0, 0.0, 1.57],
    }
    
    # Convert to XYZQUAT for freeflyer joints
    q_robot = np.array(robot_joints)
    q_objects = []
    
    for pose_xyzrpy in object_poses.values():
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_objects.extend(pose_xyzquat.tolist())
    
    q_init = np.concatenate([q_robot, q_objects])

Visualization Example
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab import ManipulationPlanner
    import numpy as np
    import time
    
    planner = ManipulationPlanner(backend="corba")
    
    # ... setup robot and objects ...
    
    # Visualize different configurations
    configs = [q_init, q_mid, q_goal]
    
    for i, q in enumerate(configs):
        print(f"Showing config {i+1}/3")
        planner.visualize(q)
        time.sleep(2)
    
    # Solve and animate
    if planner.solve():
        print("Playing solution path...")
        planner.play_path(0)

Configuration Builder Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab.config import (
        InitialConfigurations,
        RobotJoints
    )
    from agimus_spacelab.utils.config_utils import ConfigBuilder
    import numpy as np
    
    builder = ConfigBuilder()
    
    # Build configuration from components
    q = builder.build_from_components([
        InitialConfigurations.UR10,
        InitialConfigurations.VISPA_BASE,
        InitialConfigurations.VISPA,
    ])
    
    # Add object configurations
    object_configs = [
        InitialConfigurations.RS1,
        InitialConfigurations.SCREW_DRIVER,
    ]
    
    for obj_xyzrpy in object_configs:
        obj_xyzquat = builder.xyzrpy_to_xyzquat(obj_xyzrpy)
        q = np.concatenate([q, obj_xyzquat])

Rule Generation Examples
------------------------

Auto Rules from Valid Pairs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from agimus_spacelab.config import RuleGenerator, ManipulationConfig
    
    # Generate rules automatically from VALID_PAIRS
    rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
    
    # Print summary
    RuleGenerator.print_rule_summary(rules, ManipulationConfig)

Priority-Based Rules
~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # Define priority map (higher = more preferred)
    priority_map = {
        ("ur10_gripper", "frame_gripper/h_FG_tool"): 10,
        ("ur10_gripper", "screw_driver/h_SD_tool"): 8,
        ("vispa_gripper", "cleat_gripper/h_CG_tool"): 9,
        ("vispa_gripper", "RS1/h_RS_front"): 7,
    }
    
    rules = RuleGenerator.generate_priority_rules(
        ManipulationConfig,
        priority_map
    )

Sequential Task Rules
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # Define task sequence
    task_sequence = [
        ("ur10_gripper", "frame_gripper/h_FG_tool"),
        ("vispa_gripper", "cleat_gripper/h_CG_tool"),
        ("ur10_gripper", "RS1/h_RS_top"),
    ]
    
    rules = RuleGenerator.generate_sequential_rules(
        ManipulationConfig,
        task_sequence
    )

Testing Examples
----------------

Unit Testing Your Setup
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import pytest
    from agimus_spacelab import ManipulationPlanner
    
    def test_robot_loading():
        """Test that robot loads successfully."""
        planner = ManipulationPlanner(backend="corba")
        
        robot = planner.load_robot(
            name="test_robot",
            urdf_path="package://test/robot.urdf"
        )
        
        assert robot is not None
    
    def test_planning():
        """Test basic planning."""
        planner = ManipulationPlanner(backend="pyhpp")
        
        # Setup
        planner.load_robot("robot", "package://robot/urdf/robot.urdf")
        planner.set_initial_config(q_init)
        planner.add_goal_config(q_goal)
        
        # Solve
        success = planner.solve(max_iterations=1000)
        
        assert success, "Planning should find a solution"

Integration Testing
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    def test_full_workflow():
        """Test complete manipulation workflow."""
        planner = ManipulationPlanner(backend="corba")
        
        # Load models
        planner.load_robot("robot", "package://robot/urdf/robot.urdf")
        planner.load_environment("scene", "package://scene/urdf/scene.urdf")
        planner.load_object("box", "package://box/urdf/box.urdf")
        
        # Create graph
        graph = planner.create_constraint_graph(
            name="test_graph",
            grippers=["robot/gripper"],
            objects={"box": {"handles": ["box/handle"], "contact_surfaces": []}},
            rules="all"
        )
        
        assert graph is not None
        
        # Set configurations
        planner.set_initial_config(q_init)
        planner.add_goal_config(q_goal)
        
        # Solve
        success = planner.solve()
        
        if success:
            path = planner.get_path()
            assert path is not None

Running Examples
----------------

All examples can be run from the command line:

.. code-block:: bash

    # Spacelab CORBA example
    python3 script/examples/spacelab_corba_example.py
    
    # Spacelab PyHPP example
    python3 script/examples/spacelab_pyhpp_example.py
    
    # Unified API example
    python3 script/examples/unified_api_example.py
    
    # With options
    python3 script/examples/spacelab_corba_example.py --solve
    python3 script/examples/spacelab_pyhpp_example.py --solve --no-viz

See Also
--------

- :doc:`usage` - Detailed usage guide
- :doc:`api` - Complete API reference
- :doc:`installation` - Installation instructions
