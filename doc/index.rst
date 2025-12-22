Agimus Spacelab Documentation
==============================

Welcome to the documentation for **Agimus Spacelab**, a generalized manipulation planning framework.

Overview
--------

Agimus Spacelab provides a unified interface for manipulation planning that supports both:

- **CORBA Server** backend (hpp-manipulation-corba)
- **PyHPP** backend (hpp-python direct bindings)

Features
--------

- **Dual backend support**: Switch between CORBA and PyHPP seamlessly
- **Unified API**: Same code works with both backends
- **Modular design**: Reusable components for various manipulation tasks
- **Flexible installation**: pip or CMake
- **Optional dependencies**: Install only what you need
- **Well-tested**: Unit tests for reliability
- **Docker-ready**: Complete development environment

User guide
----------

.. toctree::
    :maxdepth: 2
    :caption: User Guide

    installation
    usage
    examples

API documentation
-----------------

.. toctree::
    :maxdepth: 2
    :caption: API Documentation

    api
    pyhpp_api_documentation_md
    corba_api_documentation_md

Additional
----------

.. toctree::
    :maxdepth: 1
    :caption: Additional

    comparison_corba_vs_pyhpp_md
    improvement_plan_md
    hpp_manipulation_complete_motion_plan_md

Quick example
-------------

.. code-block:: python

    from agimus_spacelab import create_planner

    # Create planner (choose backend)
    planner = create_planner(backend="pyhpp")

    # Load robot and environment
    planner.load_robot("spacelab", robot_urdf, robot_srdf)
    planner.load_environment("ground", env_urdf)

    # Setup and solve
    planner.set_initial_config(q_init)
    planner.add_goal_config(q_goal)
    success = planner.solve()

    # Visualize
    if success:
         planner.visualize()
         planner.play_path()

Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

