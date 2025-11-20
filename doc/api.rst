API Reference
=============

This section provides detailed API documentation for all modules in the
agimus_spacelab package.

Core Module
-----------

.. automodule:: agimus_spacelab.core
   :members:
   :undoc-members:
   :show-inheritance:

Base Classes
~~~~~~~~~~~~

.. autoclass:: agimus_spacelab.core.ManipulationTask
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.core.RobotConfig
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.core.ObjectConfig
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.core.PlanningResult
   :members:
   :undoc-members:


CORBA Backend
-------------

.. automodule:: agimus_spacelab.corba
   :members:
   :undoc-members:
   :show-inheritance:

CorbaManipulationPlanner
~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: agimus_spacelab.corba.CorbaManipulationPlanner
   :members:
   :undoc-members:
   :show-inheritance:


PyHPP Backend
-------------

.. automodule:: agimus_spacelab.pyhpp
   :members:
   :undoc-members:
   :show-inheritance:

PyHPPManipulationPlanner
~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: agimus_spacelab.pyhpp.PyHPPManipulationPlanner
   :members:
   :undoc-members:
   :show-inheritance:


Configuration Module
--------------------

.. automodule:: agimus_spacelab.config
   :members:
   :undoc-members:
   :show-inheritance:

Configuration Classes
~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: agimus_spacelab.config.RobotJoints
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.config.InitialConfigurations
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.config.JointBounds
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.config.ManipulationConfig
   :members:
   :undoc-members:

Rule Generation
~~~~~~~~~~~~~~~

.. automodule:: agimus_spacelab.config.rules
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.config.rules.RuleGenerator
   :members:
   :undoc-members:


Utilities Module
----------------

.. automodule:: agimus_spacelab.utils
   :members:
   :undoc-members:

Transformation Functions
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: agimus_spacelab.utils.xyzrpy_to_xyzquat

.. autofunction:: agimus_spacelab.utils.xyzrpy_to_se3

.. autofunction:: agimus_spacelab.utils.se3_to_xyzquat

.. autofunction:: agimus_spacelab.utils.xyzquat_to_se3

Configuration Utilities
~~~~~~~~~~~~~~~~~~~~~~~

.. automodule:: agimus_spacelab.utils.config_utils
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.utils.config_utils.ConfigBuilder
   :members:
   :undoc-members:

.. autoclass:: agimus_spacelab.utils.config_utils.BoundsManager
   :members:
   :undoc-members:


Planner Module
--------------

.. automodule:: agimus_spacelab.planner
   :members:
   :undoc-members:

ManipulationPlanner
~~~~~~~~~~~~~~~~~~~

.. autoclass:: agimus_spacelab.planner.ManipulationPlanner
   :members:
   :undoc-members:
   :show-inheritance:

Functions
~~~~~~~~~

.. autofunction:: agimus_spacelab.planner.check_backend
