# HPP-Python Bindings: Complete API Reference

**Version:** 6.1.0  
**Repository:** [humanoid-path-planner/hpp-python](https://github.com/humanoid-path-planner/hpp-python)  
**Branch:** v6.1.0  
**Documentation Date:** November 2025

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Quick Start Guide](#quick-start-guide)
- [Device API](#1-device-robot-representation)
- [Problem API](#2-problem-planning-problem-definition)
- [Graph API](#3-graph-constraint-graph-for-manipulation)
- [Constraints API](#4-constraints-geometric-constraints)
- [Planning API](#5-planning-and-path-manipulation)
- [Visualization](#6-visualization)
- [Migration from CORBA](#migration-from-corba)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `hpp-python` package provides **native Python bindings** for the HPP (Humanoid Path Planner) C++ libraries using Boost.Python. Unlike the CORBA server which uses inter-process communication, these bindings give direct in-process access to HPP's manipulation planning capabilities.

### Key Features

✅ **Direct C++ Access** - No serialization overhead, direct object manipulation  
✅ **Type Safety** - Python objects wrap C++ objects with proper type checking  
✅ **Memory Safety** - Automatic memory management via shared pointers  
✅ **Full API Coverage** - Access to all HPP manipulation planning features  
✅ **NumPy Integration** - Seamless Eigen ↔ NumPy array conversion via eigenpy  
✅ **Pythonic Interface** - Python-friendly wrappers while preserving C++ performance

### When to Use PyHPP vs CORBA

| Use Case | Recommended Approach |
|----------|---------------------|
| Interactive development/research | **PyHPP Bindings** ⭐ |
| Performance-critical applications | **PyHPP Bindings** ⭐ |
| Custom algorithms in Python | **PyHPP Bindings** ⭐ |
| Production multi-process systems | CORBA Server |
| Multi-language integration | CORBA Server |
| Standard manipulation planning | Either (similar APIs) |

---

## Architecture

### Module Structure

```
pyhpp/
├── __init__.py                     # Initializes eigenpy for NumPy conversion
├── pinocchio/                      # Robot kinematics and dynamics
│   ├── Device                      # Robot device wrapper
│   └── urdf                        # URDF loading utilities
├── core/                           # Core planning algorithms
│   ├── ProblemSolver              # High-level problem solver
│   ├── BiRRTPlanner               # Bidirectional RRT
│   ├── DiffusingPlanner           # Diffusing tree planner
│   └── ConfigurationShooter       # Random configuration sampling
├── constraints/                    # Constraint definitions
│   ├── Transformation             # Absolute pose constraints
│   ├── RelativeTransformation     # Relative pose constraints
│   ├── Position                   # Position-only constraints
│   ├── Orientation                # Orientation-only constraints
│   ├── Implicit                   # Implicit constraint wrapper
│   ├── LockedJoint                # Fixed joint values
│   └── ComparisonType             # Constraint comparison operators
└── manipulation/                   # Manipulation-specific features
    ├── Device                      # Device wrapper (re-exported)
    ├── urdf                        # URDF utilities (re-exported)
    ├── Problem                     # Manipulation problem definition
    ├── Graph                       # Constraint graph
    ├── ManipulationPlanner         # High-level manipulation planner
    ├── TransitionPlanner           # Transition-specific planner
    ├── ProgressiveProjector  # Path projection factories
    └── constraint_graph_factory.py # Python helper for graph generation
```

### Binding Technology

HPP-Python uses **Boost.Python** to expose C++ classes to Python. Each C++ component is wrapped in a lightweight Python object that:

1. **Holds a shared pointer** to the C++ object (`std::shared_ptr`)
2. **Exposes methods** via Boost.Python's `.def()` mechanism
3. **Handles type conversion** between Python and C++ types automatically
4. **Manages memory** through Python's reference counting

**Example binding code:**
```cpp
BOOST_PYTHON_MODULE(bindings) {
    INIT_PYHPP_MODULE;
    boost::python::import("pyhpp.pinocchio");
    boost::python::import("pyhpp.constraints");
    
    pyhpp::manipulation::exposeDevice();
    pyhpp::manipulation::exposeProblem();
    pyhpp::manipulation::exposeGraph();
}
```

### Key Design Patterns

#### 1. Wrapper Classes
Each C++ class has a Python wrapper holding a shared pointer:

```cpp
class PyWDevice {
    hpp::manipulation::DevicePtr_t obj;  // Shared pointer to C++ Device
    // Python-accessible methods...
};

class PyWGraph {
    hpp::manipulation::graph::GraphPtr_t obj;  // Shared pointer to C++ Graph
    PyWDevicePtr_t robot;                      // Associated device
    PyWProblemPtr_t problem;                   // Associated problem
    // Python-accessible methods...
};
```

```python
# Python wrapper holds C++ object
class PyWDevice:
    hpp::manipulation::DevicePtr_t obj  # C++ shared pointer
    
class PyWGraph:
    hpp::manipulation::graph::GraphPtr_t obj
    PyWDevicePtr_t robot
    PyWProblemPtr_t problem
```

#### 2. Constructors vs. `.create(...)`

In `pyhpp`, construction style is **per-class**: some objects use a Python constructor, while others use a static `.create(...)`.

Also note a third (common) pattern: **the Python constructor exists, but it is implemented as** `make_constructor(&Type::create)` in Boost.Python. In that case you still write `Type(...)` in Python, even though the underlying C++ factory is `Type::create`.

The lists below reflect the current bindings under [hpp/src/hpp-python/src/pyhpp](hpp/src/hpp-python/src/pyhpp).

**Call as constructor in Python (internally calls `::create`)**

- `pyhpp.constraints.Implicit(...)`
- `pyhpp.core.WeighedDistance(...)`
- `pyhpp.core.Roadmap(...)`
- `pyhpp.core.RandomShortcut(...)`
- `pyhpp.core.SimpleShortcut(...)`
- `pyhpp.core.PartialShortcut(...)`
- `pyhpp.core.SimpleTimeParameterization(...)`
- `pyhpp.core.RSTimeParameterization(...)`
- `pyhpp.manipulation.RandomShortcut(...)`
- `pyhpp.manipulation.EnforceTransitionSemantic(...)`

**Call as static factory in Python: `Type.create(...)`**

- `pyhpp.core.ProblemSolver.create()`
- `pyhpp.core.Parameter.create(obj)` and `pyhpp.core.Parameter.create_bool(bool)`
- `pyhpp.core.StraightPath.create(...)`
- `pyhpp.core.path.Vector.create(...)`
- `pyhpp.core.problem_target.GoalConfigurations.create(...)`
- `pyhpp.constraints.RelativeCom.create(...)`
- `pyhpp.constraints.Explicit.create(...)`
- `pyhpp.manipulation.Handle.create(...)`
- `pyhpp.pinocchio.Gripper.create(...)`

**Module-level factories (not `Type.create`, but still create-based helpers)**

- `pyhpp.manipulation.GraphRandomShortcut(problem)` / `pyhpp.manipulation.GraphPartialShortcut(problem)`
- `pyhpp.core.create(duration, ndof)` (creates a `PathWrap` instance)

```python
# Common constructors
f = Transformation(name, robot, joint_id, frame2, frame1, mask)
c = Implicit(f, comparisonTypes, mask)
lj = LockedJoint(robot, "joint_name", q_joint)

# Common static factories
rc = RelativeCom.create("com", robot, joint, reference, mask)
ex = Explicit.create(configSpace, f, inArg, outArg, inDer, outDer, comp)
```

#### 3. Result Objects
Operations that can fail return result objects:

```python
class ConstraintResult:
    success: bool            # Whether operation succeeded
    configuration: np.ndarray  # Resulting configuration
    error: float             # Constraint error magnitude
```

---

## Quick Start Guide

### Minimal Working Example

```python
#!/usr/bin/env python3
"""Minimal HPP-Python manipulation planning example."""

import numpy as np
from pinocchio import SE3, Quaternion

from pyhpp.manipulation import Device, urdf, Problem, Graph
from pyhpp.constraints import (
    Transformation,
    Implicit,
    ComparisonTypes,
    ComparisonType,
)

# 1. Create and load robot
robot = Device("ur5_robot")
urdf.loadModel(
    robot, 0, "ur5", "anchor",
    "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf",
    "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf",
    SE3.Identity()
)

# 2. Load manipulable object
urdf.loadModel(
    robot, 0, "object", "freeflyer",
    "package://hpp_environments/urdf/ur_benchmark/pokeball.urdf",
    "package://hpp_environments/srdf/ur_benchmark/pokeball.srdf",
    SE3.Identity()
)

# Set object bounds
robot.setJointBounds(
    "object/root_joint",
    [-1, 1, -1, 1, 0, 2, -1.001, 1.001, -1.001, 1.001, -1.001, 1.001, -1.001, 1.001]
)

# 3. Create problem
problem = Problem(robot)

# 4. Create constraint graph
graph = Graph("manipulation_graph", robot, problem)

# 5. Create states
free_state = graph.createState("free", False, 0)
grasp_state = graph.createState("grasp", False, 0)

# 6. Create transition
edge = graph.createTransition(free_state, grasp_state, "take", 1, free_state)

# 7. Create constraint: object must stay on table
joint_id = robot.model().getJointId("object/root_joint")
mask = [False, False, True, True, True, False]  # Fix z, roll, pitch

constraint = Transformation(
    "placement",
    robot,
    joint_id,
    SE3.Identity(),
    SE3(Quaternion(0, 0, 0, 1), np.array([0, 0, 0.1])),
    mask
)

# 8. Wrap in Implicit
cts = ComparisonTypes()
cts[:] = tuple([ComparisonType.EqualToZero] * 3)
implicit_constraint = Implicit(constraint, cts, [True, True, True])

# 9. Add to graph
graph.addNumericalConstraint(grasp_state, implicit_constraint)

# 10. Initialize graph (REQUIRED!)
graph.maxIterations(100)
graph.errorThreshold(1e-5)
graph.initialize()

# 11. Use the graph
q_init = np.zeros(robot.configSize())
success, q_proj, residual = graph.applyStateConstraints(grasp_state, q_init)

if success:
    print("✓ Projection succeeded")
    print(f"  Configuration: {np.around(q_proj, 3)}")
    print(f"  Residual: {residual:.6f}")
else:
    print(f"✗ Projection failed (residual={residual})")
```

### Execution Flow

1. **Device Creation** → Create `Device` to represent your robot
2. **URDF Loading** → Load robot and object models
3. **Problem Setup** → Create `Problem` for planning configuration
4. **Graph Creation** → Create `Graph` for constraint-based planning
5. **States & Transitions** → Define discrete modes and transitions
6. **Constraints** → Create geometric constraints using factory methods
7. **Graph Configuration** → Set parameters and initialize
8. **Planning** → Apply constraints, generate configurations, solve

Note: for constraints, construction style depends on the class: some use constructors (e.g. `Transformation`, `Implicit`, `LockedJoint`), while others use static `.create(...)`.

---

## 1. Device: Robot Representation

The `Device` class represents a robot with its geometric and kinematic properties.

### 1.1 Module Import Flexibility

**Important:** `Device` and `urdf` are available from both modules.

```python
# Option 1: Import from pyhpp.pinocchio (core planning)
from pyhpp.pinocchio import Device, urdf

# Option 2: Import from pyhpp.manipulation (manipulation planning)
from pyhpp.manipulation import Device, urdf

# Notes:
# - pyhpp.manipulation.Device is a subclass of pyhpp.pinocchio.Device
# - pyhpp.manipulation.Device adds manipulation-specific helpers (handles, grippers, ...)
```

### 1.2 Creating a Device

```python
from pyhpp.manipulation import Device

# Create empty device
robot = Device("robot_name")
```

### 1.3 Loading URDF Models

#### Method 1: Full Control with loadModel

```python
from pinocchio import SE3
import numpy as np

urdf.loadModel(
    robot,                              # Device object
    0,                                  # prefix index (usually 0)
    "model_name",                       # name for this model in scene
    "anchor",                           # root joint type
    "package://path/to/model.urdf",    # URDF file
    "package://path/to/model.srdf",    # SRDF file
    SE3.Identity()                      # initial pose
)
```

**Root Joint Types:**

| Type | Description | DOF | Use Case |
|------|-------------|-----|----------|
| `"anchor"` | Fixed base | 0 | Manipulator arms, static objects |
| `"freeflyer"` | 6-DOF floating | 7 | Mobile bases, free objects (x,y,z,qx,qy,qz,qw) |
| `"planar"` | 3-DOF planar | 3 | Planar mobile robots (x,y,θ) |

#### Method 2: Simplified API

```python
urdf.loadRobotModel(
    robot,
    "anchor",                # root joint type
    "package_directory",     # package directory
    "model_name",            # model name
    "_suffix",               # URDF suffix
    "_suffix"                # SRDF suffix
)
```

### 1.4 Device Configuration

#### Set Joint Bounds

Required for freeflyer and planar joints:

```python
# Freeflyer joint: 7 DOF (x, y, z, qx, qy, qz, qw)
robot.setJointBounds(
    "object/root_joint",
    [
        -1.0, 1.0,          # x bounds [min, max]
        -1.0, 1.0,          # y bounds
        0.0, 2.0,           # z bounds
        -1.0001, 1.0001,    # qx (quaternion x)
        -1.0001, 1.0001,    # qy (quaternion y)
        -1.0001, 1.0001,    # qz (quaternion z)
        -1.0001, 1.0001,    # qw (quaternion w)
    ]
)

# Revolute joint: 1 DOF
robot.setJointBounds("shoulder_joint", [-3.14, 3.14])
```

#### Device Properties

```python
# Get device information
name = robot.name()                    # Device name string
n_dof = robot.numberDof()              # Number of degrees of freedom
config_size = robot.configSize()       # Configuration vector size
config_space = robot.configSpace()     # Configuration space object

# Configuration access
q = robot.currentConfiguration()       # Get current config (numpy array)
ok = robot.currentConfiguration(q_new) # Set current config, returns bool

# Forward kinematics
from pyhpp.pinocchio import ComputationFlag

robot.computeForwardKinematics(ComputationFlag.JOINT_POSITION)
robot.computeFramesForwardKinematics() # Compute all frame transforms
robot.updateGeometryPlacements()       # Update collision geometry placements
```

#### Access Pinocchio Model

```python
# Get underlying Pinocchio objects
model = robot.model()                  # Pinocchio Model
data = robot.data()                    # Pinocchio Data

geom_model = robot.geomModel()         # GeometryModel (collision)
geom_data = robot.geomData()           # GeometryData (collision)

visual_model = robot.visualModel()     # GeometryModel (visual)

# Get joint and frame IDs (required for constraints)
joint_id = model.getJointId("joint_name")
frame_id = model.getFrameId("frame_name")
```

#### Kinematics Helpers

`pyhpp.pinocchio.Device` also provides a few convenience methods:

```python
# Center of mass (requires forward kinematics; will internally compute COM)
com = robot.getCenterOfMass()  # 3-vector

# Get a frame pose from current model/data (requires frames FK already computed)
robot.computeForwardKinematics(ComputationFlag.JOINT_POSITION)
robot.computeFramesForwardKinematics()

pose = robot.getJointPosition("some_frame_or_joint_name")
# pose is [x, y, z, qx, qy, qz, qw]

# Batch query: set configuration + compute FK internally
poses = robot.getJointsPosition(q, ["joint1", "joint2"])
# poses is a list of [x, y, z, qx, qy, qz, qw]

# Joint ranks in the configuration vector
ric = robot.rankInConfiguration  # dict: {"joint_name": rank_in_configuration}
```

#### Manipulation-Specific Extensions (pyhpp.manipulation.Device)

When you import `Device` from `pyhpp.manipulation`, you get additional helpers:

```python
from pyhpp.manipulation import Device
from pinocchio import SE3

robot = Device("robot_name")

# If your device contains multiple robots, set the root pose of one robot.
robot.setRobotRootPosition("robotName", SE3.Identity())

# Access SRDF-defined handles and grippers
handles = robot.handles()   # HandleMap: dict-like {name: Handle}
grippers = robot.grippers() # GripperMap: dict-like {name: Gripper}

handle = handles["some_handle"]
print(handle.name)
handle.localPosition = SE3.Identity()

# Joint utilities
names = robot.getJointNames()               # list[str]
segment = robot.getJointConfig("joint")    # list[float]

# Explicitly cast to a pinocchio device pointer when needed
pin_device = robot.asPinDevice()
```

The `Handle` type exposed in `pyhpp.manipulation` has:

- Properties: `name`, `localPosition`, `mask`, `maskComp`, `clearance`
- Methods: `createGrasp()`, `createPreGrasp()`, `createGraspComplement()`, `createGraspAndComplement()`

#### Grippers and Handles

The manipulation stack uses two related concepts:

- `pyhpp.pinocchio.Gripper`: a frame attached to a robot joint (the “tool”).
- `pyhpp.manipulation.Handle`: a frame attached to an object joint (the “graspable feature”).

In practice, you usually **load** these from SRDF/URDF via `urdf.loadModel(...)` and then query them from `pyhpp.manipulation.Device`:

```python
handles = robot.handles()    # HandleMap (dict-like): {name: Handle}
grippers = robot.grippers()  # GripperMap (dict-like): {name: Gripper}

handle = handles["object/handle"]
gripper = grippers["robot/gripper"]
```

##### `Gripper` (pyhpp.pinocchio)

Static factory (exposed by the bindings):

```python
from pyhpp.pinocchio import Gripper

gripper = Gripper.create("gripper_name", robot.asPinDevice())
```

Properties:

- `localPosition`: SE3-like transform of the grasped object in the gripper joint frame.
- `clearance`: `float` (get/set)

##### `Handle` (pyhpp.manipulation)

Static factory (exposed by the bindings):

```python
from pyhpp.manipulation import Handle
from pinocchio import SE3

# Signature:
# Handle.create(name: str, localPosition: SE3, robot: Device, joint: Joint) -> Handle
```

Properties (get/set unless noted):

- `name`: `str`
- `localPosition`: `SE3` (pose of the handle in the joint frame)
- `mask`: `list[bool]` of length 6 (grasp symmetry mask)
- `maskComp`: `list[bool]` of length 6 (complement mask)
- `clearance`: `float`

Notes:

- Updating `mask` also resets `maskComp` in the underlying C++ class.

Constraint builders (return an `Implicit` constraint object usable in graphs):

```python
# Grasp constraint (non-parameterizable)
implicit = handle.createGrasp(gripper, "")

# Complement (parameterizable, used for leaf/target constraints)
implicit_comp = handle.createGraspComplement(gripper, "")

# Full relative transformation (grasp + complement)
implicit_both = handle.createGraspAndComplement(gripper, "")

# Pregrasp: shift along handle x-axis
implicit_pre = handle.createPreGrasp(gripper, 0.02, "")
```

### 1.5 Random Configuration Sampling

⚠️ **Critical:** Use `problem.configurationShooter()`, not `robot.shootRandomConfig()`!

```python
# ✅ CORRECT: Get configuration shooter from problem
shooter = problem.configurationShooter()
q_rand = shooter.shoot()

# Shoot multiple configurations
for i in range(100):
    q = shooter.shoot()
    # Use configuration...

# ❌ INCORRECT: This method doesn't exist on Device!
# q_rand = robot.shootRandomConfig()  # AttributeError!
```

---

## 2. Problem: Planning Problem Definition

The `Problem` class encapsulates the planning problem: initial and goal configurations, components (distance, steering method, validations), and a constraint set used for projection.

There are two Python entry points:

- `pyhpp.core.Problem`: wraps `hpp::core::Problem`.
- `pyhpp.manipulation.Problem`: derives from `pyhpp.core.Problem` and wraps `hpp::manipulation::Problem` (adds graph integration).

### 2.1 Creating a Problem

```python
from pyhpp.manipulation import Problem

# Create problem for a device
problem = Problem(robot)
```

Notes:

- `pyhpp.manipulation.Problem(robot)` expects a manipulation-capable device (typically `pyhpp.manipulation.Device`, or a device created/loaded via `pyhpp.manipulation.urdf`).
- If you do not need a constraint graph, you can also use `pyhpp.core.Problem(robot)`.
```

### 2.2 Setting Initial and Goal Configurations

```python
import numpy as np

# Define configurations (size must match robot.configSize())
q_init = np.zeros(robot.configSize())
q_goal = np.array([...])  # Desired goal configuration

# Set in problem
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)

# You can add multiple goal configurations
problem.addGoalConfig(q_goal2)
problem.addGoalConfig(q_goal3)
```

### 2.3 Accessing and Configuring Problem Components

The bindings expose these components as **getter/setter methods** (call with no argument to get, or with one argument to set):

- `distance()` / `distance(d)`
- `steeringMethod()` / `steeringMethod(sm)`
- `pathValidation()` / `pathValidation(pv)`
- `pathProjector()` / `pathProjector(pp)`
- `configurationShooter()` / `configurationShooter(cs)`
- `configValidation()` / `configValidation(cv)`
- `target()` / `target(t)`

They also expose `robot()` to retrieve the underlying device.

```python
# Get problem components
distance = problem.distance()               # Distance metric
steering = problem.steeringMethod()         # Steering method
validator = problem.pathValidation()        # Path validator

# Configuration sampling
shooter = problem.configurationShooter()    # Configuration shooter
q_random = shooter.shoot()                  # Sample random config
```

Config validation helpers:

```python
# Add a built-in config validation by name
problem.addConfigValidation("CollisionValidation")
problem.addConfigValidation("JointBoundValidation")

# Or clear all config validations
problem.clearConfigValidations()
```

Parameter API:

```python
# Parameters are stored in the underlying C++ problem.
# The binding supports float/int directly.
problem.setParameter("some_parameter", 0.05)
problem.setParameter("some_parameter", 10)

param = problem.getParameter("some_parameter")
```

Note: `setParameter` checks the declared parameter type on the C++ side; a wrong type raises an exception.

### 2.4 Constraints and Projection on Configurations

The Python wrapper maintains a `ConstraintSet` (internally named "Default constraint set") used by the following helpers:

```python
# Project a configuration onto the active constraint set
success, q_proj, residual = problem.applyConstraints(q)

# Validate a configuration against config validations
valid, report = problem.isConfigValid(q)
if not valid:
    print(report)
```

Config projector utilities:

```python
# These attributes control the config projector created by
# addNumericalConstraintsToConfigProjector(...)
problem.errorThreshold = 1e-4
problem.maxIterProjection = 20

# Add numerical (Implicit) constraints into a ConfigProjector.
# If no ConfigProjector exists yet, it is created.
problem.addNumericalConstraintsToConfigProjector(
    "proj",
    [implicit1, implicit2],
    [0, 1],  # priorities (optional overload)
)

# When constraints use the 'Equality' comparison type, you can update
# their right-hand side from a reference configuration.
problem.setRightHandSideFromConfig(q_ref)
```

You can also install an externally-created constraint set on the problem:

```python
problem.setConstraints(constraint_set)
```

### 2.5 Constraint Factory Helpers (Problem)

The bindings expose a few C++ helpers that directly create `Implicit` constraints:

```python
# Relative COM constraint:
# - comName == "" uses the full robot COM
# - comName != "" uses a partial COM previously registered with addPartialCom
implicit = problem.createRelativeComConstraint(
    "rel_com",
    "",                 # comName
    "some_joint",
    [0.0, 0.0, 0.0],     # point
    [True, True, True],
)

# Transformation constraint (overloaded):
# - joint1Name == "" constrains joint2 in the world frame
# - joint1Name != "" constrains joint2 relative to joint1
implicit = problem.createTransformationConstraint(
    "tf",
    "joint1",           # or ""
    "joint2",
    M,
    [True, True, True, True, True, True],
)

# COM between feet constraint (with optional partial COM)
implicit = problem.createComBetweenFeet(
    "com_between_feet",
    "",                 # comName
    "left_foot",
    "right_foot",
    pointL,
    pointR,
    "",                 # jointRefName ("" means root joint)
    pointRef,
    [True, True, True],
)

# Control whether the constraint uses a constant RHS (EqualToZero)
# or a configurable RHS (Equality).
problem.setConstantRightHandSide(implicit, True)
```

Partial COM helpers:

```python
problem.addPartialCom("arm_com", ["joint1", "joint2", "joint3"])
com = problem.getPartialCom("arm_com")
```

### 2.6 Path Projection

Path projectors ensure that paths satisfy constraints:

```python
from pyhpp.manipulation import (
    NoneProjector,
    ProgressiveProjector,
    DichotomyProjector,
    GlobalProjector,
    RecursiveHermiteProjector,
)

# Progressive projector (RECOMMENDED for manipulation)
# Projects path incrementally with given step size
projector = ProgressiveProjector(
    problem.distance(),           # Distance metric
    problem.steeringMethod(),     # Steering method
    0.1                           # Step size
)
problem.pathProjector(projector)

# Dichotomy projector (binary search approach)
# Uses dichotomy to find projection
projector = DichotomyProjector(
    problem.distance(),
    problem.steeringMethod(),
    1e-3                          # Precision threshold
)

# Attach to the problem
problem.pathProjector(projector)

# Global projector (single projection)
# Projects entire path at once
projector = GlobalProjector(
    problem.distance(),
    problem.steeringMethod(),
    1e-3
)

# Optional: Recursive Hermite projector
projector = RecursiveHermiteProjector(
    problem.distance(),
    problem.steeringMethod(),
    1e-3
)

# Disable projection (nullptr)
problem.pathProjector(NoneProjector())
```

Notes:

- In the Python bindings, the projector constructors are exposed as functions named
    `ProgressiveProjector`, `DichotomyProjector`, `GlobalProjector`, `RecursiveHermiteProjector`.
    There are no `createDichotomyProjector` / `createGlobalProjector` functions.
- All of these factory functions take `(distance, steeringMethod, step)`.
- In `pyhpp.manipulation`, these functions accept either a regular steering method or a graph steering method
    (the binding extracts the inner steering method automatically).

### 2.7 Direct Path Utility

The binding provides `directPath(q1, q2, validate)` as a convenience wrapper:

```python
success, path, report = problem.directPath(q1, q2, True)
if not success:
    print(report)
```

Behavior (when `validate=True`):

- Uses the current `steeringMethod()` to build a direct path.
- If a `pathProjector()` is set, tries to project the path.
- Validates the (projected) path with `pathValidation()`.

Return value:

- `success`: `bool`
- `path`: a `PathVector` (or `None` if the steering method failed)
- `report`: `str` (empty on success)

### 2.8 Manipulation-Specific Additions (`pyhpp.manipulation.Problem`)

`pyhpp.manipulation.Problem` adds graph integration:

```python
from pyhpp.manipulation import Graph, Problem

problem = Problem(robot)
graph = Graph("g", robot, problem)

problem.constraintGraph(graph)
problem.checkProblem()  # Raises if init/goal invalid or if graph is missing
```

Steering method note:

- In manipulation, the underlying C++ default steering method is a *graph steering method*.
- The Python getter `problem.steeringMethod()` returns the *inner* steering method when the graph steering method is active.
- The setter `problem.steeringMethod(...)` accepts either a regular steering method wrapper or a graph steering method wrapper.

---

## 3. Graph: Constraint Graph for Manipulation

The `Graph` class implements constraint-based manipulation planning using a graph where nodes represent discrete modes (states) and edges represent feasible transitions.

The Python bindings also expose lightweight wrapper objects:

- `State`: has `name()`
- `Transition`: has `name()` and `pathValidation()`

### 3.1 Creating a Graph

```python
from pyhpp.manipulation import Graph

# Create constraint graph
graph = Graph(
    "graph_name",   # Graph name (string)
    robot,          # Device object
    problem         # Problem object
)

# Configure graph parameters
graph.maxIterations(100)        # Max iterations for constraint solving
graph.errorThreshold(1e-5)      # Convergence threshold
```

### 3.2 States (Nodes)

States represent discrete modes where specific constraints are active.

```python
# Create a state
state = graph.createState(
    "state_name",    # Unique state name
    False,           # waypoint: True if intermediate waypoint state
    0                # priority: lower = higher priority (for tie-breaking)
)

# Create multiple states
states = {}
states['free'] = graph.createState("free", False, 0)
states['grasp'] = graph.createState("grasp", False, 0)
states['place'] = graph.createState("place", False, 0)

# Query states
state = graph.getState("state_name")             # Get state by name
all_state_names = graph.getStateNames()          # List all state names
state = graph.getStateFromConfiguration(q)       # Find state containing config
```

### 3.3 Transitions (Edges)

Transitions represent feasible motions between states.

#### Standard Transition

```python
edge = graph.createTransition(
    from_state,        # Source state
    to_state,          # Target state
    "edge_name",       # Unique edge name
    1,                 # weight (int, for probabilistic selection)
    containing_state   # State whose constraints apply to the path
)

# Example: self-loop for exploration within a state
transit = graph.createTransition(
    states['free'],
    states['free'],
    "transit",
    1,
    states['free']
)
```

#### Waypoint Transition

Path must pass through waypoint configurations:

```python
edge = graph.createWaypointTransition(
    from_state,
    to_state,
    "waypoint_edge",
    2,                 # Number of waypoints
    1,                 # Weight (int)
    containing_state,
    False              # automaticBuilder
)

# If automaticBuilder=True, intermediate waypoint states/edges are created
# automatically and wired into the WaypointTransition.
```

#### Level-Set Transition

For constrained motion along a manifold:

```python
edge = graph.createLevelSetTransition(
    from_state,
    to_state,
    "levelset_edge",
    1,
    containing_state
)
```

### 3.4 Querying States and Transitions

```python
# Get transition by name
edge = graph.getTransition("edge_name")

# Get state by name
state = graph.getState("state_name")

# Get all names
edge_names = graph.getTransitionNames()
state_names = graph.getStateNames()

# Get wrapper objects (State / Transition)
states = graph.getStates()
edges = graph.getTransitions()

# Get connected states
from_state, to_state = graph.getNodesConnectedByTransition(edge)

# Get/set edge properties
weight = graph.getWeight(edge)
graph.setWeight(edge, 2)

is_short = graph.isShort(edge)  # Check if transition is "short"

# Get/set which state a transition is "in" (its containing node)
graph.setContainingNode(edge, states['free'])
containing = graph.getContainingNode(edge)  # returns a state name (string)

# For waypoint transitions only: set which (edge,state) pair to use at a waypoint index
# index is 0..nbWaypoints-1
graph.setWaypoint(waypoint_edge, 0, some_edge, some_state)
```

### 3.5 State Queries

```python
# Return the name (string) of the state that contains configuration q
state_name = graph.getStateFromConfiguration(q)
```

### 3.6 Constraint Management in the Graph

Numerical constraints (implicit constraints) can be attached to states, transitions, or the whole graph.

```python
# Add a single numerical constraint to a state
graph.addNumericalConstraint(grasp_state, implicit_constraint)

# Add a list of numerical constraints
graph.addNumericalConstraintsToState(grasp_state, [implicit1, implicit2])
graph.addNumericalConstraintsToTransition(edge, [implicit_edge])
graph.addNumericalConstraintsToGraph([implicit_global])

# Add numerical constraints that apply to PATHS inside a state
graph.addNumericalConstraintsForPath(grasp_state, [implicit_path])

# Query constraints
state_constraints = graph.getNumericalConstraintsForState(grasp_state)
edge_constraints = graph.getNumericalConstraintsForEdge(edge)
graph_constraints = graph.getNumericalConstraintsForGraph()

# Reset constraints of a state
graph.resetConstraints(grasp_state)

# Advanced: register constraint/complement/both triplets for graph initialization
# (initialize() calls registerConstraints() internally on these)
graph.registerConstraints(constraint, complement, both)
```

### 3.7 Built-in Constraint Helpers

The Graph bindings provide helpers that create common manipulation constraints.

```python
# Placement constraint between two surface sets (lists of triangle set names)
constraint, complement, both = graph.createPlacementConstraint(
    "place", ["object_surface"], ["table_surface"], 1e-4
)

# Overload without explicit margin: defaults to 1e-4
constraint, complement, both = graph.createPlacementConstraint(
    "place", ["object_surface"], ["table_surface"]
)

# Pre-placement constraint (width margin)
preplace = graph.createPrePlacementConstraint(
    "preplace", ["object_surface"], ["table_surface"], 0.02
)

# Grasp constraints return a list: [constraint, complement, both]
grasp_triplet = graph.createGraspConstraint("grasp", "gripper", "handle")

# Pre-grasp constraint
pregrasp = graph.createPreGraspConstraint("pregrasp", "gripper", "handle")
```

### 3.8 Error Checks and Projection Utilities

These methods are useful for debugging constraints and building valid configurations.

```python
# Check state constraints at a configuration
# Returns: (error_vector, belongs)
err, ok = graph.getConfigErrorForState(grasp_state, q)

# Check transition constraints at a configuration
# Returns: (error_vector, belongs)
err, ok = graph.getConfigErrorForTransition(edge, q)

# Check foliation leaf/target validity for a transition
err, ok = graph.getConfigErrorForTransitionLeaf(edge, q_leaf, q)
err, ok = graph.getConfigErrorForTransitionTarget(edge, q_leaf, q)

# Apply constraints (returns: success, q_out, residual_error)
success, q_proj, residual = graph.applyStateConstraints(grasp_state, q)

# Apply transition leaf constraints (requires q_rhs as right-hand-side)
success, q_proj, residual = graph.applyLeafConstraints(edge, q_rhs, q)

# Generate a valid target configuration along a transition
success, q_target, residual = graph.generateTargetConfig(edge, q_rhs, q_seed)
```

Notes:

- `residual_error` is taken from the underlying ConfigProjector when available; it can be `NaN` if no projector is attached.
- `applyLeafConstraints()` uses `q_rhs` to set the projector right-hand-side (`rightHandSideFromConfig`).

### 3.9 Level-Set Transitions

For `LevelSetTransition` edges, you can define the foliation by attaching constraints:

```python
graph.addLevelSetFoliation(levelset_edge, cond_constraints, param_constraints)
```

### 3.10 Collision/Relative-Motion Helpers (Per Transition)

```python
# Security margins matrix (returned as a list of lists)
matrix = graph.getSecurityMarginMatrixForTransition(edge)

# Set collision security margin for a pair of joints along a transition
graph.setSecurityMarginForTransition(edge, "joint1", "joint2", 0.01)

# Relative motion matrix (list of lists of ints)
rel = graph.getRelativeMotionMatrix(edge)

# Remove collision pair from a transition (sets relative motion constrained)
graph.removeCollisionPairFromTransition(edge, "joint1", "joint2")
```

### 3.11 Subgraphs and Debug Display

```python
# Create a subgraph with guided state selection (requires a Roadmap instance)
graph.createSubGraph("subgraph", roadmap)

# Set target node list for GuidedStateSelector
graph.setTargetNodeList([free_state, grasp_state])

# Human-readable dumps of constraints
print(graph.displayStateConstraints(grasp_state))
print(graph.displayTransitionConstraints(edge))
print(graph.displayTransitionTargetConstraints(edge))

# Output graph in DOT format
graph.display("graph.dot")
```

### 3.12 Graph Initialization

⚠️ **CRITICAL:** You MUST call `initialize()` before using the graph!

```python
# After adding all states, edges, and constraints:
graph.initialize()

# This method:
# - Validates graph structure
# - Precomputes constraint data structures
# - Prepares graph for runtime use
# - Must be called AFTER all graph construction

# Typical construction order:
# 1. Create graph
# 2. Create states
# 3. Create transitions
# 4. Create and add constraints
# 5. Configure parameters (maxIterations, errorThreshold)
# 6. Initialize ← REQUIRED!
```

---

## 4. Constraints: Geometric Constraints

Constraints define geometric relationships that configurations must satisfy.

### 4.1 Overview: Differentiable Functions vs. Constraints

In `pyhpp.constraints`, there are two layers:

- **Geometric functions** (subclasses of `DifferentiableFunction`) such as `Transformation`, `RelativeTransformation`, `Position`, `Orientation`, `RelativeCom`, …
    - They can be evaluated (`f(q)`) and differentiated (`f.J(q)`).
    - They are **not** directly “numerical constraints” until wrapped.
- **Numerical constraints** (`Implicit` and subclasses) that can be added to a constraint graph / numerical constraint set.
    - `Implicit(function, comparisonTypes, mask)` wraps a `DifferentiableFunction`.
    - Some constraints are already `Implicit` subclasses (e.g. `LockedJoint`, and also `Explicit`).

Important for Python API usage: some classes are constructed with Python `__init__`, others use static `.create(...)`.

### 4.2 `DifferentiableFunction` (evaluation API)

All `DifferentiableFunction` instances expose a small “Pythonic” API:

```python
value = f(q)   # same as f.__call__(q)
J = f.J(q)     # Jacobian

# Sizes
ni = f.ni      # input size
ndi = f.ndi    # input derivative size
no = f.no      # output size
ndo = f.ndo    # output derivative size
```

### 4.3 `Transformation`: Absolute pose function (world ↔ joint)

`Transformation` is a `DifferentiableFunction`. It is constructed in Python as:

```python
Transformation(name, robot, joint2, frame2, frame1, mask)
```

Semantics (world is used as “joint1”): it enforces the equality

`oM_joint2 * frame2 = frame1`

where `frame1` is an SE(3) in the world frame, and `frame2` is an SE(3) fixed in `joint2`.

Example: constrain a joint frame to a target pose in the world.

```python
from pyhpp.constraints import Transformation
from pinocchio import SE3
import numpy as np

joint2 = robot.model().getJointId("object/root_joint")

# [x, y, z, rx, ry, rz] mask: True means the component is used in the error.
mask = [False, False, True, True, True, False]  # constrain z, rx, ry

target = SE3.Identity()
target.translation = np.array([0.3, 0.0, 0.1])

f = Transformation(
        "placement",
        robot,
        joint2,
        SE3.Identity(),  # frame2 in joint2
        target,         # frame1 in world
        mask,
)
```

**DOF Mask Details:**

| Index | DOF | True means... | False means... |
|-------|-----|---------------|----------------|
| 0 | X position | X is fixed | X is free |
| 1 | Y position | Y is fixed | Y is free |
| 2 | Z position | Z is fixed | Z is free |
| 3 | Roll (rx) | Roll is fixed | Roll is free |
| 4 | Pitch (ry) | Pitch is fixed | Pitch is free |
| 5 | Yaw (rz) | Yaw is fixed | Yaw is free |

**Common Mask Patterns:**

```python
# Keep object on table (fix z, roll, pitch; free x, y, yaw)
mask_on_table = [False, False, True, True, True, False]

# Fix position only (free orientation)
mask_position = [True, True, True, False, False, False]

# Fix orientation only (free position)
mask_orientation = [False, False, False, True, True, True]

# Fix everything (all 6 DOF)
mask_full = [True, True, True, True, True, True]
```

### 4.4 `RelativeTransformation`: Relative pose function (joint ↔ joint)

Constrains relative pose between two joints (e.g., gripper grasping object).

```python
from pyhpp.constraints import RelativeTransformation
from pinocchio import SE3
import numpy as np

gripper = robot.model().getJointId("ur5/wrist_3_joint")
obj = robot.model().getJointId("pokeball/root_joint")

mask_full = [True] * 6

# “grasp” is a transform fixed in the gripper joint.
grasp_in_gripper = SE3.Identity()
grasp_in_gripper.translation = np.array([0.0, 0.137, 0.0])

# RelativeTransformation(name, robot, joint1, joint2, frame1, frame2, mask)
# Enforces: oM_joint1 * frame1 = oM_joint2 * frame2
f = RelativeTransformation(
    "grasp",
    robot,
    gripper,
    obj,
    grasp_in_gripper,  # frame1 in joint1
    SE3.Identity(),    # frame2 in joint2
    mask_full,
)
```

**Interpretation:**
- When this is satisfied: `oM_gripper * grasp_in_gripper = oM_object`
- Maintains a fixed relative pose between the two joints

### 4.5 `Position`: Position-only function

```python
from pyhpp.constraints import Position
from pinocchio import SE3

joint2 = robot.model().getJointId("object/root_joint")
mask_pos = [True, True, True]

f_pos = Position(
    "position",
    robot,
    joint2,
    SE3.Identity(),
    SE3.Identity(),
    mask_pos,
)
```

### 4.6 `Orientation`: Orientation-only function

```python
from pyhpp.constraints import Orientation
from pinocchio import SE3

joint2 = robot.model().getJointId("object/root_joint")
mask_orient = [True, True, True]

f_ori = Orientation(
    "orientation",
    robot,
    joint2,
    SE3.Identity(),
    SE3.Identity(),
    mask_orient,
)
```

### 4.7 `LockedJoint`: fixed joint value (already an `Implicit`)

```python
from pyhpp.constraints import LockedJoint
import numpy as np

locked = LockedJoint(
    robot,
    "joint_name",
    np.array([value])  # 1D array in the joint configuration space
)

# Example: Lock a revolute joint at 90 degrees
locked_elbow = LockedJoint(robot, "elbow_joint", np.array([np.pi / 2]))
```

### 4.8 `Implicit`: wrapping a function into a numerical constraint

`Implicit` turns a `DifferentiableFunction` into a numerical constraint by specifying:

- a **comparison type per output derivative row** (`ComparisonTypes`),
- a **mask** saying which rows are active.

Note: not everything needs wrapping. For example, `LockedJoint` is already an `Implicit` subclass.

```python
from pyhpp.constraints import Implicit, ComparisonTypes, ComparisonType

# Define comparison types for each constrained DOF
comparison_types = ComparisonTypes()
comparison_types[:] = tuple([ComparisonType.EqualToZero] * f.ndo)

# Define implicit mask (which rows are active). If empty, defaults to all True.
implicit_mask = [True] * f.ndo

# Wrap base constraint
implicit_constraint = Implicit(
    f,                   # A DifferentiableFunction (Transformation, etc.)
    comparison_types,     # How to compare constraint values
    implicit_mask        # Which DOF are implicit
)
```

**ComparisonType Options:**

| Type | Mathematical Meaning | Use Case |
|------|---------------------|----------|
| `ComparisonType.EqualToZero` | h(q) = 0 | Equality constraints |
| `ComparisonType.Superior` | h(q) ≥ 0 | Inequality (lower bound) |
| `ComparisonType.Inferior` | h(q) ≤ 0 | Inequality (upper bound) |

`ComparisonType.Equality` is also available. In HPP, rows with `Equality` contribute to the implicit constraint **parameterization** (see `Implicit.parameterSize()`), and can be used with a right-hand side.

**Example with Inequality:**

```python
# Keep object above table (z ≥ 0.05)
comparison_types = ComparisonTypes()
comparison_types[:] = tuple([ComparisonType.Superior])  # z ≥ 0
implicit_mask = [True]

implicit_constraint = Implicit(height_constraint, comparison_types, implicit_mask)
```

### 4.9 Constraint evaluation

```python
# Evaluate a DifferentiableFunction at configuration q
value = f(q)
J = f.J(q)

# Sizes
output_size = f.no
output_derivative_size = f.ndo
```

### 4.10 Solvers (optional, direct use)

The constraints module exposes two solvers:

- `HierarchicalIterative(configSpace)`: can be configured (`maxIterations`, `errorThreshold`) and constraints can be added via `add(constraint, priority)`.
- `BySubstitution(configSpace)`: extends `HierarchicalIterative` and additionally exposes `solve(q)` in Python.

```python
from pyhpp.constraints import BySubstitution

solver = BySubstitution(robot.configSpace())
solver.maxIterations = 50
solver.errorThreshold = 1e-4
solver.add(implicit_constraint, 0)

q_proj, status = solver.solve(q)
```

If you modify the solver explicit constraint set via `solver.explicitConstraintSet()`, call `solver.explicitConstraintSetHasChanged()` afterwards.

### 4.11 `RelativeCom`

`RelativeCom` is a `DifferentiableFunction` that constrains the robot center of mass relative to a joint.

```python
from pyhpp.constraints import RelativeCom
import numpy as np

joint = robot.getJointByName("base_joint")
reference = np.array([0.0, 0.0, 0.0])
mask = [True, True, True]

f_com = RelativeCom.create("com", robot, joint, reference, mask)
```

---

## 5. Planning and Path Manipulation

This section documents the planning loop and the main runtime objects involved in planning and path handling.

### 5.1 Graph constraints: attaching numerical constraints

Attach constraints to a state:

- `graph.addNumericalConstraint(state, implicit)`
- `graph.addNumericalConstraintsToState(state, [implicit, ...])`
- `graph.addNumericalConstraintsForPath(state, [implicit, ...])` (path constraints inside a state)

Attach constraints to a transition:

- `graph.addNumericalConstraintsToTransition(edge, [implicit, ...])`

Attach constraints globally:

- `graph.addNumericalConstraintsToGraph([implicit, ...])`

Reset:

- `graph.resetConstraints(state)`

### 5.2 Graph projection helpers and constraint diagnostics

Projection / generation utilities:

- `graph.applyStateConstraints(state, q)` → `(success: bool, qProj, residual)`
- `graph.applyLeafConstraints(edge, qRhs, q)` → `(success: bool, qProj, residual)`
- `graph.generateTargetConfig(edge, qRhs, qSeed)` → `(success: bool, qTarget, residual)`

Error diagnostics:

- `graph.getConfigErrorForState(state, q)` → `(error, belongs)`
- `graph.getConfigErrorForTransition(edge, q)` → `(error, belongs)`
- `graph.getConfigErrorForTransitionLeaf(edge, qLeaf, q)` → `(error, belongs)`
- `graph.getConfigErrorForTransitionTarget(edge, qLeaf, q)` → `(error, belongs)`

### 5.3 High-level graph generation: `ConstraintGraphFactory` (pure Python helper)

The module `pyhpp.manipulation.constraint_graph_factory` provides a high-level graph generator with an API intentionally close to the CORBA-side factory.

Entry point:

```python
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule

factory = ConstraintGraphFactory(graph)
```

Main API:

- `factory.setGrippers(grippers: list[str])`
- `factory.setObjects(objects, handlesPerObjects, contactSurfacesPerObjects)`
- `factory.environmentContacts(envContacts)`
- `factory.setRules(rules: list[Rule])`
- `factory.setPossibleGrasps(graspsDict)`
- `factory.setPreplacementDistance(objectName, distance)` / `factory.getPreplacementDistance(objectName)`
- `factory.setPreplaceGuide(preplaceGuide)`
- `factory.generate()`

Important note:

- This factory creates states/transitions using the bound `Graph` API (including waypoint and level-set transitions), and attaches constraints using `graph.addNumericalConstraintsToState(...)` / `graph.addNumericalConstraintsToTransition(...)`.

### 5.4 Manipulation planning

High-level manipulation planner:

```python
from pyhpp.manipulation import ManipulationPlanner

planner = ManipulationPlanner(problem)
planner.maxIterations(5000)
planner.timeOut(60.0)

path = planner.solve()  # returns a PathVector (or None)
```

Transition-level planning:

```python
from pyhpp.manipulation import TransitionPlanner

tp = TransitionPlanner(problem)
tp.setEdge(edge)
path = tp.planPath(q_start, q_goals, resetRoadmap=True)
```

Core planners are also exposed (examples):

- `pyhpp.core.BiRRTPlanner(problem)`
- `pyhpp.core.DiffusingPlanner(problem)`

### 5.5 Manipulation steering methods (graph-aware wrappers)

Wrappers used in graph-based planning:

```python
from pyhpp.manipulation import GraphSteeringMethod

gsm = GraphSteeringMethod(problem.steeringMethod())
```

`EndEffectorTrajectorySteeringMethod` is constructed from a core `hpp::core::Problem`:

```python
from pyhpp.manipulation import EndEffectorTrajectorySteeringMethod
from pyhpp.core import ProblemSolver

ps = ProblemSolver.create()
ps.robot(robot.asPinDevice())
cpp_problem = ps.problem()

eet = EndEffectorTrajectorySteeringMethod(cpp_problem)
eet.setTrajectoryConstraint(implicit_constraint)
eet.setTrajectory(path, se3Output=True)
```

### 5.6 Core `ProblemSolver` (binding-accurate API)

Construction:

- `ProblemSolver.create()` (static)

Core accessors:

- `ps.problem()` → core `Problem`
- `ps.robot()` / `ps.robot(device)` (getter/setter)
- `ps.createRobot(robotName)`

Environment / obstacles:

- `ps.addObstacle(obstacleDevice, collision, distance)`

Initial and goal configurations:

- `ps.initConfig()` / `ps.initConfig(q)` (getter/setter)
- `ps.addGoalConfig(q)`
- `ps.resetGoalConfigs()`

Solve lifecycle:

- `ps.resetProblem()`
- `ps.resetRoadmap()`
- `ps.solve()` → `bool`
- Step-by-step API: `ps.prepareSolveStepByStep()`, `ps.executeOneStep()`, `ps.finishSolveStepByStep()`

Paths container management:

- `ps.paths()` → internal paths container (indexable from Python)
- `ps.path(i)` → returns the `i`-th stored path (`PathVector`)
- `ps.addPath(pathVector)`
- `ps.erasePath(i)`

Path optimization pipeline:

- `ps.createPathOptimizers()`
- `ps.optimizePath(pathVector)`
- `ps.addPathOptimizer(name, optimizer)` / `ps.clearPathOptimizers()`
- `ps.pathOptimizers` (container)

Component selection by type-name (string):

- `ps.robotType()` / `ps.robotType(name)`
- `ps.pathPlannerType()` / `ps.pathPlannerType(name)` and `ps.pathPlanner()`
- `ps.distanceType()` / `ps.distanceType(name)`
- `ps.steeringMethodType()` / `ps.steeringMethodType(name)`
- `ps.configurationShooterType()` / `ps.configurationShooterType(name)`

Config validation stack:

- `ps.addConfigValidation(name, validation)`
- `ps.configValidationTypes()`
- `ps.clearConfigValidations()`

Path validation / projection types:

- `ps.pathValidationType(name, parameters)`
- `ps.pathProjectorType(name, parameters)`

Tuning parameters:

- `ps.maxIterProjection()` / `ps.maxIterProjection(n)`
- `ps.maxIterPathPlanning()` / `ps.maxIterPathPlanning(n)`
- `ps.errorThreshold()` / `ps.errorThreshold(eps)`

Named builders/containers:

- `ps.pathPlanners` (container-like object)
- `ps.pathOptimizers` (container-like object)

Both containers expose methods like `keys()`, `has(key)`, `erase(key)`, `clear()`, `__getitem__(key)`, and `add(key, valueOrBuilder)`.

Example (same pattern as `hpp-python/tests/problem-solver.py`):

```python
from pyhpp.core import ProblemSolver
import numpy as np

ps = ProblemSolver.create()
robot = ps.createRobot("ur3")
ps.robot(robot)

qinit = np.zeros((robot.model().nq, 1))
qgoal = qinit.copy()
qgoal[0, 0] = 1

ps.initConfig(qinit)
ps.addGoalConfig(qgoal)

ok = ps.solve()
if ok:
    path = ps.paths()[0]  # PathVector (inherits Path)
```

### 5.7 Core paths: `Path`, `StraightPath`, and `PathVector`

Common `Path` API (bound methods):

- `path(t)` / `path.eval(t)` → `(q, success)`
- `path(qOut, t)` / `path.eval(qOut, t)` → `success` (fills `qOut`)
- `path.derivative(t, order)`
- `path.copy()`
- `path.extract(t0, t1)` (also accepts an interval)
- `path.timeRange()` / `path.paramRange()`
- `path.length()`
- `path.initial()` / `path.end()`
- `path.constraints()`
- `path.outputSize()` / `path.outputDerivativeSize()`

Example for the two `__call__` overloads:

```python
q, success = path(0.3)

qout = np.zeros_like(q)
success = path(qout, path.length())
```

`StraightPath` creation:

- `StraightPath.create(...)` (static, with overloads)

`PathVector` is exposed as `pyhpp.core.path.Vector` (and inherits `Path`):

- `Vector.create(configSize, numberDof)` (static)
- `pv.numberPaths()`
- `pv.pathAtRank(i)`
- `pv.rankAtParam(t)`
- `pv.appendPath(path)`
- `pv.concatenate(otherPathVector)`
- `pv.flatten()`

### 5.8 Core roadmap: `Roadmap`

`pyhpp.core.Roadmap` is a C++ roadmap object that can be used directly from Python (e.g. to implement RRT/PRM experiments).

Construction (as used in `hpp-python/tests/rrt.py`):

```python
from pyhpp.core import Roadmap, WeighedDistance

weighedDistance = WeighedDistance(robot)
roadmap = Roadmap(weighedDistance, robot)
```

Core API surface (bound methods):

- `roadmap.clear()`
- `roadmap.initNode(q)` (also overload returning a node object)
- `roadmap.addNode(q)`
- `roadmap.addNodeAndEdges(...)` / `roadmap.addNodeAndEdge(...)`
- `roadmap.addEdge(q1, q2, path)` (also overload)
- `roadmap.addEdges(...)`
- `roadmap.merge(otherRoadmap)`
- `roadmap.insertPathVector(pathVector)`
- `roadmap.addGoalNode(q)` / `roadmap.resetGoalNodes()`
- `roadmap.goalNodes()`
- `roadmap.nodes()`
- `roadmap.nodesConnectedComponent(i)`
- `roadmap.connectedComponents()`
- `roadmap.numberConnectedComponents()`
- `roadmap.getConnectedComponent(i)`
- `roadmap.connectedComponentOfNode(q)`
- Nearest queries: `roadmap.nearestNode(...)` (multiple overloads) and `roadmap.nearestNodes(...)`
- `roadmap.nodesWithinBall(q, radius)`
- `roadmap.pathExists()`
- `roadmap.distance()`

Typical nearest-neighbor usage (from `hpp-python/tests/rrt.py` pattern):

```python
q_near, d = roadmap.nearestNode(q_rand, cc)
```

### 5.9 Core path validation and path optimization

#### Path validation

`pyhpp.core.PathValidation.validate(path)` returns a tuple `(valid, validPart, report)`.

The module also exposes factory functions:

```python
from pyhpp.core import DiscretizedCollisionAndJointBound, Progressive, Dichotomy

pv = DiscretizedCollisionAndJointBound(robot.asPinDevice(), stepSize=0.05)
ok, valid_part, report = pv.validate(path)

# Continuous validators
pv2 = Progressive(robot.asPinDevice(), tolerance=1e-3)
pv3 = Dichotomy(robot.asPinDevice(), tolerance=1e-3)
```

#### Path optimization

The base class is `pyhpp.core.PathOptimizer`, with `optimize(pathVector)`.

Concrete optimizers constructed via `__init__` include:

- `RandomShortcut(problem)`
- `SimpleShortcut(problem)`
- `PartialShortcut(problem)`
- `SimpleTimeParameterization(problem)`
- `RSTimeParameterization(problem)`

The manipulation module adds:

- `RandomShortcut(problem)` (manipulation variant)
- `EnforceTransitionSemantic(problem)`
- Graph-aware factories: `GraphRandomShortcut(problem)` and `GraphPartialShortcut(problem)`

### 5.10 Core submodules: `pyhpp.core.path`, `pyhpp.core.path_optimization`, `pyhpp.core.problem_target`

These are imported by `pyhpp.core` and provide advanced types.

#### `pyhpp.core.path`

```python
from pyhpp.core.path import Vector, SplineB1, SplineB3

pv = Vector.create(robot.configSize(), robot.numberDof())
```

#### `pyhpp.core.path_optimization`

This module exposes low-level optimization primitives (linear constraints, quadratic programs) and spline-gradient-based abstract optimizers:

```python
from pyhpp.core.path_optimization import LinearConstraint, QuadraticProgram

lc = LinearConstraint(10, 5)
qp = QuadraticProgram(5)
```

#### `pyhpp.core.problem_target`

`GoalConfigurations` is exposed as a `ProblemTarget` implementation:

```python
from pyhpp.core.problem_target import GoalConfigurations
from pyhpp.core import ProblemSolver

ps = ProblemSolver.create()
ps.robot(robot.asPinDevice())
cpp_problem = ps.problem()
target = GoalConfigurations.create(cpp_problem)
```

### 5.11 Complete Planning Workflow

```python
# 1. Setup
robot = Device("robot")
# ... load models ...

problem = Problem(robot)
graph = Graph("graph", robot, problem)

# 2. Build graph
# ... create states and transitions ...
# ... add constraints ...

# 3. Configure
graph.maxIterations(100)
graph.errorThreshold(1e-5)
graph.initialize()  # REQUIRED!

problem.constraintGraph(graph)

# 4. Set initial and goal
shooter = problem.configurationShooter()

# Find valid initial configuration
for i in range(100):
    q = shooter.shoot()
    success, q_init, residual = graph.applyStateConstraints(initial_state, q)
    if success:
        break

# Find valid goal configuration
for i in range(100):
    q = shooter.shoot()
    success, q_goal, residual = graph.applyStateConstraints(goal_state, q)
    if success:
        break

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)

# 5. Configure path projector
from pyhpp.manipulation import ProgressiveProjector

projector = ProgressiveProjector(
    problem.distance(),
    problem.steeringMethod(),
    0.1
)
problem.pathProjector(projector)

# 6. Plan
from pyhpp.manipulation import ManipulationPlanner

planner = ManipulationPlanner(problem)
planner.maxIterations(5000)

path = planner.solve()
if path is not None:
    print("✓ Success!")
else:
    print("✗ Planning failed")
```

---

## 6. Visualization

### 6.1 Gepetto Viewer

```python
from pyhpp.gepetto.viewer import Viewer

# Create viewer
viewer = Viewer(robot)

# Display configuration
viewer(q)  # Call viewer as function

# Alternative syntax
viewer.display(q)

# Load visualization model explicitly
viewer.loadViewerModel(robot, "robot_name")
```

### 6.2 Animating Configurations

```python
import time

def animate_configurations(viewer, configs, dt=0.5):
    """Animate a sequence of configurations."""
    for i, q in enumerate(configs):
        print(f"Config {i+1}/{len(configs)}")
        viewer(q)
        time.sleep(dt)

# Use it
configs = [q1, q2, q3, q4, q5]
animate_configurations(viewer, configs, dt=1.0)
```

### 6.3 Animating Path

```python
def animate_path(viewer, path, dt=0.01):
    """Animate a path."""
    t = 0.0
    length = path.length()
    
    print(f"Animating path (length={length:.3f})...")
    
    while t <= length:
        q = path(t)
        viewer(q)
        time.sleep(dt)
        t += dt
    
    print("Animation complete!")

# Use it
animate_path(viewer, path, dt=0.02)
```

---

## Migration from CORBA

### API Differences

| Feature | CORBA Server | PyHPP Bindings |
|---------|-------------|----------------|
| **Communication** | CORBA RPC (out-of-process) | Direct C++ (in-process) |
| **Performance** | Serialization overhead | Near-native speed |
| **API Style** | String-based names | Object references |
| **Random Config** | `robot.shootRandomConfig()` | `problem.configurationShooter().shoot()` |
| **Constraint Creation** | Constructor | Mix of constructors and static `.create()` |
| **Graph Initialization** | Automatic | Explicit `graph.initialize()` |
| **Constraint Wrapping** | Optional | Required (`Implicit()`) |
| **Type Conversion** | CORBA sequences | NumPy arrays |

### Common Migration Issues

#### 1. Random Configuration Sampling

```python
# CORBA
q_rand = robot.shootRandomConfig()

# PyHPP
shooter = problem.configurationShooter()
q_rand = shooter.shoot()
```

#### 2. Constraint Creation

```python
# CORBA
constraint = Transformation(name, device, joint_id, ...)

# PyHPP
# Many constraints are constructed directly (e.g. Transformation),
# while some use a static .create(...) (e.g. RelativeCom, Explicit).
constraint = Transformation(name, robot, joint_id, frame2, frame1, mask)
```

#### 3. Graph Initialization

```python
# CORBA (automatic)
graph = ConstraintGraph(...)
# Ready to use immediately

# PyHPP (explicit)
graph = Graph(...)
# ... add states, edges, constraints ...
graph.initialize()  # REQUIRED!
```

#### 4. Constraint Wrapping

```python
# CORBA (optional)
graph.addNumericalConstraint(state, constraint)

# PyHPP (required)
implicit = Implicit(constraint, comparison_types, mask)
graph.addNumericalConstraint(state, implicit)
```

---

## Troubleshooting

### Common Errors

#### 1. `AttributeError: 'Device' object has no attribute 'shootRandomConfig'`

**Problem:** Trying to call `robot.shootRandomConfig()` directly.

**Solution:** Use `problem.configurationShooter().shoot()` instead.

```python
# ❌ Wrong
q = robot.shootRandomConfig()

# ✅ Correct
shooter = problem.configurationShooter()
q = shooter.shoot()
```

#### 2. `TypeError: __init__() missing required argument`

**Problem:** Trying to use constructor instead of factory method.

**Solution:** Double-check the signature. In `pyhpp.constraints`, some classes use constructors and others use `.create(...)`.

```python
# Example: Transformation is constructed directly
f = Transformation(name, robot, joint_id, frame2, frame1, mask)

# Example: RelativeCom uses a static factory
f_com = RelativeCom.create("com", robot, joint, reference, mask)
```

#### 3. `RuntimeError: Graph not initialized`

**Problem:** Forgot to call `graph.initialize()`.

**Solution:** Call `initialize()` after building graph.

```python
# ❌ Wrong
graph = Graph(...)
# ... add states/edges ...
result = graph.applyStateConstraints(state, q)  # Error!

# ✅ Correct
graph = Graph(...)
# ... add states/edges ...
graph.initialize()  # Required!
result = graph.applyStateConstraints(state, q)
```

#### 4. `TypeError: incompatible function arguments`

**Problem:** Wrong argument types (e.g., passing list instead of SE3).

**Solution:** Use proper Pinocchio types.

```python
# ❌ Wrong
transform = [0, 0, 0, 0, 0, 0, 1]

# ✅ Correct
from pinocchio import SE3, Quaternion
import numpy as np
transform = SE3(Quaternion(0, 0, 0, 1), np.array([0, 0, 0]))
```

#### 5. Projection Fails with High Error

**Problem:** Constraints are incompatible or configuration is far from manifold.

**Solutions:**
1. Increase `graph.maxIterations()`
2. Relax `graph.errorThreshold()`
3. Try different random configurations
4. Check constraint definitions for conflicts

```python
# Increase solver parameters
graph.maxIterations(500)
graph.errorThreshold(1e-4)

# Try multiple seeds
shooter = problem.configurationShooter()
for i in range(1000):
    q = shooter.shoot()
    success, q_valid, residual = graph.applyStateConstraints(state, q)
    if success and residual < 1e-5:
        break
```

### Debugging Tips

#### 1. Check Constraint Errors

```python
# Evaluate constraint before adding to graph
constraint = Transformation(...)
value = constraint(q)
print(f"Constraint value: {value}")
print(f"Constraint error: {np.linalg.norm(value)}")
```

#### 2. Verify Joint IDs

```python
# Print all joint names and IDs
model = robot.model()
for i in range(model.njoints):
    name = model.names[i]
    print(f"Joint {i}: {name}")
```

#### 3. Inspect Graph Structure

```python
# Print all states
states = graph.getStateNames()
print(f"States: {states}")

# Print all transitions
edges = graph.getTransitionNames()
print(f"Transitions: {edges}")

# Check connectivity
for edge_name in edges:
    edge = graph.getTransition(edge_name)
    from_state_name, to_state_name = graph.getNodesConnectedByTransition(edge)
    print(f"{edge_name}: {from_state_name} → {to_state_name}")
```

#### 4. Enable Verbose Output

```python
# Some components support verbose mode
problem.setVerbose(True)
planner.setVerbose(True)
```

---

## Source Code Reference

### File Locations

```
/path/to/hpp-python/
├── src/pyhpp/
│   ├── manipulation/
│   │   ├── bindings.cc          # Module entry (212 lines)
│   │   ├── graph.cc             # Graph implementation (1342 lines)
│   │   ├── device.cc            # Device wrapper (259 lines)
│   │   ├── problem.cc           # Problem wrapper (152 lines)
│   │   ├── path-planner.cc      # Planner wrappers
│   │   └── constraint_graph_factory.py  # Python helper (1298 lines)
│   ├── core/                    # Core planning
│   ├── pinocchio/               # Kinematics
│   └── constraints/             # Constraint bindings
└── tests/                       # Test files (excellent examples!)
    ├── constraint_graph.py      # Graph usage examples
    ├── graph_factory.py         # Factory examples
    ├── motion_planner.py        # Planning examples
    └── load_ur3.py              # Device loading examples
```

### Example Files

The `tests/` directory contains excellent working examples:

- **constraint_graph.py** - Complete graph creation and usage
- **graph_factory.py** - Using ConstraintGraphFactory
- **motion_planner.py** - Motion planning workflow
- **load_ur3.py** - Loading URDF models
- **rrt.py** - RRT planner usage
- **path-optimizer.py** - Path optimization

### Repository Information

- **Repository:** https://github.com/humanoid-path-planner/hpp-python
- **Branch:** v6.1.0
- **Documentation:** https://humanoid-path-planner.github.io/hpp-doc/
- **Main HPP Website:** https://humanoid-path-planner.github.io

---

## Summary: Key Takeaways

### ✅ Critical Rules

1. **Always** use the correct construction style (constructor vs `.create(...)`) for the specific constraint
2. **Always** call `graph.initialize()` before using the graph
3. **Always** wrap `DifferentiableFunction` constraints in `Implicit()` before adding to the graph as numerical constraints
4. **Always** use `problem.configurationShooter().shoot()` for random configs
5. **Always** use `robot` when creating constraints

### 📚 Best Practices

- Import `Device` and `urdf` from `pyhpp.manipulation` for manipulation tasks
- Use `ProgressiveProjector()` for path projection in manipulation
- Set appropriate `maxIterations` and `errorThreshold` for your problem
- Test constraint projections before planning
- Use test files in `tests/` directory as reference examples

### 🚀 Typical Workflow

1. Create `Device` and load URDFs
2. Create `Problem`
3. Create `Graph` and build structure (states, transitions)
4. Create constraints using constructors or `.create(...)` as appropriate
5. Wrap `DifferentiableFunction` constraints in `Implicit()`
6. Add constraints to states/edges
7. Configure graph parameters
8. **Initialize graph** (`graph.initialize()`)
9. Set initial/goal configurations
10. Create planner and solve

---


## Conclusion

The hpp-python bindings provide a **thin, efficient wrapper** around the C++ HPP libraries. Unlike the CORBA server which uses string-based remote procedure calls, PyHPP gives direct access to C++ objects with minimal overhead. The `constraint_graph_factory.py` module adds convenience for common manipulation planning patterns, making it comparable to the CORBA interface for standard use cases while maintaining the flexibility to drop down to low-level APIs when needed.

**Document Version:** 1.0  
**Last Updated:** November 18, 2025
