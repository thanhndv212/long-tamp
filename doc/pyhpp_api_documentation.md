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

#### 2. Factory Methods
Constraints use **static factory methods**, not constructors:

```python
# ✅ CORRECT: Static factory method
constraint = Transformation(name, device, joint_id, ...)

# ❌ INCORRECT: Constructor (not exposed)
# constraint = Transformation(name, device, ...)  # Will fail!
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
from pinocchio import SE3, Quaternion, StdVec_Bool as Mask

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

The `Problem` class encapsulates the planning problem including initial/goal configurations, distance metrics, steering methods, and path validation.

### 2.1 Creating a Problem

```python
from pyhpp.manipulation import Problem

# Create problem for a device
problem = Problem(robot)

# Alternative: Using ProblemSolver (high-level interface)
from pyhpp.core import ProblemSolver
ps = ProblemSolver.create()
robot = ps.createRobot("robot_name")
ps.robot(robot)
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

### 2.3 Accessing Problem Components

```python
# Get problem components
distance = problem.distance()               # Distance metric
steering = problem.steeringMethod()         # Steering method
validator = problem.pathValidation()        # Path validator

# Configuration sampling
shooter = problem.configurationShooter()    # Configuration shooter
q_random = shooter.shoot()                  # Sample random config
```

### 2.4 Path Projection

Path projectors ensure that paths satisfy constraints:

```python
from pyhpp.manipulation import (
    ProgressiveProjector,
    createDichotomyProjector,
    createGlobalProjector
)

# Progressive projector (RECOMMENDED for manipulation)
# Projects path incrementally with given step size
projector = ProgressiveProjector(
    problem.distance(),           # Distance metric
    problem.steeringMethod(),     # Steering method
    0.1                           # Step size
)
problem.pathProjector = projector

# Dichotomy projector (binary search approach)
# Uses dichotomy to find projection
projector = createDichotomyProjector(
    problem.distance(),
    problem.steeringMethod(),
    1e-3                          # Precision threshold
)

# Global projector (single projection)
# Projects entire path at once
projector = createGlobalProjector(
    problem.distance(),
    problem.steeringMethod()
)
```

### 2.5 Path Planning

#### Method 1: Direct Solve

```python
# Solve problem directly
success = problem.solve()

if success:
    paths = problem.paths()
    path = paths[0]
    print(f"Found path of length {path.length()}")
else:
    print("No path found")
```

#### Method 2: Using ManipulationPlanner

```python
from pyhpp.manipulation import ManipulationPlanner

# Create planner
planner = ManipulationPlanner(problem)
planner.maxIterations(5000)

# Solve
success = planner.solve()

if success:
    path = planner.path()
    print(f"Path length: {path.length()}")
```

### 2.6 Path Operations

```python
# Evaluate path at parameter t
q, success = path(0.5)          # Get config at t=0.5 (50%)
q_initial = path.initial()      # Start configuration
q_final = path.end()            # End configuration

# Path properties
length = path.length()          # Total path length (time parameter)
time_range = path.timeRange()   # (t_min, t_max)

# Iterate through path
dt = 0.01
t = 0.0
configs = []
while t <= path.length():
    q = path(t)
    configs.append(q)
    t += dt

print(f"Generated {len(configs)} configurations along path")
```

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

### 4.1 Constraint Factory Pattern

⚠️ **Critical:** Constraints use static factory methods (`.create()`), NOT constructors!

```python
from pyhpp.constraints import (
    Transformation,
    RelativeTransformation,
    Position,
    Orientation,
    Implicit,
    LockedJoint,
    ComparisonTypes,
    ComparisonType,
)
from pinocchio import SE3, Quaternion, StdVec_Bool as Mask
import numpy as np

# ✅ CORRECT: Use .create() factory method
constraint = Transformation(...)

# ❌ INCORRECT: Constructors not exposed to Python
# constraint = Transformation(...)  # Will fail!
```

### 4.2 Transformation: Absolute Pose Constraint

Constrains a joint's pose relative to the world frame.

```python
# Get joint ID from Pinocchio model
joint_id = robot.model().getJointId("object/root_joint")

# Define DOF mask: [x, y, z, roll, pitch, yaw]
# True = constrained (fixed), False = free
mask = [False, False, True, True, True, False]  # Fix z, roll, pitch

# Create target SE3 transform
quaternion = Quaternion(0, 0, 0, 1)  # Identity rotation (qx, qy, qz, qw)
position = np.array([0.3, 0.0, 0.1])  # Target position
target = SE3(quaternion, position)

# Create constraint
constraint = Transformation(
    "placement",           # Constraint name
    robot,  # Pinocchio device wrapper
    joint_id,             # Joint ID to constrain
    SE3.Identity(),       # Reference frame (identity = world frame)
    target,               # Target transform
    mask                  # DOF mask
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

### 4.3 RelativeTransformation: Relative Pose Constraint

Constrains relative pose between two joints (e.g., gripper grasping object).

```python
# Get joint IDs
gripper_id = robot.model().getJointId("ur5/wrist_3_joint")
object_id = robot.model().getJointId("pokeball/root_joint")

# Full 6-DOF mask (constrain all relative DOF)
mask_full = Mask()  # Pinocchio's StdVec_Bool
mask_full[:] = (True,) * 6

# Define relative transform (grasp pose)
# This is the transform from gripper to object when grasping
q = Quaternion(0.5, 0.5, -0.5, 0.5)  # 90° rotations
offset = np.array([0, 0.137, 0])      # 137mm forward
grasp_transform = SE3(q, offset)

# Create relative constraint
constraint = RelativeTransformation(
    "grasp",
    robot,
    gripper_id,           # First joint (reference frame)
    object_id,            # Second joint (target frame)
    grasp_transform,      # Transform: T_gripper_to_object
    SE3.Identity(),       # Additional frame offset (usually identity)
    mask_full
)
```

**Interpretation:**
- When this constraint is satisfied: `T_gripper * grasp_transform = T_object`
- Maintains fixed relative pose between gripper and object

### 4.4 Position: Position-Only Constraint

```python
mask_pos = Mask()
mask_pos[:] = (True, True, True)  # Constrain x, y, z

position_constraint = Position.create(
    "position",
    robot,
    joint_id,
    SE3.Identity(),
    target_transform,
    mask_pos
)
```

### 4.5 Orientation: Orientation-Only Constraint

```python
mask_orient = Mask()
mask_orient[:] = (True, True, True)  # Constrain roll, pitch, yaw

orientation_constraint = Orientation.create(
    "orientation",
    robot,
    joint_id,
    SE3.Identity(),
    target_transform,
    mask_orient
)
```

### 4.6 LockedJoint: Fixed Joint Value

```python
locked = LockedJoint.create(
    robot,
    "joint_name",
    np.array([value])     # Locked value (1D array even for 1 DOF)
)

# Example: Lock a revolute joint at 90 degrees
locked_elbow = LockedJoint.create(
    robot,
    "elbow_joint",
    np.array([np.pi/2])
)
```

### 4.7 Implicit Constraints

⚠️ **All constraints must be wrapped in `Implicit()` before adding to graph!**

```python
# Define comparison types for each constrained DOF
comparison_types = ComparisonTypes()
comparison_types[:] = tuple([ComparisonType.EqualToZero] * 3)

# Define implicit mask (which DOF contribute to implicit constraint)
implicit_mask = [True, True, True]

# Wrap base constraint
implicit_constraint = Implicit(
    constraint,           # Base constraint (Transformation, etc.)
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

**Example with Inequality:**

```python
# Keep object above table (z ≥ 0.05)
comparison_types = ComparisonTypes()
comparison_types[:] = tuple([ComparisonType.Superior])  # z ≥ 0
implicit_mask = [True]

implicit_constraint = Implicit(height_constraint, comparison_types, implicit_mask)
```

### 4.8 Constraint Evaluation

```python
# Evaluate constraint at configuration q
value = constraint(q)        # Constraint value vector

# Get Jacobian
J = constraint.J(q)          # Constraint Jacobian matrix

# Check constraint dimension
dim = constraint.dimension()  # Number of constraint equations
```

---

## 5. Planning and Path Manipulation

### 5.1 Adding Constraints to Graph

#### Add to States

```python
# Add single constraint to a state (RECOMMENDED)
graph.addNumericalConstraint(state, implicit_constraint)

# Add multiple constraints one by one
graph.addNumericalConstraint(state, implicit_constraint1)
graph.addNumericalConstraint(state, implicit_constraint2)

# Alternative: Add multiple at once (may be deprecated)
graph.addNumericalConstraintsToState(state, [constraint1, constraint2])
```

#### Add to Transitions

```python
# Add constraints to an edge
graph.addNumericalConstraintsToTransition(edge, [constraint1, constraint2])

# Note: Edge constraints are also inherited from the containing_state
# parameter passed to createTransition()
```

#### Add Globally

```python
# Apply constraints to all states/edges
graph.addNumericalConstraintsToGraph([constraint1, constraint2])
```

#### Reset Constraints

```python
# Remove all constraints from a state
graph.resetConstraints(state)
```

### 5.2 Applying Constraints

#### Project onto State

```python
# Apply all state constraints to project a configuration
success, q_projected, residual = graph.applyStateConstraints(state, q_input)

if success:
    print(f"✓ Projection succeeded (residual: {residual:.6f})")
    print(f"  Projected config: {q_projected}")
else:
    print(f"✗ Projection failed (residual: {residual})")

# Use projected configuration
if success:
    # Configuration satisfies state constraints
    pass
```

#### Project onto Transition

```python
# Apply edge (leaf) constraints
success, q_projected, residual = graph.applyLeafConstraints(edge, q_rhs, q_input)
```

#### Generate Target Configuration

Useful for sampling-based planning:

```python
# Get configuration shooter
shooter = problem.configurationShooter()

# Generate target satisfying edge constraints
for i in range(100):
    q_rand = shooter.shoot()
    success, q_target, residual = graph.generateTargetConfig(edge, q_start, q_rand)
    
    if success:
        # Use q_target for planning
        break
```

#### Check Constraint Satisfaction

```python
# Check if configuration satisfies state constraints
error_vector, is_satisfied = graph.getConfigErrorForState(state, q)

if is_satisfied:
    print(f"✓ Config satisfies state constraints")
    print(f"  Error norm: {np.linalg.norm(error_vector)}")
else:
    print(f"✗ Config violates constraints")
    print(f"  Error: {error_vector}")

# Check transition constraints
error_vector, is_satisfied = graph.getConfigErrorForTransition(q, edge)
```

#### Using ConstraintGraphFactory

```python
from pyhpp.manipulation.constraint_graph_factory import (
    ConstraintGraphFactory,
    Rule,
)

# Create factory
factory = ConstraintGraphFactory(graph)

# Configure factory
factory.setGrippers(["robot/gripper1", "robot/gripper2"])
factory.setObjects(
    ["object1", "object2"],                    # object names
    [["object1/handle"], ["object2/handle"]],  # handles per object
    [["object1/surface"], ["object2/surface"]] # contact surfaces per object
)
factory.environmentContacts(["table/surface", "wall/surface"])

# Set rules
rules = [
    Rule(["gripper.*"], ["handle.*"], True),   # Allow all gripper-handle pairs
    Rule(["gripper1"], [""], False),            # Disallow gripper1 empty
]
factory.setRules(rules)

# Generate graph
factory.generate()
```

### 5.3 Manipulation Planning

#### Using ManipulationPlanner

```python
from pyhpp.manipulation import ManipulationPlanner

# Create planner
planner = ManipulationPlanner(problem)

# Configure planner
planner.maxIterations(5000)
planner.timeOut(60.0)  # seconds

# Solve
print("Solving manipulation planning problem...")
success = planner.solve()

if success:
    path = planner.path()
    print(f"✓ Found path of length {path.length()}")
    
    # Get planning statistics
    n_nodes = planner.roadmap().numNodes()
    n_edges = planner.roadmap().numEdges()
    print(f"  Roadmap: {n_nodes} nodes, {n_edges} edges")
else:
    print("✗ Planning failed")
```

#### Using TransitionPlanner

For planning specific transitions:

```python
from pyhpp.manipulation import TransitionPlanner

# Create transition planner
transition_planner = TransitionPlanner(problem)

# Plan specific transition
path = transition_planner.planTransition(edge, q_start, q_goal)
```

#### Lower-Level Planners

```python
from pyhpp.core import BiRRTPlanner, DiffusingPlanner

# Bidirectional RRT
birrt = BiRRTPlanner(problem)
birrt.solve()

# Diffusing planner
diffusing = DiffusingPlanner(problem)
diffusing.solve()
```

### 5.4 Complete Planning Workflow

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
problem.pathProjector = projector

# 6. Plan
planner = ManipulationPlanner(problem)
planner.maxIterations(5000)

if planner.solve():
    path = planner.path()
    print(f"✓ Success! Path length: {path.length()}")
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
| **Constraint Creation** | Constructor | Static `.create()` methods |
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
# CORBA (implied constructor)
constraint = Transformation(name, device, joint_id, ...)

# PyHPP (explicit factory method)
constraint = Transformation(name, device, joint_id, ...)
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

**Solution:** Use `.create()` static methods for constraints.

```python
# ❌ Wrong
constraint = Transformation(name, device, ...)

# ✅ Correct
constraint = Transformation(name, device, ...)
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

1. **Always** use `.create()` factory methods for constraints
2. **Always** call `graph.initialize()` before using the graph
3. **Always** wrap constraints in `Implicit()` before adding to graph
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
4. Create constraints using `.create()` methods
5. Wrap constraints in `Implicit()`
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
