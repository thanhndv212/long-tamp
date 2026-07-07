# HPP CORBA Python Clients: API Reference

**Focus:** `hpp-corbaserver`, `hpp-manipulation-corba`, `gepetto-viewer-corba` (Python client side)

This document describes the **CORBA-based Python API** used to drive HPP and Gepetto Viewer through omniORB stubs. It mirrors the structure of the PyHPP documentation, but the programming model is different:

- You call methods on a **remote CORBA server** (out-of-process).
- Most HPP entities are referenced by **names** (`str`) and/or **IDs** (`int`) rather than Python-wrapped C++ objects.
- Many calls return simple Python types (lists of floats, tuples, strings) because data is serialized over CORBA.

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

The CORBA HPP stack provides a **client/server** API:

- The **server** hosts the C++ HPP libraries.
- The **Python client** talks to the server via CORBA IDL stubs.

This API is widely used in scripting and in ROS-based pipelines because it supports:

- Running HPP in a dedicated process (or on a remote machine)
- A stable, string-based API surface
- Easy interop with Gepetto Viewer via CORBA

---

## Architecture

### Module Structure (Python)

The primary entry points are:

- `hpp.corbaserver` (core planning server)
- `hpp.corbaserver.manipulation` (manipulation + constraint graph server)
- `gepetto.corbaserver` (Gepetto GUI server)

Files of interest (client side):

- HPP core client wrappers:
  - [hpp/src/hpp-corbaserver/src/hpp/corbaserver/client.py](hpp/src/hpp-corbaserver/src/hpp/corbaserver/client.py)
  - [hpp/src/hpp-corbaserver/src/hpp/corbaserver/robot.py](hpp/src/hpp-corbaserver/src/hpp/corbaserver/robot.py)
  - [hpp/src/hpp-corbaserver/src/hpp/corbaserver/problem_solver.py](hpp/src/hpp-corbaserver/src/hpp/corbaserver/problem_solver.py)
- HPP manipulation client wrappers:
  - [hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/client.py](hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/client.py)
  - [hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/robot.py](hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/robot.py)
  - [hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/problem_solver.py](hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/problem_solver.py)
  - [hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/constraint_graph.py](hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/constraint_graph.py)
  - [hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/constraint_graph_factory.py](hpp/src/hpp-manipulation-corba/src/hpp/corbaserver/manipulation/constraint_graph_factory.py)
- Gepetto client wrappers:
  - [hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/client.py](hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/client.py)
  - [hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/tools.py](hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/tools.py)

### Connection model

You usually connect to:

- HPP NameService / tools:
  - Defaults can be influenced by `HPP_HOST` / `HPP_PORT`.
- Gepetto NameService / GUI:
  - Defaults can be influenced by `GEPETTO_VIEWER_HOST` / `GEPETTO_VIEWER_PORT`.

The Python clients will typically:

1. Initialize omniORB
2. Resolve a CORBA object (either direct `corbaloc:` or through NameService)
3. Narrow it to the expected IDL interface

---

## Quick Start Guide

### Minimal Core Planning Example

```python
from hpp.corbaserver import Client
from hpp.corbaserver import Robot, ProblemSolver

# 1) Connect to the CORBA server
client = Client()

# 2) Load a robot model (see “Device” section for the usual pattern)
robot = Robot("ur5", "anchor")

# 3) Create a problem solver wrapper
ps = ProblemSolver(robot)

# 4) Set init/goal configurations
q_init = [0.0] * robot.getConfigSize()
q_goal = q_init[:]  # replace with a real goal
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

# 5) Choose components (string-based selectors)
ps.selectPathPlanner("BiRRTPlanner")
ps.selectPathValidation("Dichotomy", 1e-3)
ps.selectPathProjector("Progressive", 1e-3)
ps.selectSteeringMethod("SteeringMethodStraight")
ps.selectDistance("WeighedDistance")

# 6) Solve
ok = ps.solve()
print("Solved:", ok)
print("Number of paths:", ps.numberPaths())
```

### Minimal Manipulation (Constraint Graph) Example

```python
from hpp.corbaserver.manipulation import Robot, ProblemSolver, ConstraintGraph

robot = Robot("ur5", "anchor")
ps = ProblemSolver(robot)

# Create a graph (server-side object)
graph = ConstraintGraph(robot, "g")

# Create states and transitions
graph.createNode(["free", "grasp"], waypoint=False, priority=[0, 1])
graph.createEdge("free", "grasp", "take")

# You must typically finalize graph creation on the server side.
# (Exact initialization workflow depends on how you build constraints and
# whether you use ConstraintGraphFactory.)
```

---

## 1. Device: Robot Representation

The CORBA API uses `hpp.corbaserver.Robot` (and a manipulation specialization) as a **remote robot model**.

Compared to PyHPP, this `Robot` object is primarily a **client-side convenience wrapper** around CORBA services:

- It holds a `client` (CORBA stubs) and forwards most calls to `client.robot`.
- It builds useful local caches such as `rankInConfiguration` and `rankInVelocity` by querying the server.
- It may also include higher-level helpers (e.g. stability constraint factories).

### 1.1 Core Robot wrapper

Entry point: `hpp.corbaserver.robot.Robot`.

#### 1.1.1 Construction and connection

Most users either:

- instantiate a `Robot(...)` directly (expects the server to already know the model paths, or that you provide them through class attributes), or
- subclass `Robot` to define where URDF/SRDF live.

Common usage pattern is to subclass `Robot` and provide URDF/SRDF info:

```python
from hpp.corbaserver.robot import Robot

class MyRobot(Robot):
    packageName = "example-robot-data"
    urdfName = "ur5"
    urdfSuffix = ""
    srdfSuffix = ""

robot = MyRobot("ur5", "anchor")
```

If you already have an initialized server-side robot and only want to attach to it, you can instantiate with `robotName=None` and it will query the current robot name from the server.

#### 1.1.2 URDF/SRDF loading

The wrapper can load URDF/SRDF from either:

- filenames computed from `(packageName, urdfName, urdfSuffix, srdfSuffix)`;
- explicit filenames via `urdfFilename` / `srdfFilename` attributes;
- raw XML strings via `RobotXML` (see below).

The underlying load call is one of:

- `loadRobotModel(robotName, rootJointType, urdfFilename, srdfFilename)`
- `loadRobotModelFromString(robotName, rootJointType, urdfString, srdfString)`

Root joint types are the usual CORBA strings: `"anchor"`, `"freeflyer"`, `"planar"`.

#### 1.1.3 Sizes and joint ordering

- `robot.getConfigSize()` returns the configuration size.
- `robot.getNumberDof()` returns the velocity size.

Joint ordering is server-defined; the wrapper exposes it via:

- `robot.getJointNames()` (ordered like the configuration)
- `robot.getJointTypes()`
- `robot.getAllJointNames()` (includes fixed/anchor frames depending on the model)

The wrapper builds (client-side) rank maps:

- `robot.rankInConfiguration: dict[str, int]`
- `robot.rankInVelocity: dict[str, int]`

They are refreshed by `robot.rebuildRanks()` (called automatically after loading).

#### 1.1.4 Kinematic tree helpers

Convenience methods to query the kinematic tree:

- `robot.getParentFrame(frameName)`
- `robot.getChildFrames(frameName, recursive=False)`
- `robot.getParentJoint(jointName)` (skips anchor joints)
- `robot.getChildJoints(jointName, recursive=False)` (skips anchor joints)

These methods maintain internal caches (computed on first use).

#### 1.1.5 Poses and transforms

The server returns poses as CORBA-friendly sequences (typically 7 floats: translation + quaternion).

- `robot.getJointPosition(jointName)`
- `robot.getRootJointPosition()` / `robot.setRootJointPosition(position)`
- `robot.setJointPosition(jointName, position)` (sets the constant transform of a joint w.r.t. its parent)
- `robot.getCurrentTransformation(jointName)` (transform in world frame for the current configuration)

URDF-link oriented pose:

- `robot.getLinkPosition(linkName)` returns the URDF link pose in world frame.
- `robot.getLinkNames(jointName)` lists links attached to a joint.

#### 1.1.6 Joint sizes and bounds

- `robot.getJointNumberDof(jointName)`
- `robot.getJointConfigSize(jointName)`
- `robot.setJointBounds(jointName, bounds)`
- `robot.getJointBounds(jointName)`

Bounds are returned as a flattened sequence `[v0_min, v0_max, v1_min, v1_max, ...]`.

There is also a client-side helper:

- `hpp.corbaserver.robot.shrinkJointRange(robot, joints, ratio)`

#### 1.1.7 Current configuration and sampling

The CORBA server maintains a *current* configuration/velocity:

- `robot.setCurrentConfig(q)` / `robot.getCurrentConfig()`
- `robot.setCurrentVelocity(v)` / `robot.getCurrentVelocity()`

Random sampling (CORBA-only convenience that does not exist in PyHPP `Device`):

- `robot.shootRandomConfig()`

#### 1.1.8 Collision checking and distances

- `robot.isConfigValid(cfg)` returns `(valid: bool, msg: str)`.
- `robot.configIsValid(cfg)` returns only the boolean.
- `robot.distancesToCollision()` returns distance and nearest-point information.
- `robot.getRobotAABB()` returns a 6-float AABB.

#### 1.1.9 Objects attached to joints

- `robot.getJointInnerObjects(jointName)`
- `robot.getJointOuterObjects(jointName)`
- `robot.getObjectPosition(objectName)` returns an `hpp.Transform` wrapper.
- `robot.removeObstacleFromJoint(objectName, jointName)` removes an obstacle from collision checking for that joint.

#### 1.1.10 Mass and center of mass

- `robot.getMass()`
- `robot.getCenterOfMass()`
- `robot.getJacobianCenterOfMass()`

#### 1.1.11 Humanoid helpers (core)

The core module also provides:

- `hpp.corbaserver.robot.HumanoidRobot` (loads a humanoid via `loadHumanoidModel`)
- `hpp.corbaserver.robot.RobotXML` (loads a robot model from URDF/SRDF XML strings)

Additionally, the mixin `StaticStabilityConstraintsFactory` provides utilities to create stability-related constraints (stored server-side in the ProblemSolver constraint map).

### 1.2 Manipulation Robot wrapper

Entry point: `hpp.corbaserver.manipulation.Robot`.

It extends the core robot wrapper and uses a manipulation-aware CORBA client (services `graph`, `problem`, `robot`).

Key differences vs core `Robot`:

- It supports *composite robots* (multiple sub-robots inserted into a single device).
- It provides insert/load helpers that target the manipulation server interface.

#### 1.2.1 Composite robot and insertion API

Typical pattern:

```python
from hpp.corbaserver.manipulation import Robot

class Composite(Robot):
    packageName = "example-robot-data"

robot = Composite(compositeName="world", robotName="ur5", rootJointType="anchor")
```

Insertion helpers (server-side composition):

- `robot.insertRobotModel(robotName, rootJointType, urdfPath, srdfPath)`
- `robot.insertRobotModelOnFrame(robotName, frameName, rootJointType, urdfPath, srdfPath)`
- `robot.insertRobotModelFromString(robotName, rootJointType, urdfString, srdfString, frame="universe")`

Humanoid insertion variants:

- `robot.insertHumanoidModel(...)`
- `robot.insertHumanoidModelFromString(...)`
- `robot.loadHumanoidModel(...)` (alias)

Environment model:

- `robot.loadEnvironmentModel(urdfPath, srdfPath, envName)`

SRDF layering:

- `robot.insertRobotSRDFModel(robotName, srdfPath)` allows loading multiple SRDF files for a given robot.

#### 1.2.2 Multi-robot root placement

For composite robots, root placement is per-sub-robot:

- `robot.setRootJointPosition(robotName, position)`

#### 1.2.3 Handles and grippers (manipulation SRDF)

The manipulation server also exposes helper calls around SRDF-defined handles/grippers:

- `robot.getGripperPositionInJoint(gripperName)`
- `robot.getHandlePositionInJoint(handleName)`
- `robot.setHandlePositionInJoint(handleName, position)`
- `robot.getHandleApproachingDirection(handleName)` / `robot.setHandleApproachingDirection(handleName, direction)`

These are commonly used together with the manipulation constraint-graph API (grasp / pregrasp constraints).

---

## 2. Problem: Planning Problem Definition

The CORBA API central object is the client wrapper `hpp.corbaserver.problem_solver.ProblemSolver`.

Unlike PyHPP, this object is primarily a **facade over CORBA services**, and most entities are referenced by:

- names (`"RandomShortcut"`, `"Progressive"`, `"WeighedDistance"`, ...)
- constraint names (`"placement"`, `"grasp"`, ...)
- path IDs (ints)

### 2.1 Creating a ProblemSolver

Entry point: `hpp.corbaserver.problem_solver.ProblemSolver`.

```python
from hpp.corbaserver import Robot, ProblemSolver

robot = Robot("ur5", "anchor")
ps = ProblemSolver(robot)
```

You can also manage multiple problems on the server:

- `ps.selectProblem(name)`
- `hpp.corbaserver.problem_solver.newProblem(client=None, name=None)`
- `ps.movePathToProblem(pathId, problemName, jointNames)` moves a stored path to another problem (optionally extracting a subchain).

Notes:

- `selectProblem(name)` returns `True` when a new problem was created.
- `newProblem(...)` resets the current server-side problem (or selects/resets a named one).

### 2.2 Plugins, reproducibility, and threading

- `ps.loadPlugin(pluginName)` loads a plugin into the current problem.
  - `pluginName` can be an absolute path or a name resolved under `hppPlugins`.
  - This is reset each time the server problem is reset.
- `ps.setRandomSeed(seed)` sets the random seed.
- `ps.setMaxNumThreads(n)` / `ps.getMaxNumThreads()` configures concurrent access in thread-safe areas.

### 2.3 Inspecting available/selected components

The CORBA server exposes registries of components (planner types, steering methods, validators, …) as string lists.

- `ps.getAvailable(type)` returns a list of available elements of the given `type`.
- `ps.getSelected(type)` returns a list of selected elements of the given `type`.

Tip: pass `"type"` to discover the supported type categories.

### 2.4 Initial and Goal Configurations

```python
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

q0 = ps.getInitialConfig()
goals = ps.getGoalConfigs()
ps.resetGoalConfigs()
```

Goal as constraints (instead of explicit goal configurations):

- `ps.setGoalConstraints(constraints)` sets a task/constraint-based goal.
- `ps.resetGoalConstraints()` clears it.

### 2.5 Parameters

Parameters are set by name; Python values are converted to `CORBA.Any` automatically.

```python
ps.setParameter("some_parameter", 0.05)
ps.setParameter("flag", True)
ps.setParameter("vec", [1.0, 2.0, 3.0])

value = ps.getParameter("some_parameter")
doc = ps.getParameterDoc("some_parameter")
```

Implementation detail: the wrapper converts floats/bools/ints/strings and sequences into the expected `CORBA.Any` types.

### 2.6 Choosing components (string selectors)

The typical CORBA workflow is to select implementations by name:

- `ps.selectPathPlanner(pathPlannerType)`
- `ps.selectConfigurationShooter(configurationShooterType)`
- `ps.selectPathValidation(pathValidationType, tolerance)`
- `ps.selectPathProjector(pathProjectorType, tolerance)`
- `ps.selectDistance(distanceType)`
- `ps.selectSteeringMethod(steeringMethodType)`

For example:

```python
ps.selectPathPlanner("BiRRTPlanner")
ps.selectPathValidation("Dichotomy", 1e-3)
ps.selectPathProjector("Progressive", 1e-3)
```

You can also control:

- Path optimizers:
  - `ps.addPathOptimizer(pathOptimizerType)`
  - `ps.clearPathOptimizers()`
- Configuration validations (in addition to default checks):
  - `ps.addConfigValidation(configValidationType)`
  - `ps.clearConfigValidations()`

### 2.7 Obstacles (environment geometry)

Obstacles are managed through a dedicated CORBA service (`client.obstacle`), but exposed as `ProblemSolver` helpers.

- `ps.loadObstacleFromUrdf(urdfFilename, prefix)` loads collision geometry from a URDF (kinematic structure is ignored).
  - The legacy 3-argument form exists but is deprecated.
- `ps.getObstacleNames(collision: bool, distance: bool)` lists known obstacles.
- `ps.getObstaclePosition(objectName)` / `ps.getObstacleLinkPosition(objectName)` queries obstacle pose.
- `ps.getObstacleLinkNames()` lists obstacle link names.
- `ps.moveObstacle(objectName, cfg)` moves an obstacle (note: not added to some local collision maps).
- `ps.cutObstacle(objectName, aabb)` cuts an obstacle by an AABB (`[xmin, ymin, zmin, xmax, ymax, zmax]`).
- `ps.removeObstacleFromJoint(objectName, jointName, collision: bool, distance: bool)` removes obstacle from a joint’s outer objects.

### 2.8 Constraints and projection

The CORBA API is name-based: constraints are created on the server and stored under string keys.

Create constraints (examples of the core helpers):

- `ps.createTransformationConstraint(name, joint1, joint2, ref, mask)`
- `ps.createOrientationConstraint(name, joint1, joint2, quat, mask)`
- `ps.createPositionConstraint(name, joint1, joint2, p1, p2, mask)`
- `ps.createRelativeComConstraint(name, comName, jointName, point, mask)`
- `ps.createComBeetweenFeet(name, comName, jointL, jointR, pointL, pointR, jointRef, mask)`
- `ps.createDistanceBetweenJointConstraint(name, joint1, joint2, distance)`
- `ps.createDistanceBetweenJointAndObjects(name, joint1, objects, distance)`
- `ps.createLockedJoint(name, jointName, value, comp=None)`
- `ps.createLockedExtraDof(name, index, value)`

Center of mass computations (stored server-side):

- `ps.addPartialCom(comName, jointNames)` creates a partial COM computation from joint subtrees.
- `ps.getPartialCom(comName)` retrieves the joint list defining a partial COM.

Manage stored constraints:

- `ps.resetConstraintMap()` clears the ProblemSolver constraint map (numerical + locked).
- `ps.resetConstraints()` resets constraints applied to the current problem.

Insert constraints into the active config projector:

- `ps.addNumericalConstraints(projectorName, [names...], priorities=[...])`
- `ps.addLockedJointConstraints(projectorName, [names...])`

Right-hand side (RHS) management:

- `ps.setConstantRightHandSide(constraintName, True/False)` / `ps.getConstantRightHandSide(constraintName)`
- `ps.setRightHandSide(rhs)` sets RHS for all non-constant constraints.
- `ps.setRightHandSideByName(constraintName, rhs)` sets RHS for one constraint.
- `ps.setRightHandSideFromConfig(q)` / `ps.setRightHandSideFromConfigByName(constraintName, q)`

Projection and local optimization:

- `ps.applyConstraints(q)` returns the projected configuration (raises on failure).
- `ps.optimize(q)` returns `(q_opt, residualError)`.
- `ps.computeValueAndJacobian(q)` returns `(valueVector, jacobianMatrix)`.
- `ps.generateValidConfig(maxIter)` tries to sample/project a valid configuration.

Passive degrees of freedom:

- `ps.addPassiveDofs(name, dofNames)` creates a passive-dofs vector stored server-side (Jacobian columns for these dofs are set to 0).

Numerical settings:

- `ps.getErrorThreshold()` / `ps.setErrorThreshold(threshold)`
- `ps.setDefaultLineSearchType(type)` (see server-side enum)
- `ps.getMaxIterProjection()` / `ps.setMaxIterProjection(iterations)`
- `ps.getMaxIterPathPlanning()` / `ps.setMaxIterPathPlanning(iterations)`
- `ps.getTimeOutPathPlanning()` / `ps.setTimeOutPathPlanning(seconds)`

Collision-pair filtering:

- `ps.filterCollisionPairs()` builds/filters collision pairs based on joint relative motions.

### 2.9 Solving, paths, and roadmap

Planning:

- `ps.solve()` runs planning and stores resulting paths on the server.
- Step-by-step planning:
  - `ps.prepareSolveStepByStep()`
  - `ps.executeOneStep()`
  - `ps.finishSolveStepByStep()`
- `ps.interruptPathPlanning()` attempts to interrupt planning (effective with multi-thread policy).

Direct path utility:

- `ps.directPath(q1, q2, validate)` returns `(success: bool, pathId: int, report: str)`.
- `ps.appendDirectPath(pathId, q_end, validate)` appends a direct segment to an existing path.

Path storage and queries (paths live server-side and are referenced by IDs):

- `ps.numberPaths()`
- `ps.pathLength(pathId)`
- `ps.configAtParam(pathId, s)`
- `ps.derivativeAtParam(pathId, order, s)`
- `ps.getWaypoints(pathId)`

Path editing:

- `ps.concatenatePath(startId, endId)` concatenates in-place (result stays at `startId`).
- `ps.extractPath(pathId, s0, s1)` creates a new stored path.
- `ps.erasePath(pathId)`
- `ps.projectPath(pathId)` projects a path with the selected projector.

Path optimization:

- `ps.optimizePath(pathId)` runs the configured optimizer chain and returns the output path ID.

Roadmap inspection (mostly for debugging/analysis):

- `ps.nodes()` / `ps.node(nodeId)`
- `ps.numberNodes()` / `ps.numberEdges()`
- `ps.edge(edgeId)`
- `ps.numberConnectedComponents()` / `ps.nodesConnectedComponent(ccId)`
- `ps.getNearestConfig(q, connectedComponentId=-1)`
- `ps.addConfigToRoadmap(q)` / `ps.addEdgeToRoadmap(q1, q2, pathId, bothEdges)`
- `ps.clearRoadmap()`
- `ps.saveRoadmap(filename)` / `ps.loadRoadmap(filename)`

### 2.10 Manipulation ProblemSolver extensions

Entry point: `hpp.corbaserver.manipulation.problem_solver.ProblemSolver`.

It subclasses the core wrapper, but forwards some calls to the manipulation CORBA interfaces.

Contact surfaces (used in placement states):

- `ps.getEnvironmentContactNames()` / `ps.getRobotContactNames()`
- `ps.getEnvironmentContact(name)` / `ps.getRobotContact(name)`

Placement constraints helpers:

- `ps.createPlacementConstraints(placementName, shapeName, envContactName, width=0.05)`
  - returns `(placementConstraintName, prePlacementConstraintName)` when `width` is not `None`.

Stability / registration:

- `ps.createQPStabilityConstraint(...)` (server-defined signature)
- `ps.registerConstraints(...)` (server-defined signature)

Convenience joint locking:

- `ps.lockFreeFlyerJoint(freeflyerBaseName, lockConstraintBaseName, values=(x,y,z,qx,qy,qz,qw))`
- `ps.lockPlanarJoint(jointName, lockConstraintName, values=(x,y,cos,sin))`

Manipulation planning target:

- `ps.setTargetState(stateId)` sets the goal to “any configuration in graph state `stateId`”.

---

## 3. Graph: Constraint Graph for Manipulation

The manipulation CORBA stack provides a **constraint-graph** service hosted by `hpp-manipulation-corba`.

Main wrapper:

- `hpp.corbaserver.manipulation.ConstraintGraph` (client-side convenience wrapper)

It keeps local name→ID caches:

- `graph.nodes[name] -> nodeId`
- `graph.edges[name] -> edgeId`

The underlying CORBA stubs are also accessible for advanced use:

- `graph.client` (manipulation problem service)
- `graph.clientBasic` (core/basic problem service)
- `graph.graph` (graph service)

### 3.1 Graph lifecycle

Create a new server-side graph:

```python
from hpp.corbaserver.manipulation import Robot, ConstraintGraph

robot = Robot("ur5", "anchor")
graph = ConstraintGraph(robot, "manipulation_graph")
```

Attach to an existing graph on the server:

- `ConstraintGraph(robot, name, makeGraph=False)` attempts to fetch the current graph and populate `graph.nodes` / `graph.edges`.

Finalize / initialize:

- `graph.initialize()` calls the server-side graph initialization.

One-shot automatic builder:

- `ConstraintGraph.buildGenericGraph(robot, name, grippers, objects, handlesPerObjects, shapesPerObjects, envNames, rules=[])` creates, fetches, and initializes a graph.

### 3.2 States (nodes)

Create one or multiple nodes:

- `graph.createNode(node, waypoint=False, priority=None)`

Notes:

- `node` can be a string or a list of strings.
- `waypoint=True` creates waypoint states.
- `priority` orders the states: when the server tries to classify a configuration into a node, it checks nodes in priority/creation order.

Example:

```python
graph.createNode(["free", "grasp"], waypoint=False, priority=[0, 1])
graph.createNode(["w0"], waypoint=True)
```

### 3.3 Transitions (edges)

Create a standard edge:

- `graph.createEdge(nodeFrom, nodeTo, name, weight=1, isInNode=None)`

About `weight`:

- It controls edge selection probability among outgoing edges of a node.
- Setting `weight=0` creates an edge that is **not selected** by M-RRT but can still be valid for other operations.

Containing node:

- `graph.setContainingNode(edge, node)`
- `graph.getContainingNode(edge)`

“Short” edges:

- `graph.setShort(edge, isShort)`
- `graph.isShort(edge)`

Example:

```python
graph.createEdge("free", "grasp", "take", weight=1)
graph.setContainingNode("take", "grasp")
graph.setShort("take", True)
```

### 3.4 Waypoint edges

Create a waypoint edge:

- `graph.createWaypointEdge(nodeFrom, nodeTo, name, nb=1, weight=1, isInNode=None, automaticBuilder=True)`

Notes:

- With `automaticBuilder=True` (default), the wrapper creates additional waypoint nodes/edges with generated names.
- With `automaticBuilder=False`, it only creates the server-side WaypointEdge; wiring individual waypoint segments is typically done through low-level CORBA calls (`graph.graph.setWaypoint(...)`) or via `ConstraintGraphFactory`.

### 3.5 Level-set edges and foliation

Create a level-set edge:

- `graph.createLevelSetEdge(nodeFrom, nodeTo, name, weight=1, isInNode=None)`

Attach foliation constraints:

- `graph.addLevelSetFoliation(edge, condGrasps=None, condPregrasps=None, condNC=[], condLJ=[], paramGrasps=None, paramPregrasps=None, paramNC=[], paramLJ=[])`

Important notes:

- `condLJ` and `paramLJ` are rejected by the wrapper: locked joints are handled as **numerical constraints** in this API.
- Grasp / pregrasp names are expanded into the corresponding numerical constraints created by `graph.createGrasp` / `graph.createPreGrasp`.

### 3.6 Grasp / pregrasp constraints registration

Create grasp constraints (stored on the server by name):

- `graph.createGrasp(name, gripper, handle)`
- `graph.createPreGrasp(name, gripper, handle)`

These create named numerical constraints on the server; they are later referenced by name via the `Constraints` container and `graph.addConstraints(...)`.

### 3.7 Attaching constraints to graph elements

Attach constraints to:

- the whole graph: `graph.addConstraints(graph=True, constraints=...)`
- a node: `graph.addConstraints(node="nodeName", constraints=...)`
- an edge: `graph.addConstraints(edge="edgeName", constraints=...)`

The `constraints` argument must be an instance of `hpp.corbaserver.manipulation.Constraints`.

For nodes, the wrapper applies constraints to both the node and the “path constraints for the node” (server side):

- `graph.graph.addNumericalConstraints(nodeId, nc)`
- `graph.graph.addNumericalConstraintsForPath(nodeId, nc)`

### 3.8 Visualization and introspection

DOT/PDF rendering helper:

- `graph.display(dotOut="/tmp/constraintgraph.dot", pdfOut="/tmp/constraintgraph", format="pdf", open=True)`

Optional text→TeX replacement used by the display helper:

- `graph.addTextToTeXTranslation(text, tex)`
- `graph.setTextToTeXTranslation(textToTex: dict)`

Connectivity and membership queries:

- `graph.getNodesConnectedByEdge(edge)`
- `graph.getNode(config)` returns the name of the node the configuration belongs to (or raises if the ID is unknown locally).

Constraint inspection strings:

- `graph.displayNodeConstraints(node)`
- `graph.displayEdgeConstraints(edge)`
- `graph.displayEdgeTargetConstraints(edge)`

### 3.9 Applying constraints and generating configurations

Projection / constraint application utilities:

- `graph.applyNodeConstraints(node, input)` → `(output, error)`
- `graph.applyEdgeLeafConstraints(edge, qfrom, input)` → `(output, error)`
- `graph.generateTargetConfig(edge, qfrom, input)` → `(output, error)`

Building and projecting a path along a specific edge:

- `graph.buildAndProjectPath(edge, qb, qe)` → `(success, indexNotProj, indexProj)`

Notes:

- No path validation is performed by `buildAndProjectPath`.
- Returned indices are IDs in the server-side “path vector” (usable with `ProblemSolver.configAtParam(...)`).

### 3.10 Constraint error diagnostics

- `graph.getConfigErrorForNode(node, config)` → `(error, isSatisfied)`
- `graph.getConfigErrorForEdge(edge, config)` → `(error, isSatisfied)`
- `graph.getConfigErrorForEdgeLeaf(edge, leafConfig, config)` → `(error, isSatisfied)`
- `graph.getConfigErrorForEdgeTarget(edge, leafConfig, config)` → `(error, isSatisfied)`

### 3.11 Collision validation tuning along edges

Remove a collision pair for a specific edge:

- `graph.removeCollisionPairFromEdge(edge, joint1, joint2)`

Per-edge security margins (path validation margins):

- `graph.setSecurityMarginForEdge(edge, joint1, joint2, margin)`
- `graph.getSecurityMarginMatrixForEdge(edge)`

### 3.12 High-level graph generation: ConstraintGraphFactory

Entry point: `hpp.corbaserver.manipulation.ConstraintGraphFactory`.

This is the classic CORBA-side way to generate large manipulation graphs by exploring the combinatorics of (gripper, handle) associations.

Main API:

- `factory = ConstraintGraphFactory(graph)`
- `factory.setGrippers(grippers)`
- `factory.setObjects(objects, handlesPerObjects, contactsPerObjects)`
- `factory.environmentContacts(envContacts)`
- `factory.setRules(rules)` where `rules` is a list of `hpp.corbaserver.manipulation.Rule` (IDL type)
- `factory.setPossibleGrasps(graspsDict)` (simple allow-list by gripper)
- `factory.setPreplacementDistance(obj, distance)` / `factory.getPreplacementDistance(obj)`
- `factory.setPreplaceGuide(preplaceGuide)`
- `factory.generate()`

The factory creates nodes/edges and also creates/attaches grasp, pregrasp, placement, and preplacement constraints using the server-side manipulation services.

---

## 4. Constraints: Geometric Constraints

In the CORBA API, constraints are usually created and stored server-side by **name**.
Most calls create named constraints in a registry on the server and later refer to them by string.

### 4.1 Creating constraints (core)

Common constructors are exposed as `ProblemSolver` helpers:

- `ps.createTransformationConstraint(name, joint1Name, joint2Name, ref, mask)`
- `ps.createOrientationConstraint(name, joint1Name, joint2Name, quat, mask)`
- `ps.createPositionConstraint(name, joint1Name, joint2Name, p1, p2, mask)`
- `ps.createRelativeComConstraint(name, comName, jointName, point, mask)`
- `ps.createLockedJoint(name, jointName, value, comp=None)`

Parameter conventions (as used by the CORBA wrappers):

- Poses are typically `[x, y, z, qx, qy, qz, qw]`.
- `mask` is a 6-length list/tuple (constraining translation/rotation components) with 0/1 entries.
- Locked joint values are passed as a list (joint config subvector).

Example:

```python
# World-to-joint transform constraint (joint1Name="" means world)
ps.createTransformationConstraint(
    "placement",
    "",
    "object/root_joint",
    [0, 0, 0, 0, 0, 0, 1],
    [0, 0, 1, 1, 1, 0],
)

ps.createLockedJoint("lock_elbow", "elbow_joint", [1.57])
```

### 4.2 Adding constraints to the config projector

After creating constraints, you typically insert them into the active projector:

```python
ps.addNumericalConstraints("proj", ["placement"], priorities=[0])
ps.addLockedJointConstraints("proj", ["lock_elbow"])
```

Right-hand side management:

- `ps.setConstantRightHandSide(constraintName, True/False)`
- `ps.setRightHandSide(rhs)`
- `ps.setRightHandSideByName(constraintName, rhs)`
- `ps.setRightHandSideFromConfig(q)`

Notes:

- The “active projector” is the one selected/created in the server for the current problem.
- Right-hand side handling is essential for constraints that depend on a reference configuration (e.g., relative transforms).

### 4.3 Applying constraints

```python
q_proj = ps.applyConstraints(q)
q_opt, residual = ps.optimize(q)
```

### 4.4 Manipulation constraint sets: `hpp.corbaserver.manipulation.Constraints`

Manipulation graph APIs use a dedicated container type:

- `hpp.corbaserver.manipulation.Constraints`

It stores three sets of names:

- `grasps`: names registered via `ConstraintGraph.createGrasp(name, ...)`
- `pregrasps`: names registered via `ConstraintGraph.createPreGrasp(name, ...)`
- `numConstraints`: names of “plain” numerical constraints (including what used to be locked joints)

Construction and set algebra:

- `Constraints(grasps=[], pregrasps=[], numConstraints=[], lockedJoints=[])`
- Supports `+`, `-`, `+=`, `-=` to union/difference constraint sets.
- `lockedJoints` is accepted but **deprecated**; elements are merged into `numConstraints`.

Useful helpers:

- `c.empty()`
- `c.grasps`, `c.pregrasps`, `c.numConstraints` (properties returning lists)

### 4.5 How graph constraint attachment expands grasps/pregrasps

When you call `ConstraintGraph.addConstraints(...)`, the wrapper expands:

- each grasp name into one or more numerical constraint names (created on the server by `createGrasp`),
- each pregrasp name similarly.

Special case handled by the wrapper:

- When adding grasp constraints to an **edge**, it adds the corresponding `"/complement"` constraint instead of the main grasp constraint.

This behavior matches the wrapper code and is important to reproduce expected manipulation-graph semantics.

### 4.6 Rules and possible grasps (factory-side filtering)

Graph generation typically uses `ConstraintGraphFactory` filters:

- `hpp.corbaserver.manipulation.Rule` is an IDL type imported from `hpp_idl.hpp.corbaserver.manipulation`.
- `factory.setRules([Rule(...), ...])` supports regular expressions for matching grippers/handles.
- `factory.setPossibleGrasps({"gripper": ["object/handle", ...], ...})` provides a simple allow-list.

### 4.7 Security margins helper

Module: `hpp.corbaserver.manipulation.security_margins.SecurityMargins`.

This helper computes and applies per-edge collision margins by calling `ConstraintGraph.setSecurityMarginForEdge(...)`, taking into account grasp and placement constraints detected along each edge.

Typical usage pattern:

```python
from hpp.corbaserver.manipulation import ConstraintGraphFactory
from hpp.corbaserver.manipulation.security_margins import SecurityMargins

factory = ConstraintGraphFactory(graph)
# ... configure factory and generate graph ...

sm = SecurityMargins(ps, factory, robotsAndObjects=["robot", "object"])
sm.setSecurityMarginBetween("robot", "object", 0.01)
sm.apply()
```

---

## 5. Planning and Path Manipulation

In CORBA, planned paths are stored server-side and referenced by **path IDs** (`int`).

The primary interface is:

- `hpp.corbaserver.ProblemSolver` (core)
- `hpp.corbaserver.manipulation.ProblemSolver` (adds manipulation-specific helpers like `setTargetState`)

### 5.1 Solving

```python
ok = ps.solve()
```

Step-by-step solving is also available:

- `ps.prepareSolveStepByStep()`
- `ps.executeOneStep()`
- `ps.finishSolveStepByStep()`

Interrupting a solve (effective only with multi-thread policy on the server):

- `ps.interruptPathPlanning()`

### 5.2 Direct path utility

```python
success, path_id, report = ps.directPath(q1, q2, validate=True)
```

Notes:

- If `validate=True`, only the collision-free prefix may be stored as a path on the server.
- `report` is a server-generated string that may explain why validation failed.

### 5.3 Path storage and queries

- `ps.numberPaths()`
- `ps.pathLength(pathId)`
- `ps.configAtParam(pathId, s)`
- `ps.derivativeAtParam(pathId, order, s)`
- `ps.getWaypoints(pathId)`

Path IDs refer to the current problem’s server-side path vector.

Path editing:

- `ps.appendDirectPath(pathId, q_end, validate)`
- `ps.concatenatePath(startId, endId)`
- `ps.extractPath(pathId, s0, s1)`
- `ps.erasePath(pathId)`
- `ps.projectPath(pathId)`

### 5.4 Path optimizers

Register and run optimizers:

```python
ps.clearPathOptimizers()
ps.addPathOptimizer("RandomShortcut")
ps.addPathOptimizer("PartialShortcut")

out_id = ps.optimizePath(path_id)
```

#### Available Path Optimizers

**Built-in (hpp-core):**

| Name | Description |
|------|-------------|
| `"RandomShortcut"` | Random shortcut optimizer - tries random shortcuts between configurations |
| `"SimpleShortcut"` | Simple shortcut optimizer |
| `"PartialShortcut"` | Partial shortcut optimizer - shortcuts that preserve some path structure |
| `"SimpleTimeParameterization"` | Adds time parameterization to paths |
| `"RSTimeParameterization"` | Reeds-Shepp time parameterization |

**Manipulation-specific (hpp-manipulation):**

| Name | Description |
|------|-------------|
| `"RandomShortcut"` | Manipulation-aware random shortcut (overrides core version) |
| `"Graph-RandomShortcut"` | Graph-aware random shortcut for constraint graphs |
| `"PartialShortcut"` | Manipulation-aware partial shortcut |
| `"Graph-PartialShortcut"` | Graph-aware partial shortcut for constraint graphs |
| `"EnforceTransitionSemantic"` | Enforces transition semantics in manipulation graphs |

**Plugin-based (require `ps.loadPlugin()`):**

| Name | Plugin | Description |
|------|--------|-------------|
| `"SplineGradientBased_bezier1"` | `spline-gradient-based.so` | Spline optimization with Bezier basis, order 1 |
| `"SplineGradientBased_bezier3"` | `spline-gradient-based.so` | Spline optimization with Bezier basis, order 3 |
| `"SplineGradientBased_bezier5"` | `spline-gradient-based.so` | Spline optimization with Bezier basis, order 5 |
| `"TOPPRA"` | `toppra.so` | Time-optimal path parameterization |

Example with plugin:

```python
# Load plugin first
ps.loadPlugin("spline-gradient-based.so")

# Then use the optimizer
ps.clearPathOptimizers()
ps.addPathOptimizer("SplineGradientBased_bezier3")
out_id = ps.optimizePath(path_id)
```

### 5.5 Roadmap inspection (debugging)

These methods expose the roadmap produced during planning:

- `ps.nodes()`
- `ps.node(nodeId)`
- `ps.numberNodes()`
- `ps.numberEdges()`
- `ps.edge(edgeId)`

### 5.6 Manipulation-specific planning with constraint graphs

Targeting a graph state (manipulation only):

- `ps.setTargetState(stateId)` makes the goal “any configuration in the given graph state”.

Selecting constraints from a graph element:

- `graph.setProblemConstraints(name, target)` sets the problem constraints to a node or edge.
  - If `name` is a node: `target` is ignored.
  - If `name` is an edge: `target=True` uses the edge target constraints, `target=False` uses the edge path constraints.

Edge-based helpers:

- `graph.generateTargetConfig(edge, qfrom, input)`
- `graph.buildAndProjectPath(edge, qb, qe)`

These are useful when debugging or when you want to explicitly force a specific transition rather than relying on the planner to pick edges.

#### 5.6.1 ManipulationPlanner vs TransitionPlanner (conceptual comparison)

At the C++ level, HPP-Manipulation distinguishes:

- **Global graph planning** (`hpp::manipulation::ManipulationPlanner`): explores the whole constraint graph, automatically choosing transitions, growing a roadmap until init and goal connect.
- **Transition-level planning** (`hpp::manipulation::pathPlanner::TransitionPlanner`): plans for one *selected* transition (edge) by configuring an inner problem with that edge’s constraints/steering/validation.

In the CORBA Python API, you most often interact with the *global* planner via:

```python
ok = ps.solve()
```

To get **transition-level behavior** in CORBA (edge-focused, controlled transition), you typically emulate what `TransitionPlanner` does by explicitly choosing and using one edge:

1) Pick the edge you want (by name or by inspecting the graph)
2) Generate a target config consistent with the edge leaf:

```python
success, q_target, residual = graph.generateTargetConfig(edge, q_from, q_seed)
```

3) Build a constrained path for that edge and project it:

```python
success, path_id, report = graph.buildAndProjectPath(edge, q_from, q_target)
```

4) Optionally validate / post-process the stored path (server-side):

- `ps.projectPath(path_id)`
- `ps.optimizePath(path_id)`

Key difference to keep in mind:

- Global planning (`ps.solve`) decides which transitions to attempt.
- Edge-focused planning (using `graph.generateTargetConfig` + `graph.buildAndProjectPath`) keeps the discrete transition under your control and is ideal for debugging a specific manipulation transition.

#### 5.6.2 Using `TransitionPlanner` from CORBA (edge-focused planning)

The CORBA stack exposes a transition-level planner (C++ `hpp::manipulation::pathPlanner::TransitionPlanner`) that you can use when you want to:

- force planning along a specific graph edge (no automatic edge choice),
- validate or debug a single transition,
- build a longer motion by chaining multiple edge-local paths.

There are two common construction patterns in existing AGIMUS demos:

**Pattern A: create a manipulation TransitionPlanner (simple)**

```python
tp = ps.client.manipulation.problem.createTransitionPlanner()
```

This returns a CORBA object with TransitionPlanner methods (set edge, planPath, validateConfiguration, …).

**Pattern B: create a TransitionPlanner via the generic path-planner factory (advanced)**

Some code creates a dedicated roadmap and then instantiates the planner with:

```python
tp = ps.client.basic.problem.createPathPlanner(
    "TransitionPlanner",
    cproblem,
    croadmap,
)
```

This is useful when you want a custom `Roadmap` / `Distance` object.

##### Minimal configuration

In practice, you should set *both* timeout and maxIterations on the TransitionPlanner (some demos note this avoids crashes / undefined behavior):

```python
tp.timeOut(3.0)
tp.maxIterations(3000)
tp.setPathProjector("Progressive", 0.2)
```

You can also attach path optimizers to the transition context:

```python
tp.addPathOptimizer("EnforceTransitionSemantic")
tp.addPathOptimizer("RandomShortcut")

# If using plugin optimizers, make sure the plugin is loaded in the server first.
tp.addPathOptimizer("SplineGradientBased_bezier3")
```

##### Selecting which transition to plan

TransitionPlanner plans along **one selected edge**.

In CORBA manipulation graphs, edges are commonly referenced by string name and mapped to an integer ID via `graph.edges[...]`:

```python
edge_id = graph.edges["Loop | f"]
tp.setEdge(edge_id)
```

##### Planning APIs you typically use

- `tp.planPath(q_start, [q_goal], resetRoadmap)`
  - returns a path object when successful.
  - `resetRoadmap=True` is common when you want repeatable, edge-local planning.

- `tp.directPath(q1, q2, validate)`
  - returns `(path, success, msg)`.
  - if it succeeds, you often time-parameterize explicitly (some demos do this because directPath may not apply time-parameterization automatically).

- `tp.timeParameterization(pathVector)`
  - returns a time-parameterized path vector.

- `tp.validateConfiguration(q, edge_id)`
  - returns `(ok, msg)` and is useful to quickly check whether a generated target config is valid for a given edge.

##### Typical “edge-focused” workflow

1) Generate a target configuration consistent with the edge leaf:

```python
success, q_target, residual = graph.generateTargetConfig(edge_name_or_id, q_leaf, q_seed)
```

2) Optionally validate the configuration in the transition context:

```python
ok, msg = tp.validateConfiguration(q_target, graph.edges[edge_name])
```

3) Plan locally along that edge:

```python
tp.setEdge(graph.edges[edge_name])
path = tp.planPath(q_leaf, [q_target], True)
```

4) If you use `directPath`, explicitly time-parameterize the returned vector when needed:

```python
p, ok, msg = tp.directPath(q1, q2, True)
pv = p.asVector()
pv_time = tp.timeParameterization(pv)
```

Note on object lifetime: many CORBA path objects expose `deleteThis()`; many demos use `hpp.corbaserver.wrap_delete` to avoid leaks when handling lots of temporary paths.

---

## 6. Visualization

The CORBA visualization stack is `gepetto.corbaserver`.

### 6.1 Connecting to Gepetto GUI

```python
from gepetto.corbaserver import gui_client, start_server

start_server()

gui = gui_client(window_name="window")
```

Or explicitly:

```python
from gepetto.corbaserver import Client

client = Client()
gui = client.gui
gui.createWindow("window")
```

### 6.2 Drawing helpers

The module [hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/tools.py](hpp/src/gepetto-viewer-corba/src/gepetto/corbaserver/tools.py) provides helpers like `Vector6`, `Linear`, `Angular`.

---

## Migration from CORBA

If you are migrating to PyHPP:

- Replace string-based selection (`selectPathPlanner("BiRRTPlanner")`) by constructing bound objects and setting them on a `pyhpp.core.Problem` / `pyhpp.core.ProblemSolver`.
- Replace string constraint names by Python-wrapped constraint objects (`Transformation`, `Implicit`, etc.) and direct method calls.

See also: [hpp/src/agimus_spacelab/doc/comparison_corba_vs_pyhpp.md](hpp/src/agimus_spacelab/doc/comparison_corba_vs_pyhpp.md)

---

## Troubleshooting

### 1) Cannot connect to CORBA server

- Check the server is running and NameService is reachable.
- Try passing a direct `url` to `hpp.corbaserver.Client(url=...)`.
- Check environment variables: `HPP_HOST`, `HPP_PORT`.

### 2) Gepetto connection fails

- Start `gepetto-gui` (or call `gepetto.corbaserver.start_server()`).
- Check environment variables: `GEPETTO_VIEWER_HOST`, `GEPETTO_VIEWER_PORT`.
- If running remotely, use `url = "corbaloc:iiop:<host>:<port>/NameService"`.

### 3) “incompatible function arguments” / serialization issues

- CORBA interfaces expect basic types (lists/tuples of floats, strings, ints).
- Ensure configuration vectors have the right size (`robot.getConfigSize()`).

