# HPP-Manipulation: Complete Motion Plan

This page is a Sphinx-friendly wrapper around the original Markdown document whose filename contains characters that can be awkward for static doc builders.

```{include} HPP-Manipulation: Complete Motion Plan.md
```
# HPP-Manipulation: Complete Motion Planning Architecture Analysis

## 1. **Package Overview**

HPP-Manipulation extends HPP-Core to solve **manipulation planning problems** where robots must:
- Grasp and place objects
- Maintain contacts with surfaces
- Switch between different contact modes (grasped/placed/free)
- Navigate through state spaces defined by constraints

**Key Innovation**: Uses a **Constraint Graph** to encode discrete contact states and allowable transitions, making manipulation planning tractable.

---

## 2. **Core Architecture Components**

### 2.1 **Problem Formulation** (problem.hh, `problem.cc`)

```
Problem
├── Inherits from core::Problem
├── Adds: constraintGraph_ (the manipulation graph)
├── Manages: PathValidation, SteeringMethod
└── Validates problem setup before solving
```

**Key Methods:**
- `constraintGraph()`: Set/get the constraint graph defining manipulation modes
- `pathValidationFactory()`: Creates validation for constrained paths
- `checkProblem()`: Validates initial/goal configs satisfy constraints

### 2.2 **Constraint Graph** (graph.hh, `graph/graph.cc`)

The **heart of the formulation**. A directed graph where:

**Nodes (States)**: Represent *contact modes*
- Example: "Object on table", "Robot grasping object"
- Each state has **configuration constraints** that must be satisfied
- Implemented in state.hh

**Edges (Transitions)**: Represent *motion primitives* between states
- Example: "Approach object", "Lift object"
- Each edge has:
  - **Path constraints**: Must hold along the entire path
  - **Target constraints**: Must hold at path end (for leaf foliation)
- Implemented in edge.hh

**Graph Structure:**
```text
Graph
├── StateSelector (determines which state a configuration belongs to)
│   └── States (contact modes)
│       └── Edges (transitions between states)
│           ├── pathConstraint() - constraints along path
│           ├── targetConstraint() - constraints at target
│           └── steeringMethod() - how to connect configs
└── ConstraintsAndComplements (registered constraint pairs)
```

**Key Concept - Foliation:**
The graph defines a *foliation* of the configuration space:
- Each edge represents a manifold (constraint surface)
- The manifold is *foliated* into leaves by the right-hand side of constraints
- Example: "Object grasped" edge - each leaf corresponds to a fixed object position

---

## 3. **Planning Algorithm** (manipulation-planner.hh, manipulation-planner.cc)

### 3.1 **Main Planning Loop** (`oneStep()` method)

```
1. Sample random configuration q_rand
2. For each connected component in roadmap:
   3. For each graph state:
      4. Find nearest node n_near in that state
      5. Extend from n_near toward q_rand:
         a. Choose edge from constraint graph
         b. Generate target config in edge's leaf
         c. Build path using edge's steering method
         d. Project path onto edge constraints
         e. Validate collision-free
      6. If successful, add new node and edges to roadmap
7. Try to connect new nodes together
8. Try to connect new nodes to existing roadmap
```

### 3.2 **Extension Strategy** (`extend()` method)

**Critical steps:**

1. **Edge Selection** (`graph->chooseEdge(n_near)`):
   - Selects outgoing edge from state of n_near
   - Uses probabilistic weights (higher weight = more likely)
   - Stored in `State::neighbors_` weighted list

2. **Target Generation** (`edge->generateTargetConfig()`):
   ```cpp
   // Projects q_rand onto the TARGET state constraint
   // while respecting the edge's path constraint right-hand side
   bool generateTargetConfig(core::NodePtr_t nStart, 
                            ConfigurationOut_t q) const;
   ```
   - Sets constraint RHS from nStart configuration
   - Projects q_rand onto target state manifold
   - Ensures result is reachable via this edge

3. **Path Building** (`edge->build(path, q_near, qProj_)`):
   - Uses edge-specific steering method
   - Creates path in configuration space

4. **Path Projection** (if pathProjector exists):
   - Projects path onto edge constraints
   - Handles constraint drift during interpolation
   - May shorten path if projection fails

5. **Path Validation**:
   - Checks collision-free
   - Validates constraints satisfied along path
   - Returns longest valid sub-path

### 3.3 **Success Tracking**

```cpp
enum TypeOfFailure {
  PATH_PROJECTION_SHORTER = 0,  // Projection reduced path length
  PATH_VALIDATION_SHORTER = 1,  // Collision shortened path
  REACHED_DESTINATION_NODE = 2, // Success!
  FAILURE = 3,                  // Generic failure
  PROJECTION = 4,               // Target projection failed
  STEERING_METHOD = 5,          // Steering failed
  PATH_VALIDATION_ZERO = 6,     // No valid length
  PATH_PROJECTION_ZERO = 7      // Projection completely failed
};
```

Statistics tracked per edge to identify bottlenecks.

### 3.4 **ManipulationPlanner vs TransitionPlanner** (global vs local planning)

HPP-Manipulation provides two complementary planning styles:

- `ManipulationPlanner` is a **global** planner: it explores the *constraint graph* and grows a roadmap across many states/edges until the init and goal are connected.
- `TransitionPlanner` is a **local / transition-level** planner: it plans **only for a chosen transition edge** (or a specific pair of states), typically to debug or to force a particular discrete transition.

#### 3.4.1 Scope and contract

| Aspect | `ManipulationPlanner` | `TransitionPlanner` |
|---|---|---|
| Discrete structure | Chooses edges automatically via `graph->chooseEdge(...)` and explores many state/edge combinations | Requires an edge (transition) to be selected (e.g. via `setEdge(...)`) |
| Continuous manifold | Uses the edge’s **path constraint** manifold and its foliation leaves implicitly during expansion | Plans on the **selected edge** manifold (edge path constraint + edge validation + edge steering) |
| Goal handling | Goal is usually “reach a goal configuration / goal state” in the full manipulation problem | Goals must satisfy the **same leaf** (right-hand side) induced by the start configuration (otherwise it rejects them) |
| Output | A path (often a `PathVector`) that may traverse many graph transitions | A path (`PathVector`) for that single transition/edge context |

#### 3.4.2 How constraints and foliation are handled

- `ManipulationPlanner`:
   - Picks an outgoing edge from the current node’s state.
   - Uses `edge->generateTargetConfig(...)` to project a random sample onto a reachable target configuration **consistent with the edge leaf**.
   - Builds a path with `edge->build(...)`, then optionally projects it and validates it.
   - The roadmap is partitioned by *graph state* and *leaf connected components* (two configs in the same state but different leaves generally cannot be connected within that state).

- `TransitionPlanner`:
   - Takes an edge and sets the inner problem to match it: path constraints, path validation, steering method.
   - Initializes the constraint right-hand side from the start configuration (effectively selecting the leaf).
   - Rejects goal configurations that do not satisfy the same leaf constraint.
   - Can optionally reset the roadmap for each call (useful for deterministic debugging).

#### 3.4.3 When to use which

- Prefer `ManipulationPlanner` when you want an **end-to-end manipulation plan** and you are OK with the planner choosing which discrete transitions to attempt.
- Prefer `TransitionPlanner` when you already know the discrete transition you want (or you want to test a single edge), and you need a **repeatable, edge-focused** plan for debugging, benchmarking, or building higher-level orchestration.

---

## 4. **Constraint Management**

### 4.1 **Handle and Gripper** (`handle.hh`, `handle.cc`)

**Handle**: Grasp location on an object
```cpp
class Handle {
  Transform3s localPosition_;        // Pose in object frame
  vector3_t approachingDirection_;   // Pre-grasp approach vector
  std::vector<bool> mask_;           // Which DOFs are constrained
  std::vector<bool> maskComp_;       // Complement mask
  
  // Creates grasp constraint: gripper must match handle pose
  ImplicitPtr_t createGrasp(GripperPtr_t gripper);
  
  // Creates pre-grasp: gripper offset along approach direction
  ImplicitPtr_t createPreGrasp(GripperPtr_t gripper, value_type shift);
  
  // Creates complement: free DOFs during grasp
  ImplicitPtr_t createGraspComplement(GripperPtr_t gripper);
};
```

**Key Method - `createPreGrasp()`:**
```cpp
Transform3s M = gripper->objectPositionInJoint() *
                Transform3s(I3, shift*approachingDirection_);
```
Offsets gripper by `shift` distance along handle's approach vector.

### 4.2 **Constraint Registration**

```cpp
graph->registerConstraints(constraint, complement, both);
```

Registers constraint triples:
- **constraint**: Main grasp/placement constraint
- **complement**: Free DOFs (e.g., sliding on surface)
- **both**: Explicit combined constraint (more efficient)

During graph construction, when both constraint and complement appear together, they're automatically replaced by the explicit "both" version.

---

## 5. **Roadmap Structure** (roadmap.hh, `roadmap.cc`)

### 5.1 **Specialized Roadmap Node** (`roadmap-node.hh`)

```cpp
class RoadmapNode : public core::Node {
  graph::StatePtr_t graphState_;  // Which graph state this config is in
  LeafConnectedCompPtr_t leafCC_; // Leaf connected component
};
```

### 5.2 **Leaf Connected Components**

**Innovation**: Roadmap is partitioned not just by connectivity, but by **graph state AND constraint leaf**.

```
ConnectedComponent (standard roadmap connectivity)
└── LeafConnectedComp (nodes in same state + same constraint leaf)
    └── Nodes in same foliation leaf
```

**Purpose**: Two nodes in same state but different leaves cannot be connected by a simple path - they require a transition through other states.

### 5.3 **Nearest Neighbor by State**

```cpp
RoadmapNodePtr_t nearestNodeInState(
  ConfigurationIn_t configuration,
  const ConnectedComponentPtr_t& connectedComponent,
  const graph::StatePtr_t& state,
  value_type& minDistance) const;
```

Finds nearest neighbor **restricted to a specific graph state**. Essential for the planning loop.

---

## 6. **Path Validation** (`graph-path-validation.hh`)

**GraphPathValidation**: Validates paths respect constraint graph transitions.

```cpp
bool validate(const PathPtr_t& path, bool reverse,
              PathPtr_t& validPart, PathValidationReportPtr_t& report);
```

Checks:
1. Path starts in valid state
2. Path follows valid edge in graph
3. Constraints satisfied along path
4. No collisions

---

## 7. **Helper Functions for Graph Construction** (helper.hh)

Provides utilities to build common graph patterns:

### 7.1 **Foliated Manifold**

```cpp
struct FoliatedManifold {
  NumericalConstraints_t nc;      // Manifold constraints
  LockedJoints_t lj;              // Locked joints
  NumericalConstraints_t nc_path; // Path constraints
  NumericalConstraints_t nc_fol;  // Foliation definition
  LockedJoints_t lj_fol;          // Foliation locked joints
};
```

Encapsulates both the manifold definition and its foliation structure.

### 7.2 **Edge Creation Templates**

```cpp
template <int gCase>
Edges_t createEdges(
  const StatePtr_t& from, const StatePtr_t& to,
  const FoliatedManifold& grasp,
  const FoliatedManifold& pregrasp,
  const FoliatedManifold& place,
  const FoliatedManifold& preplace,
  ...);
```

Automatically creates forward/backward edges with appropriate waypoints for:
- Grasp-only transitions
- Grasp with pre-grasp
- Placement transitions
- Combined grasp and placement

---

## 8. **Problem Solver Interface** (problem-solver.hh)

High-level API for users:

```cpp
class ProblemSolver : public core::ProblemSolver {
  // Setup
  void constraintGraph(const std::string& graph);
  void initConstraintGraph();
  
  // Constraint creation
  void createGraspConstraint(name, gripper, handle);
  void createPreGraspConstraint(name, gripper, handle);
  void createPlacementConstraint(name, surfaces...);
  void createPrePlacementConstraint(name, surfaces...);
  
  // Planning
  void resetProblem();
  void resetRoadmap();
  void setTargetState(graph::StatePtr_t state);
};
```

---

## 9. **Complete Planning Workflow**

### User Workflow:

```python
# 1. Setup
ps = ProblemSolver()
ps.robot(manipulationRobot)

# 2. Define constraints
ps.createGraspConstraint("grasp", "gripper", "handle")
ps.createPlacementConstraint("placement", surfaces1, surfaces2)

# 3. Build graph
graph = ps.constraintGraph("myGraph")
graph.createNode("free")
graph.createNode("grasping")
graph.createEdge("free", "grasping", "approach", weight=1)

# 4. Initialize
ps.initConstraintGraph()

# 5. Set problem
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

# 6. Solve
ps.solve()
paths = ps.getPaths()
```

### Internal Workflow:

```
1. ManipulationPlanner::oneStep() called repeatedly
   ↓
2. For each connected component + state pair:
   - Find nearest node in that (CC, state)
   - Attempt extension:
     ↓
3. Extension process:
   a. Choose outgoing edge from state
   b. Generate target in edge's target state
   c. Build path using edge steering method
   d. Project path onto edge constraints
   e. Validate path (collision + constraints)
   ↓
4. If successful:
   - Add new node to roadmap
   - Create bidirectional edges
   - Update connected components
   - Update leaf connected components
   ↓
5. Connection attempts:
   - Try connecting new nodes to each other
   - Try connecting new nodes to existing roadmap
   ↓
6. Repeat until goal reached or timeout
```

---

## 10. **Key Algorithmic Insights**

### 10.1 **Why Constraint Graph Works**

**Problem**: Manipulation has hybrid discrete/continuous structure
- Discrete: Contact modes (grasping, placing, free)
- Continuous: Configuration space within each mode

**Solution**: 
- Graph explicitly encodes discrete structure
- Each node/edge defines a continuous manifold
- Planner explores both discrete (state transitions) and continuous (configurations) simultaneously

### 10.2 **Foliation for Manipulation**

**Concept**: Each edge defines not one manifold, but a *family* of manifolds (leaves)

Example: "Grasp" edge
- Manifold: Gripper-handle relative pose fixed
- Leaves: Different object positions in world
- Each leaf: Robot can move freely while maintaining grasp at that specific object position

**Benefits**:
- Reachability: Can only connect configs in same leaf via that edge
- Efficiency: Constraint RHS defines leaf membership
- Completeness: Exploring different leaves = exploring different object placements

### 10.3 **Extension vs Classical RRT**

**Classical RRT**:
```
extend(q_near, q_rand):
  path = steer(q_near, toward q_rand)
  if collision_free(path):
    add q_new to roadmap
```

**Manipulation RRT**:
```
extend(q_near, q_rand):
  edge = choose_edge_from_graph(q_near)
  q_target = project_onto_target_state(q_rand, edge)
  path = edge.steer(q_near, q_target)
  path = project_onto_constraints(path)
  if collision_free(path):
    add q_new to roadmap
```

**Key Difference**: Target is projected onto reachable manifold defined by graph edge, not directly toward random config.

---

## 11. **Optimization and Path Post-Processing**

### 11.1 **Graph Optimizer** (`graph-optimizer.hh`)

Optimizes paths while respecting constraint graph structure:
- Cannot optimize across different states
- Must maintain edge constraints
- Uses graph-aware path shortcutting

### 11.2 **Path Optimization Methods**

Located in `path-optimization/`:
- **SmallSteps**: Discretizes path into smaller segments
- **Keypoints**: Identifies critical configurations to preserve

---

## 12. **Statistics and Debugging** (`graph/statistics.hh`)

Tracks planning performance:
- Success/failure rates per edge
- Bottleneck identification
- Histogram of visited states

Helps users:
- Identify problematic transitions
- Tune edge weights
- Validate graph design

---

## Summary

**HPP-Manipulation formulates manipulation planning as exploration of a constraint graph:**

1. **States** = Contact modes with configuration constraints
2. **Edges** = Transitions with path constraints and steering methods  
3. **Planning** = RRT-like sampling guided by graph structure
4. **Foliation** = Enables efficient exploration of multi-modal spaces
5. **Leaf CCs** = Tracks connectivity within constraint manifolds

**The planner solves the problem by:**
- Sampling random configurations
- Projecting them onto reachable states (via graph edges)
- Building constrained paths between projected configs
- Maintaining a roadmap partitioned by state and leaf
- Connecting nodes until init and goal are in same component

This architecture makes manipulation planning tractable by **explicitly encoding the discrete structure** that classical planners struggle with.

---

## Appendix A. **ConstraintGraphFactory (automatic graph generation)**

The `ConstraintGraphFactory` builds a manipulation constraint graph *automatically* from:
- a list of **grippers** $G$,
- a list of **handles** $H$ (grouped per object),
- optional **placement contact surfaces** (per object) and **environment contacts**,
- optional **rules** or **possible grasps** restrictions.

### A.1 Contact-mode states (nodes)

The factory defines a discrete **grasp assignment vector** `grasps` of length $|G|$:
- `grasps[i] = None` means gripper $G_i$ is free,
- `grasps[i] = j` means gripper $G_i$ holds handle $H_j$.

Each *node/state* corresponds to one such assignment. The initial node is the “all free” assignment.

### A.2 Recursion and pruning

The graph topology is generated by recursion:
1. Start from “all free”.
2. Pick an available gripper and an available handle.
3. Create a new state by adding that grasp.
4. Create transitions between the previous state and the new state.
5. Recurse.

The recursion is *pruned* by validation callbacks:
- `setRules(...)` installs regexp-based link/unlink rules.
- `setPossibleGrasps({...})` restricts which (gripper, handle) pairs are allowed.

### A.3 Manifold vs foliation constraints

Internally, each state aggregates two sets of numerical constraints:
- **Manifold**: constraints that define which configurations belong to the state
   (e.g., active grasps + placements for objects not currently grasped).
- **Foliation**: constraints that parameterize motion on outgoing transitions
   (typically *complements* of the state’s placements/grasps).

This is how the same discrete state can represent a *family of continuous leaves*
(different object poses consistent with the same active contacts).

### A.4 Waypoint transitions (pregrasp / intersection / preplace)

When creating a transition that adds/removes a grasp, the factory may insert waypoint
states and build a “waypoint transition” composed of multiple segments:
- `..._pregrasp` if a pregrasp constraint exists,
- `..._intersec` if both grasp and placement constraints exist,
- `..._preplace` if a preplacement constraint exists.

Each waypoint state has its own manifold constraints, and each segment carries the
appropriate foliation/path constraints so that the motion stays on the intended
constrained manifold.

### A.5 Naming conventions (important for interoperability)

The factory uses fixed constraint names:
- Grasp: `"{gripper} grasps {handle}"`
- Grasp complement: add `"/complement"`
- Pregrasp: `"{gripper} pregrasps {handle}"`
- Placement: `"place_{object}"`
- Placement complement: add `"/complement"`
- Preplacement: `"preplace_{object}"`

### A.6 How `agimus_spacelab` uses the factory

In this repo, `agimus_spacelab.planning.graph.GraphBuilder` supports factory mode
for both CORBA and PyHPP:

```python
graph = GraphBuilder(planner, robot, ps, backend="pyhpp")
graph.build_graph_for_task(MyTaskConfig, mode="factory")
```

`agimus_spacelab.planning.constraints.FactoryConstraintLibrary` centralizes the
factory naming conventions so code can reliably reference factory-created
constraint names.