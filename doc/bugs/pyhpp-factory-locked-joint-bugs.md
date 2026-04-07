# Bug Report: PyHPP Factory Mode — LockedJoint Constraint Failures

**Package**: agimus_spacelab / hpp-python / hpp-manipulation  
**Component**: `PyHPPConstraintGraphFactory.buildPlacement()`, `GraphBuilder.create_factory_graph()`  
**Severity**: High — all PyHPP factory-mode planning fails silently  
**Affects**: PyHPP backend with `--factory` mode, no-contact placement objects

---

## Summary

Three interrelated bugs caused `generate_via_edge` to fail 100% of the time for grasp
edges in PyHPP factory mode. All three involve `LockedJoint` constraints created during
`buildPlacement()` for objects with no contact surfaces:

1. **Wrong comparison type** — `Equality` instead of `EqualToZero` triggers a
   use-after-free in `ExplicitConstraintSet::rightHandSideFromInput()`.
2. **Incorrect `ComparisonTypes` constructor call** — `ComparisonTypes(nv)` is not valid;
   the correct call is `ComparisonTypes()`.
3. **Robot configuration not set before `factory.generate()`** — `getJointConfig()` reads
   `currentConfiguration()`, which defaults to identity, locking the object at the origin
   instead of its actual starting position.

---

## Bug 1: Wrong Comparison Type (use-after-free)

### Problem

`buildPlacement()` in `constraint_graph_factory.py` created `LockedJoint` constraints
using the default constructor:

```python
self.registerConstraint(
    LockedJoint(self.graph.robot, n, np.array(q)),  # ← defaults to Equality
    n,
)
```

The default comparison type is `Equality`. This makes the `LockedJoint` a **foliation
constraint**. During `Edge::generateTargetConfig`, the planner calls
`proj->rightHandSideFromConfig` → `ExplicitConstraintSet::rightHandSideFromInput` →
`d.equalityIndices.lview/rview`, which triggers a use-after-free via buggy
`MatrixBlocks` iterators in `libhpp-constraints.so`.

### Observed Behavior

- **Residual**: consistently 0.1773906184320775 (not random — repeatable use-after-free
  pattern corrupting the projector state)
- **Failure rate**: 1000/1000 attempts
- **Edge**: `spacelab/g_ur10_tool > frame_gripper/h_FG_tool | f_01`
- **No crash** — the use-after-free is silent; the projector simply returns a wrong
  (non-zero) residual

### Root Cause

`EqualToZero` semantics: the constraint has a fixed right-hand side; `rightHandSideFromInput` is a no-op (equalityIndices is empty → no MatrixBlocks access → safe).

`Equality` semantics: the right-hand side is computed from the configuration at every
projection step, which traverses `equalityIndices` → triggers the buggy MatrixBlocks
use-after-free.

### Fix

Pass `EqualToZero` via a `ComparisonTypes` object to `LockedJoint`:

```python
comp = ComparisonTypes()
comp[:] = tuple([ComparisonType.EqualToZero] * nv)
self.registerConstraint(
    LockedJoint(self.graph.robot, n, np.array(q), comp),
    n,
)
```

**Files changed**:
- `src/pyhpp/manipulation/constraint_graph_factory.py` (HPP upstream)

---

## Bug 2: Invalid `ComparisonTypes` Constructor Call

### Problem

When implementing the `EqualToZero` fix above, the `ComparisonTypes` object was
constructed as:

```python
comp = ComparisonTypes(nv)   # ← WRONG: no int constructor exposed
```

This raises `Boost.Python.ArgumentError: ComparisonTypes.__init__(ComparisonTypes, int)`
and crashes the graph construction entirely.

### Root Cause

The Python binding for `ComparisonTypes` (a `std::vector<ComparisonType>`) is exposed via
`eigenpy::StdVectorPythonVisitor`. Its `__init__` accepts either no arguments (empty
vector) or another `ComparisonTypes` instance (copy); it does **not** accept an integer
size argument.

### Fix

```python
comp = ComparisonTypes()          # ← Correct: empty vector
comp[:] = tuple([ComparisonType.EqualToZero] * nv)
```

**Files changed**:
- `src/pyhpp/manipulation/constraint_graph_factory.py` (HPP upstream)

---

## Bug 3: Robot `currentConfiguration` Not Set Before `factory.generate()`

### Problem

`buildPlacement()` determines the value to lock each object joint at by reading:

```python
q = self.graph.robot.getJointConfig(joint_name)
```

`getJointConfig()` (C++ binding) reads `robot->currentConfiguration()`. At the time
`factory.generate()` is called, the robot's current configuration is the **default
identity** (`[0, 0, 0, 0, 0, 0, 1]` for an SE(3) joint), not the actual initial scene
configuration.

Result: the `LockedJoint` locks the object at the **origin**, not where it actually
starts. All IK attempts fail because the constraint forces the object to a position
inconsistent with `q_init`.

### Observed Behavior

- `robot.getJointConfig('frame_gripper/root_joint')` → `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]`
- `q_init[14:21]` → `[-0.133, 1.332, -1.213, 0.707, -0.707, ~0, ~0]`
- IK residual: 3.4–10.8 (large, non-repeatable — object is simply in the wrong place)
- Failure rate: 1000/1000 attempts (for any edge involving the locked object)

### Root Cause

`GraphBuilder.create_factory_graph()` called `self.factory.generate()` without first
setting the robot's current configuration. The `q_init` value computed in
`ManipulationTask.setup()` was not propagated to the factory graph construction step.

### Fix

1. Add `q_init` parameter to `create_factory_graph()` in `graph.py`:

```python
def create_factory_graph(
    self,
    config: BaseTaskConfig,
    pyhpp_constraints=None,
    graph_constraints=None,
    q_init=None,          # ← NEW
) -> Any:
    ...
    if self.backend == "pyhpp" and q_init is not None:
        self.robot.currentConfiguration(np.array(q_init, dtype=float))
    self.factory.generate()
```

2. Pass `self.q_init` from `create_graph()` in `base.py`:

```python
return self.graph_builder.create_factory_graph(
    self.task_config,
    pyhpp_constraints=self.pyhpp_constraints,
    graph_constraints=graph_constraints,
    q_init=self.q_init,   # ← NEW
)
```

**Files changed**:
- `src/agimus_spacelab/planning/graph.py`
- `src/agimus_spacelab/tasks/base.py`

---

## Related: `skip_placement` for PyHPP No-Contact Objects

Before the above fixes could take effect, a fourth issue existed:
`FactoryConstraintRegistry.register_from_defs()` was pre-registering placement constraints
as `RelativeTransformation` (CORBA-style) before factory graph creation. This caused
`buildPlacement()` to see `placeAlreadyCreated=True` and skip the `LockedJoint` path
entirely — but the pre-registered `RelativeTransformation` is wrong for PyHPP's foliation
model for no-contact objects.

**Fix**: Added `skip_placement=True` logic in `base.py._create_factory_constraints()` for
PyHPP + no-contact case, and a corresponding `skip_placement` parameter in
`constraints.py.FactoryConstraintRegistry.register_from_defs()`.

**Files changed**:
- `src/agimus_spacelab/tasks/base.py`
- `src/agimus_spacelab/planning/constraints.py`

---

## Final Outcome

After all fixes, `generate_via_edge` for the factory graph succeeds:

```
✓ Set robot current configuration for factory graph construction
✓ Generated graph structure
✓ SUCCESS q_wp_0_...|_f generated via edge '...| f' after 5 attempts
✓ Goal state: spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool
```

---

## Reproduction

```bash
# Before fix: fails 1000/1000 on f_01, then f after partial fix
cd script/spacelab
python task_grasp_frame_gripper.py --factory --backend pyhpp --no-viz

# After fix: succeeds
python task_grasp_frame_gripper.py --factory --backend pyhpp --no-viz
```

---

## References

- `hpp-python/src/pyhpp/manipulation/constraint_graph_factory.py` — `buildPlacement()`
- `hpp-constraints/include/hpp/constraints/locked-joint.hh` — `LockedJoint::create()`
- `hpp-manipulation/src/graph/edge.cc` — `ExplicitConstraintSet::rightHandSideFromInput()`
- `hpp-python/src/pyhpp/manipulation/device.cc` — `getJointConfig()`, `currentConfiguration()`
