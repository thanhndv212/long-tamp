# Motion Quality — Redundant Waypoints, Path Smoothness & Time Parameterization

## Overview

Paths from manipulation planning (RRT + constraint graph) contain redundant
waypoints and execute at undesirable speeds even after basic optimization.
This document tracks the 6-phase improvement plan, implementation status, and
all tunable parameters.

### Phase Implementation Status

| Phase | Description | Status |
|---|---|---|
| 1 | Increase `NumberOfLoops` (shortcut attempts) | ✅ done |
| 2 | Add `GraphPartialShortcut` to all pipelines | ✅ done (partial — CORBA transit only) |
| 3 | Prepend `EnforceTransitionSemantic` to all pipelines | ✅ done (partial — PyHPP pregrasp only; commented out) |
| 4 | Disable zero-velocity at state junctions | ✅ done |
| 5 | Minimum RRT iterations via distance-tuning floor | ⬜ pending |
| 6 | Allow disabling `isShort` on pregrasp sub-edges (opt-in) | ⬜ pending |
| — | `SimpleTimeParameterization/safety` wiring | ✅ done |

---

## Symptoms

Two visually observable problems after running `test_full_sequence.py`:

1. **Pregrasp → grasp path too swift** — the short constrained approach motion
   executes at full joint-velocity-limit speed, making it hard to follow visually
   and giving no margin for real-robot execution.

2. **Transit / pregrasp paths sluggy after optimization** — long free-space motions
   remained rough and jerky despite path optimizers having been applied.  In the
   CORBA backend the spline smoother ran *before* the shortcut optimizer, so the
   shortcut re-kinked paths that had already been smoothed, leaving the final
   result worse than either optimizer alone.

---

## Root Causes

### 1. `SimpleTimeParameterization/safety` never set (both backends)

`SimpleTimeParameterization` scales joint velocities by a `safety` factor
(range [0, 1]).  The default in HPP-core is `1.0` (full velocity limits).
Neither backend ever called `setParameter("SimpleTimeParameterization/safety", …)`
so all paths executed at 100 % of the robot's velocity limits.

**PyHPP additional bug**: `_apply_transition_planner_defaults` never called
`setParameter` for *any* `SimpleTimeParameterization` key (`order`,
`maxAcceleration`, `safety`), even though the fields were stored on the object.

### 2. CORBA transit optimizer pipeline in wrong order

```
# Before (wrong — smoothing before shortcutting):
_transit_edge_optimizers = ["SplineGradientBased_bezier3"]

# After (correct — shorten path first, then smooth the result):
_transit_edge_optimizers = ["Graph-PartialShortcut", "SplineGradientBased_bezier5"]
```

Running the spline smoother first yields a curve-fitted version of the raw RRT
path.  The subsequent `Graph-PartialShortcut` then re-introduces kinks by taking
chord shortcuts across the smooth curve.  Reversing the order fixes this.

### 3. `bezier3` (cubic) insufficient for complex long paths

`SplineGradientBased_bezier3` fits 3rd-order Bézier segments.  On long transit
paths with many via-points the cubic segments cannot represent the required
curvature smoothly.  `SplineGradientBased_bezier5` (5th-order, 6 control
points/segment) produces visually smoother results on the same paths.

### 4. `_time_param_max_accel` default too conservative

The previous default of `0.2` was unreasonably tight and interacted badly with
a `safety` of `1.0`, causing excessive time stretching on paths already near the
acceleration limit.  Default raised to `1.0` (effectively unconstrained —
safety is the dominating lever).

---

## Changes Made

### `src/agimus_spacelab/backends/pyhpp.py`

| What | Before | After |
|---|---|---|
| `_waypoint_pregrasp_optimizers` spline | `bezier3` | `bezier5` |
| `_time_param_max_accel` default | `0.2` | `1.0` |
| `_time_param_safety` field | missing | `0.95` |
| `configure_time_parameterization` | no `safety` arg | `safety` arg added |
| `_apply_transition_planner_defaults` | no `setParameter` calls | calls `setParameter` for `order`, `maxAcceleration`, `safety` on `self.problem` |

### `src/agimus_spacelab/backends/corba.py`

| What | Before | After |
|---|---|---|
| `_transit_edge_optimizers` | `["SplineGradientBased_bezier3"]` | `["Graph-PartialShortcut", "SplineGradientBased_bezier5"]` |
| `_waypoint_pregrasp_optimizers` spline | `bezier3` | `bezier5` |
| `_time_param_max_accel` default | `0.2` | `1.0` |
| `_time_param_safety` field | missing | `0.95` |
| `configure_time_parameterization` | no `safety` arg | `safety` arg added, live-applied via CORBA |
| `_apply_transition_planner_defaults` | no `safety` setParameter | `safety` setParameter added |

### `src/agimus_spacelab/config/base_config.py`

Added to `Defaults` and `BaseTaskConfig`:
```python
TIME_PARAM_SAFETY: float = 0.95   # fraction of velocity limits
TIME_PARAM_ORDER: int = 2         # 1=linear, 2=cubic, 3=quintic
```

### `src/agimus_spacelab/config/yaml_loader.py`

Parses `time_param_safety` and `time_param_order` from the `optimization:` YAML
section and stores them as `TIME_PARAM_SAFETY` / `TIME_PARAM_ORDER` on the
generated config class.

### `src/agimus_spacelab/tasks/grasp_sequence.py`

`plan_sequence` now calls `configure_time_parameterization` from `task_config`
immediately after `configure_transition_planner`:

```python
if hasattr(self.planner, "configure_time_parameterization"):
    tp_kwargs = {}
    for field, kwarg in (("TIME_PARAM_SAFETY", "safety"), ("TIME_PARAM_ORDER", "order")):
        val = getattr(self.task_config, field, None)
        if val is not None:
            tp_kwargs[kwarg] = val
    if tp_kwargs:
        self.planner.configure_time_parameterization(**tp_kwargs)
```

### Script config files

`spacelab_config.py`, `graspball_config.py` — added `TIME_PARAM_SAFETY`,
`TIME_PARAM_ORDER` class attributes.

`spacelab_config.yaml`, `graspball_config.yaml` — added under `optimization:`:
```yaml
time_param_safety: 0.5    # SpaceLab: half speed (was 0.95 default)
time_param_order: 2
```

### `script/spacelab/test_full_sequence.py`

- Removed hard-coded `TIMEOUT_PER_EDGE` / `MAX_ITERATIONS_PER_EDGE` constants;
  the backend now reads its own defaults (set during `task.setup()`).
- Both `KeyboardInterrupt` handlers now call `_interactive_replay()` before
  returning, so stopping mid-run no longer skips the replay prompt.

---

## How to Tune

### Execution speed (`time_param_safety`)

This is the primary lever.  Edit `script/config/spacelab_config.yaml`:

```yaml
optimization:
  time_param_safety: 0.5   # 0.5 = half speed, 1.0 = full speed
```

| Value | Effect |
|---|---|
| `1.0` | Full joint velocity limits (default HPP-core behaviour) |
| `0.95` | Backend default — barely perceptible reduction |
| `0.5` | **Half speed** — recommended for visual inspection / demo |
| `0.25` | Quarter speed — use for slow-motion debugging |

The SpaceLab YAML is set to `0.5`; graspball uses `0.95`.  To change it
at runtime without editing the YAML:

```python
task.planner.configure_time_parameterization(safety=0.5)
```

### Polynomial order (`time_param_order`)

Controls the smoothness profile at path endpoints:

| Value | Polynomial | Boundary conditions |
|---|---|---|
| `1` | linear | none (constant speed) |
| `2` | cubic (3rd-order) | zero velocity at start/end |
| `3` | quintic (5th-order) | zero velocity *and* acceleration at start/end |

Use `3` for the smoothest ramp-up/ramp-down; use `1` for diagnostic runs where
you want constant-speed motion.

### Optimizer pipeline

To change the optimizer list for a specific edge type, edit the backend
initializer (`__init__`) or call the helper at runtime:

```python
# CORBA
task.planner._transit_edge_optimizers = ["Graph-PartialShortcut", "SplineGradientBased_bezier5"]

# PyHPP
task.planner._waypoint_pregrasp_optimizers = ["ManipulationRandomShortcut", "SplineGradientBased_bezier5"]
```

**Rule**: always put shortcut optimizers *before* spline smoothers.

### Shortcut iterations (`random_shortcut_loops`)

```yaml
optimization:
  random_shortcut_loops: 50   # HPP default is 5; higher = better quality, slower
```

More loops give shorter, cleaner paths before the spline pass.  50 is a good
all-round value; raise to 100–200 for offline planning where time is not critical.

---

## 6-Phase Improvement Plan

### Phase 1 — Increase `NumberOfLoops` ✅

**Problem**: HPP default is 5 shortcut attempts.  Under manipulation constraints
most attempts fail (the shortcutted path violates a constraint), so the optimizer
quits before making progress on complex paths.

**Fix**: `_random_shortcut_loops = 50` in both backends; propagated to
`setParameter("PathOptimization/RandomShortcut/NumberOfLoops", …)` inside
`_apply_transition_planner_defaults`.  Overridable via YAML.

**Files changed**: `backends/corba.py`, `backends/pyhpp.py`,
`config/base_config.py`, `config/yaml_loader.py`

---

### Phase 2 — Add `GraphPartialShortcut` to optimizer pipelines ✅ (partial)

**Problem**: Full-DOF `RandomShortcut` rarely finds valid shortcuts under
manipulation constraints because all joints must simultaneously satisfy them.
`GraphPartialShortcut` attempts shortcuts one joint at a time, making it far more
likely to succeed.

**Fix**: `Graph-PartialShortcut` (CORBA) / `GraphPartialShortcut` (PyHPP) inserted
after `RandomShortcut`, before `SplineGradientBased`, in all 4 optimizer profile
lists.

**Current state**: CORBA transit pipeline has `Graph-PartialShortcut` before
`SplineGradientBased_bezier5`.  PyHPP uses `ManipulationRandomShortcut` only on
the pregrasp pipeline (no explicit `GraphPartialShortcut` — the bindings exist but
aren't wired into the list yet).

**CORBA optimizer name → PyHPP class mapping**:

| YAML / CORBA name | PyHPP class |
|---|---|
| `Graph-PartialShortcut` | `GraphPartialShortcut` |
| `EnforceTransitionSemantic` | `EnforceTransitionSemantic` |
| `SplineGradientBased_bezier3` | `SplineGradientBased_bezier3` |
| `SplineGradientBased_bezier5` | `SplineGradientBased_bezier5` |
| `RandomShortcut` | `RandomShortcut` |
| `ManipulationRandomShortcut` | `ManipulationRandomShortcut` |

**Files changed**: `backends/corba.py __init__`, `backends/pyhpp.py __init__`

---

### Phase 3 — Prepend `EnforceTransitionSemantic` ✅ (partial)

**Problem**: After a shortcut removes waypoints, the sub-paths may no longer have
consistent edge-membership labels.  Subsequent optimizers (especially spline) can
then mis-classify path segments and violate constraints.

**Fix**: `EnforceTransitionSemantic` as first element in all 4 optimizer profile
lists in both backends.  It is an O(n) pass (<1 ms per edge) and has zero
collision risk.

**Current state**: Imported and registered as a factory in PyHPP.  In the
pregrasp pipeline it is present but commented out.  CORBA does not yet prepend it.

**Note**: No plugin loading needed — `EnforceTransitionSemantic` is built into
`hpp-manipulation`.

**Files changed**: `backends/corba.py __init__`, `backends/pyhpp.py __init__`

---

### Phase 4 — Disable zero-velocity at state junctions ✅

**Problem**: `SplineGradientBased` defaults to
`zeroDerivativesAtStateIntersection = true`, which forces zero joint velocity at
every intermediate waypoint state (the boundary between sub-paths).  This prevents
the spline from reducing path length across state junctions.

**Fix**: `_spline_zero_derivatives_at_state = False` in both backends.
`_apply_transition_planner_defaults` calls
`setParameter("SplineGradientBased/zeroDerivativesAtStateIntersection", False)`.
Only affects edges where `SplineGradientBased_*` is actually in the pipeline.

**Files changed**: `backends/corba.py`, `backends/pyhpp.py`,
`config/base_config.py`, `config/yaml_loader.py`

---

### Phase 5 — Minimum RRT iterations via distance-tuning floor ⬜ PENDING

**Problem**: Distance-based auto-tuning (`_enable_distance_tuning = True`) can
scale iterations very low for short edges.  This produces a poor initial RRT path
with many redundant segments — the optimizer then has more work to do and may not
remove all of them within the iteration budget.

**Fix plan**:
1. Add `_min_transition_iterations: int = 500` to both backends
2. In `_compute_planning_budget`, clamp:
   ```python
   max_iter = max(self._min_transition_iterations, scaled_value)
   ```
3. Add YAML fields: `transition_min_iterations`, `transition_max_iterations`,
   `transition_time_out`
4. Wire through `Defaults` → `BaseTaskConfig` → `yaml_loader` → `grasp_sequence`

**YAML schema (to add)**:
```yaml
optimization:
  transition_max_iterations: 10000
  transition_time_out: 60.0
  transition_min_iterations: 500
```

**Files to change**: `backends/corba.py`, `backends/pyhpp.py`,
`config/base_config.py`, `config/yaml_loader.py`

---

### Phase 6 — Allow disabling `isShort` on pregrasp sub-edges ⬜ PENDING (opt-in)

**Problem**: `ManipulationRandomShortcut::shootTimes()` skips any time sample that
falls in an `isShort` edge.  `WaypointEdge` sub-edges (`_01` / `_10`) are marked
`isShort = true` by default, so the pregrasp / postgrasp approach motions are
**never shortcutted**, even if the raw RRT path is very long.

**Fix plan** (opt-in, off by default):
1. Add `_allow_shortcut_through_pregrasp: bool = False` to both backends
2. After graph initialization, iterate all `WaypointEdge` sub-edges and call
   `subEdge.setShort(False)` on `_01` / `_10` edges when flag is `True`
3. Guard: only safe when `handle.clearance() + gripper.clearance()` is within the
   path projector convergence radius (typically 0.02 m for `Progressive(0.2)`)
4. Add `optimization.allow_shortcut_through_pregrasp: false` to YAML

**Risk**: If clearance > convergence radius, the shortcutted pregrasp path may not
project back onto the constraint manifold → collision.  Test on a single
gripper/handle pair first, confirm with `validatePath` before enabling globally.

**Files to change**: `backends/corba.py`, `backends/pyhpp.py`,
`planning/graph.py`, `config/yaml_loader.py`

---

## Complete YAML Schema (`optimization:` section)

```yaml
optimization:
  # --- Phase 1 ---
  # Number of shortcut attempts per RandomShortcut pass.
  # HPP default is 5; 50 is a good general-purpose value.
  random_shortcut_loops: 50

  # --- Phase 4 ---
  # When false, SplineGradientBased may carry velocity through state junctions,
  # producing smoother, shorter splines.
  spline_zero_derivatives_at_state: false

  # --- Phase 5 (pending) ---
  # TransitionPlanner time budget.  Used by _compute_planning_budget.
  transition_max_iterations: 10000
  transition_time_out: 60.0
  transition_min_iterations: 500    # floor when distance tuning scales down

  # --- Phase 6 (pending, opt-in) ---
  allow_shortcut_through_pregrasp: false

  # --- Time parameterization ---
  # Fraction of joint velocity limits used during time parameterization.
  # 1.0 = full speed (HPP default), 0.5 = half speed.
  time_param_safety: 0.5
  # 1=linear, 2=cubic (zero-vel ends), 3=quintic (zero-vel+accel ends)
  time_param_order: 2

  # --- Phase 2-3 (advanced override) ---
  # Override the optimizer pipeline for each edge type.
  # Omit to use backend defaults.
  # YAML uses CORBA names; PyHPP backend maps internally.
  #
  # optimizer_pipelines:
  #   transit:
  #     - EnforceTransitionSemantic
  #     - Graph-PartialShortcut
  #     - SplineGradientBased_bezier5
  #   pregrasp:
  #     - EnforceTransitionSemantic
  #     - RandomShortcut
  #     - Graph-PartialShortcut
  #     - SplineGradientBased_bezier5
  #   grasp:
  #     - EnforceTransitionSemantic
  #     - RandomShortcut
  #   default:
  #     - EnforceTransitionSemantic
  #     - RandomShortcut
```

---

## Verification Checklist

### Per-phase regression testing

```python
# Measure config-space path length for a reference edge
import numpy as np

pid = task.planner.ps.numberPaths() - 1
path = task.planner.ps.getPath(pid)
length = path.length()
print(f"Path {pid} length = {length:.4f}")
```

Run `validatePath` on every optimized path:
```python
valid, report = task.planner.ps.robot.client.basic.problem.validatePath(pid)
assert valid, f"Path {pid} invalid: {report}"
```

### Phase-specific checks

| Phase | What to verify |
|---|---|
| 1 — NumberOfLoops | Path lengths decrease vs baseline (more shortcut attempts → shorter paths) |
| 2 — GraphPartialShortcut | Constrained pregrasp paths visibly shorter; no new collisions |
| 3 — EnforceTransitionSemantic | `validatePath` passes; no change to path length expected |
| 4 — zeroDerivativesAtState | Spline paths cross state junctions with non-zero velocity; smoother visual |
| 5 — min iterations | Short edges no longer produce jagged paths; iteration count ≥ floor |
| 6 — isShort | Only test on one gripper/handle pair; always run `validatePath` |

### YAML propagation test

```python
# Confirm extreme values propagate end-to-end
from agimus_spacelab.config.yaml_loader import YamlTaskLoader
loader = YamlTaskLoader("script/config/spacelab_config.yaml")
cfg = loader.task_config
assert cfg.RANDOM_SHORTCUT_LOOPS == 50
assert cfg.TIME_PARAM_SAFETY == 0.5
print("Config chain OK")
```

---

## Design Decisions

- **YAML uses CORBA optimizer names** — PyHPP backend maps them via an internal
  translation dict so user-facing config is backend-agnostic.
- **All new parameters have sensible defaults** that reproduce the previous
  behaviour without requiring YAML changes.
- **Phase 6 is opt-in** (`allow_shortcut_through_pregrasp: false`) due to
  collision risk when clearance exceeds projector convergence radius.
- **Phases 1–3 are independent** and can be enabled/disabled individually via
  YAML without touching other phases.
- **Phase 4 requires SplineGradientBased in the pipeline** — setting
  `zeroDerivativesAtState` has no effect if the spline optimizer is not running.
- **Phase 6 depends on graph initialization** — must run after
  `factory.generate()` produces the `WaypointEdge` sub-edges.

