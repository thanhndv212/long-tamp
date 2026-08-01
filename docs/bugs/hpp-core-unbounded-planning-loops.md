# Bug Report: Unbounded Loops in HPP Planning/Optimization Cause Indefinite Hangs

**Package**: agimus_spacelab / hpp-core / hpp-manipulation
**Component**: `GraspSequencePlanner._plan_release_subphase()`, `hpp-core::PathPlanner`,
`hpp-core::continuousValidation::Progressive`, `hpp-core::PathOptimizer`
(`RandomShortcut`, `SplineGradientBased`)
**Severity**: Critical — long sequential grasp/release plans could hang indefinitely with
no error, no timeout, and no way to distinguish "still working" from "will never return"
**Affects**: Any multi-phase sequential grasp task (`GraspSequencePlanner`) with more than
a few simultaneous grasps; observed on `script/spacelab/test_full_sequence.py`'s 13-phase
SpaceLab assembly sequence, specifically the auto-release step ahead of phase 4
(`frame_gripper` releasing `RS1` before grasping `RS6`) and recurring intermittently at
later phases (6, 10) due to unseeded randomness

---

## Summary

A 13-phase sequential grasp-and-release task would occasionally freeze for 30–45+ minutes
at a time with zero console output and no crash, indistinguishable from a true infinite
loop. Root-caused to **five separate, independent bugs** stacked across the planning
stack — one in `agimus_spacelab`'s own retry logic, and four in upstream `hpp-core`'s
motion-planning/optimization pipeline, each missing a wall-clock bound that every sibling
algorithm in the same codebase already has. None of these are inherent to the planning
problem's difficulty — the underlying task is solvable in seconds once each loop is
properly bounded and allowed to retry with a fresh random seed instead of hanging forever
on one unlucky sample.

---

## Bug 1: `_plan_release_subphase` Has No Retry-on-Failure

### Problem

`GraspSequencePlanner._plan_release_subphase()` (`src/agimus_spacelab/tasks/grasp_sequence.py`)
plans the two waypoint edges of an auto-release ("grasped → pregrasp → free") in a single
shot each. If `generate_via_edge()`'s underlying IK solver or `plan_transition_edge()`'s RRT
search draws an unlucky random seed, the whole release attempt raises immediately — with no
retry — even though the exact same pattern (regenerate a fresh target and try again, up to
`_MAX_COLLISION_RETRIES` times) was already used everywhere else in the same file for the
main grasp-edge planning loop (see the two pre-existing loops at lines 1827 and 2892).

### Observed Behavior

A single call to `plan_transition_edge()` inside the release step could occasionally run
for 30+ minutes with zero progress output before the container was killed to recover —
because that one call was itself deep inside one of Bugs 2–5 below, with no outer retry
to ever get a second chance at a luckier random seed.

### Root Cause

`_plan_release_subphase()` (`src/agimus_spacelab/tasks/grasp_sequence.py:219`) called
`generate_via_edge()`/`plan_transition_edge()` once per waypoint edge (`_21`, `_10`, and the
direct-edge fallback) with no surrounding retry loop, unlike the main grasp-edge code path.

### Fix

Wrapped all three planning calls (`_21`, `_10`, and the direct-release-edge fallback) in the
same regenerate-and-retry pattern already used by the main loop, bounded by the existing
`self._MAX_COLLISION_RETRIES = 10` (line 158):

```python
for _attempt_21 in range(self._MAX_COLLISION_RETRIES):
    try:
        path21, _ = self.planner.plan_transition_edge(
            edge=edge_21, q1=q_start, q2=q_pregrasp
        )
        if path21 is None:
            raise RuntimeError(f"Planning failed for edge '{edge_21}'")
        plan_err_21 = None
        break
    except Exception as e:
        plan_err_21 = e
        path21 = None
        if _attempt_21 < self._MAX_COLLISION_RETRIES - 1:
            _ok21, _q_new21 = self.config_gen.generate_via_edge(
                edge_name=edge_01, q_from=q_start,
                config_label=f"q_autorelease_{gripper}_pregrasp",
            )
            if _ok21 and _q_new21 is not None and np.all(np.isfinite(_q_new21)):
                q_pregrasp = _q_new21
```

(mirrored for `_10` and the direct-edge fallback)

**Files changed**: `src/agimus_spacelab/tasks/grasp_sequence.py`

**Verification**: With only this fix applied, the previously-stuck phase-4 release completed
in ~20–25s (comparable to every other phase), across multiple full test runs.

---

## Bug 2: `ConfigGenerator.generate_via_edge` Has No Wall-Clock Timeout

### Problem

`generate_via_edge()` (`src/agimus_spacelab/planning/config.py`) shoots up to
`max_attempts` (1000) random configurations through HPP's Newton-Raphson constraint
projector, bounded only by attempt *count*, never by wall-clock time. Each C++ solve is
itself bounded (`graph.maxIterations()`, 10000 per attempt — see `graph.py`'s
`finalize_manual_graph`/`create_factory_graph`), but as the constraint graph accumulates
more simultaneous grasps in later phases, individual solves become more expensive, and
1000 × 10000 iterations of increasingly-expensive work has no overall ceiling.

Additionally, the function only ever printed progress on a *successful* solve — a target
that's hard to reach (or reachable but always in collision) silently burns through up to
1000 attempts with zero output, indistinguishable from a hang from the outside.

### Fix

Added a `timeout: Optional[float] = 30.0` parameter, checked at the top of the attempt
loop, plus periodic progress logging every 200 attempts (attempt count, solver-fail vs.
collision-invalid counts, last errors) so a long call is visible instead of a black box:

```python
_t_start = time.time()
for i in range(self.max_attempts):
    if timeout is not None and (time.time() - _t_start) > timeout:
        return False, None
    if verbose and i > 0 and i % 200 == 0:
        print(f"       [progress] '{edge_name}': {i}/{self.max_attempts} attempts "
              f"({n_solver_fail} solver-failed, {n_collision_invalid} invalid/in-collision)")
    ...
```

**Files changed**: `src/agimus_spacelab/planning/config.py`

**Note**: this alone is insufficient on its own — see Bugs 3–5, which are the calls that
actually needed a Python-level timeout to have any effect. It's still valid defense in
depth for the IK-sampling path specifically.

---

## Bug 3: `Astar::findPath()` Has No Bound At All

### Problem

**File**: `hpp-core/src/astar.hh`
**Function**: `Astar::findPath()` (private helper of `GoalConfigurations::computePath()`)

When `tryConnectInitAndGoals()` succeeds immediately (a direct connection is found), the
RRT's own bounded main loop (`PathPlanner::solve()`, which correctly checks `maxIterations`/
`timeOut` between steps) is skipped entirely, and the roadmap-to-path extraction goes
straight to `computePath()` → `GoalConfigurations::computePath()` → `Astar::findPath()`, an
A\* search over the roadmap graph — this one had **neither an iteration cap nor a
wall-clock bound**, unlike every other search in the same file.

### Root Cause

`findPath()`'s `while (!open_.empty())` loop terminates only when the goal is found or the
open set is exhausted — no timeout, no iteration cap. `open_`/`closed_` are `std::list`
with `O(n)` `std::find` membership checks redone for every outgoing edge of every expanded
node, so pathological roadmap shapes can blow this up well past what any caller expects
from a "path already found" step.

### Observed Behavior

Traced via targeted C++ tracing bracketing every step of `PathPlanner::solve()`
(`startSolve()`, `tryConnectInitAndGoals()`, `target()->reached()`, `computePath()`) — this
was the first of several call sites found to hang silently for 10+ minutes with no output,
confirmed reproducible via a standalone repro script (`repro_phase_range.py`, see below)
run in a loop until caught.

### Fix

Added a 30-second wall-clock bound inside the search loop, throwing the same kind of
catchable exception the loop already throws on genuine exhaustion:

```cpp
bpt::ptime astarStart(bpt::microsec_clock::universal_time());
const double astarTimeOutSeconds = 30.0;
while (!open_.empty()) {
  bpt::ptime astarNow(bpt::microsec_clock::universal_time());
  double astarElapsed = static_cast<double>(
      (astarNow - astarStart).total_milliseconds()) / 1000.0;
  if (astarElapsed > astarTimeOutSeconds) {
    throw std::runtime_error(
        "A* timed out extracting the solution path from the roadmap.");
  }
  ...
}
```

**Files changed**: `hpp-core/src/astar.hh`

---

## Bug 4: `continuousValidation::Progressive::validateStraightPath` Can Take an
Astronomical Number of Steps

### Problem

**File**: `hpp-core/src/continuous-validation/progressive.cc`
**Function**: `Progressive::validateStraightPath()`

This is the shared continuous path-validation routine used by `directPath()`,
`tryConnectInitAndGoals()`, and every RRT extend step. Its step size (`t = second(interval,
reverse)`) is a conservative-advancement distance derived from the nearest collision pair's
separation. When two bodies pass very close to or nearly tangent to each other along a
candidate path — exactly what happens when a gripper retracts away from an object it just
released — that safe step can shrink toward zero, making the number of iterations needed to
cover the path's time range astronomically large in practice, even though the loop is
mathematically guaranteed to terminate eventually.

### Observed Behavior

Confirmed via unbuffered (`python3 -u`) tracing that this exact call — reached via
`directPath()`'s "trying directPath first" branch on a release-retraction edge — was the
hang site in multiple independent repro runs, with no output at all (not even the
1,000,000-iteration diagnostic counter added in a first fix attempt, which turned out to be
far too generous relative to real per-iteration cost to ever practically trigger).

### Fix

Replaced an initial (too-generous) fixed iteration cap with a proper wall-clock bound,
checked every 1000 steps to avoid the timing check itself adding overhead to the common,
fast-converging case:

```cpp
bpt::ptime progStart(bpt::microsec_clock::universal_time());
const double progTimeOutSeconds = 15.0;
unsigned long int nStep = 0;
while (finished < 2 && valid) {
  ++nStep;
  if (nStep % 1000 == 0) {
    double progElapsed = /* ms since progStart */ / 1000.0;
    if (progElapsed > progTimeOutSeconds) {
      report = PathValidationReportPtr_t(new PathValidationReport(
          t, ValidationReportPtr_t(new ProjectionError())));
      valid = false;
      break;
    }
  }
  ...
}
```

**Files changed**: `hpp-core/src/continuous-validation/progressive.cc`

---

## Bug 5: `PathOptimizer/timeOut` Exists But Was Never Armed, and Then Not Propagated

### Problem

**File**: `hpp-core/src/path-optimizer.cc` (base class); consumed by
`RandomShortcut::optimize()` and `SplineGradientBased::optimize()`
(`hpp-core/src/path-optimization/{random-shortcut,spline-gradient-based}.cc`)

`PathOptimizer` already has a correctly-designed stopping mechanism:
`shouldStop()` checks both `maxIterations_` and `timeOut_`, armed via `monitorExecution()`
at the start of every `optimize()` call. But both default to infinity
(`path-optimizer.cc:41-42`), and **nothing in `agimus_spacelab` ever set a finite value** —
so an optimizer that keeps finding genuine (if vanishingly small) cost improvements, or
hits a pathological path geometry, had no wall-clock backstop at all. Traced via full
`SplineGradientBased` iteration tracing: one call ran 5,560+ fast iterations (~4ms each,
zero collisions, cost never converging below threshold) with no way to stop short of
`shouldStop()`.

Once a `PathOptimizer/timeOut` value was set via `self.problem.setParameter(...)`, the hang
persisted — because `RandomShortcut`/`SplineGradientBased` instances are constructed
against `TransitionPlanner`'s separate `innerProblem_`, not `self.problem`, and a value set
only on the latter never reaches them (the exact same class of propagation gap the
pre-existing `SimpleTimeParameterization` parameter block in the same function already had
a documented workaround for).

### Fix

Set the parameter on **both** `self.problem` and `tp.innerProblem()`, matching the existing
`SimpleTimeParameterization` pattern immediately above it in the same function:

```python
try:
    self.problem.setParameter("PathOptimizer/timeOut", 30.0)
except Exception as e:
    print(f"      [TP] ✗ PathOptimizer/timeOut on problem failed: {e}")
try:
    tp.innerProblem().setParameter("PathOptimizer/timeOut", 30.0)
    print("      [TP] ✓ PathOptimizer/timeOut=30.0s (problem + innerProblem)")
except Exception as e:
    print(f"      [TP] ✗ PathOptimizer/timeOut on innerProblem failed: {e}")
```

**Files changed**: `src/agimus_spacelab/backends/pyhpp.py`
(`_apply_transition_planner_defaults`)

---

## Final Outcome

An isolated repro of the two hardest phases (12–13 of the 13-phase sequence) — previously
hanging indefinitely or up to 90–150s+ per attempt — now completes (success or a clean,
retriable failure) within a bounded window every time, confirmed via a 400-second external
cap with room to spare. A full end-to-end 13-phase run subsequently reached the final phase
for the first time all session, resolving two separate legitimate mid-sequence failures
(phases 5 and 10, genuine collisions after all random seeds were exhausted) via fast, clean
auto-resume instead of hangs.

---

## Reproduction

```bash
# Fast, isolated repro of the hardest phases using a checkpoint (see below),
# instead of the ~20-25 min needed to run phases 1-11 first:
cd script/spacelab
AGIMUS_CHECKPOINT_DIR=/tmp/agimus_checkpoints python3 test_full_sequence.py --no-viz  # once, to populate checkpoints
python3 -u repro_phase_range.py --start 11 --end 12 --checkpoint-dir /tmp/agimus_checkpoints
```

`repro_phase_range.py` (new, `script/spacelab/repro_phase_range.py`) replays any phase
range directly from a `GraspSequencePlanner.plan_sequence()`/`resume_sequence()` checkpoint
(dumped via `AGIMUS_CHECKPOINT_DIR`), seeding grasp state and the true scene-initial
configuration (`q_scene_init`, a new optional parameter on `plan_sequence()`) without
re-running earlier phases.

---

## References

- `src/agimus_spacelab/tasks/grasp_sequence.py` — `_plan_release_subphase()`,
  `_MAX_COLLISION_RETRIES`
- `src/agimus_spacelab/planning/config.py` — `ConfigGenerator.generate_via_edge()`
- `src/agimus_spacelab/backends/pyhpp.py` — `_apply_transition_planner_defaults()`,
  `plan_transition_edge()`
- `hpp-core/src/path-planner.cc` — `PathPlanner::solve()`, `tryConnectInitAndGoals()`
- `hpp-core/src/astar.hh` — `Astar::findPath()`
- `hpp-core/src/problem-target/goal-configurations.cc` — `GoalConfigurations::computePath()`
- `hpp-core/src/continuous-validation/progressive.cc` —
  `Progressive::validateStraightPath()`
- `hpp-core/src/path-optimizer.cc` — `PathOptimizer::shouldStop()`,
  `PathOptimizer::monitorExecution()`
- `hpp-core/src/path-optimization/random-shortcut.cc` — `RandomShortcut::optimize()`
- `hpp-core/src/path-optimization/spline-gradient-based.cc` —
  `SplineGradientBased::optimize()`
