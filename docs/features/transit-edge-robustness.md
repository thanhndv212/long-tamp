# Arm-transit robustness: spline-optimizer fallback and phase-index sync

**Script**: `script/spacelab/test_screwdriving_sequence.py` —
`ScrewdrivingSequenceTask._attempt_move_to_target()`, the single attempt behind
`move_arm_to_target_nonstop()` and `_search_via_point()`
**Status**: Implemented 2026-08-14

Two independent fixes in the same code path — the "move an arm to a target pose" transit
used for VISPA/UR10 home moves and via-points.

---

## 1. Deterministic dodge for `SplineGradientBased_bezier3` "more than 2 IPs"

**Symptom.** `plan_transition_edge()` throws with `more than 2 IPs` in the message, killing
an otherwise-successful transit.

**Cause.** `SplineGradientBased_bezier3` is the last stage of the default transit-edge
optimizer chain (`_transit_edge_optimizers` in `backends/pyhpp.py`). It hard-throws
whenever `GraphRandomShortcut` / `GraphPartialShortcut` hand it a shortcut segment whose
constraint projection needed more than 2 interpolation points to track the manifold — see
hpp-core's `SplineGradientBasedAbstract::appendEquivalentSpline`.

This is **not** a sign the motion is infeasible. The shortcut optimizers already produced a
valid, collision-free path before this purely cosmetic smoothing pass choked on it.

**Why not just retry.** Blind retry only works by luck — a different random shortcut choice
happening to avoid a >2-IP segment. With enough simultaneously-held objects (5, by RS6's
post-`CON1`-release state) that luck is not reliable.

**Fix.** On catching a `more than 2 IPs` exception, retry the *same* attempt with just that
optimizer dropped from this edge's chain:

```python
self.planner.set_transition_optimizers(
    0, ["GraphRandomShortcut", "GraphPartialShortcut"],
)
path, _ = self.planner.plan_transition_edge(edge=0, q1=q_current, q2=q_target_raw)
```

The override is deliberately **left in place** afterward rather than cleared: the same
held-grasps combination reproduces the same edge name and the same failure, so there is no
benefit to re-discovering it on a future attempt. Any other exception, or a failure of the
retry itself, falls through to the previous behavior (log and return failure).

---

## 2. Missing `set_phase_indices()` sync after `build_phase_graph()`

**Symptom.** `plan_transition_edge()` rejecting the transit with `Initial configuration ...
does not satisfy the constraints of the problem` — **deterministically, on every retry**,
because `q_current` never changes between attempts within one move.

**Cause.** After building this move's degenerate phase graph, the tracker's phase indices
were never synced to that graph's phase-local factory ordering.
`get_current_state_name()` then silently falls back to the tracker's **global**
gripper/handle indices (`GraspStateTracker._get_abbreviated_state()` warns about exactly
this), which almost never match the reduced gripper/handle set of a degenerate graph. The
resulting `state_name` doesn't resolve to a real state in *this* graph, so
`apply_state_constraints()` either projects onto the wrong thing or no-ops — `q_current`
passes through effectively unprojected, and the edge planner rejects it.

**Fix.** Sync before computing any state name:

```python
if hasattr(self.graph_builder, "_phase_grippers"):
    seq_planner.grasp_tracker.set_phase_indices(
        self.graph_builder._phase_grippers,
        self.graph_builder._phase_handles,
    )
```

Every other `build_phase_graph()` call site in the codebase already did this
(`_setup_release_phase_graph`, `_build_phase_graph_and_constraints`) — this was the one
place it was missing.

The existing re-snap of `q_current` onto the freshly-built graph's node still runs after the
sync, and now has a correct state to snap onto.
