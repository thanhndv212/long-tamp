# Bug Report: ConstraintGraphFactory Redundant Recursion Causes Combinatorial Blowup

**Package**: hpp-python
**Component**: `pyhpp.manipulation.constraint_graph_factory.GraphFactoryAbstract._recurse()`
**Severity**: Critical — graph construction (not planning) could run for 20+ minutes of
genuine CPU work for phases with many simultaneously-held grippers/objects, with no error
and no way to distinguish "still working" from "will never return"
**Affects**: Any `ConstraintGraphFactory`-based graph construction with more than ~6-7
simultaneous grippers/handles, regardless of how restrictive the grasp filter is;
observed on the final phase (13/13) of a real 13-phase, multi-arm assembly sequence,
the phase with the most grippers (8) and objects (7) held simultaneously in the whole
sequence

---

## Summary

Constraint graph construction (`ConstraintGraphFactory.generate()`) is a *separate*
subsystem from motion planning (see the sibling report,
`hpp-core-unbounded-planning-loops.md`, for the planning-side hangs) — it builds the
states/transitions for a phase's grasp combinations *before* any planning happens. After
fixing every planning-side hang, a full 13-phase end-to-end run reached the final phase for
the first time all session, and then stalled there for 20+ minutes of continuous,
genuinely-computing CPU activity (confirmed via multi-threaded CPU time exceeding
wall-clock time) with zero progress output.

Root-caused to a missing negative-memoization bug in the factory's core recursive
state-space search: rejected grasp combinations are never cached, so every one of them gets
its entire descendant subtree redundantly re-explored from every distinct order in which
grippers/handles can be assigned to reach it. The module's own docstring documents the
*intended* behavior as O(N) for a restrictive sequential filter — this was a regression
from that intent, not inherent complexity, and was empirically confirmed (not just reasoned
about) via a standalone, HPP-independent reproduction.

---

## Bug: Rejected Grasp Combinations Are Never Memoized

### Problem

**File**: `hpp-python/src/pyhpp/manipulation/constraint_graph_factory.py`
**Function**: `GraphFactoryAbstract._recurse()`

```python
def _recurse(self, grippers, handles, grasps, depth):
    isAllowed = self.graspIsAllowed(grasps)
    if isAllowed:
        current = self._makeState(grasps, depth)
    if len(grippers) == 0 or len(handles) == 0:
        return
    for ig, g in enumerate(grippers):
        ngrippers = grippers[:ig] + grippers[ig + 1:]
        isg = self.grippers.index(g)
        for ih, h in enumerate(handles):
            nhandles = handles[:ih] + handles[ih + 1:]
            ish = self.handles.index(h)
            nGrasps = grasps[:isg] + (ish,) + grasps[isg + 1:]

            nextIsAllowed = self.graspIsAllowed(nGrasps)
            isNewState = not self._existState(nGrasps)   # BUG
            if nextIsAllowed:
                nnext = self._makeState(nGrasps, depth + 1)
            if isAllowed and nextIsAllowed and self.transitionIsAllowed(...):
                self.makeTransition(current, nnext, isg)
            if isNewState:
                self._recurse(ngrippers, nhandles, nGrasps, depth + 2)
```

`_existState()`/`_makeState()` (the memoization pair) read/write `self.states`, but
`self.states` is populated **only** inside `_makeState`, itself only called
`if nextIsAllowed:`. So a `nGrasps` combination *rejected* by `graspIsAllowed` (e.g. by
`long_tamp`'s `SequentialGraspFilter`, a trivial O(1) tuple-equality check — confirmed
not itself the bottleneck) is never added to `self.states`. `isNewState` is therefore `True`
on **every** visit to that combination, from **every** distinct parent path that reaches
it — not just the first.

### Root Cause

Formally and empirically proven (via a standalone stub reproduction — see Verification)
that the entire subtree rooted at a given `nGrasps` is a **pure function of its content
alone**: `ngrippers`/`nhandles` are the full gripper/handle lists with a fixed *set* of
elements removed (list-slicing), so their resulting order is independent of removal order,
and `depth` at any recursion level equals `2 × (number of non-None entries in grasps)`,
also path-independent. So re-reaching the same `nGrasps` via a different gripper-assignment
order calls `_recurse(ngrippers, nhandles, nGrasps, depth+2)` with byte-identical
arguments every time — pure wasted recomputation, never new discovery. This is exactly what
the `isNewState` check is supposed to prevent, and does correctly prevent for *accepted*
combinations — the bug is that it silently doesn't apply to rejected ones.

For 8 grippers × 7 handles, this multiplies out to a severe blowup: the same final grasp
tuple is reachable via many distinct assignment orderings, and the deeper/more-restrictive
the filter (i.e. the more combinations get rejected), the *more* of the redundant
re-exploration goes un-cached, since only accepted states are ever memoized.

### Observed Behavior

20+ minutes of continuous, multi-threaded CPU activity inside `factory.generate()`
(confirmed alive, not deadlocked, via CPU-time-vs-wall-clock comparison) for the 8-gripper/
7-object final phase, with the very next line of expected output
(`✓ Generated graph structure`) never printed. No phase earlier in the same 13-phase
sequence (each with fewer simultaneous grippers/objects) exhibited this.

### Fix

Added a separate visited-set, deliberately independent from `self.states` (which has a
different meaning — actually-created State objects — that other code depends on):

```python
# In GraphFactoryAbstract.__init__, alongside self.states = dict():
self._visitedGrasps = set()

# In _recurse(), replacing the buggy isNewState computation:
isNewState = nGrasps not in self._visitedGrasps
if isNewState:
    self._visitedGrasps.add(nGrasps)
if nextIsAllowed:
    nnext = self._makeState(nGrasps, depth + 1)
```

`_existState`/`_makeState` are untouched, so the set of created states/transitions is
unaffected — confirmed by diffing the resulting `{grasps}` state-set and
`{(from, to, ig)}` transition-set between unpatched and patched `_recurse` across multiple
gripper/handle counts, including a deliberately adversarial non-monotonic filter (one that
rejects most intermediate combinations except one specific deep target, forcing many
different orderings to converge on a rejected node before finally reaching it) — states and
transitions matched exactly in every case; only the redundant recursion was eliminated.

**Files changed**: `hpp-python/src/pyhpp/manipulation/constraint_graph_factory.py`

### Measured Complexity Reduction

Standalone reproduction (trivial stub `makeState`/`makeTransition`, no HPP/robot/scene),
`_recurse()` call counts before/after, under the exact `SequentialGraspFilter` shape used in
production:

| grippers × handles | buggy `_recurse` calls | fixed `_recurse` calls |
|---|---|---|
| 2 × 1 | 3 | 3 |
| 3 × 2 | 18 | 13 |
| 4 × 3 | 222 | 73 |
| 5 × 4 | 4,548 | 501 |
| 6 × 5 | 137,266 | 4,051 |
| 7 × 6 | > 3,000,000 (aborted) | 37,633 |
| **8 × 7** (production worst case) | *(not measured — would dwarf 7×6)* | **394,353** |

The fixed algorithm's call count matches the closed-form combinatorial floor
`Σ_{k=0}^{min(nG,nH)} C(nG,k)·P(nH,k)` exactly (2 states actually created in every case,
regardless of N — the filter's job). 394,353 calls of cheap Python-level work is a
fundamentally different regime from the multi-million-and-climbing count the buggy version
does for the same input.

### Note on remaining complexity

The fix eliminates *redundant* exploration but the algorithm still visits every distinct
partial gripper→handle matching once — that count is still combinatorial in grippers ×
handles (just no longer multiplied by redundant revisits). This is expected to be
sufficient for scenes at least up to the production 8×7 case (low seconds to tens of
seconds, not 20+ minutes). If future scenes grow substantially larger, the next lever would
be filter-aware early pruning (skip whole subtrees that can provably never reach an
accepted combination, rather than only deduplicating already-considered ones) — not
required for this fix, noted here as the natural next step if graph construction cost
becomes a problem again at greater scale.

---

## Final Outcome

Confirmed fixed and verified end-to-end on the real 13-phase assembly sequence that
surfaced this bug — phase 13's graph construction, previously stalled indefinitely, now
completes and the full sequence runs to completion.

---

## Verification

Standalone, HPP-independent reproduction (runs in seconds, no Docker/robot/scene needed):

1. Stub the two module-level dependencies `constraint_graph_factory.py` needs
   (`pyhpp.constraints.{Implicit,LockedJoint}`, `numpy`) — only touched by
   `ConstraintFactory`/`ConstraintGraphFactory` (the concrete robot-facing subclass), never
   by `GraphFactoryAbstract._recurse` itself.
2. Load the real source file directly (`importlib.util.spec_from_file_location`).
3. Subclass `GraphFactoryAbstract` with trivial `makeState`/`makeLoopTransition`/
   `makeTransition` that record into plain `set()`s instead of touching real graph objects
   (legal — these are the only `@abc.abstractmethod`s).
4. Reuse the real `SequentialGraspFilter`
   (`long_tamp/src/long_tamp/planning/sequential_grasp_filter.py`, pure Python,
   no compiled deps) plus one deliberately adversarial non-monotonic filter.
5. For several N (2..8 grippers/handles), diff the resulting state/transition sets between
   unpatched and patched `_recurse` (must match exactly) and assert call counts drop
   sharply for the patched version.

Deployment (pure-Python `install(FILES ...)` target — no codegen, no pybind11
recompilation triggered):

```bash
docker exec -it hpp-agimus-arm64 bash
source ~/devel/hpp/dockers/hpp-arm64/config.sh
cd ~/devel/hpp/src/hpp-python/build-rel
cmake --build . --target install
```

Note: `install/` inside the container is a named Docker volume, not a host bind-mount — it
can only be inspected/updated via `docker exec`, unlike `src/`, which is a live bind mount.

---

## References

- `hpp-python/src/pyhpp/manipulation/constraint_graph_factory.py` —
  `GraphFactoryAbstract._recurse()`, `_existState()`, `_makeState()`
- `long_tamp/src/long_tamp/planning/sequential_grasp_filter.py` —
  `SequentialGraspFilter.__call__()`
- `long_tamp/src/long_tamp/planning/graph.py` — `create_factory_graph()`,
  `build_phase_graph()` (pre-existing partial mitigation: restricts grippers/objects to
  phase-relevant ones, which reduces N for every phase except the last, where nearly
  everything is already held)
- `long_tamp/src/long_tamp/planning/sequential_graph_factory.py` —
  `SequentialConstraintGraphFactory`/`SequentialTransitionFilter` (currently unused dead
  code path; shares the same underlying `_recurse` and would automatically benefit from
  this fix if ever adopted)
