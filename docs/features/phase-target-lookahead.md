# Phase-target lookahead (`find_feasible_phase_target` + `phase_q_hints`)

**Package**: long_tamp
**Component**: `long_tamp.tasks.grasp_sequence.GraspSequencePlanner`
**Status**: Implemented 2026-08-14. The planner-side API and its tests are in this commit;
a caller wires it into its own retry loop the same way (see §Caller wiring below).
**Motivating failure**: in a long, multi-phase assembly mission, a later phase's grasp
target failing ~2300+ consecutive target-generation attempts (and, on another part,
878/878 draws per attempt) because an earlier phase's random commitment had already
made it unreachable

---

## Summary

A phase's target configuration is produced by `ConfigGenerator.generate_via_edge()`, which
is a **randomized** random-restart IK solve. Whichever valid solution it happens to land on
gets committed, and that commitment can pin constraints a *later* phase depends on — with
no way back. Retrying the later phase cannot help: it is not the phase that made the bad
choice.

This feature adds a **lookahead**: before committing phase N, draw a candidate, probe
whether phase N+1 is still reachable from it, and only commit candidates that pass. The
winning candidate is then replayed into the real planning call as a warm start
(`phase_q_hints`), so the real run reproduces the validated candidate instead of drawing a
fresh (possibly bad) one.

---

## The failure class this fixes

Two simultaneous fixed-offset grasps on the same rigid object fully determine that
object's orientation. In a multi-gripper assembly mission, a part can be held by one
gripper (e.g. carried by a first arm) **and** by a second gripper (a second-phase grasp)
at the same time. Once both are committed, the part's orientation is frozen — and the
next phase's grasp on that same part has no remaining freedom to fix a bad orientation.

Evidence from live runs:

| Observation | Measurement |
| --- | --- |
| Target generation after a bad second-grasp commitment | ~2300+ consecutive failures, **0** ever reaching collision-checking |
| …with the next arm frozen, and after the 30-failure unfreeze escalation fired | Same — freeing the frozen arm cannot move an object already pinned by two *other* fixed grasps |
| Visual confirmation (viser) | The target grasp face was turned away from the approaching tool |
| Same edge shape, a different part | 878/878 target draws failing at solver convergence |

So the retry loop was structurally incapable of succeeding: it was hammering phase N+1
against a phase-N commitment it could not undo.

---

## Mechanism

### 1. `GraspSequencePlanner.find_feasible_phase_target(...)`

```python
chain = seq_planner.find_feasible_phase_target(
    phase_n=("arm2/gripper", "part/handle_a"),
    phase_n1=("arm3/gripper", "part/handle_b"),
    q_current=q_current,
    q_scene_init=q_scene_init,
    frozen_arms_n=["arm1_"],
    frozen_arms_n1=["arm2", "arm3"],
    probe_timeout=5.0,
    max_candidates=100,
)
# -> list[list[float]] (one config per edge of phase N) or None
```

Per candidate it:

1. builds phase N's graph, chains phase N's edges from `q_current`, keeping **every**
   intermediate config;
2. commits the candidate on a **throwaway `grasp_tracker.copy()`**;
3. builds phase N+1's graph and probes its edge chain with a short `probe_timeout`,
   discarding the resulting config — only "does a solution exist" matters.

Two full `build_phase_graph()` calls per candidate, not one: `self.graph` is a
**singleton**, so building phase N+1's graph tears down phase N's. There
is no way to hold both alive and compare candidates side by side.

The real `self.grasp_tracker` is never mutated. Committing a hypothetical grasp to the real
tracker while merely probing is exactly the corruption class
`_restore_grasp_tracker_for_resume()` exists to fix — it would silently corrupt the state
the subsequent real `plan_sequence(phase_q_hints=...)` call depends on.
`tests/test_grasp_state_copy.py` pins the `copy()` isolation invariant that safety argument
rests on.

### 2. The hint is a **chain**, not a single config

`find_feasible_phase_target()` returns the whole per-edge config chain (pregrasp configs
first, committed target last) — and this is load-bearing, not a convenience.

`generate_via_edge()` takes an edge's constraint RHS from its predecessor's end config, and
free DOFs it doesn't constrain pass straight through. Hinting only the terminal edge leaves
the pregrasp edge randomized, so the last edge solves from a `q_from` the probe never saw
and its result drifts off the validated candidate — taking the phase N+1 guarantee with it.

That is not hypothetical. On 2026-08-13 with terminal-only hinting, a pregrasp edge was
redrawn 6× (5 planning failures), moving the free arm and with it the held part itself;
the next grasp the lookahead had *just verified* then failed 878/878 draws. The lookahead
reported success and the run still lost that grasp.

### 3. `_edge_hints_for_phase(phase_hint, n_edges)` — accepted shapes

| Input | Behavior |
| --- | --- |
| Chain, `len == n_edges` | One hint per edge; an uninterrupted run reproduces the probed candidate exactly |
| Single config (legacy) | Applied to the **last edge only**; earlier waypoint edges stay unhinted |
| Chain, length mismatch | Degrades to legacy single-config behavior (uses `hint[-1]`) rather than mis-aligning configs onto edges they were never solved for |
| `None` / empty | No hints |

Length, not truthiness, is used for the empty check — a hint may arrive as a numpy array,
whose truth value is ambiguous for more than one element.

### 4. `phase_q_hints` plumbing

`phase_q_hints: dict[int, list[list[float]] | list[float]]` is accepted by
`plan_sequence()` and `resume_sequence()`, forwarded through `_run_phase_loop` into
`_plan_phase_edges()`, expanded per-edge, and passed as `q_hint` to each edge's
`generate_via_edge()` call.

For `resume_sequence()` the key is the **absolute** phase index within
`self.original_sequence`, not re-based to the remaining sequence.

The hint is deliberately **not** applied to the collision-retry regeneration call. That
call exists to draw a genuinely fresh sample when RRT planning fails on an otherwise-valid
target; re-feeding the same hint would just reconverge to the same unplannable point.

### 5. `invalidated_phase_hints` — when the chain breaks

That same collision-retry redraw does break the chain. When it replaces a hinted edge's
target, `_plan_phase_edges()` records the phase in `self.invalidated_phase_hints` and logs
a warning. Everything downstream of that edge now solves from a `q_from` the lookahead
never probed, so the "next phase stays reachable" guarantee is void.

Reset semantics: cleared at the start of every `plan_sequence()` call, **not** in
`resume_sequence()` — a resume continues the same call's block and must keep an
already-broken chain visible to the caller across every resume attempt.

### 6. `reset_grasp_tracker_to_call_start()`

New public method, thin wrapper over
`_restore_grasp_tracker_for_resume(replay_completed_phases=False)`.

Resuming *keeps* the block's completed grasps — that is the point of a resume. Replanning a
block from its start must **not**, or the rebuilt phase graph would see the block's own
grasps as already held and plan a structurally different, unsatisfiable sequence.

---

## Caller wiring

A caller that repeats the same two-phase grasp shape across many parts/units typically
wires this in as:

- A phase-pair constant identifying which (phase N, phase N+1) shape needs protecting —
  one pair usually covers every repetition of that shape in the mission.
- A helper that calls `find_feasible_phase_target()` in an **unbounded** round loop, with
  periodic `gc.collect()` (e.g. every 5 rounds) to bound the C++ graph memory the retries
  accumulate. It should never fall back to blind, unhinted generation — that fallback *is*
  the failure mode this feature replaces. Each round explores genuinely fresh candidates,
  so more rounds keep buying real coverage (unlike blind retries against a dead
  commitment).
- A block-runner that takes a hint **factory**, not a precomputed dict, so that if the
  block breaks the chain mid-phase it can roll the tracker back with
  `reset_grasp_tracker_to_call_start()`, re-run the lookahead for a genuinely fresh
  candidate, and replan the block from its start rather than resuming forward on a void
  guarantee.
- Gating: only run the lookahead where the shape actually recurs — a plan entry that
  doesn't match the protected phase-pair shape can't be protected by *this* lookahead
  even in principle.

---

## Cost model

Not flat-rate cheap. `build_phase_graph()`'s cost scales with how many objects/grippers are
already held, not a constant:

| Checkpoint | Objects/grippers held at that point | Measured per `build_phase_graph()` |
| --- | --- | --- |
| Early in the mission | One held | ~65 ms |
| Mid-mission | Three held | ~6.2–6.4 s |

Target generation itself stays fast throughout (~30 ms/attempt, confirmed still true
mid-mission). So a candidate that reaches the phase-N+1 probe costs roughly **7–12 s** by
that point, and worse the further into the mission it gets.

`max_candidates` defaults to **100** for that reason: 20 was tried and starved the search
mid-mission (whole budget exhausted in ~22 s, while a slightly luckier draw found a
candidate on attempt #2). 100 gives up to ~15–20 min worst case — still trivial next to
the multi-hour blind-retry stalls it replaces.

---

## Scope and limitations

- **Grasp phases only** (`handle is not None`) for both N and N+1. Release phases use
  `get_release_edge_sequence`, not wired in — the one real caller never needs it.
- **Manual frozen-arms resolution only** (explicit list per phase; not `auto` /
  `interactive` / `global`), matching `frozen_arms_mode="manual"`.
- **One phase of lookahead depth.** Protecting a later turn that itself depends on an
  even-earlier turn's commitment would need a deeper lookahead spanning that earlier
  turn too. Not built — no evidence yet that it's needed.
- A hint for a phase a resume starts *after* is inert: that phase is already committed, and
  resuming cannot undo a bad commitment.

---

## Tests

| File | Kind | Covers |
| --- | --- | --- |
| `tests/test_grasp_sequence_phase_q_hints.py` | Unit (fakes) | Which `generate_via_edge()` call site receives which hint: full-chain expansion, length-mismatch fallback, single-edge and multi-edge phases, wrong-`phase_idx` hints ignored, omitted-hints backward compatibility, retry-regeneration call receiving **no** hint, and chain-break recording in `invalidated_phase_hints` (including that an unhinted phase's redraw is *not* recorded). |
| `tests/test_grasp_state_copy.py` | Unit (pure Python) | `GraspStateTracker.copy()` mutation isolation in both directions, and that phase indices are not carried over. |

Both need `pyhpp`, so they run inside the `hpp-arm64` container like every other test
that imports the package. An HPP-integration test that reproduces the real failure case
against actual SRDF constraints (not fakes) also exists in the private validation
environment this feature was developed against, seeded from a real mission checkpoint —
not included here since that checkpoint is mission-specific data.
