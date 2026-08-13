# Full codebase refactor: audit and phased plan

> **Status:** ✅ Complete — branch `refactor/split-manipulation-task-run`
> (branched from `main`). Every phase has a final disposition as of
> 2026-08-09: Phases 1, 1B, 2, 2B, and 3 are done; Phase 4 is excluded
> (CORBA being deprecated); Phase 5 was decided against (already
> coherent enough). Two cross-cutting bugs found during the refactor
> (decoupled from it) were also fixed 2026-08-09: the Phase 1 logging
> asymmetry and the hardcoded personal video-output-path default — see
> their sections below. The `print()`-vs-structured-logging migration
> (originally left open/unscheduled) was also completed 2026-08-09 —
> see the cross-cutting section below. See "Status: refactor complete"
> near the end of this doc for the full table. Phase 2's detailed step
> list (formerly the standalone `refactor-manipulation-task-run.md`) is
> inlined below and that file has been removed.
>
> **2026-08-09 update:** re-ran the AST audit (same methodology, script
> re-derived — see "How this audit was done") to get current line
> numbers/depths now that Phases 1-2 have landed and shifted everything
> below them, and to check for hotspots that appeared or grew during
> those phases. Findings folded in below: Phase 3's four candidates now
> have a full investigation + step list (same rigor as Phase 1's Step
> 1.0), `setup()` gets its follow-up plan (Phase 2B), and two new
> discoveries get their own sections — `GraspSequencePlanner`'s two other
> oversized methods (Phase 1B) and a lower-priority CLI/interactive-glue
> cluster (Phase 5).

## Tracking: before → after

Lines/depth from the same AST audit described below, re-run after each
step lands — not estimates. Depth = max if/for/while/try/with nesting.

### Refactor targets

| Target | Before (lines/depth) | After (lines/depth) | Status | Commit(s) |
|---|---|---|---|---|
| `grasp_sequence.py::plan_sequence()` | 1145 / 7 | 208 / 3 | ✅ Done | `fcd6cb6`…`be98588` |
| `grasp_sequence.py::resume_sequence()` | 919 / 7 | 165 / 4 | ✅ Done | `fcd6cb6`…`be98588` |
| `tasks/base.py::run()` | 365 / 7 | 108 / 4 | ✅ Done | `52036f3`…`fa90b98` |
| `grasp_sequence.py::_plan_release_subphase()` | 409 / 4 | 160 / — | ✅ Done — Phase 1B | `730d411` |
| `grasp_sequence.py::replay_sequence()` | 144 / 6 | 115 / — | ✅ Done — Phase 1B | `730d411` |
| `tasks/base.py::setup()` | 169 / 3 | 79 / — | ✅ Done — Phase 2B | `56cfa56` |
| `planning/graph.py::build_phase_graph()` | 299 / 4 | 114 / — | ✅ Done — Phase 3.4 | `750494c` |
| `backends/pyhpp.py::plan_transition_edge()` | 271 / 4 | 123 / — | ✅ Done — Phase 3.3 | `f8f8ded` |
| `planning/scene.py::disable_collisions_between_subtrees()` | 213 / 4 | 42 / — | ✅ Done — Phase 3.1 | `1941ea8` |
| `planning/config.py::generate_via_edge()` | 175 / 7 | 82 / — | ✅ Done — Phase 3.2 | `1941ea8` |
| `utils/interactive.py::interactive_menu()` | 100 / 8 | — | ⛔ Decided against — Phase 5, already coherent enough | — |
| `grasp_sequence.py::_offer_replay_or_browse()` | 90 / 8 | — | ⛔ Decided against — Phase 5, already coherent enough | — |
| `backends/corba.py` (whole file, 2150 lines) | — | — | ⛔ Excluded — being phased out entirely | — |

Note: `_plan_phase_edges()` (531/6, `grasp_sequence.py`) is Phase 1's own
output (Step 1.5), already explained there as large *by design* — not
re-listed as a new target.

### Where Phase 1's duplication actually went

`_plan_release_subphase()` (409 lines, pre-existing, untouched — already
shared by both public methods before this refactor started) is excluded
from the "new methods" list below; everything else is new.

| New shared method | Lines/depth | Replaces duplication from | Commit |
|---|---|---|---|
| `_plan_release_entry_phase()` | 115 / 4 | explicit-release blocks in both | `fcd6cb6` |
| `_plan_auto_release_if_needed()` | 131 / 5 | auto-release blocks in both | `fcd6cb6` |
| `_build_phase_graph_and_constraints()` | 212 / 6 | graph-build blocks in both | `82f2651` |
| `_compute_and_project_edge_sequence()` | 117 / 2 | edge-sequence/projection blocks in both | `a4a111d` |
| `_plan_phase_edges()` | 531 / 6 | the per-edge planning loops in both — the largest block, still large because it's where the two functions' *real* behavioral differences live (NaN-check, attempt bookkeeping, RunLogger events), not because it's under-refactored | `57c7462` |
| `_finalize_phase_result()` | 131 / 3 | phase-complete bookkeeping in both | `00f7b8f` |
| `_run_phase_loop()` | 178 / 3 | the phase-iteration wrapper itself | `be98588` |

Net for Phase 1: two 1000+-line, depth-7 functions with ~90% duplicated
logic → two ~200-line, depth-3/4 orchestrators calling into seven named,
independently-testable, single-responsibility methods (one of which is
still substantial by design, not by neglect).

## How this audit was done

Not impressions — measured. An AST pass over every function/method in
`src/agimus_spacelab` (490 total) ranked by line count and max
if/for/while/try/with nesting depth. Full output kept in this doc's
findings table; raw numbers are reproducible by re-running the same
AST walk. `script/` was excluded from the function-level audit (it's entry
points/examples, not the library), but its file sizes were surveyed too
(see "Script directory" below).

## Verified: container-based verification workflow

The `hpp-agimus-arm64` container (image `hpp-agimus:arm64`, volume
`hpp-arm64-install`) was stopped; started it and confirmed it bind-mounts
the **live working tree** directly
(`/Users/thanhndv212/Develop/agimus-ws/agimus_spacelab` →
`.../src/agimus_spacelab` in-container) — branch switches and edits are
visible immediately, no rebuild needed to change what files exist.

Two things had to be true to actually import and run code, now confirmed
working:
1. Source `dockers/hpp-arm64/config.sh` inside the container — activates
   the `hpp` conda env and sets `PYTHONPATH`/`LD_LIBRARY_PATH` to the
   compiled HPP stack in the `hpp-arm64-install` volume. Without it,
   `import pyhpp` fails even inside the container (wrong/no env active).
2. **The installed `agimus_spacelab` under `PYTHONPATH` is a static CMake
   install copy**, not the live source — editing `src/agimus_spacelab/*.py`
   does **not** change what `import agimus_spacelab` resolves to by
   default. Fix: prepend the live bind-mounted source dir to `PYTHONPATH`
   so edits are picked up immediately:

```bash
docker start hpp-agimus-arm64
docker exec hpp-agimus-arm64 bash -lc '
  source /home/thanhndv212/devel/hpp/dockers/hpp-arm64/config.sh &&
  export PYTHONPATH=/home/thanhndv212/devel/hpp/src/agimus_spacelab/src:$PYTHONPATH &&
  cd /home/thanhndv212/devel/hpp/src/agimus_spacelab &&
  <verification command here>
'
```

This is the standard verification recipe for every step in every phase
below — no step should claim "verified" without actually running this.

## Findings: size/complexity hotspots (`src/agimus_spacelab`)

| Lines | Max depth | Location | Note |
|---|---|---|---|
| 1145 | 7 | `tasks/grasp_sequence.py:957` `plan_sequence()` | largest function in the codebase, 3x the original refactor target |
| 919 | 7 | `tasks/grasp_sequence.py:2146` `resume_sequence()` | docstring says "Returns: Same as `plan_sequence()`" — strong duplication signal, see below |
| 409 | 4 | `tasks/grasp_sequence.py:219` `_plan_release_subphase()` | already shared by both of the above |
| 365 | 7 | `tasks/base.py:460` `run()` | the original target — Phase 2, now done, see below |
| 299 | 4 | `planning/graph.py:1038` `build_phase_graph()` | |
| 271 | 4 | `backends/pyhpp.py:1945` `plan_transition_edge()` | |
| 215 | 4 | `backends/corba.py:1403` `load_path_from_waypoints()` | **deprecated backend**, see scoping question |
| 213 | 4 | `planning/scene.py:265` `disable_collisions_between_subtrees()` | |
| 175 | 7 | `planning/config.py:288` `generate_via_edge()` | tied for deepest nesting outside grasp_sequence.py |
| 169 | 3 | `tasks/base.py:263` `setup()` | not yet in scope of the Phase 2 doc — candidate to add |

Codebase-wide: 24 functions over 100 lines, 8 over 200 lines, 21 with
nesting depth ≥ 5. `grasp_sequence.py` alone is 3912 lines — one class,
`GraspSequencePlanner` (lines 113-3327, ~3200 lines by itself) — and zero
of that class's methods are covered by `tests/` beyond an incidental
mention in `test_sequential_filter.py`.

### Duplication hypothesis: `plan_sequence()` / `resume_sequence()`

Checked which `self.*()` helpers each of the two giant methods calls.
Both already call the same 7 shared helpers (`_plan_release_subphase`,
`compute_phase_locked_joints`, `_get_arm_for_gripper`,
`_get_active_joints_for_unfrozen_arms`, `_auto_save_phase_paths`,
`_dump_phase_checkpoint`, `interactive_arm_selector_callback`) — so some
extraction already happened historically, but the *orchestration loop*
around those calls (iterate phases → iterate edges → plan → validate →
handle failure → track state) appears to be independently reimplemented
in each 1000-line function rather than shared. This is a hypothesis, not
yet confirmed by a full read of both bodies — Phase 1's first step is to
confirm or refute it before committing to specific extraction points.

## Script directory (`script/`, 6985 LOC total, excluded from the
## function-level audit)

Largest files: `interactive_planning.py` (969), `spacelab_config.py` (882),
`graspball_pyhpp_example.py` (860), `graspball_inbox_corba_example.py`
(615), `task_graspball_inbox.py` (557). These are CLI entry points and
worked examples, not library code with a stable contract — lower priority
than `src/`. Revisit after `src/` phases land; not scheduled as its own
phase yet.

## Phased roadmap

Ordered by measured size/risk, not by how the conversation happened to
arrive at them. Each phase gets its own detailed step list (same
granularity as Phase 2's existing doc: small, single-purpose, one commit
each, verified via the container recipe above) written **when that phase
starts**, not all up front — a 1145-line function's real seams won't be
known until it's actually read closely, and speccing 15 steps against a
guess would just get rewritten.

**2026-08-09 update**: Phases 1-2 are done. This update writes the
"when that phase starts" investigation for everything that was still a
placeholder — Phase 3's four candidates (previously "not yet detailed"),
plus two follow-ups this session's re-audit surfaced: Phase 1B
(`GraspSequencePlanner`'s other two oversized methods, found once Phase
1's extractions shifted the codebase's size ranking) and Phase 2B
(`setup()`, flagged but unplanned when Phase 2 landed). Phase 5 is new
too (CLI/interactive glue, deepest nesting in the codebase but lowest
priority) — recorded for visibility, deliberately *not* stepped out, per
the same "not all up front" rule this paragraph states, since nothing
points to it being worked on soon. **2026-08-09, later the same day**:
Phase 1B, Phase 2B, and all four Phase 3 steps landed (commits
`730d411`, `56cfa56`, `1941ea8`, `f8f8ded`, `750494c`), and Phase 5 was
decided against (user: already coherent enough, not worth refactoring)
— see each phase's section below. Phase 4 stays excluded. Every phase
now has a final disposition; see "Status: refactor complete" at the end
of this doc.

### Phase 1 — `GraspSequencePlanner` (`tasks/grasp_sequence.py`) — highest priority

The actual production-critical class: this is what pairs with the BT to
execute assembly missions. Highest line count, highest nesting depth,
zero test coverage, and a strong duplication signal between its two
largest methods. Highest risk *and* highest value in the codebase — should
happen before Phase 2 finishes, not after.

#### Step 1.0 — Investigation (done, no code change)

Read `plan_sequence()` (957-2101, 1145 lines) and `resume_sequence()`
(2146-3064, 919 lines) in full. **Duplication confirmed**: ~90% of both
functions is structurally identical —

| Block | `plan_sequence()` | `resume_sequence()` | Diff |
|---|---|---|---|
| Explicit-release handling | 1134-1222 | 2295-2380 | only `frozen_arms_mode` vs. resolved `use_fm` |
| Auto-release detection | 1229-1324 | 2383-2474 | same |
| Phase-graph build + locked-joint constraints | 1326-1492 | 2476-2609 | same, plus a harmless if/elif reordering for `config_gen` update |
| Edge-sequence compute + state projection | 1493-1575 | 2611-2680 | identical |
| Per-edge planning loop (generate target → plan → retry-on-collision) | 1577-1990 | 2682-2993 | identical except `start_edge_idx`/`attempt_num` resume bookkeeping |
| Phase-complete bookkeeping (timing, auto-save, pregrasp cache, result dict) | 1991-2065 | 2995-3037 | identical |

Genuinely different: the **preamble** (`plan_sequence` resets state fresh;
`resume_sequence` calls `get_resumable_state()`, replays completed grasps
to rebuild `self.grasp_tracker`, computes `remaining_sequence` and
`start_edge_idx`) and the **epilogue** (see logging finding below).

**Found while reading, not something I went looking for — logging/verbosity
asymmetries between the two paths (five now, growing as extraction reaches
each block):**
1. `resume_sequence()`'s edge loop never calls
   `self.run_logger.log("edge_start"/"edge_end", ...)` — `plan_sequence()`
   does, every edge (lines 1659-1669, 1731-1746, 1886-1901, 1956-1971).
2. `resume_sequence()` never emits `"phase_end"` — `plan_sequence()` does
   (2046-2063).
3. `resume_sequence()` never emits `"run_end"` or calls
   `self.run_logger.close()` on success — `plan_sequence()` does
   (2078-2092).
4. (found extracting Step 1.3) `resume_sequence()` never emits
   `"phase_start"` either — `plan_sequence()` does, right before computing
   locked-joint constraints.
5. (found extracting Step 1.3) `resume_sequence()` never prints the
   verbose confirmation lines around the graph/ConfigGenerator update
   (`"✓ Updated planner graph reference"`, `"✓ Initialized/Updated
   ConfigGenerator..."`) even when its own `verbose=True` — `plan_sequence()`
   always does when `verbose=True`. Preserved via the same `emit_logs` flag
   used for the RunLogger asymmetries, not unified.

Net effect: **a grasp sequence that succeeds via `resume_sequence()` is
invisible to `RunLogger`-based analysis/replay for every phase it
completes on the resume path.** This looks like an oversight from the
two functions evolving independently after being copy-pasted, not a
deliberate design choice — nothing in either docstring suggests
resume runs are meant to be unlogged. **Not fixing this as part of the
refactor** — per "preserve behavior exactly," the shared core will take
an `emit_logs` flag so the current asymmetry is preserved bit-for-bit.
Flagging it here as a separate bug to decide on later, not bundling a
behavior change into a refactor commit.

#### Steps 1.1-1.9 — extraction (small, one shared block at a time)

Each step below pulls one already-identical block into a private method,
called from both `plan_sequence()` and `resume_sequence()` unchanged. Do
them in this order — each shrinks both functions further and de-risks the
next step, ending with the one genuinely hard consolidation (1.7).

1. **1.1** — `_plan_release_entry_phase(...)`: the explicit-release block.
2. **1.2** — `_plan_auto_release_if_needed(...)`: the auto-release block.
3. **1.3** — `_build_phase_graph_and_constraints(...)`: graph build +
   locked-joint constraints. Done. Reconciled the if/elif reordering
   difference (both orders are equivalent since the two conditions are
   mutually exclusive — unified on `plan_sequence()`'s order). Verified:
   phase 3 (auto-release scenario, now fast post-rebuild) produces a
   structurally identical result (success, final_config, per-phase
   gripper/handle/complete/edges/num_paths) to the post-rebuild baseline.
4. **1.4** — `_compute_and_project_edge_sequence(...)`: edge-sequence
   compute + state projection. Done. Found a 6th verbose-print asymmetry
   here (two pairs of prints around the projection step that
   `plan_sequence()` always made when `verbose=True` and
   `resume_sequence()` never made regardless of `verbose`) — preserved via
   `emit_logs`, same pattern as before. Verified: phase 3 structurally
   identical to the post-rebuild baseline.
5. **1.5** — `_plan_phase_edges(...)`: the per-edge planning loop (the
   biggest single block, ~530 lines with docstring). Done. This block
   turned out to have more than logging differences: `plan_sequence()`
   additionally does a NaN/inf finite-check on the generated target and
   attempts viewer visualization before planning (`resume_sequence()`
   does neither), and `resume_sequence()` tracks a real `attempt` counter
   via `self.edge_stats` lookback plus an `is_resume: True` edge_stat key
   and a different failure message ("Failed after Xs (attempt #N)" vs.
   "Stored partial phase result: N edges completed"). All preserved
   exactly via a single `is_resume` parameter (not just a logging flag —
   documented per-difference in the method's docstring) plus
   `start_edge_idx` for where resume continues from.

   Verified both branches for real: `is_resume=False` via phase 3 again
   (structurally identical to baseline). `is_resume=True` required its
   own harness (`resume_sequence()` is never called cold in production —
   always after a real `plan_sequence()` call establishes graph_builder/
   roadmap state) — first attempt called it on a freshly-constructed
   planner with no prior `plan_sequence()` call and hit a **segfault**
   inside the compiled `computePath` C++ call on the very first edge.
   Re-ran with a realistic setup (real `plan_sequence()` on phase 0
   first, then a synthetic phase-1 failure, then `resume_sequence()`) —
   completed successfully, no crash, `is_resume=True` edge_stats present
   only where expected. Conclusion: the segfault is real but only
   reachable via an invocation pattern (`resume_sequence()` with no prior
   `plan_sequence()` call in the same process) that no actual call site
   in this codebase uses — not blocking this refactor, not caused by it
   (the crash is inside unmodified backend code), but worth a mental note
   if anyone ever adds a "resume from a serialized checkpoint in a fresh
   process" entry point.
6. **1.6** — `_finalize_phase_result(...)`: timing totals, auto-save,
   pregrasp caching, result dict, optional `phase_end` log. Done. Two
   more provably-safe order unifications found here (auto-save vs.
   pregrasp-cache order, and grasp-tracker-update vs. timing-calc order —
   neither pair reads the other's output). Uses `is_resume` (not
   `emit_logs`) since the differences go beyond logging: different print
   wording at different points in the two originals, not just presence/
   absence. Verified: phase 3 structurally identical to baseline (one
   verification run hit a 300s timeout during scene/graph setup —
   unrelated code this step never touches, and the immediate retry
   completed in the usual ~19s with an identical result, consistent with
   container resource contention from this session's many back-to-back
   runs rather than a regression).
7. **1.7** — Done. Extracted `_run_phase_loop(...)`, the shared
   phase-iteration wrapper around the six helpers from 1.1-1.6. Folded in
   two more provably-safe unifications while doing it: `resume_sequence()`
   was re-resolving `frozen_arms_mode`'s `None -> "global"` three times
   per iteration (once per sub-call) even though the value never changes
   across the loop — now resolved once before calling; and
   `getattr(self, "_q_scene_init", None)` was being recomputed every
   iteration for the same reason — now resolved once too. Both callers
   now just pass their own (`grasp_sequence`/`remaining_sequence`,
   `starting_phase_idx`, `total_phase_count_for_display`, `is_resume`)
   and get `q_current` back.

   **Result**: `plan_sequence()` 1145 → 208 lines (nesting depth 7 → 3);
   `resume_sequence()` 919 → 165 lines (depth 7 → 4). Verified both
   branches for real: `plan_sequence()` via phase 3 (structurally
   identical to baseline), `resume_sequence()` via the realistic
   plan-then-resume harness (success, correct `is_resume` tagging).

8-9. **Steps 1.8/1.9 turned out to already be done** — by the time 1.7
   landed, both `plan_sequence()` (208 lines) and `resume_sequence()`
   (165 lines) were already just preamble → `_run_phase_loop(...)` →
   epilogue, with the pre-existing logging asymmetries intact
   (`plan_sequence()` still emits `run_end`; `resume_sequence()` still
   doesn't). No further slimming needed.

Verification: no existing tests (confirmed — only an incidental mention
in `test_sequential_filter.py`). Given the complexity here, worth
recording an actual baseline (not just console-diffing like Phase 2):
run a real multi-phase sequence via the container, capture
`phase_results` (success/timing/final_config per phase) as JSON, and
diff that structurally after each step — console output alone is too
easy to eyeball-match while missing a real behavior change in this much
branching.

#### Found during Steps 1.1/1.2 verification: a real, unrelated hang bug

Baseline capture for phase 3 (`frame_gripper/g_FG_part` auto-releasing
`RS1/h_RS1_FG` before grasping `RS6/h_RS6_FG`, seeded from the existing
`/tmp/agimus_checkpoints/phase_03.json`) was launched *before* any
extraction edits. It ran for **~10h46m of continuous CPU time with zero
further output** after printing `Release edge sequence: [...]` — i.e. it
hung somewhere inside `_plan_release_subphase()`'s edge-planning loop, or
one level deeper in `plan_transition_edge`/`generate_via_edge`, **despite
`timeout_per_edge=60.0` being explicitly configured**
(`configure_transition_planner(time_out=60.0, ...)` logged earlier in the
run). Killed manually (`kill -9`).

This predates and is unrelated to Steps 1.1/1.2: `_plan_release_subphase`
is a pre-existing method neither step touches, and both extractions call
it with the same arguments the original inline code did.

**Update, resolved (not a new bug)**: `docs/bugs/hpp-core-unbounded-planning-loops.md`
(committed `96cd127`, pre-dating this refactor) documents **this exact
scenario** — `frame_gripper` auto-releasing `RS1` before grasping `RS6`,
ahead of phase 4 — as Bug 1 of five stacked unbounded-loop bugs, all with
fixes already landed in source: the Python-side retry loop (agimus_spacelab)
and `generate_via_edge` timeout are confirmed present in the current
source; the C++ fixes (`hpp-core/src/astar.hh`, `hpp-core/src/continuous-validation/progressive.cc`)
are confirmed present in the local `hpp-core` checkout. But the
**container's compiled `libhpp-core.so` was built 2026-08-07 12:47, and
the two relevant hpp-core fix commits landed 2026-08-07 14:52** — the
running container's compiled HPP stack is ~2 hours stale relative to its
own source tree. The Python-side fixes are live (bind-mounted source,
interpreted directly); the C++ fixes are not, until `hpp-core`/
`hpp-manipulation`/`hpp-python` are rebuilt and reinstalled into the
`hpp-arm64-install` volume. Not a code defect to fix — an environment
sync action, left for the user to run on their own schedule (rebuild is a
non-trivial, multi-minute C++ compile). Verification for the rest of this
refactor uses scenarios that don't hit this stale-build gap (see below)
until/unless a rebuild happens.

**Steps 1.1 + 1.2 verification, adapted**: re-ran against phase 0 instead
(simplest case: first grasp, no auto-release) with a hard `timeout 240`
this time — completed cleanly in 17.45s, `success=True`, correct final
state. This exercises the no-op branch of both new methods (the
restructuring in Step 1.2 that made the call unconditional and pushed the
guard inside the method) and confirms no import/control-flow regression.
It does **not** re-exercise the "actual release happened" branch inside
either new method — that requires the exact scenario that hit the
pre-existing hang. Not re-attempted given the hang risk; confidence here
instead rests on the extraction being a verified byte-identical relocation
(both new methods were built via exact line-range copy from the original
inline blocks, diffed block-by-block, not rewritten) plus the call into
`_plan_release_subphase` being unchanged in arguments and order. Disclosed
gap, same pattern as the thin-coverage steps flagged elsewhere in this
plan — not silently claimed as fully covered.

## Decision needed: the logging asymmetry — ✅ Fixed (2026-08-09)

Fix it (make `resume_sequence()` log like `plan_sequence()` does) as an
explicit, separately-committed bug fix — before, after, or decoupled
from this refactor? **User confirmed**: fix it now, decoupled from the
refactor (separate commit, not bundled into any phase's structure-only
diff).

**What changed**: every `self.run_logger.log(...)`/`.close()` call in
the shared `_build_phase_graph_and_constraints` (`phase_start`),
`_plan_phase_edges` (`edge_start`/`edge_end`/interrupt `run_end`), and
`_finalize_phase_result` (`phase_end`) helpers was gated by `not
is_resume` (or, for `phase_start`, bundled into the `emit_logs` flag
alongside unrelated console-print verbosity). All of these gates are
removed — RunLogger events now fire for both `plan_sequence()` and
`resume_sequence()`. `resume_sequence()` also gained its own
`loop_start_time` (previously never passed to `_run_phase_loop`, so the
interrupt-path `run_end`'s `total_time` would have crashed — caught
silently by the existing `except Exception: pass`, now computed
correctly) and a `run_end`+`close()` epilogue on success, mirroring
`plan_sequence()`'s — the reasoning being that `resume_sequence()` is
the call that actually brings a resumed run to completion, so it should
be the one to close out the log, not leave it dangling open forever.

**What deliberately did not change**: the purely-cosmetic console-print
asymmetries found alongside this (points 4-6 in the Step 1.0
investigation above, plus the `emit_logs`-gated verbose confirmation
lines and the two projection-step print pairs) — those aren't part of
"RunLogger-based analysis/replay," which is what this bug was actually
about, and touching them would be a broader, separate cosmetic-parity
decision. Also unchanged: every genuinely-different `is_resume`-gated
*behavior* (attempt-count bookkeeping, the NaN/inf finite-check +
viewer-visualization skip, the `edge_stat["is_resume"]` tag, and the
differing console message wording on failure) — those are real,
intentional differences between a first attempt and a resume attempt,
not logging bugs.

**Verification**: 7 new unit tests in `tests/test_grasp_sequence_logging.py`
directly exercise each fixed call site with `is_resume=True` (and, for
`_finalize_phase_result`, `is_resume=False` too) against a fake
`run_logger` that records every event, confirming `phase_start`,
`edge_start`, `edge_end` (success and failure), `phase_end`, and
`resume_sequence()`'s new `run_end`+`close()` all fire. Full regression:
204 tests pass (was 197), same 7 pre-existing unrelated failures. No
real-HPP container re-run was done for this specific fix — reasoning
recorded rather than skipped silently: every edit in this fix is of the
shape `if not is_resume and X:` → `if X:`; for `plan_sequence()`
(`is_resume=False`), `not is_resume` was already always `True`, so the
condition's truth value — and therefore `plan_sequence()`'s observable
behavior — is provably unchanged. The only new behavior is additional
logging on the `is_resume=True` path, and only when `self.run_logger is
not None` (i.e., only when a caller has configured `log_dir` at all) —
for any real script that doesn't, this fix is a complete no-op.

### Phase 1B — `GraspSequencePlanner`'s remaining hotspots — ✅ Done

Found by the 2026-08-09 re-audit, after Phase 1 landed and shifted line
numbers. Both methods below live in `GraspSequencePlanner` — the same
production-critical class Phase 1 already proved the extract-verify-commit
workflow on — so this is a direct continuation of Phase 1, not a new
pattern. Two candidates, in risk order (lower first):

| Method | Lines/depth | Called from |
|---|---|---|
| `replay_sequence()` (`grasp_sequence.py:2797`) | 144 / 6 | External (CLI: `InteractiveGraspSequenceBuilder._offer_replay_or_browse`, see Phase 5) |
| `_plan_release_subphase()` (`grasp_sequence.py:219`) | 409 / 4 | Only `_plan_release_entry_phase()` (line 1027) and `_plan_auto_release_if_needed()` (line 1170) — both Phase 1 outputs |

#### Step 1B.0 — Investigation (done, no code change, this session)

**`replay_sequence()`**: a `for phase in self.phase_results` loop; each
phase's inner per-path playback is a 4-way dispatch (record via
`play_and_record_path_vector` → visualizer via
`play_path_vector_with_viz` → plain `play_path_vector` → unsupported-backend
warning, lines 2896-2927) — structurally the same "3(+1)-way duplicated
try/dispatch block" shape Phase 2 already solved once via
`_play_and_record`. Low risk, same playbook.

**Also found, cross-cutting**: `replay_sequence(output_dir: str =
"/home/dvtnguyen/devel/demos", ...)` is a **second instance** of the
hardcoded personal path already flagged in `tasks/base.py::run()`.
`grep -rn "dvtnguyen/devel/demos" src/` actually finds it in **6 places**
across 4 files (`tasks/base.py`, `tasks/grasp_sequence.py`,
`backends/corba.py` ×2, `backends/pyhpp.py` ×2,
`visualization/video_recorder.py` ×2) — wider than the original
cross-cutting note suggested. Updated below.

**`_plan_release_subphase()`** (219-627) is materially riskier than
anything Phase 1 touched: it's the exact function whose edge-planning
loop hung for **~10h46m** during Phase 1's own verification (see "Found
during Steps 1.1/1.2 verification" above) — a real bug, since resolved by
an `hpp-core` fix not yet compiled into the container's `libhpp-core.so`.
Reading the full body (not done before — Phase 1 only called this method
unchanged) finds five sequential blocks:

1. **Graph setup + tracker sync** (243-291): build `release_held_grasps`,
   call `graph_builder.build_phase_graph(...)`, wire `self.planner.graph`
   and `self.config_gen`, sync `grasp_tracker` phase indices. Self-contained,
   own try/except. → `_setup_release_phase_graph(...)`.
2. **Source-state projection** (297-319): project `q_current` onto
   `source_state` via `apply_state_constraints`. Small, self-contained
   try/except. → `_project_onto_release_source_state(...)`.
3. **Edge `_21` (grasped→pregrasp)** (339-415): generate `q_pregrasp`
   *once* via the forward edge `edge_01` (not `edge_21` — see the
   in-source comment explaining why: `edge_21`'s own fold conflicts with
   the pregrasp leaf), then a `_MAX_COLLISION_RETRIES` loop that plans
   `edge_21` and **regenerates via `edge_01` on failure**.
4. **Edge `_10` (pregrasp→free), primary path** (417-491), reached only
   if step 3 succeeded: a `_MAX_COLLISION_RETRIES` loop that
   **regenerates and plans the same edge (`edge_10`) every iteration**.
5. **Direct fallback edge** (492-569), reached only if step 3 exhausted
   all retries: **structurally identical** loop shape to block 4 —
   regenerate-and-plan-every-iteration — just against `direct_edge`
   (`edge_21` stripped of its `_21` suffix), no `q_hint`.

**Duplication found**: blocks 4 and 5 are the same
"generate-via-edge-then-plan-that-same-edge, retry both together up to
`_MAX_COLLISION_RETRIES` times" loop, parameterized only by
`(edge_name, config_label, q_hint)` and their error-message wording — a
real consolidation opportunity, same category as Phase 1's headline
finding. Block 3 does **not** fit that same shape (its generate-edge
`edge_01` differs from its plan-edge `edge_21`, and only the plan step
retries, not the initial generation) — flagging that now rather than
discovering it mid-extraction and forcing a false unification. The two
`phase_info` dict literals at the end (578-598 direct-path vs. 599-626
waypoint-path) are also near-identical, differing only in which
paths/edges/edge_stats list they contain — a `_build_release_phase_info(...)`
builder, same shape as Phase 2's `_build_result`.

#### Steps 1B.1-1B.5 — extraction, in this order

1. **1B.1** — `_setup_release_phase_graph(gripper, q_current,
   phase_graph_constraints, verbose)`: block 1 above. Returns
   `release_edges` (needed by the caller for logging) plus whatever the
   graph/tracker side effects require — exact return shape TBD when
   writing this step, since it mutates `self.config_gen`/`self.planner.graph`
   rather than returning them (instance method, not pure).
2. **1B.2** — `_project_onto_release_source_state(source_state, q_current,
   verbose)`: block 2. Returns the (possibly) projected `q_current`.
3. **1B.3** — `_generate_and_plan_edge_with_retry(edge_name, q_from,
   config_label, verbose, q_hint=None)`: the shared shape from blocks 4
   and 5. Returns `(path, q_result, t_gen, t_plan, error)` or raises
   after `_MAX_COLLISION_RETRIES` exhausted — mirrors block 4's exception
   message for callers that need it, block 5's caller adapts the message
   (both preserved verbatim via a `label` or edge-role parameter, not
   silently unified, per this doc's established practice for near-but-not
   -quite-identical blocks). Replaces blocks 4 and 5 with two calls to the
   same method.
4. **1B.4** — block 3 (the `_21`/`edge_01` pregrasp generation +
   regenerate-on-plan-failure loop) stays its own method,
   `_plan_release_pregrasp_edge(...)`, precisely because 1B.0 found it
   does *not* share block 4/5's shape. Extracting it anyway (as its own
   named method) still shrinks `_plan_release_subphase()` and gives it an
   independently-testable seam — just not a shared one.
5. **1B.5** — `_build_release_phase_info(q_start, used_direct, paths,
   edges, edge_stats, timings)`: consolidates the two end-of-method dict
   literals.

   **Result** (projected, not yet measured): `_plan_release_subphase()`
   drops from 409 lines to roughly 80-100 (preamble → four helper calls →
   `_build_release_phase_info` → return), similar shrink ratio to Phase
   1's `plan_sequence`/`resume_sequence`.

#### Step 1B.6 — `replay_sequence()` extraction

`_play_single_phase_path(path, edge_name, phase, idx, record, visualizer,
output_dir, video_prefix, framerate, dt, speed)`: the 4-way dispatch
block (2896-2927), same pattern and same discipline as Phase 2's
`_play_and_record`. Leave the phase-iteration loop and the
before/after `get_num_stored_paths()` bookkeeping in `replay_sequence()`
itself — there's no second caller to consolidate against here, so this
is a pure legibility split, not a duplication fix.

#### Verification model for Phase 1B

Same phase_results JSON-diffing baseline approach as Phase 1 (see
"Steps 1.1-1.9" above) — console-diffing alone is not enough for code
this branchy. **Extra discipline for 1B.1-1B.4 specifically**: this is
the exact code path that produced the documented 10h46m hang, so:
- Extractions must be verified byte-identical relocations (line-range
  copy, block-by-block diff against the original), not rewrites — same
  rule Phase 1 used for its own risky steps.
- Do not attempt to re-trigger the "actual release, edge_21 succeeds"
  and "actual release, direct-fallback" branches until the container's
  `hpp-core` is confirmed rebuilt past the 2026-08-07 14:52 fix commits
  (see the resolved-hang note above) — re-running the same stale-build
  scenario risks the same multi-hour hang for no new information.
- Prefer the phase-0/no-release smoke path (confirmed clean, 17.45s) for
  fast iteration between steps, and reserve a real release scenario for
  one final integration check per the Phase 1 Step 1.5-1.7 pattern —
  disclose if that final check can't be run before the container rebuild
  happens, rather than skipping verification silently.

### Phase 2 — `ManipulationTask.run()` (`tasks/base.py`) — ✅ Done

All 10 steps landed 2026-08-09, one commit each (`52036f3`…`fa90b98`).
`run()`: 365 lines/depth-7 → 108 lines/depth-4. Full regression gate
(Step 9) diffed clean against the Step 0 baseline modulo one disclosed,
accepted console-output line (see Step 6a below). Two behavior-preserving
decisions surfaced during execution and deliberately left unfixed: the
dead `solve_mode` gate, and a latent off-by-one in
`_parse_factory_waypoints` — both detailed below.

#### Why

[`ManipulationTask.run()`](../../src/agimus_spacelab/tasks/base.py)
(originally `tasks/base.py:460-824`, 365 lines) had grown to contain: 4
nested closures that didn't need to be closures, two structurally
different solve strategies (`manipulation-planner` segment-by-segment vs.
`transition-planner` edge-sequence) co-located in one function, a
duplicated 2-vs-N-segment special case, a play/record try/except block
repeated 3 times near-verbatim, and 2 inline return-dict literals covering
3 conceptually different outcomes (skip and 2-config and general-loop all
shared one; transition-planner had its own, with extra
`path_id`/`solve_mode` keys). It had **zero automated test coverage** —
only ever exercised end-to-end via real scripts.

Pure structure refactor: no behavior change for any caller. Also the
first concrete step toward exposing a narrow, stable BT-facing planning
entry point (`_solve_transition_planner`), a separate follow-up now that
this has landed.

**Explicitly out of scope** (tracked as follow-ups below, not touched, so
the diff stayed reviewable as pure structure change): the
`print()`-vs-structured-logging migration; the hardcoded personal
`output_dir` default; broader docstring/comment cleanup beyond `run()`
itself (Step 10 covers only `run()`).

#### Verification model

- No automated tests existed for `run()` before this phase.
- Two real call sites reach the base class's `run()` directly:
  `script/graspball/task_graspball_inbox.py` (manipulation-planner mode,
  multi-segment via 4 `preferred_configs` → the general `len(seq) > 2`
  loop) and `script/spacelab/interactive_planning.py` (the only script
  exposing `--solve-mode`, so nominally the only path to
  `transition-planner` mode — see the Step 0 finding below on why that
  path turned out to be dead). `script/spacelab/test_full_sequence.py`
  calls `task.run(no_viz=..., auto_save_dir=...)` — parameter names that
  don't exist on this signature, so it's calling an *overridden* `run()`
  elsewhere, not this method; excluded as a verification vehicle.
- **Correction found during Phase 2 review (2026-08-08), superseding the
  original plan's "host-testable" framing for Steps 1-2**: importing
  *anything* from `agimus_spacelab`, even a pure-Python helper with zero
  HPP calls, transitively requires `pyhpp` on host, because
  `agimus_spacelab/__init__.py` unconditionally does `from .planning
  import (...)`, which imports `sequential_graph_factory.py`, which
  imports `pyhpp.manipulation.constraint_graph_factory` at module load
  time — before any test-specific code runs. Confirmed by reproducing on
  an existing, already-pure test file: `pytest tests/test_run_logger.py`
  (docstring: "These tests require no HPP backend") fails collection on
  host with `ModuleNotFoundError: No module named 'pyhpp'`. So **every**
  step, including Steps 1-2, had to be verified inside the `hpp-arm64`
  container — no host-only path existed. Pre-existing condition of the
  package's eager-import structure, not something this refactor changes
  or should fix (a separate lazy-import cleanup, out of scope here). The
  container has `pytest` (9.1.1) and `pytest-cov` (7.1.0) installed and
  importing the live bind-mounted source confirmed working.
- **Baseline**: captured (Step 0) inside the `hpp-arm64` container
  (`hpp-agimus-arm64`), live bind-mounted source, `--backend pyhpp` (the
  only backend available there — `get_available_backends() ==
  ['pyhpp']`). Raw console captures kept under
  `docs/plans/refactor-manipulation-task-run/baseline/` for reference.
  Every later step diffed against this baseline, not against "looks
  right."
- One commit per step, Conventional Commits style. No step's verification
  failed, so no reverts were needed.

#### Step 0 — Baseline capture (no code change)

Ran `task_graspball_inbox.py` and `interactive_planning.py` inside the
`hpp-arm64` container.

- `task_graspball_inbox.py` (the only real call site for the
  manipulation-planner **general N-segment loop**): **blocked** — fails
  at `setup()` with `ValueError: Unable to retrieve package://
  hpp_practicals/urdf/ur5_gripper.urdf`. The `hpp_practicals` ROS package
  isn't present anywhere in this workspace or in the container's
  `ROS_PACKAGE_PATH`. Pre-existing environment gap, unrelated to this
  refactor, out of scope to fix. Consequence: the general-loop path
  (`len(seq) > 2`, non-transition-planner) had no real-script baseline in
  this environment for the whole phase — verified instead via the
  `len(seq) == 2` scenarios below (Step 6a's structural argument: 2-config
  is one iteration of the general loop) plus code-level inspection,
  disclosed rather than silently claimed as covered.
- `interactive_planning.py --goal "spacelab/g_ur10_tool grasps
  frame_gripper/h_FG_tool" --solve --no-viz`: real run, real HPP planning.
  3-node/8-edge filtered factory graph, 4 configs generated, **`solve()`
  reaches `max_iterations=5000` and fails**, script exits 0. This is
  `run()`'s `len(seq) == 2` branch. Re-ran with a second, different goal
  (`spacelab/g_vispa2_wb1 grasps RS1/h_RS1_WB`) to confirm this is a real,
  reproducible property of this solve path in this environment (both hit
  max-iterations), not a one-off unlucky seed.
- **Finding: `--solve-mode transition-planner` was a no-op whenever the
  resolved config sequence had exactly 2 entries.** `run()`'s dispatch
  only inspected `solve_mode` inside the `else` branch reached when
  `len(seq) > 2`. `_ordered_config_keys` only returns more than 2 entries
  if `configs` has `q_wp_<i>_*`-named keys or the caller passed matching
  `preferred_configs`. `interactive_planning.py` — the only script
  exposing `--solve-mode` at all — does neither, so it always fell
  through to the (dead, see below) fallback branch and got exactly
  `["q_init", "q_goal"]`. Net effect, confirmed empirically (re-running
  with `--solve-mode transition-planner` added produced output
  indistinguishable from the manipulation-planner run, right up to the
  same eventual max-iterations failure): **`--solve-mode
  transition-planner` had never actually taken effect through any real
  script in this codebase** — not "thin" coverage, zero, for a structural
  reason (a length check), not bad luck.
- **Related finding: `_ordered_config_keys`'s fallback branch is dead
  code.** The one line that would populate `mids` from arbitrary `q_*`
  configs is commented out in source. Pre-existing, preserved as-is (dead
  code relocated verbatim in Step 1), flagged for visibility.
- **Related finding (found writing Step 2 tests): `_parse_factory_waypoints`'s
  docstring invariant doesn't hold, and the branch is dead anyway.** Its
  docstring claims `len(waypoints) == len(edges) + 1` (the standard "N
  edges connect N+1 waypoints" convention). But for `k` factory-named
  `q_wp_<i>_<edge>` configs, the real code returns `edges` with `k`
  entries and `waypoints` with `k + 2` — one edge name short of what
  `plan_transition_sequence` would need for that many waypoints (no name
  for the final "last waypoint → q_goal" transition). Confirmed this
  branch is currently unreachable in practice: `grep -rn "q_wp_" src/
  script/` (excluding this method and its own tests) finds nothing — no
  config generator anywhere produces this naming convention. Not fixed
  (pure relocation only, Step 2); flagged for whoever eventually
  implements a real caller of the factory-waypoint convention.
- **Not captured — success-path / play-record branch**: both
  manipulation-planner attempts above ended in `solve()` failure, so the
  play/record block never executed in either capture, and no real script
  in this environment reliably produces `success=True` through this code
  path within the available time budget (consistent with production
  planning going through `GraspSequencePlanner`'s `TransitionPlanner`, not
  this method). Closed via a synthetic harness instead (Step 5, below).

#### Steps 1-10 — extraction

1. **`_ordered_config_keys`** (Step 1) — static method, parametrized on
   `configs` + `preferred_configs` explicitly instead of closing over
   `run()`'s outer scope. 7 unit tests added
   (`tests/test_task_base_helpers.py`), covering the dead-fallback case
   above. Direct relocation, byte-identical logic.
2. **`_parse_factory_waypoints`** (Step 2) — fully static. 4 more unit
   tests, including one documenting the docstring-invariant discrepancy
   found above.
3. **`_reset_goals_if_possible`** (Step 3) — trivial `self.ps.
   resetGoalConfigs()` guard, was duplicated at both solve call sites.
4. **`_compute_transition_inputs`** (Step 4) — instance method (needs
   `self.config_gen`) over explicit args. 7 unit tests covering every
   branch reachable without a real HPP `config_gen` (explicit waypoints +
   mismatch error, factory-waypoint dispatch, edges-without-generate-flag
   error, missing-config_gen error, the final "no inputs" error). Branch
   3's success path needs real HPP and has no real-script coverage in
   this environment, per the Step 0 finding — disclosed, not claimed.
5. **`_play_and_record`** (Step 5) — replaces 3 near-verbatim duplicated
   try/`play_and_record_path`/except/`play_path` blocks. Since no real
   script reaches `success=True` (Step 0 finding), verified via a
   synthetic harness (`TestPlayAndRecord`, a fake `self.planner` with
   controllable methods/exceptions) exercising the actual relocated
   control flow — record-flag branching, `hasattr` fallback, exception
   swallowing — directly, without needing a real successful solve.
6. **Step 6a — merge `len(seq) == 2` into the general loop.** The
   2-config branch is exactly one iteration of the general loop's body,
   so folding it in removes the special case. Two subtleties found and
   preserved exactly, not silently unified:
   - **`solve_mode` gate**: preserved as `solve_mode ==
     "transition-planner" and len(seq) > 2` — see "Decision needed"
     below.
   - **Playback gating**: the 2-config branch only called
     `_play_and_record` `if success`; the N-segment loop called it
     unconditionally (using whatever `path_ids` it had, even after a
     mid-sequence failure). Unified as `if visualize and (len(seq) > 2 or
     success)` — bit-for-bit equivalent to both originals (reduces to
     `success` when `len(seq) == 2`, always `True` otherwise).
   - **`path_ids` indexing safety**: the N-segment loop's CORBA
     `numberPaths`/`concatenatePath` tracking only does anything when
     `self.ps` has those callable methods. Confirmed via
     `dir(pyhpp.core.Problem)` inside the container: pyhpp's `Problem`
     class has neither, so for the only backend available here,
     `path_ids` always stays `[]` and resolves to the same hardcoded `0`
     the 2-config branch used directly — verified, not assumed. (CORBA,
     excluded from this refactor per the Phase 4 decision, could in
     principle differ here — not verified, not in scope.)
   - **One accepted, disclosed, non-functional deviation**: the loop
     unconditionally prints `"Segment {i+1}/{n}: {a} -> {b}"` before each
     solve, which the 2-config branch never did — for `len(seq) == 2`
     this adds exactly one new console line. A full real re-run diffed
     against the Step 0 baseline confirmed that line is the *entire*
     diff — same config generation, same eventual max-iterations failure,
     no spurious playback attempt on failure.
7. **`_solve_manipulation_planner`** (Step 6b) — mechanical extraction of
   the (already-verified) merged loop. Returns `(path_ids, success)`
   rather than the plan's originally-stated `List[int]`, since `run()`'s
   playback gate needs the last segment's `success` too.
8. **`_solve_transition_planner`** (Step 7) — the transition-planner
   solve path (resolve inputs → apply optimizer config →
   `plan_transition_sequence`) as its own method returning `int`.
   Playback and result-dict building stay in `run()`. Drops the plan's
   originally-stated `max_iterations` parameter — never actually used by
   `plan_transition_sequence`, so including it would've been dead weight.
   Confirmed (again) this branch is unreachable from any real script.
9. **`_build_result`** (Step 8) — replaces the 2 inline return-dict
   literals with one `_build_result(configs, **extra)`. 3 unit tests.
10. **Step 9 — slim `run()` to the orchestrator.** By Step 8 the
    incremental extractions had already reduced `run()` to 108 lines with
    the target shape (validate → generate configs → visualize → dispatch
    → play/record → build result) — no further code changes needed. Step
    9's job was the full-integration regression check: a complete real
    re-run of `interactive_planning.py --solve --no-viz`, diffed against
    the Step 0 baseline. **Diff: the one disclosed "Segment 1/1" line,
    nothing else.** All 25 unit tests passed. This was the main
    regression gate for the whole refactor.
11. **Step 10 — docstring cleanup.** `run()`'s docstring covered
    `visualize`/`solve`/`record`/`output_dir`/`video_name`/`framerate`
    but omitted `preferred_configs`, `max_iterations`, `solve_mode`,
    `transition_edges`, `transition_waypoints`,
    `generate_waypoints_via_edges`. Documented all of them.

### Phase 2B — `ManipulationTask.setup()` (`tasks/base.py:263`, 169/3) — ✅ Done

Flagged as a follow-up when Phase 2 landed, now detailed. Read in full
this session (263-431) — no duplication puzzle here, `setup()` is a
straight-line sequential pipeline (unlike `run()`'s branching), so this
is a legibility split, not a consolidation. Six seams, in source order:

1. **`_log_setup_snapshot(setup_params)`** — the config-snapshot
   `run_logger.log_task_config(...)` call + its own try/except (289-306).
2. Scene build + `q_init` (308-319) stays inline in `setup()` — it's the
   one truly orchestration-level step (assigns `self.planner`,
   `self.robot`, `self.ps`, `self.q_init`), extracting it would just move
   four assignments without reducing complexity.
3. **`_apply_optimizer_config()`** — the `RANDOM_SHORTCUT_LOOPS`/
   `SPLINE_ZERO_DERIVATIVES_AT_STATE` → `configure_transition_planner(...)`
   block (321-337).
4. **`_apply_time_parameterization_config()`** — the `TIME_PARAM_METHOD`/
   `TOPPRA_*` → `configure_time_parameterization_method(...)` block
   (339-357). **Note**: this and step 3 share an identical shape (build a
   kwargs dict from a `(task_config field, kwarg name)` list, call a
   `self.planner.configure_*` method if the dict is non-empty) — tempting
   to generalize into one `_apply_config_fields(configure_method, field_map)`
   helper. Decide explicitly rather than defaulting to it: two call sites
   is thin justification for an abstraction layer, and the method names
   differ per call. Recommend keeping them as two named methods unless a
   third caller of the same shape shows up.
5. **`_setup_locked_joint_constraints(freeze_joint_substrings)`** — the
   pattern-resolution + `ConstraintBuilder.create_locked_joint_constraints(...)`
   block (366-397). Returns `graph_constraints`.
6. **`_finalize_graph_setup(graph_constraints, skip_graph)`** — the
   `if skip_graph: ... else: ...` block (399-430) that either wires a
   graph-less `GraphBuilder` or creates the full graph + `ConfigGenerator`.

Verification: `setup()` is called by every real script before `run()`/
`plan_sequence()`, so Phase 2's own baseline captures
(`interactive_planning.py --solve --no-viz`, plus the `task_graspball_inbox.py`
attempt that's blocked on the missing `hpp_practicals` package — see Phase
2 Step 0) already exercise this method end-to-end on every re-run. No new
harness needed; reuse the existing regression-diff model (diff full
console output against the last-known-good Phase 2 baseline after each
step). Unit-testable pieces: steps 3-5 are pure enough (given a
`task_config`-like object and mocked `self.planner`/`self.ps`) to get
direct unit tests in `tests/test_task_base_helpers.py`, extending the
suite Phase 2 already built there rather than starting a new file.

## Decision needed: the dead `solve_mode` gate

`--solve-mode transition-planner` has never taken effect through any real
script in this codebase — the only script exposing the flag
(`interactive_planning.py`) always produces a 2-entry config sequence, and
`run()` only checked `solve_mode` when the sequence had more than 2
entries. Fix it (make the 2-config case honor `solve_mode` too) as an
explicit, separately-committed bug fix — before, after, or decoupled from
this refactor? Not doing it silently either way. Step 6a preserved the
current (likely accidental) gate exactly. **User confirmed** (2026-08-08):
don't fix now, just note it for revisit later — done, here.

### Phase 3 — mid-size hotspots (`planning/`, `backends/pyhpp.py`) — ✅ Done

All four steps landed 2026-08-09, in the planned risk order (3.1 → 3.2 →
3.3 → 3.4), commits `1941ea8` (3.1+3.2, bundled — both touch `planning/`
and share one new test file), `f8f8ded` (3.3), `750494c` (3.4). 61 new
unit tests across `tests/test_planning_helpers.py` (new) and
`tests/test_pyhpp.py` (extended). Full regression gate: a real
`interactive_planning.py --solve --no-viz` run after all four steps
produced console output **byte-identical** to the run captured after
step 3.1 alone (diffed, zero lines differed) — same 3-node/8-edge
factory graph, same 4 generated configs, same eventual
`max_iterations=5000` planning failure. Full unit suite: 197 passed
(same pre-existing 7 failures as the Phase 1B/2B baseline, all
unrelated to `planning/`/`backends/pyhpp.py` — missing `spacelab_config`
module, a viewer test stub, a config-dir lookup).

One relocation pitfall worth recording: `planning/graph.py`'s original
source had a **literal `✓`/`⚠` escape-sequence typo** (six
literal characters `\`, `u`, `2`, `7`, `1`, `3` in the source, not the
rendered ✓ glyph) in exactly one pair of print calls, inconsistent with
every other ✓/⚠ in the same file. `Read` silently renders that escape
back to the glyph when displaying file contents, which produced a
byte-for-byte mismatch on the first extraction attempt (confirmed via a
non-ASCII character-frequency diff between the original and relocated
blocks) — caught and fixed by writing the literal escape bytes directly
via a small Python script rather than through the glyph-rendering
`Edit` tool. Preserved as-is (not "fixed" to the rendered form) per this
refactor's "byte-identical relocation" rule.

All four candidates read in full this session (2026-08-09). None of them
have Phase 1's duplication puzzle — each is a single linear-ish pipeline
or a clean backend if/else split, so this phase is legibility-focused,
same character as Phase 2. Ordered below by risk (lowest first), which
is also **recommended execution order** — start with 3.1, it's the
cleanest seam in the whole plan.

#### Step 3.1 — `planning/scene.py::disable_collisions_between_subtrees()` (265-477, 213/4) — lowest risk

The entire body is one `if self.backend == "pyhpp": ... return self`
(294-398, includes three small nested closures —
`_resolve_joint_id`/`_subtree_joint_ids`/`_geom_ids_for` — used only
within this branch) followed by the CORBA-path fallthrough (400-477).
This is a pre-existing backend dispatch, not entangled logic — split
along the boundary that's already there:

1. **`_disable_collisions_pyhpp(robot_frame_or_joint, obstacle_root_joint,
   verbose, max_pairs)`** — the whole pyhpp branch, closures included,
   relocated verbatim.
2. **`_disable_collisions_corba(robot_frame_or_joint, obstacle_root_joint,
   remove_collision, remove_distance, verbose, max_pairs)`** — the whole
   CORBA branch, relocated verbatim.
3. `disable_collisions_between_subtrees()` becomes: print the header line,
   dispatch on `self.backend`, return `self`. ~10 lines.

Verification: exercised by any scene-setup call in either backend; the
pyhpp branch specifically runs on every container-based baseline capture
used throughout this doc (it's part of `SceneBuilder.build()`'s standard
collision setup). Diff console output (the `removed N collision pair(s)`
counts) against a fresh baseline before/after.

#### Step 3.2 — `planning/config.py::generate_via_edge()` (288-462, 175/7) — deepest nesting in Phase 3

A `for i in range(self.max_attempts)` retry loop; each iteration
backend-dispatches (corba vs. pyhpp, 336-413) to generate a candidate
config, then runs a finite-value/debug check (419-437), then validates
collision (`self.is_config_valid`, already extracted). The backend
dispatch — especially the pyhpp branch's freeflyer-DOF-preservation
`try/for` block (376-390) — is what drives the depth to 7; pulling it out
drops the loop body back to depth ~3-4.

1. **`_generate_candidate_config(edge_name, q_from, q_hint, use_hint)`**
   — the backend if/else (336-413), returns `(success, config, err)`.
   Pure relocation; still branches on `self.backend` internally (that's
   the existing contract, not something to unify) but as one named unit
   instead of inline.
2. **`_check_config_finite(config, edge_name, attempt_idx, verbose)`** —
   the debug-print + `np.isfinite` check (419-437), returns bool.
3. `generate_via_edge()`'s loop body becomes: timeout check → generate →
   finite check → validity check → success handling. Depth drops from 7
   to roughly 3.

Verification: this is the single most call-frequency-critical function
in Phase 3 — every phase transition in `GraspSequencePlanner` calls it
repeatedly (directly and via `_plan_release_subphase`'s blocks 3-5 in
Phase 1B). Use the same phase-0 smoke baseline as Phase 1B (17.45s, no
release) for fast per-step iteration, since this function is on that
code path too.

#### Step 3.3 — `backends/pyhpp.py::plan_transition_edge()` (1945-2215, 271/4)

A five-stage linear pipeline: setup budget → validate endpoints (optional,
`if validate:`, 2003-2097, ~95 lines) → project q2 onto the constraint
leaf → try directPath, fall back to computePath/planPath → time-parameterize
and store. Already surrounded by same-named helper conventions
(`_compute_planning_budget`, `_project_onto_leaf`,
`_configure_transition_planner_for_edge`, `_apply_time_parameterization`,
`_is_pregrasp_edge`/`_is_grasp_edge` all already exist nearby in this
file) — this step continues that existing pattern rather than inventing
a new one.

1. **`_validate_edge_endpoints(tr, edge_name, q1_arr, q2_arr)`** — the
   entire `if validate:` block (2003-2097). Self-contained: only prints
   warnings, doesn't affect control flow afterward (confirmed — nothing
   after line 2097 reads any variable this block defines). Lowest-risk
   extraction in this step.
2. **`_try_direct_path(tp, edge_name, q1_arr, q2_arr, validate,
   skip_direct_path)`** — the directPath attempt (2127-2150), returns
   `pv` or `None`.
3. **`_compute_or_plan_path(tp, q1_arr, q2_arr, reset_roadmap, edge_name,
   skip_direct_path)`** — the computePath/planPath fallback with its
   TypeError-triggered older-binding fallback (2160-2197), returns `pv`.
4. `plan_transition_edge()` keeps: setup, the `skip_direct_path`
   determination (2113-2123, cheap string checks — leave inline, not
   worth naming), calls to 1-3 in sequence, then the existing
   time-parameterize/store tail (2199-2215, already simple).

Verification: this is the actual C++-facing planning call, exercised by
every real planning run in the container (both the phase-0 smoke test and
any real script). Diff `[TP]`-prefixed console lines against baseline —
they're already structured/greppable, unlike most of this codebase's
`print()` output.

#### Step 3.4 — `planning/graph.py::build_phase_graph()` (1038-1336, 299/4) — highest line count, do last

Six sequential jobs: tear down the existing graph (1101-1134) → build
`phase_valid_pairs` from `held_grasps`/`next_grasp` (1136-1164) → derive
a filtered `phase_config` (restricted `GRIPPERS`/`OBJECTS`/
`HANDLES_PER_OBJECT`/`CONTACT_SURFACES_PER_OBJECT`, 1166-1220) → lock
non-participating objects' root joints (1222-1259) → apply
`SequentialGraspFilter` if requested (1264-1310) → reset free objects and
delegate to `create_factory_graph(...)` (1312-1336).

1. **`_teardown_existing_graph()`** — block 1. Backend-dispatched
   (CORBA deletes by name, pyhpp just drops the reference) but already a
   small, self-contained try/except.
2. **`_build_phase_valid_pairs(held_grasps, next_grasp)`** — block 2, pure
   function (no `self` access beyond the args), easiest of the six to
   unit-test directly.
3. **`_build_phase_config(config, phase_valid_pairs)`** — block 3, returns
   `(phase_config, phase_objects)` (the latter needed by block 4).
4. **`_lock_nonphase_objects(orig_objects, phase_objects, q_init,
   graph_constraints)`** — block 4, returns updated `graph_constraints`.
5. **`_apply_sequential_filter(phase_config, held_grasps, next_grasp)`**
   — block 5, mutates and returns `phase_config` (sets
   `_SEQUENTIAL_FILTER`) plus the `use_sequential_filter` fallback flag
   (it can flip to `False` on `ImportError`).
6. `build_phase_graph()` keeps: the header print, calls to 1-5 in
   sequence, the free-object reset call (already delegates to
   `self._reset_free_objects_in_q_init`, thin), and the final
   `self.create_factory_graph(...)` call.

This is the one Phase 3 target that's also on `_plan_release_subphase()`'s
call path (Phase 1B block 1 calls this directly) — sequence Phase 3.4
independently of Phase 1B (no shared code, just a caller/callee
relationship), but verify both together at the end since a regression
here would show up in Phase 1B's release-scenario tests too.

Verification: every phase transition calls this (it's what
`_build_phase_graph_and_constraints` — a Phase 1 output — wraps). Same
phase-0 smoke baseline covers the no-release path; a release scenario is
needed to exercise `_lock_nonphase_objects` with a non-empty
`_nonphase_objects` list (phase 0 has none), same caveat as Phase 1B
about the stale-`hpp-core` hang risk.

#### Phase 3 verification model

No existing test coverage for any of these four (confirmed: `grep -rln`
for each method name across `tests/` finds nothing). Given none of them
have Phase 1's "two functions must stay behaviorally identical" 
constraint, the bar is lower: relocate-and-diff-console-output is
sufficient for 3.1-3.3; 3.2 and 3.4 additionally get a handful of direct
unit tests for their now-pure sub-functions (`_build_phase_valid_pairs`,
`_check_config_finite`) in a new `tests/test_planning_helpers.py`,
following Phase 2's `tests/test_task_base_helpers.py` precedent rather
than inventing a new file-per-method convention.

### Phase 4 — `backends/corba.py` (2150 lines) — ⛔ excluded, out of scope

Confirmed with user (original scoping call, reconfirmed 2026-08-09): the
CORBA backend is being phased out entirely, not just deprecated-but-kept.
Excluded from this refactor — no investment in restructuring ~2150 lines
of code on its way to deletion. Revisit only if a decision is made to
keep it longer-term.

### Phase 5 — CLI/interactive glue — ⛔ decided against, not pursued

Found by the 2026-08-09 re-audit: two functions with the deepest nesting
in the entire codebase (depth 8, deeper than anything in Phases 1-3):

- **`utils/interactive.py::interactive_menu()`** (149-248, 100/8): an
  arrow-key terminal menu — `while True:` event loop, each iteration
  reads one keypress and branches on it (up/down/space/enter/quit), with
  small inline loops to clear previously-drawn lines.
- **`tasks/grasp_sequence.py::InteractiveGraspSequenceBuilder.
  _offer_replay_or_browse()`** (3523-3612, 90/8): CLI branching for
  "replay all / replay one phase / browse configs", each branch calling
  into `interactive_menu()` and then `GraspSequencePlanner` methods.

**User confirmed** (2026-08-09): not worth refactoring — both are
terminal-UI/CLI glue, not planning logic, and already read as coherent,
single-purpose state machines despite the raw depth number (the depth
comes from the key-handling state machine itself, not from accidental
complexity). Neither is on the BT-facing execution path, each is called
from exactly one place, and `interactive_menu()` already has test
coverage (`tests/test_refactored_modules.py::TestInteractiveMenu`) that
a refactor would have to thread carefully around for a benefit nobody
asked for. Closed, not deferred — revisit only if a future change to
either function needs to touch this code for an unrelated reason.

### Script directory (`script/`) — still deferred

Re-checked 2026-08-09: still 6985 LOC total, identical to the "Script
directory" findings above — no drift since the original audit. Still
excluded per that section's scoping call. No change to that decision.

### Cross-cutting (parallel-track, not file-scoped)
- `print()`-vs-structured-logging migration — ✅ **Done 2026-08-09**.
  Converted the progress/diagnostic `print()` call sites across all 10
  in-scope files (`planning/path_io.py`, `visualization/video_recorder.py`,
  `planning/constraints.py`, `visualization/live_graph_viz.py`,
  `tasks/base.py`, `planning/scene.py`, `visualization/viz.py` (partial),
  `planning/graph.py`, `backends/pyhpp.py`, `tasks/grasp_sequence.py`) to
  `logging.Logger` calls via `agimus_spacelab.logging.get_logger()`, with
  `configure_logging()` now wired into `ManipulationTask.__init__` so
  console output (and optional file mirror) actually appears — previously
  `configure_logging()` existed but nothing ever called it. Level policy:
  success/milestone → `INFO`, warnings/recoverable failures → `WARNING`,
  fatal errors → `ERROR`, per-attempt/verbose internals → `DEBUG`.
  Redundant `[prefix]`-style tags and severity symbols (`✓`/`⚠`/`✗`) were
  dropped where they only decorated severity now conveyed by the log
  level; kept where they're part of an actual computed value (e.g. a
  status ternary) or asserted verbatim by a pre-existing test. Decorative
  separator lines (`"="*70`) and interactive/CLI-facing output (arrow-key
  pickers, the CORBA backend, the module-level SIGINT handler, report-
  printing helpers whose `print()` output IS the product, e.g.
  `get_phase_summary()`) were deliberately left as `print()` — not in
  scope for this migration. Tests previously asserting on `print()` output
  via `capsys` were converted to `caplog`. Verified per-file via AST
  parse, full test-suite run (same 7 pre-existing unrelated failures
  throughout, no regressions), and a non-ASCII character-frequency diff
  against `git show HEAD:<file>` to catch any accidental content
  corruption. Also verified with a real end-to-end
  `interactive_planning.py --grasp-sequence ... --no-viz` run exercising
  `tasks/base.py`, `planning/scene.py`, `tasks/grasp_sequence.py`,
  `planning/constraints.py`, `planning/graph.py`, `planning/config.py`,
  and `backends/pyhpp.py` together — real HPP planning, successful
  2-edge sequence, clean leveled log output throughout.
- Hardcoded personal path default (`output_dir=
  "/home/dvtnguyen/devel/demos"`) — ✅ **Fixed 2026-08-09**, commit
  `8815609`. Turned out to be in 8 signatures across 5 functions/methods
  in 4 files (the earlier "6 places" count only found the code lines
  matching a raw path-string grep, undercounting slightly): `tasks/base.py
  ::run()`, `tasks/grasp_sequence.py::replay_sequence()`,
  `backends/corba.py::play_and_record_path`/`play_and_record_path_vector`,
  `backends/pyhpp.py::play_and_record_path`/`play_and_record_path_vector`,
  `visualization/video_recorder.py::VideoRecorder.__init__`/
  `record_path_playback`. Fixed with a single shared
  `default_video_output_dir()` in `video_recorder.py` (env var
  `AGIMUS_VIDEO_OUTPUT_DIR`, else `~/devel/demos` for whichever user is
  running) — every signature now defaults `output_dir` to `None` and
  lets it flow down the existing call chain to the one place
  (`VideoRecorder.__init__`) that actually resolves it, so no behavior
  change for any caller already passing an explicit `output_dir`.
  13 new unit tests, incl. a regression guard asserting all 8 signatures
  default to `None` (not a literal path). 217 tests pass total, same 7
  pre-existing unrelated failures.
- `run()`'s docstring gap (Phase 2 Step 10 fixes this one instance;
  worth a broader pass later, not scheduled yet).

## Decisions (confirmed by user)
1. Phase 1 (`grasp_sequence.py`) proceeds ahead of Phase 2 (`run()`).
2. Phase 4 (CORBA backend) is excluded — being phased out entirely.
   Reconfirmed 2026-08-09.
3. The `solve_mode` dead-gate (Phase 2) is not fixed now — noted for
   revisit later.
4. Phase 5 (CLI/interactive glue: `interactive_menu()`,
   `_offer_replay_or_browse()`) is not pursued — already coherent enough
   to not be worth refactoring. Confirmed 2026-08-09; see Phase 5 section
   above.
5. The Phase 1 logging asymmetry — fix now, decoupled from the refactor.
   Confirmed and landed 2026-08-09; see "Decision needed: the logging
   asymmetry" above for what changed.
6. The hardcoded `/home/dvtnguyen/devel/demos` default — fix with a
   single shared-default. Confirmed and landed 2026-08-09; see the
   cross-cutting section above for what changed.
7. The `print()`-vs-structured-logging migration — do it now. Confirmed
   and landed 2026-08-09; see the cross-cutting section above for what
   changed and what was deliberately left as `print()`.

## Decisions still open (not yet confirmed by user)
None — the last open item (whether/when to do the logging migration) was
confirmed and completed 2026-08-09.

## Status: refactor complete

As of 2026-08-09, every phase this doc identified has a final
disposition — done, excluded, or decided against — none left "not yet
decided":

| Phase | Target | Outcome |
|---|---|---|
| 1 | `GraspSequencePlanner.plan_sequence()`/`resume_sequence()` | ✅ Done |
| 1B | `_plan_release_subphase()`, `replay_sequence()` | ✅ Done |
| 2 | `ManipulationTask.run()` | ✅ Done |
| 2B | `ManipulationTask.setup()` | ✅ Done |
| 3 | `planning/`+`backends/pyhpp.py` mid-size hotspots (4 targets) | ✅ Done |
| 4 | `backends/corba.py` | ⛔ Excluded (deprecated) |
| 5 | CLI/interactive glue (2 targets) | ⛔ Decided against |

What's left is explicitly **not** refactor work. Both cross-cutting bugs
found along the way (logging asymmetry, hardcoded path default) and the
`print()`-vs-logging migration are now all fixed/done too (2026-08-09) —
nothing tracked in this doc remains open. The `script/` directory (6985
LOC) remains out of scope per its original scoping call (CLI entry
points/worked examples, not library code). None of these block calling
the codebase refactor itself finished.
