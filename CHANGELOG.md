# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
No release has shipped yet (see `pyproject.toml`'s `0.1.0`/Alpha status) --
entries accumulate under **Unreleased** until the first tagged release.

## [Unreleased]

### Added

- `create_twin_regrasp_session`, a real-mission BT scenario (`script/twin/twin_bt_session.py`,
  `src/long_tamp/tasks/task_planning/host.py`) where a gripper grasps a handle, then a
  `fallback`/`condition` guard forces a real `release()` and re-`grasp()` of that same
  handle -- the first plan document to compile a top-level `fallback`/`condition` composed
  with nested `transaction`s, proving the compiler's `Fallback`/`retry` shapes compose
  correctly one level above a single transaction. Verified via a new compiler unit test, a
  Python session-dispatch integration test, and a new opt-in `taskplan_bt_twin_regrasp`
  CTest driving the actual compiled `agimus_taskplan_bt` binary.
  (`docs/usage/behaviortree-integration.md` §11 item 1.)
- `GraspSequencePlanner.grasp()` accepts an optional `q_hint` (a
  `find_feasible_phase_target()` candidate, or a legacy single config), forwarded to
  `_plan_phase_edges()` as a `phase_q_hints` entry -- previously only `plan_sequence()`'s
  own `phase_q_hints` plumbing supported this.
- `run_sequence()` (`src/long_tamp/tasks/sequence_orchestrator.py`) accepts
  `per_phase_frozen_arms` (remapped per-call to each `grasp()`/`release()`'s own internal
  `phase_idx=0`) and `lookahead_pairs`, which probes `find_feasible_phase_target()` against
  the next phase before committing a grasp -- reproducing, for the capability-driven path,
  the same failure-class fix `find_feasible_phase_target()` already gave `plan_sequence()`
  callers (previously wired into SpaceLab's `run_block_nonstop()` only).

### Fixed

- `script/ikea_table_prototype/task_assemble_table.py`: `PER_PHASE_FROZEN_ARMS` froze
  `ur10_left` for phase 0 despite its own comment explaining that phase specifically needs
  `ur10_left` free (it parked in leg1's pregrasp corridor otherwise) -- the freeze was never
  actually lifted.
- `script/ikea_table_prototype/task_assemble_table.py`: the viewer started before planning
  (`task.planner.visualize(q_init)` right after setup), the same viser-thread-vs-native-RRT
  concurrency bug `script/twin/task_lift_ball.py`'s `run_task()` already documented and
  avoided. Fixed the same way (viewer now starts only after planning completes).

### Changed

- `script/ikea_table_prototype/`: cleared for general use. The IKEA LACK table assembly
  example was scoped as a private, local-only prototype (its own README said it must never
  be merged into `main` or pushed to `origin`, despite already being in both) -- now the
  intended proven long-horizon example for the project, so the "local prototype only" /
  "NOT for public release" language was removed from the README and the scattered per-file
  references to it.
- `script/ikea_table_prototype/task_assemble_table.py`: `GRASP_SEQUENCE` now has all 12
  phases uncommented (was 1), and `run_task()` drives it through `run_sequence()` with
  `lookahead_pairs` on every leg's grasp -> dock pair instead of `plan_sequence()`.
  Verified the lookahead mechanism finds real, better candidates (residual 0.049 vs.
  1.6-3.9 unprotected) but a full 12-phase run hasn't landed yet -- see that file's
  module docstring `STATUS` section for the remaining gap (hint invalidation on a
  mid-phase collision retry has no retry/re-roll loop yet).

### Known issues

- A native `SIGSEGV` at the "target generated -> path planning begins" transition,
  deterministic on `script/ikea_table_prototype/task_assemble_table.py`'s scene (3/3),
  intermittent elsewhere (e.g. `create_twin_session`, ~1/5 observed). Classic race-condition
  signature -- avoided when run under `gdb` (changes thread timing), not root-caused. Real
  runs of `task_assemble_table.py` should go through
  `gdb -batch -ex run -ex quit --args python3 ...` until this is fixed.
- The `f_12` pregrasp -> grasp waypoint collision documented in
  `tests/test_grasp_release_use_case_twin.py` is markedly worse than "intermittent" when it
  is the target of a release-then-regrasp cycle specifically: 100% of regrasp draws hit it
  across two independent verification runs vs. 0% of first-grasp draws in those same runs.
  `create_twin_regrasp_session`'s `grasp` capability raised `max_attempts` from 3 to 8 to
  compensate; not root-caused.
- Grasp/target-generation difficulty for a given edge is not solely a function of which
  gripper/handle pair it names -- the same `panda_right/gripper > ball/handle2` grasp plans
  in ~18s as the *second* phase of a multi-grasp sequence but failed 6/6 draws when built as
  the *only* phase of a single-gripper session. Not root-caused.
