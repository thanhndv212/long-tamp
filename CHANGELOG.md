# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
(`0.x` while the API is still moving).

## [Unreleased]

### Added

- `long_tamp.grasping`: a grasp planner, separate from motion planning.
  `GraspPlanner` samples, evaluates and ranks parallel-jaw grasps on an object's
  URDF collision primitives and closes the fingers at any grasp pose, including
  hand-written SRDF handles; planned grasps export as SRDF `<handle>`s.
  `ParallelGripperModel` presets: `ROBOTIQ_2F85` (stroke and knuckle envelope
  calibrated by forward kinematics of the repo's URDF) and `PANDA_HAND`.
  `FingerClosureTable` maps each `(gripper, handle)` pair of a task YAML to the
  finger joint values to command.
- `tasks/task_planning/grasp_capability.py`: `plan_grasp`, `grasp_feasible`,
  `close_gripper`, `open_gripper` capabilities for task plans / the BT host.
- `script/grasp_planning/`: `plan_grasps.py` (rank grasps, check an object's
  handles, emit SRDF) and `validate_closure.py` (check closures against the real
  Robotiq meshes with pinocchio + coal).

### Fixed

- The Robotiq fingers never closed on the drill in the screw-assembly viewer:
  planned paths keep them frozen open, and native playback showed exactly that
  (pads 24 mm off the handle). `MissionViewer(closures=...)` now closes them to the
  grasp planner's width at each grasp and opens them at each release.
  `replay.py` used a fixed `finger_joint = 0.6` for every object, which put the
  pads 6.8 mm into the drill handle; it now uses the same closures (0.496 on the
  drill, 0.567 on a part's tab).

## [0.1.0] - 2026-09-27

First public release, on PyPI as `long-tamp`.

### Added

- PyPI-wheel CI on pushes and pull requests, nightly one-part screw-assembly runs,
  distribution checks, and a tag-triggered trusted-publishing workflow.
- Cloud-only Claude Code SessionStart hook installing `[hpp,dev]` dependencies.

- `script/screw_assembly/`: a long-horizon, multi-arm example built from generic
  primitives. Two UR10 + Robotiq arms fasten N plates to a jig, two screws each, with a
  tool pickup, home retreats and a tool return (4 parts: 19 blocks, 31 grasp/release phases).
  Over 10 seeded 4-part runs with the cordless drill: 10/10 missions,
  0/140 blocks replanned, 13/13 failures recovered. The scene
  is generated from parameters (`build_scene.py --parts N`) and has no lineage to real
  product CAD. `run_batch.sh` / `summarize.py` run seeded batches and report replanning and
  recovery rates.
- `long_tamp.tasks.block_recovery.run_block_with_recovery()`: plans a short grasp sequence
  as one unit and escalates on failure -- in-phase redraws, then `resume_sequence()`, then
  a replan of the whole block from its entry with fresh lookahead hints (on a broken hint
  chain, an *unreachable* phase per `last_edge_failure`, or 40 failed resumes). Bounded by
  `max_replans`. `make_lookahead_hints_factory()` builds the per-attempt hints.
- `find_feasible_phase_target(verify_paths=True)`: also path-plans the candidate's own
  edges and each protected phase's edges, on a short budget (`path_check_timeout`,
  `path_check_iterations`), rejecting candidates the arms can't actually move to or that
  block a later approach.
- `long_tamp.tasks.mission_checkpoint.MissionCheckpoint`: one folder per mission run that
  is both its run log (`mission.json`: metadata and one record per block) and its resume
  point (`checkpoint.json`: next block, configuration, held grasps), rewritten atomically
  after every block. The screw-assembly runner uses it for `--run-dir` / `--resume`, and
  records the motion into the same folder for the new `replay.py` viser player.
- `configure_transition_planner(spline_optimizer=False)` drops `SplineGradientBased` from
  every edge's optimizers: its inner QP solve ignores the optimizer timeout (a run hung
  11+ minutes in one solve).
- `YamlTaskLoader` resolves relative `paths:` entries against the YAML file's folder, so
  one config works on the host and in the container.
- `configure_transition_planner(path_optimizer_timeout=...)` (default 30 s).

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
- `find_feasible_phase_target()` takes `also_reachable`: further grasps a phase-N candidate
  must also leave reachable, beyond phase N+1. A grasp that fixes a part's orientation
  fixes it for every later contact on that part, so a candidate can pass the N+1 probe and
  still strand a later grasp (solver failures only, never a collision) that no retry can
  recover. `run_sequence()` exposes it as `lookahead_also_protect` (`{i: [j, ...]}` for
  entries of `lookahead_pairs`). Ported from `agimus_spacelab`, where checking the later
  grasp took a multi-arm assembly mission from stalling in 3 of 4 runs to 10/10 completions.
- `ConfigGenerator.last_edge_failure`: the most recent failed `generate_via_edge()` call's
  attempt breakdown (`solver_failed` vs `collision_invalid`), so a caller can tell a target
  that is unreachable from the current commitment from one that is merely obstructed.

### Fixed

- The PyPI (cmeel) HPP wheels import without `LD_LIBRARY_PATH`: their extension modules
  carry no RPATH, so `long_tamp.backends.pyhpp` now preloads the HPP libraries from
  `cmeel.prefix/lib` when the plain import fails on a missing shared library. Each library
  is loaded once, by soname (the wheels ship `libhpp-util.so` and `libhpp-util.so.9.0.2`
  as two copies; mapping both corrupted the heap at exit).
- URDF mesh paths are resolved at load time (`_urdf_paths.resolve_mesh_paths`), so the
  examples load outside the hpp-agimus container: relative paths against the URDF's folder,
  and absolute paths from another machine by their tail under the URDF's parent folders.
  `script/ikea_table_prototype`'s config now names its URDFs relative to itself, and
  `screw_assembly/build_scene.py` writes the drill mesh path relative (no `--repo-root`).
- `print_joint_info()` skips joints without a configuration rank (`universe`, listed by
  HPP 9.0.2's `getJointNames()`), which raised `KeyError`.
- The PyHPP backend's import error no longer says the PyPI wheels lack long_tamp's
  bindings: HPP 9.0.2 has them (the full screw-assembly mission runs on the wheels).

- The native `SIGSEGV` at the "target generated -> path planning begins" transition (and
  "Maximal number of iterations reached" failures within seconds regardless of budget).
  `Problem(device)` builds its `WeighedDistance` right after the first robot is loaded, so
  every joint loaded later had no weight and distances read uninitialized memory. NaN
  distances left the roadmap's nearest-node search empty, and `Roadmap::addNode`
  dereferenced the null node (confirmed from a core dump). The backend now rebuilds the
  problem's distance after each load that adds joints.
- Lookahead probes (`find_feasible_phase_target`) now lock and freeze joints exactly like
  the real phase, including `config_gen.set_frozen_joints()`. They inherited the previous
  phase's frozen set, so a probe could hold the very arm it needed to move and reject every
  candidate.
- A path failure to a lookahead-hinted target is retried (`_HINTED_PATH_RETRIES`, 2) before
  the target is redrawn; the redraw voids the hint chain and forces a block replan.

- `script/ikea_table_prototype/task_assemble_table.py`: `PER_PHASE_FROZEN_ARMS` froze
  `ur10_left` for phase 0 despite its own comment explaining that phase specifically needs
  `ur10_left` free (it parked in leg1's pregrasp corridor otherwise) -- the freeze was never
  actually lifted.
- `script/ikea_table_prototype/task_assemble_table.py`: the viewer started before planning
  (`task.planner.visualize(q_init)` right after setup), the same viser-thread-vs-native-RRT
  concurrency bug `script/twin/task_lift_ball.py`'s `run_task()` already documented and
  avoided. Fixed the same way (viewer now starts only after planning completes).
- Phase-graph builds raised `AttributeError` on stock `hpp-python`: `PrunedRecursionMixin`
  used `_visitedGrasps` without creating it, relying on a patched `hpp-python` fork's
  `ConstraintGraphFactory` to. The mixin now creates it itself.
- `from long_tamp import *` raised `AttributeError`: `__all__` listed `PlanningBridge`,
  `TaskBuilder` and `TaskOrchestrator`, which don't exist.
- A gripper on a free-flying tool is now resolved to whichever arm currently holds the
  tool (recursively, via the live grasp state), and arm groups may list it under several
  arms. Auto frozen-arms mode previously froze the actual carrier whenever the "other" arm
  held the tool.
- `plan_loop()` now projects a live start configuration onto the held-grasp state, as
  `plan_sequence()` already did. A start from another stack's state estimate misses the
  grasp constraint by millimetres and was rejected by the loop edge.

### Changed

- `task_config.JOINT_GROUPS` (optional) is now keyed by the same arm keywords as
  `ALL_ARM_KEYWORDS`; the planner no longer hardcodes a keyword-to-group map for
  per-phase TOPPRA joint selection, so any robot's config can use it.
- Comments, docstrings, docs and test fixtures use generic scene names instead of
  identifiers inherited from the pre-split mission.
- Per-phase planner dumps read `LONG_TAMP_CHECKPOINT_DIR`, and the video default
  `LONG_TAMP_VIDEO_OUTPUT_DIR`; the pre-split `AGIMUS_*` names are still honored.

- Target generation seeds IK with an unvalidated random draw
  (`random_config(validate=False)`). Rejection-sampling a collision-free seed, whose object
  poses were then overwritten anyway, cost millions of wasted collision checks per mission.

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

- The two `slow_planning` TWIN integration checks
  (`test_grasp_release_use_case_twin.py`, `test_twin_regrasp_bt_session.py`) are
  unreliable: a local ARM64 rerun hit a finger/ball collision in one and the process
  time limit in the other, although both passed in an earlier cloud run. They are
  excluded from push/PR CI and run nightly.

- `SplineGradientBased` can hang inside a single QP solve (proxsuite, uncapped iterations),
  which `PathOptimizer/timeOut` cannot interrupt; one run was stuck 11+ minutes. Worked
  around with `configure_transition_planner(spline_optimizer=False)`; a proper bound
  needs an hpp-core change. See `docs/bugs/hpp-core-unbounded-planning-loops.md`, Bug 6.

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

[Unreleased]: https://github.com/thanhndv212/long-tamp/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/thanhndv212/long-tamp/releases/tag/v0.1.0
