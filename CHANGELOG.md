# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
(`0.x` while the API is still moving).

## [Unreleased]

### Added

- MuJoCo export (#17), `long_tamp.sim.mjcf`: `export_mjcf(config, out_dir)` writes a
  task's scene as one self-contained MJCF (HPP's body and joint names, objects free at
  their initial pose, Robotiq mimic joints as equalities, COLLADA meshes converted);
  `qpos_from_pinocchio` maps an HPP configuration to MuJoCo `qpos` and `fk_mismatch`
  compares the two models' kinematics (screw-assembly cell: < 1e-7 on 60 bodies). CLI:
  `python -m long_tamp.sim.mjcf CONFIG -o OUT`. New `sim` extra (mujoco, trimesh,
  pycollada).

## [0.4.0] - 2026-09-29

Milestone M3: automatic task planning. Capabilities, a world state and a goal
export to PDDL; a classical planner (Fast Downward, run directly, or any Unified
Planning engine) turns the goal into a plan skeleton that becomes a validated
TaskPlan; refinement failures come back as facts (`cannot_reach`,
`ik_unreachable`, `release_infeasible`, `lookahead_failed`, `blocks`) and a
bounded loop blocks what failed and replans. The screw assembly now has real
choices (spare clamps, free part order): with an injected `cannot_reach` it
replans the part into another clamp and completes (the M3 exit test). Validated
with the screw-assembly batch gate on planner-ordered missions (10/10, median
772 s, source-built HPP).

### Added

- PDDL export (#13), `long_tamp.tasks.task_planning.pddl`: `to_pddl(descriptors,
  init, goal)` writes a domain (one action per capability with effects; wildcards
  as `exists`/`forall`) and a problem any classical planner reads, with a
  reversible renaming of names like `ur10_left/gripper`; `static_preconditions`
  bound parameters with export-only facts. `from_pddl_plan` maps a plan back to
  capability calls. `screw_domain.pddl_problem(n)` exports the screw-assembly
  mission; Fast Downward (through Unified Planning, new `planning` extra) plans
  it for 1, 2 and 4 parts and from partly done states, and every plan is checked
  step by step against long_tamp's own semantics.
- Plans from a goal (#14), `long_tamp.tasks.task_planning.skeleton`:
  `UnifiedPlanningPlanner` solves a PDDL export into a skeleton (Fast Downward,
  else pyperplan after Unified Planning compiles the problem down to STRIPS) and
  `skeleton_document` turns it into a TaskPlan document validated like a
  hand-written one. The screw assembly plans its mission from the world state
  with `--planner up`; each step's block is rebuilt from its capability and
  parameters (`screw_domain.block_for`), and `run_batch.sh` passes mission
  options through. The `planning` extra installs `up-fast-downward` only where
  it has wheels, plus `up-pyperplan`.
- Replanning around failures (#15): refinement failures report
  `cannot_reach`, `ik_unreachable`, `release_infeasible`, `lookahead_failed`
  `(gripper, handle)` and `blocks(body, body)` from a collision;
  `to_pddl(blocked=...)` rules out bindings; `repair.plan_execute_repair` plans
  from the world state, executes, blocks what failed through a policy and
  replans (bounded). Screw assembly: `--replan ROUNDS`, `--inject-failure`, and a
  failed block resets the grasp tracker to its start.
- Screw assembly with real choices (#16): `build_scene.py --clamps M` adds spare
  jig clamps that take any part's seat (the default scene is unchanged), the
  planning domain reads the (clamp, seat) choices from the scene, the goal asks
  for every part screwed in any clamp, and part order is free. With a spare
  clamp, an injected `cannot_reach` on a clamp replans the part into another one
  (the M3 exit test). Either arm driving is left to #60 (the cell is built for
  ur10_right to drive).
- `FastDownwardPlanner` runs Fast Downward (https://github.com/aibasel/downward)
  directly on the exported PDDL, with no compilation step; `default_planner`
  prefers it when an executable is found (`LONG_TAMP_FAST_DOWNWARD`, `PATH`, or
  the one bundled with `up-fast-downward`). `UnifiedPlanningPlanner("auto")`
  warns when it falls back to pyperplan, which is slow and can hang on larger
  problems.

### Changed

- CI: TWIN is no longer checked in CI. Its handover location is random and
  often infeasible (a 5 cm ball between two Panda hands), so its checks passed
  or failed by chance; the scripts and tests stay as an example to run by hand.
  `tests/test_grasp_release_screw.py`, seeded, on the screw-assembly cell, is
  the real-scene check of `grasp()` and `release()` (new `nightly-grasp-release`
  job) (#54).
- Refiner failure facts renamed for #15: `unreachable` -> `ik_unreachable`,
  `phase_failed` -> `cannot_reach` (or `release_infeasible` for a release phase).

### Fixed

- Releases recover from more failures (#54): a release now runs in up to
  `1 + _MAX_GENERATION_RETRIES` rounds, each redrawing its pregrasp from the held
  configuration (before, the pregrasp was drawn once, and the pregrasp -> free
  step only retried from the pregrasp already reached); nothing is committed
  before a round succeeds. The held configuration is projected onto the grasp's
  constraints only when the projection stays collision-free.
- TWIN: the ball's y bound widens to +-0.6 m. A dual-arm hold could leave the
  ball at y = -0.43..-0.44, and every release from there was rejected as out of
  bounds (#54).
- The screw-assembly `--run-dir` help now lists every file in the run folder.

## [0.3.0] - 2026-09-28

Milestone M2: executor contract and refiner interface. Planned motion runs on
pluggable execution backends under a supervised contract (heartbeats,
duration-scaled deadlines, BUSY retries, pause/stop/breakpoints); a Python
executor runs TaskPlans on it; refinement sits behind a `Refiner` interface that
reports failures as facts; the Python executor and the BehaviorTree.CPP host write
the same event stream; and a mission killed mid-run resumes from world state.
Validated with the screw-assembly batch gate on the executor and on the refiner
(10/10 missions each, 0 % replanning, 100 % recovery), kill-and-resume at three
kill points, and 4/4 initial-state scenarios.

### Added

- Execution contract (#8, ADR-0004), `long_tamp.execution`, ROS-free: backends
  implement a polled `start` / `poll` / `cancel` protocol with `Feedback`
  heartbeats; `run_command` supervises a command (BUSY retried with backoff,
  cancelled on heartbeat silence or past a deadline scaled by the command's
  duration and a minimum real-time factor, with the reason reported);
  `ExecutionControl` pauses, resumes, stops and sets breakpoints at step
  boundaries; `MockBackend` produces every status for tests.
- Python TaskPlan executor (#9): `PlanExecutor(session, backend).run()` plans
  each step through the session and executes the motion its capability submitted
  (`executor.submit(command)`) on the backend under `run_command`, with
  pause/stop/breakpoints at step boundaries; a failed execution fails its step.
  `run_plan` gained `before_step` / `after_step` hooks. `PathPlaybackBackend`
  plays time-parameterized paths (to a viewer or headless). Screw assembly runs on
  it, with `--backend none|mock|playback` (default: planning only).
- Refiner interface (#10, ADR-0001), `long_tamp.tasks.refiner`: a `Refiner`
  binds a `RefinementStep` (a grasp sequence, frozen arms, optional `Lookahead`)
  to geometry and returns a `Refinement`, which on failure carries ground facts
  for the task planner (`refinement_failed(step)`, `unreachable(gripper, handle)`,
  `phase_failed(...)`, `lookahead_failed(...)`). `GraspSequenceRefiner` wraps
  `run_block_with_recovery` and the phase-target lookahead; screw assembly uses it.
  `run_block_with_recovery` results gained a structured `failure` field.
- Mission event stream (#11), schema `long-tamp.events/1`
  (`docs/usage/events.md`): one JSONL event per status change of a plan node.
  `run_plan(..., on_event=)` and `PlanExecutor(..., on_event=)` emit it (the
  executor adds `motion` events with execution metrics), `JsonlEventWriter`
  writes it, and the C++ host writes the same stream with `--events <path>`.
  The compiler stamps `_ir_id` / `_ir_role` on every BT element it emits for an
  IR node (`COMPILER_VERSION` 1.1). The `taskplan_bt_events` CTest checks the host
  and the Python runner produce the same transitions. Screw assembly writes
  `events.jsonl` in its run folder. The fake host session gained a
  `"shape": "composite"` option.
- Kill-and-resume test (#12): `tests/test_kill_resume.py` SIGKILLs a mission
  mid-step and restarts it; the restarted run skips the completed steps from the
  recorded world state, redoes the interrupted one and completes.
  `script/screw_assembly/kill_resume.py` does the same on the screw-assembly mission
  (kill during or after a chosen step, then `--resume`), checked from the event
  stream. Documented as part of V4.

### Fixed

- Screw assembly: a mission killed after a part's clamp + screw block but before
  its release skipped the release on resume, ending with ur10_left still holding
  the clamped part. The `part_done` / `all_parts_done` guards now also require the
  carrying arm to have let go (the guard is labelled "partN assembled"). Found by
  the kill-and-resume check (#12).

## [0.2.0] - 2026-09-28

Milestone M1: state model and effect-based resume. Capabilities declare
preconditions and effects, plans are checked before any geometry runs, and a
mission skips work whose effect already holds in the world, so it resumes after a
restart and starts from partly done states. The screw-assembly example runs as a
TaskPlan. Validated with the screw-assembly batch gate (10/10 missions, 0 %
replanning, 100 % recovery) and 7/7 initial-state scenarios.

### Added

- Initial-state scenarios, validation level V4 (#6):
  `script/screw_assembly/scenarios.py` (4 scenarios) and `script/twin/scenarios.py`
  (3) plan part of the mission to reach a start state, then run the unchanged plan
  and check that the work already done is skipped, not planned again.
- `task_planning.runner.run_plan(session)`: runs a plan synchronously with the
  compiled BehaviorTree's semantics (sequence, fallback, retry, condition,
  transaction with the effect guard, precondition check and attempt budget);
  screw assembly and the scenarios run on it.
- Screw assembly runs as a TaskPlan (#5): `script/screw_assembly/screw_domain.py`
  declares the mission as capabilities with preconditions and effects (`holds`
  observed from the grasp tracker, `screwed` recorded in the run folder's
  `facts.json`) and one transaction per block; `task_screw_assembly.py` runs it
  step by step through a `TaskPlanningSession`. `--resume` now restores the world
  state and skips steps whose effects hold, instead of resuming from a block index.
- Plan simulation treats a transaction whose declared effects already hold as
  complete, matching the run-time effect guard, so one plan validates from any
  start state the guard can handle.
- Plan diagrams (#7): `task_planning.visualize.to_mermaid(plan)` and
  `to_dot(plan)` render a TaskPlan (fallback alternatives as dashed `else` edges,
  attempt budgets, optionally each step's grounded effects with `registry=`).
- Effect-based completion (#4, ADR-0002): with a `world_state`, a transaction
  is complete exactly when its grounded effects hold in the world. A step whose
  effect already holds is skipped without running (`effect already holds`), and
  one whose effect was undone runs again; `is_step_complete` reports the reason.
  Steps without declared effects, and sessions without a world state, keep the
  in-memory record. The TWIN sessions use the grasp tracker as their world state.
- World-state providers (#3, ADR-0002): `GraspTrackerState(planner)` reports
  observed `holds(gripper, handle)` atoms from the planner's grasp tracker
  (re-read on every call, since the planner replaces it on resume/reset); `RecordedFacts`
  holds facts no sensor shows afterwards (e.g. `screwed(part, hole)`), persisted
  atomically and reloaded on restart; `CompositeWorldState` merges sources into a
  session's `world_state=`. A session built with `recorded=` writes a step's
  grounded effects on the recorded predicates only when the step completes.
- Capability preconditions and effects (#2, ADR-0002): `CapabilityDescriptor`
  takes `preconditions=` and `effects=` as literals over the step's parameters
  (`"not holds(?gripper, _)"`, `"holds(?gripper, ?handle)"`), parsed and checked
  against `required_parameters` at construction
  (`long_tamp.tasks.task_planning.predicates`). A plan may declare an
  `initial_state`; `TaskPlan.from_dict` then simulates every branch and rejects a
  plan whose preconditions can fail, naming the step, literal and state, before
  any geometry runs. Sessions take an optional `world_state=` callable, and
  `TaskStepReady` then evaluates the step's preconditions against it. Both TWIN
  missions declare real literals and are checked this way.

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
- Screw-assembly batch gate: `script/screw_assembly/summarize.py --gate --baseline
  <results.json>` exits non-zero unless every mission completed, the replanning
  trigger rate is < 2 %, the recovery rate is > 95 % and the median time is
  ≤ 1.25× the baseline; `--json` records a result file in the committed schema.
- Contributor process: `CONTRIBUTING.md`, `docs/development/workflow.md` (issue →
  PR → release lifecycle, definition of done), `docs/development/validation.md`
  (validation levels V0–V4 on the screw-assembly mission), `docs/plans/roadmap.md`
  (milestones M1–M6), architecture decision records under `docs/adr/`, PR and issue
  templates, a pre-commit config mirroring the lint job, and Dependabot for actions.
- CI: a `docs` job (`mkdocs build --strict`, which now also fails on pages missing
  from the nav) and a `changelog` job on pull requests (library or example changes
  need a `CHANGELOG.md` entry unless labelled `skip-changelog`).

### Fixed

- The mission viewer played every completed block, then the whole mission again,
  in real time even with no browser connected (#35). Batch runs (which also
  started a viewer) lingered after each mission replaying it, grew to ~7 GB per
  process, and six in parallel exhausted a 16 GB container: the OOM killer took
  down running missions. Playback now happens only while a browser is connected
  (paths are still recorded for later), and `run_batch.sh` runs with `--no-viewer`.
- Joints frozen by `task.setup(freeze_joint_substrings=...)` (e.g. gripper
  fingers) were only kept frozen on idle arms: phase graphs rebuild the locked
  joints from the frozen *arms*, so the moving arm's "frozen" fingers took random
  widths in every generated configuration unless `GraspSequencePlanner` was also
  given the patterns (#28). On TWIN, a finger closing to 14.8 mm inside the 25 mm
  ball made the grasp pose collide, the cause of the long-standing flaky TWIN
  checks. `GraspSequencePlanner` now inherits the patterns `setup()` froze (pass
  `freeze_joint_substrings=[]` to opt out); this also fixes the templates and
  `interactive_grasp_sequence_builder`, which never passed them. The TWIN fingers
  are frozen fully open (0.04, clear of the ball by ≥ 15 mm; 0.025 was flush), and
  the regrasp scenario is back to `max_attempts=3`.
- The screw-assembly mission hung before its first block whenever nobody
  opened the viewer (#29). `MissionViewer` passed `open=True` to pyhpp_viser,
  which opens a browser and blocks until a client connects, which never happens
  on a headless machine; the nightly mission job timed out every run. The viewer
  now only serves the scene and prints its URL (`open_browser=True` restores the
  old behaviour), and the nightly job runs with `--no-viewer`.
- Broken links in the docs site: links from included root files (README,
  ARCHITECTURE) and from `docs/` to files outside it now use absolute GitHub URLs;
  archived legacy pages point at the pages' current locations.
- The Robotiq fingers never closed on the drill in the screw-assembly viewer:
  planned paths keep them frozen open, and native playback showed exactly that
  (pads 24 mm off the handle). `MissionViewer(closures=...)` now closes them to the
  grasp planner's width at each grasp and opens them at each release.
  `replay.py` used a fixed `finger_joint = 0.6` for every object, which put the
  pads 6.8 mm into the drill handle; it now uses the same closures (0.496 on the
  drill, 0.567 on a part's tab).

### Deprecated

- `CapabilityDescriptor(effects=("grasp_state",))`: bare state tags now belong
  in the new `writes=` field. They are moved there automatically, with a
  `DeprecationWarning`; `effects` declares literals.

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

[Unreleased]: https://github.com/thanhndv212/long-tamp/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/thanhndv212/long-tamp/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/thanhndv212/long-tamp/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/thanhndv212/long-tamp/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/thanhndv212/long-tamp/releases/tag/v0.1.0
