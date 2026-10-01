# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
(`0.x` while the API is still moving).

## [Unreleased]

### Added

- Web mission viewer, `long_tamp.viewer` (#24). It shows the plan tree with each node's state
  (attempt k/N, skipped because its effect held, drift, paused), a Gantt timeline you can play
  and scrub, the event list and per-node details.
  - `python -m long_tamp.viewer replay <run>` writes one self-contained HTML file per run,
    with no network needed. `serve` follows a running mission.
  - `ViewerServer` serves it live and can pause, resume and stop through an
    `ExecutionControl`. The screw assembly's `--web-port` serves it next to the Viser scene.
  - Customizable: `ViewerConfig` (panels, colors, theme, metric labels, scene URL, JSON
    file) and `window.LongTamp` in your own scripts (panels, badges, metric formats, event
    hooks). See `docs/usage/viewer.md`.
- Events: `plan` (opt-in, `plan_event(plan)`, carrying the IR document) and `pause` (from the
  executor, when an `ExecutionControl` holds it at a step boundary).

## [0.7.0] - 2026-10-01

Milestone M6: AI model integration (ADR-0006). Any model behind an Anthropic- or
OpenAI-compatible API works through one gateway (`long_tamp.ai`), and models act through
typed, checked roles: the grounder, the goal writer, the plan reviewer and the execution
supervisor. A model writes problems and decisions, never plans, and every proposal passes a
deterministic check. An operator can talk to the mission through gated tools (`--chat`),
and a mission can run from one instruction to the end without a human (`--instruction ...
--supervise`). Exit test: 10/10 such missions completed in MuJoCo through an injected
failure, with every decision audited. The spline path optimizer is also back on, with its QP
solve capped (needs the hpp-core patch; without it, the optimizer stays off).

### Changed

- The spline path optimizer is back on in the screw assembly, with its QP solve capped (#26).
  - `PyHPPBackend.configure_transition_planner(qp_max_iterations=N)` sets hpp-core's
    `SplineGradientBased/QPMaxIterations`, which bounds the optimizer by
    `path_optimizer_timeout` plus one capped solve. The mission uses 1000.
  - The parameter comes from a local hpp-core patch (`fix/qp-max-iterations`, see
    `docs/bugs/hpp-core-unbounded-planning-loops.md`, bug 6). With an hpp-core that lacks
    it, which includes the PyPI wheels, the backend drops the spline optimizer as before.

### Added

- The M6 exit test (#91), `script/screw_assembly/autonomy_batch.py`. Over N seeds, one
  instruction runs to the end with no human input, under the supervisor, with a part's
  clamping failure injected.
  - Each run is classified as completed, escalated cleanly (a report in its folder), or
    unclean.
  - Each run is audited: every decision is an allowed action, and every relaxed goal is a
    strict subset of the original.
  - It totals model calls and tokens per run. The gate passes with no unclean run and no
    unchecked decision.
  - Result: 10/10 missions completed with no human input, each after one goal relaxation that
    dropped only the failed part; 48 model calls, about 42k tokens. The first batch also
    showed a clean escalation.

- The operator chat (#89, ADR-0006), `long_tamp.ai.chat`. A model talks with the operator and
  acts only through gated tools.
  - Each answer is one JSON object (`say`, `actions`, `done`). Tools run in order, and their
    results go back to the model, for up to `max_steps` model calls per message.
  - A tool refuses its arguments with a reason (`ToolRejected`), which goes back to the
    model. This works on every endpoint the gateway reaches, since it needs no
    provider-specific function calling.
  - `task_screw_assembly.py --chat` reads operator messages from stdin. The tools are
    `state`, `set_goal`, `add_constraint` / `remove_constraint`, `plan` (optionally reaching
    some goal literals `first`, e.g. part 2 before part 1; the planner orders the rest),
    `run`, and `explain_failure`, plus `write_goal`, which hands an instruction to the goal
    writer role. Each is checked like the Python API, and every call is written to
    `events.jsonl` as a `tool` event. The chat model is given the domain (predicates,
    objects, notes) and moves the robot only when the operator asks it to run.
- The execution supervisor (#88, ADR-0006), `long_tamp.tasks.task_planning.supervisor`. When
  the deterministic repair loop gives up, a model decides at goal level, within limits, and
  never improvises.
  - Actions: `retry`, `relax_goal` (keep a strict subset of the original goal's literals,
    checked and reachable), `abort`, `escalate`.
  - Limits: decisions, wall-clock, model tokens, and an allowlist of actions. Past a limit,
    or without a valid decision, it escalates.
  - Abort and escalation write `escalation.json` and `escalation.md` to the run folder: what
    failed, what was tried, the decisions, the state.
  - `task_screw_assembly.py --instruction ... --supervise` runs repair loops under it.
    `run_with_repair` now also works when no first plan exists, and reports why repair
    gave up.
- Typed model roles (#87, ADR-0006), `long_tamp.ai.roles`: a model proposes, a deterministic
  checker accepts or explains, the explanation goes back to the model, for bounded rounds.
  A fallback that needs no model covers API errors, refusals, instructions it can't
  express, and rounds run out.
  - `refine(propose, check, max_rounds, fallback)` runs that loop; `ModelRole` makes a
    proposer from any gateway client. The goal writer is now one such role.
  - Grounder (`language.ground_instruction`): which scene objects an instruction refers to,
    checked against the cell. Its fallback matches names, and its result is a note for the
    goal writer.
  - Plan reviewer (`task_planning.review.review_constraints`): what an instruction rules out
    ("don't use clamp 1"), as blocked bindings for the task planner, never steps. Each
    constraint must name a real capability, parameter and object, and the planner must
    still reach the goal with it. The fallback is no constraint.
  - `task_screw_assembly.py --instruction` runs grounder, goal writer and plan reviewer on
    one client. The constraints apply to planning and to every `--replan` round.
  - An unusable answer (not JSON, cut off, malformed) costs a round, with feedback, not the
    role. Constraints are read in the shapes models write, since not every gateway enforces
    the schema.
- One gateway to AI models (#86, ADR-0006), `long_tamp.ai`.
  - Any model behind an Anthropic- or OpenAI-compatible API, named `<api>:<model>`
    (`anthropic:claude-opus-5-5`, `openai:<model>`). `make_client(model)` returns a client
    whose `complete_json(system, user, schema, role)` returns the parsed JSON answer.
  - Structured output, falling back to plain JSON mode where an endpoint refuses a schema.
    Claude on Anthropic's endpoint also gets an effort level and the refusal fallback.
  - Typed errors from either SDK (`AIAuthError`, `AIBillingError`, `AIRateLimitError`,
    `AIConnectionError`, `AIRefusalError`, `AIOutputError`, `AIRequestError`).
  - Each call becomes a `CallRecord` (role, model, tokens, seconds, error), which the mission
    writes to `events.jsonl` as `model` events.
  - Configuration: `configure()` / `load_env_file()` read env files the way a shell does
    (`--ai-env` or `LONG_TAMP_AI_ENV`). Inside a container, endpoints on `localhost` map to
    `host.docker.internal`.
  - Extras `ai-anthropic`, `ai-openai` and `ai`. Setup guide: `docs/usage/ai-models.md`.
- Goals from natural language (#22), `long_tamp.tasks.task_planning.language`. A model
  writes the goal of the planning problem, never the plan.
  - `goal_from_instruction(instruction, writer, vocabulary, reachable)` checks each written
    goal (syntax, known predicates and arities, known objects; `check_goal`), then whether
    the task planner reaches it from the current state. What fails goes back to the model,
    for up to three attempts.
  - The initial state stays the observed one; motion failures are still replanned
    deterministically, never through the model.
  - `Vocabulary.from_domain` builds what a goal may say from the capabilities' literals,
    the objects and the state, with free-text notes.
  - `ModelGoalWriter` writes goals through the gateway; `goal_writer(model)` builds one.
  - `task_screw_assembly.py --instruction "assemble part 2" --goal-model <api>:<model>
    [--ai-env FILE]` plans the written goal with the task planner (implies
    `--planner up`). `screw_domain.goal_vocabulary` describes the domain, and
    `pddl_problem(goal=...)` takes a goal other than the full mission's.

### Fixed

- Missions started together no longer crash writing the same cached URDF
  (`backends._urdf_paths`): each writer uses its own temporary file and an atomic rename.

## [0.6.0] - 2026-10-01

Milestone M5: multi-arm partial-order execution. Plans become partial orders:
`parallelize` groups steps with no ordering between them (by the resources capabilities
declare and their literals) into `parallel` lanes, which BehaviorTree.CPP gets as
`Parallel`. The executor plans a group's lanes, then runs their motions together,
merged joint by joint and collision-checked in HPP, or one after another when they can't
be merged. On identical plans, concurrent execution beats the sequential one on
wall-clock for every seed tested (5/5, 5.8% of motion time saved on the two-part screw
assembly). The BT session path is also checked on the real screw cell, in Python
nightly and through the C++ host.
### Added
- The BehaviorTree.CPP session path on the screw-assembly cell (#58).
  - `host.create_screw_session` builds a short seeded plan: pick the driver, home it while
    the left arm grasps part 1 (a `parallel` node), rack it
    (`script/screw_assembly/screw_bt_session.py`).
  - `tests/test_screw_bt_session.py` runs it through the session on the real scene, in the
    nightly. It replaces the TWIN session check, which passed or failed by chance.
  - The opt-in `taskplan_bt_screw_cell` CTest runs the compiled tree in
    `agimus_taskplan_bt`. It is the first real-scene run of BT.CPP's `Parallel` lowering,
    and it passes (131 s).
- Partial-order plans (#21, ADR-0005), `long_tamp.tasks.task_planning.partial_order`.
  - A `parallel` node in the TaskPlan IR: lanes of steps with no ordering constraint between
    them. At load time, no step of a lane may depend on another lane's.
  - `parallelize(document, registry)` rewrites runs of consecutive independent steps into
    `parallel` nodes. Steps depend on each other when they share a resource (capabilities'
    `resources` name the parameters a step holds; `ur10_left` and `ur10_left/gripper` are
    the same arm), when their literals interfere, or when a step declares neither.
  - The runner plans lanes one after another and reports each group to executors
    (`on_group`). The BehaviorTree.CPP compiler (1.2) lowers `parallel` to `Parallel`.
    Mermaid and DOT diagrams show it.
  - The screw assembly's capabilities declare their resources: each part's "right arm home"
    runs alongside the left arm's release.
- Concurrent arms (#21), `long_tamp.execution.concurrent`. With
  `PlanExecutor(concurrent=True, validate_config=...)`, a `parallel` group's steps are
  planned first, then their lanes' motions run together.
  - `merge_lanes` pairs the lanes' k-th commands into one command whose path takes each
    configuration entry from the lane that moves it. The lanes were planned one after
    another, each with the other arms still, so each lane moves only its own entries.
  - The merge is refused, and the group's motions run one after another in planning order,
    when a lane has a skill command, when two lanes move the same entry, or when
    `validate_config` (HPP's collision check) rejects a configuration along the merged
    motion.
  - The group's steps commit once all of it has run, with or without plan-ahead.
    `PlanRun.timing` counts `merged_groups` and `sequential_groups`.
  - `task_screw_assembly.py --concurrent`: independent steps run in parallel lanes. On the
    planner-ordered two-part mission, the right arm goes home while the left arm releases
    the part and grasps the next one. Motion time drops from 132 s to 115 s (seed 1).
- `script/screw_assembly/concurrency_ab.py`: the #21 exit test on identical plans. It plans a
  mission once, then runs the same motions sequentially and merged on fresh MuJoCo backends.
  On the two-part mission, seeds 1-5, concurrent execution saves 6.5-10 s of motion on
  every seed (5.8% on average), so it wins on wall-clock with planning shared.

## [0.5.0] - 2026-09-30

Milestone M4: execution in simulation. The planning scene exports to MuJoCo, and a
MuJoCo execution backend runs the planned paths under tracking control (retimed within the
joints' limits), with grasps as welds or, optionally, held by the fingers' friction. Skills
carry their own controllers (a screwing skill with a virtual screw); the executor checks
drift before each step and replans from the observed state, and can plan the next step
while the current one runs, with the simulation in its own process. The example cells now
use the vendors' own robot models (the UR10 from Universal Robots, the Robotiq 2F-85 from
PickNik), and missions can be recorded and replayed in MuJoCo's viewer. Validated with the
screw-assembly batch gate on the new models (10/10, median 913 s, source-built HPP).

### Changed

- The example cells' UR10 now comes from Universal Robots' own description package (#67).
  - Source: `UniversalRobots/Universal_Robots_ROS2_Description` at a pinned commit, vendored
    by `build_assets.py --only ur10-official` (URDF with relative mesh paths, meshes, BSD-3
    LICENSE, SOURCE.md). It replaces the Gepetto/example-robot-data export, and so brings
    UR's masses, inertias and joint limits.
  - Kinematics: `tool0` relative to `base_link` is identical (6e-10 m), and every collision
    mesh's world bounding box matches within 1 mm, so grasps and scenes are unchanged.
  - The base geometry is now on `base_link_inertia`, and the SRDF's adjacent pair follows.
- The example cells' Robotiq 2F-85 now comes from PickNik's `ros2_robotiq_gripper` (#71).
  - Source: `PickNikRobotics/ros2_robotiq_gripper` at a pinned commit, vendored by
    `build_assets.py --only robotiq-picknik` (the world link and ros2_control tags dropped;
    pad frames `robotiq_85_{left,right}_finger_pad` added on the fingertips' flat inner
    faces).
  - Joint and link names change to PickNik's: the driver is
    `robotiq_85_left_knuckle_joint` (0 open .. 0.8 closed), with five mimic joints. Configs,
    SRDF collision pairs and the example scripts follow.
  - `long_tamp.grasping.ROBOTIQ_2F85` is now this gripper, recalibrated from the merged
    URDF (84.9 mm stroke). The ros-industrial model stays available as
    `ROBOTIQ_2F85_ROS_INDUSTRIAL`.
  - `gripper_tcp` and the HPP gripper frame are unchanged. The screw assembly now closes to
    0.484 on the driver and 0.555 on a part, and `validate_closure.py` shows both pads
    touching and no other link in collision.

### Added

- Contact grasps in the MuJoCo backend (#71): `MuJoCoBackend(grasp="contact", fingers=...,
  grip=..., pads=...)`, and `task_screw_assembly.py --backend mujoco --grasp contact`.
  Welds remain the default.
  - The planner keeps the fingers open, so `grip(object, carrier)` gives the closure
    (`GripTable` builds it from a table; the mission uses the grasp planner's closures).
  - A new grasp snaps the object into place, closes the fingers for `grip_time` while the
    object still rests, then lets it go. The path runs with the fingers squeezing: they
    track the closure plus `grip_torque` towards it. A release welds the object where it is
    and opens the fingers before the path runs.
  - Physics after MuJoCo Menagerie's `robotiq_2f85`: elliptic cone, `impratio` 10,
    armature on the finger joints, stiff mimic equalities, and box pads on the fingertips
    that alone grip (a mesh fingertip touches a flat face at a point or two, and a long
    part pivots). The no-slip solver stops the creep MuJoCo's soft friction allows under a
    steady load. Unless `contacts` is on, only fingers and objects collide. The default
    `grip_torque` (15 N m) squeezes with about 95 N per pad (the 2F-85 is rated 20-235 N).
  - `slip` reports how far a gripped object moved in the hand, and gripping fingers are
    left out of `tracking_error` and `drift`. A 0.27 kg part carried through a 2 s arm
    move slips 0.4 mm (`tests/test_sim_contact_grasp.py`).
  - The screwing skill aims at the hole where its part actually is, and takes as its tool
    the carried object the approach moves (the other arm may hold the part still).
  - Known limit: execution is open loop. On the two-part mission, objects sit up to 3 mm
    from the plan in the fingers; part 1's screws go in, part 2's first misses the 2 mm
    alignment tolerance by 0.5 mm. Welds stay the default for missions.
- Replay a MuJoCo mission in MuJoCo's viewer: `MuJoCoBackend(record=folder)` saves `qpos` at
  30 frames per simulated second (one chunk per command), `task_screw_assembly.py
  --sim-record` records into the run folder, and `script/screw_assembly/view_mujoco.py` replays
  it (`mjpython` on macOS). The planning container has no display; the replay needs only
  `mujoco`.
- Drift check and planning ahead (#20), `long_tamp.execution`.
  - **Drift check.** Before a step's motion runs, `PlanExecutor` asks the backend how far
    the robot is from where the plan starts (`start_error`). Beyond
    `ExecutionPolicy.max_start_drift`, it emits a `drift` event and calls
    `on_drift(node, observe)` to replan the step from the observed configuration.
  - **Planning ahead.** With `plan_ahead=True`, step k+1 is planned while step k's motion
    runs in a worker thread. Paths are sampled to arrays first (`SampledPath`), and
    preconditions see pending effects.
  - **`ProcessBackend`.** It runs a backend in its own process, so a simulator doesn't
    share the planner's interpreter lock (in a thread, the MuJoCo motion ran 3.6 times
    slower). `PlanRun.timing` reports wall, execution, idle and drift-replan figures.
  - **MuJoCo backend.** It gains `start_error`, `observed_config` (`QposMap.inverse`) and
    `disturb`.
  - **Screw assembly.** New flags `--plan-ahead`, `--max-drift`, `--sim-speed` and
    `--inject-drift LABEL:JOINT:RAD`. The simulation runs in a `ProcessBackend`.
  - **Results.** A 0.2 rad drift before a grasp is replanned and the mission completes.
    At real-time speed, planning ahead cut a 2-part mission from 773 s to 581 s.
- Skills (#19), `long_tamp.tasks.task_planning.skills`. A `SkillSpec` declares a step that
  ends in controller-level behaviour: pre- and postconditions, failure facts, start and end
  poses. `descriptor()` turns it into a capability.
  - A `SkillCommand` is the payload a backend runs, and it is also its approach path, so
    backends without the skill play it.
  - `long_tamp.sim.ScrewDriving` is a screwing stub for the MuJoCo backend: a compliant
    approach, an alignment check, a virtual screw with force feed-forward, and a torque
    threshold. It reports `screwed(part, hole)`, `screw_misaligned` or `screw_no_contact`.
  - The screw assembly sends each screw insertion as a skill; `--hole-error` models a
    perception error.
  - `Feedback` and `ExecutionResult` carry `facts`.
  - With a backend, a step's recorded effects are written only after its motion executed
    (they used to be written when it was planned).
  - Docs: `docs/usage/skills.md`.
- MuJoCo export (#17), `long_tamp.sim.mjcf`: `export_mjcf(config, out_dir)` writes a
  task's scene as one self-contained MJCF (HPP's body and joint names, objects free at
  their initial pose, Robotiq mimic joints as equalities, COLLADA meshes converted);
  `qpos_from_pinocchio` maps an HPP configuration to MuJoCo `qpos` and `fk_mismatch`
  compares the two models' kinematics (screw-assembly cell: < 1e-7 on 60 bodies). CLI:
  `python -m long_tamp.sim.mjcf CONFIG -o OUT`. New `sim` extra (mujoco, trimesh,
  pycollada).
- MuJoCo execution backend (#18), `long_tamp.sim.MuJoCoBackend`: runs planned paths in
  the exported scene under tracking control, with the following behaviour.
  - Torque motors with PD, gains scheduled on each joint's apparent inertia plus any
    carried payload, gravity and Coriolis compensation, and net torque limited to the
    URDF effort.
  - Grasps are welds to the carrying link: a grasp snaps within 2 cm, or fails as
    "grasp missed".
  - Every path is retimed rest to rest within velocity and acceleration limits, and
    stretched until inverse dynamics fits within 80 % of effort.
  - Motion events report `tracking_error`, `drift`, `start_drift`, `object_drift`,
    `grasp_error` and `time_scale`.
  - The screw assembly runs on it with `--backend mujoco`; `--summary` adds an
    `execution` block.
  - A 2-part mission completes in simulation: 37 commands, worst tracking error
    0.015 rad, object drift 0.05 mm.
  - The export keeps the URDF velocity limits and frees mimic followers' own limits.
  - `Feedback` and `ExecutionResult` carry `metrics`, which the executor adds to
    `motion` events.

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

[Unreleased]: https://github.com/thanhndv212/long-tamp/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/thanhndv212/long-tamp/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/thanhndv212/long-tamp/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/thanhndv212/long-tamp/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/thanhndv212/long-tamp/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/thanhndv212/long-tamp/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/thanhndv212/long-tamp/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/thanhndv212/long-tamp/releases/tag/v0.1.0
