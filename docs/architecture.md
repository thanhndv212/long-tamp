---
title: Architecture
---

# Architecture

**Last reviewed against source:** 2026-10-26, commit `cbb1a88` (M8 mission domains, M11
RoboCerebra exporter). If a module isn't mentioned here, check
`git log --since=2026-10-26 -- src/long_tamp` before assuming it's missing rather than
undocumented.

!!! tip "Interactive tour"
    The same story as this page, with diagrams you can step through: follow one mission
    from instruction to motion, watch the recovery ladder fire, compare graph sizes.
    **[Open the architecture tour](architecture-tour.html)** (one self-contained page, works
    offline).

This page explains how long_tamp is put together and why. It starts with the big picture,
follows one mission through the code, then covers failure handling, models, processes and
the rules that keep the layers apart. The [module map](#module-map) at the end lists where
everything lives. How to *use* each part is under [Usage](usage/README.md); the reasons
behind the big decisions are the [ADRs](adr/README.md).

## The short version

A long-horizon manipulation mission has to answer four questions. long_tamp gives each one
to a different part of the code, and each part hands a checked result to the next.

| Question | Answered by | It produces |
|---|---|---|
| **What should be true at the end?** | a person, or a model through the goal writer | a *goal*: literals such as `screwed(part1, part1/h_hole1)` |
| **Which steps get there?** | a task planner (Fast Downward, Unified Planning, or a hand-written plan) | a *TaskPlan*: a validated tree of capability calls |
| **How exactly, in geometry?** | the **refiner**: `GraspSequencePlanner` on HPP | grasps, configurations and collision-free paths |
| **Do it, and watch it** | an executor (Python, or BehaviorTree.CPP) and an execution backend | motion on a robot or simulator, and an **event stream** |

Two loops close it. When geometry fails, the refiner reports *why* as facts, and the task
planner plans around them. When that runs out, a supervisor decides at goal level (retry,
relax the goal, abort, or hand over to a human). Everything that happens is written to the
event stream, which the viewer, the watchdog and the logs read.

```mermaid
flowchart LR
    goal["Goal<br/>what should be true"] --> plan["TaskPlan<br/>which steps"]
    plan --> refine["Refiner<br/>how, in geometry"]
    refine --> exec["Executor + backend<br/>do it"]
    refine -- "failure facts" --> plan
    exec -. "events.jsonl" .-> watch["viewer · watchdog · logs"]
    plan -- "no plan left" --> sup["supervisor<br/>retry · relax · abort · escalate"]
    sup --> goal
```

## Two stacks, one seam

The code splits along the line between **symbols** and **geometry**
([ADR-0001](adr/0001-planner-refiner-executor.md)).

- The **mission stack** reasons about symbols: which gripper holds which handle, which
  screw is in. It plans, validates, executes and records. It never imports HPP, so the plan
  format, the compiler, the C++ host and the viewer all work without it.
- The **motion stack** reasons about geometry: constraint graphs, configurations, paths. It
  is the part that took the most work, and it knows nothing about plans or goals.
- They meet at the **refiner**. A plan step goes in (`grasp(ur10_left/gripper,
  part1/h_grasp)`); a path comes out, or facts about why there isn't one
  (`ik_unreachable`, `blocks(part1, clamp2)`). That is the whole interface.

```mermaid
flowchart TB
    subgraph mission["Mission stack: symbols"]
        direction TB
        mission_pkg["mission/<br/>domains, runner, chat, UI"]
        tp["task_planning/<br/>goals, plans, checks, events"]
        execution["execution/<br/>run plans on a backend"]
        ai["ai/<br/>model gateway, roles"]
        sim["sim/<br/>MuJoCo backend"]
        viewer["viewer/<br/>web mission viewer"]
    end
    refiner{{"Refiner<br/>plan step → path, or failure facts"}}
    subgraph motion["Motion stack: geometry"]
        direction TB
        tasks["tasks/<br/>GraspSequencePlanner, recovery"]
        planning["planning/<br/>scenes, constraints, graphs"]
        backends["backends/<br/>PyHPPBackend"]
    end
    mission_pkg --> execution
    mission_pkg --> tp
    mission_pkg -. "domain's scene" .-> tasks
    execution --> tp
    tp -.-> ai
    sim --> execution
    viewer -. events .-> tp
    tp --> refiner
    refiner --> tasks
    tasks --> planning --> backends

    style refiner fill:#2f6fed,stroke:#1d2330,color:#fff
    style backends fill:#4c566a,stroke:#2e3440,color:#fff
    style planning fill:#5e81ac,stroke:#2e3440,color:#fff
    style tasks fill:#81a1c1,stroke:#2e3440,color:#fff
```

A mission connects the two through a **mission domain** ([ADR-0011](adr/0011-mission-domain.md)):
a class, outside the library, that answers everything specific to one robot and task
(capabilities, how a plan step becomes a grasp sequence, the scene, the backends). The
generic `MissionRunner` ([ADR-0012](adr/0012-mission-runner.md)) runs any domain on the
same planner → refiner → executor → repair → supervisor loop, and `mission/` never
imports HPP itself: the domain's `setup()` builds the scene. The screw assembly's domain is
`script/screw_assembly/screw_mission.py`; the smallest complete one, with no HPP, is
`long_tamp.mission.template`. What the screw assembly still keeps to itself is listed in
[what the library runs, what a mission supplies](#what-the-library-runs-what-a-mission-supplies).

## Following one mission

The reference mission ([ADR-0003](adr/0003-screw-assembly-reference-track.md)): two UR10
arms with Robotiq grippers fasten plates to a jig, two screws per plate. Here is what runs,
in order.

1. **Scene.** `build_scene.py` writes URDF/SRDF and a task YAML. `YamlTaskLoader` reads it;
   `SceneBuilder` loads robots, jig and parts into `PyHPPBackend`.
2. **Instruction → goal** (optional). "Assemble part 1 and put the driver away" goes to the
   goal writer (`task_planning/language.py`). The model writes only goal literals; they are
   checked for syntax, known predicates and objects, and reachability. If they fail, the
   reasons go back to the model; if the model can't help, the mission asks for a goal.
3. **Goal → plan.** `pddl.to_pddl()` exports the capabilities (`grasp`, `release`,
   `clamp_and_screw`, `rack`, `home`, with preconditions and effects), the current world
   state and the goal. Fast Downward returns a sequence of calls; `skeleton_document()`
   turns it into a `TaskPlan`, and `parallelize()` puts independent arm motions side by side
   ([ADR-0005](adr/0005-partial-order-plans.md)).
4. **Validation.** `TaskPlan.from_dict` checks every call against the `CapabilityRegistry`
   and simulates the plan from the initial state. Every plan passes this gate, whoever
   wrote it.
5. **Run.** `PlanExecutor` walks the plan with `run_plan` (the same semantics as the
   compiled behavior tree). Before each step, a world-state check: if the step's effects
   already hold (a resumed run, a part already screwed) it is skipped
   ([ADR-0002](adr/0002-effects-as-runtime-guards.md)).
6. **Refine.** The step's capability asks `GraspSequencePlanner` for motion. Per phase it
   builds a small constraint graph for just that transition, picks the edge, draws a target
   (with a lookahead so the next phase stays reachable), and solves. See
   [inside a step](#inside-a-step-why-on-not-on).
7. **Execute.** The path goes to the execution backend (MuJoCo, playback, or a robot) under
   `run_command`: retries while busy, heartbeats, a deadline scaled to the path's duration
   ([ADR-0004](adr/0004-execution-contract.md)). Before it runs, a drift check compares the
   robot with the path's start and replans if they differ. With `plan_ahead`, the next step
   plans while this one moves.
8. **Record.** Effects that no sensor can see later (a screw driven) are recorded only after
   the motion ran. Every status change goes to `events.jsonl`.

The same plan can instead be compiled to BehaviorTree.CPP XML and run in a C++ host, which
calls back into the same Python session and writes the same events. See
[BehaviorTree.CPP integration](usage/behaviortree-integration.md).

## Inside a step: why O(N), not O(N!)

HPP plans manipulation on a *constraint graph*: a node per grasp state ("left gripper holds
part 1"), an edge per transition. One graph over every combination of grippers and handles
grows factorially with the number of grasps, and most of it is never visited.

`GraspSequencePlanner` builds a new, minimal graph for each phase instead.
`SequentialConstraintGraphFactory` keeps only the states and edges on the planned sequence
(`SequentialGraspFilter`), so the total work grows linearly with the number of phases.
`GraspStateTracker` remembers what is held and names the edge to take next; joints of arms
not involved in the phase are locked.

```mermaid
flowchart TD
    plan["GraspSequencePlanner: grasp / release / plan_sequence"]
    plan --> p1["1. build_phase_graph<br/>only this phase's states and edges"]
    p1 --> p2["2. GraspStateTracker picks the edge"]
    p2 --> p3["3. ConfigGenerator draws a target<br/>(lookahead hints if given)"]
    p3 --> p4["4. backend.solve, optimize, time-parameterize"]
    p4 --> p5["5. record path, log phase"]
    p5 -->|next phase| p1
    p5 --> result["concatenated multi-phase path, O(N) planning cost"]
```

The **lookahead** (`find_feasible_phase_target`) handles a failure this creates: a target
that is fine for phase N can leave the object in an orientation phase N+1 can't grasp. The
lookahead tries candidates for N on a copy of the tracker and keeps one from which N+1 is
reachable. See [phase-target lookahead](features/phase-target-lookahead.md).

## When something fails

Failures are handled at the lowest level that can fix them, and each level only escalates
what it can't.

| Level | Where | What it can change |
|---|---|---|
| 1. Redraw the target | inside `plan_sequence()` | another target for the same edge |
| 2. Resume the block | `block_recovery`, `resume_sequence()` | replan the failing phase, keep earlier phases |
| 3. Replan the block | `block_recovery` | undo the block's earlier commitments, new lookahead |
| 4. Replan the mission | `repair.plan_execute_repair()` | block the failing binding (`clamp2 can't take part1`) and plan around it, or retry a missed grasp from the observed world (ADR-0010) |
| 5. Decide at goal level | `supervisor.py` (a model role) | retry, drop part of the goal, abort, or escalate |
| 6. A human | escalation report | anything |

Levels 1–4 are deterministic. The refiner turns a geometric failure into facts
(`cannot_reach`, `ik_unreachable`, `release_infeasible`, `lookahead_failed`, `blocks`), and
the task planner treats them as constraints. Only when no plan avoids them does a model get
a say, and only at goal level.

A crash is a failure too. Because completion is checked against the world state, a killed
mission restarts by running the same plan again: finished steps are skipped and the
interrupted one is redone (`kill_resume.py` checks this).

## Models propose, long_tamp checks

Language models are optional, and they never produce motion or plan steps
([ADR-0006](adr/0006-ai-model-integration.md)). Every use goes through the same loop in
`ai/roles.py`: the model proposes, a deterministic checker accepts or gives reasons, the
reasons go back for a bounded number of rounds, and a model-free fallback applies after
that.

| Role | Proposes | Checked by |
|---|---|---|
| Grounder | which scene objects an instruction means | known objects |
| Goal writer | goal literals | syntax, vocabulary, the task planner finding a plan |
| Plan reviewer | constraints, blocked bindings (never steps) | the planner validating them |
| Execution supervisor | retry / relax goal / abort / escalate | allowed actions; a relaxed goal must be a strict subset |
| Step watchdog | wait / skip / abort a slow step | a hard time limit the model can't extend |
| Operator chat | tool calls (plan, run, pause, block, …) | each tool's own argument checks |

`ai/` is one gateway to any Anthropic- or OpenAI-compatible API (`<api>:<model>`); every
call is recorded in the event stream with its tokens and latency. See
[AI models](usage/ai-models.md).

## What runs where

HPP planning holds Python's interpreter lock for seconds at a time. Anything that must stay
responsive while a step plans therefore runs in its own process:

| Process | Runs | Why separate |
|---|---|---|
| Mission | executor, refiner, HPP | the main process |
| Simulator (`ProcessBackend`) | MuJoCo backend | in a thread, MuJoCo motion ran 3.6× slower than real time |
| Robot host (`RemoteBackend`, optional) | any backend, over TCP | the robot's hardware is on another host or in another environment; stops on disconnect (ADR-0007) |
| Viewer front (`separate_process=True`) | page and event serving | the page answers in milliseconds instead of seconds |
| 3D scene (`SceneProcess`) | Viser scene | doesn't freeze while a step plans |
| C++ host (optional) | BehaviorTree.CPP tree | calls Python in-process (embedded CPython), no RPC |

They talk through the event stream (a JSONL file), pipes, a local socket for scene frames,
and HTTP for the viewer's controls, which act on `ExecutionControl` at step boundaries.

## Dependency rules

What the imports look like (checked at the review commit):

- `tasks/task_planning/` imports only `ai/` and `grasping/` from the package. Missions
  connect it to the motion stack in their own adapters, loaded lazily by `host.py`.
- `execution/` depends on `task_planning/` (events, `run_plan`, skills), never on
  `planning/` or `backends/`. Motion reaches it as opaque commands.
- `sim/` implements `execution/`'s contract; `viewer/` only reads events.
- `mission/` imports `tasks/` (the refiner, checkpoints, `task_planning`), `execution/`,
  `ai/` and `viewer/`, never `planning/` or `backends/`: the domain's `setup()` builds the
  HPP scene. `eval/` imports only `execution/` (the remote backend's wire format).
- Motion stack: `tasks` → `planning` → `backends`. Known lazy exceptions: `backends/pyhpp.py`
  imports `planning.constraints` and `visualization` inside functions; `tasks/` imports
  `execution.activity` to report progress while planning.
- `backends/` is the only place `pyhpp` is imported, behind `try/except ImportError`
  (`HAS_PYHPP`). The pure-Python tier installs and runs without HPP; HPP, TOPPRA, Unified
  Planning, MuJoCo and model SDKs are extras (`hpp`, `toppra`, `planning`, `sim`, `ai`).

## What the library runs, what a mission supplies

The screw assembly came first, as one script, and the library grew out of it. M8
(#131–#136) moved the mission loop into `long_tamp.mission`, and a second domain with no
HPP (`mission.template`) runs on the same code. This is where the line stood at the review
commit.

**Moved into the library.** The screw script now only calls these:

| What | Now in | Issue |
|---|---|---|
| The mission loop: run a plan, repair, supervise, goal from an instruction | `mission.runner.MissionRunner` (`run_mission`, `run_with_repair`, `run_supervised`, `plan_from_goal`, `understand_instruction`) | #132, #134 |
| The robot- and task-specific part, behind one interface, with generic defaults (PDDL export, vocabulary, skeleton → plan, repair policy, missed-grasp retry) | `MissionDomain`, `BaseMissionDomain`, `check_domain` | #131 |
| Operator chat tools, the mission UI page and the launcher that restarts on Reset | `mission.chat`, `mission.ui` (`python -m long_tamp.mission ui --domain …`); `mission_ui.py` is a 40-line wrapper | #133 |
| The 3D scene in its own process, fed with simulated states | `visualization.SceneProcess`, `MissionViewer` | #106 |
| Failure injection specs, the trajectory file, the run summary | `parse_injection`, `write_trajectory`, `execution_summary`, `fact_count`, `metric_max` | #132 |
| Object-level goals (`grasped`, `placed_at`) | `mission.goals` | #131 |
| The simulator and execution layer: MJCF export, `MuJoCoBackend`, `QposMap`, `GripTable`, process and remote backends, drift check, plan-ahead, parallel arms, watchdog, grasp verification | `sim/`, `execution/` (generic from the start) | M4–M7 |

**Generic, but still in the script.** The candidates to port next:

| What | Where it lives | What porting it takes |
|---|---|---|
| The headless mission command: `--replan`, `--resume`, `--inject-failure/-drift/-miss`, `--instruction … --supervise`, `--summary`, `--check`, `--plan-ahead`, `--concurrent`, `--remote`, `--grasp contact`, `--sim-record` | `task_screw_assembly.main()`, about 450 lines | a `python -m long_tamp.mission run --domain …` next to `ui` and `serve` (which only run the chat-driven mission) |
| Seeding HPP's shooter (libc `srand`), pinocchio, NumPy and `random` | `seed_everything` | a library helper; every HPP mission needs it to be reproducible |
| Fault injection around a backend: bump a joint, move an object, just before a step | `DriftInjector`, `MissInjector` | `execution/` wrappers; they use only `disturb` / `displace`, which `MuJoCoBackend` has |
| MuJoCo grasp wiring: the grip table, weld or contact grasps, finger pads | `grip_table`, `weld_grasps`, `contact_grasps`, `robotiq_pads` | a `sim/` helper from a `FingerClosureTable` and the gripper models (the pad size is already on `GripperModel`) |
| Replanning from the observed world, keeping carried objects as planned; opening hands | `observed_start`, `screw_mission.opened_config` | `BaseMissionDomain` defaults; today they find hands by the Robotiq joint names |
| Moving one arm to a named pose with what it holds | `run_home_move`, `home_target` | a `Move` refinement in `tasks/` with named poses from the task YAML (they are `build_scene` constants now) |
| The `mock` / `playback` / `mujoco` backends for an HPP scene | `task_screw_assembly.make_backend` | a `BaseMissionDomain` helper for any domain with an HPP scene and an MJCF export |
| Replaying a run: Viser with finger overlay, MuJoCo's viewer on a recorded simulation | `replay.py`, `view_mujoco.py` | `long_tamp.viewer` / `sim` commands; the library already writes `trajectory.json` and the recording |
| Batches and release gates | `run_batch.sh`, `summarize.py --gate`, `kill_resume.py`, `concurrency_ab.py`, `autonomy_batch.py` | a batch command with per-domain gates; the scripts name screw steps and thresholds |

**The screw assembly's own.** These stay in `script/screw_assembly/`, as every mission keeps
its own:

| File | What it holds |
|---|---|
| `build_scene.py`, `config/`, `assets/` | the cell: URDF/SRDF for parts, driver, jig and clamps, the task YAML |
| `screw_domain.py` | capabilities (`grasp`, `release`, `clamp_and_screw`, `rack`, `home`), static facts, clamp seats, the hand-written plan, step expansion, repair policy |
| `screw_mission.py` | `ScrewAssemblyDomain`, the reference `MissionDomain`; maps the driver tip's insertion edge to the screw skill |
| `scenarios.py` | the V4 start states (driver held, part clamped, …) |
| `screw_bt_session.py` | the session the C++ host's `create_screw_session` builds |

**Screw-specific code inside the library.** The opposite direction, worth moving out:

- `sim/skills.py`: the `ScrewDriving` controller and its `SCREW` spec, exported from
  `long_tamp.sim`. The skill machinery (`SkillSpec`, `SkillCommand`) is generic; this
  controller belongs to the cell.
- `tasks/task_planning/host.py`: `create_screw_session` (and the two TWIN factories) put
  `script/…` on `sys.path` and import from it, so they work only from a source checkout.
- `examples/behaviortree/src/screw_assembly_plugin.cpp`: an example, where it belongs.

## Module map

| Package | Its job | Main types | Docs |
|---|---|---|---|
| `backends/` | the one HPP binding | `BackendBase`, `PyHPPBackend`, `create_planner` | [Install](INSTALL.md) |
| `planning/` | scenes, constraints, graphs, configs, path recording | `SceneBuilder`, `ConstraintBuilder`, `GraphBuilder`, `SequentialConstraintGraphFactory`, `GraspStateTracker`, `ConfigGenerator`, `PathRecorder` | [Standalone usage](usage/standalone-usage.md) |
| `tasks/` | multi-phase grasp sequences, recovery, the refiner | `GraspSequencePlanner`, `run_block_with_recovery`, `Refiner`, `GraspSequenceRefiner`, `MissionCheckpoint`, `ManipulationTask` | [Refiner](usage/refiner.md) |
| `mission/` | mission domains and the generic loop around them, operator chat, mission UI | `MissionDomain`, `BaseMissionDomain`, `MissionRunner`, `MissionChat`, `check_domain`, `TemplateDomain` | [Write your own mission](usage/write-your-own-mission.md), [ADR-0011](adr/0011-mission-domain.md), [ADR-0012](adr/0012-mission-runner.md) |
| `tasks/task_planning/` | plan format, validation, task planners, repair, BT compiler, events, model roles | `TaskPlan`, `CapabilityRegistry`, `TaskPlanningSession`, `run_plan`, `to_pddl`, `FastDownwardPlanner`, `plan_execute_repair`, `compile_behavior_tree`, `SkillSpec` | [Task planning](usage/task-planning-pddl.md), [Events](usage/events.md), [Skills](usage/skills.md) |
| `execution/` | run plans and motion on a backend | `PlanExecutor`, `ExecutionBackend` contract, `run_command`, `ExecutionControl`, `ProcessBackend`, `RemoteBackend`, `GripperActuation`, `ExpectedGrasp`, `StepWatchdog` | [Execution](usage/execution.md) |
| `sim/` | MuJoCo scene export and backend | `export_mjcf`, `MuJoCoBackend` | [Simulation](usage/simulation-mujoco.md) |
| `eval/` | benchmark adapters: the RoboCerebra env client and its scene export | `RoboCerebraClient`, `write_export`, `load_export` | |
| `ai/` | model gateway, roles, operator chat | `make_client`, `refine`, `ModelRole`, `ChatSession` | [AI models](usage/ai-models.md) |
| `viewer/` | web mission viewer, live or replay | `ViewerServer`, `write_replay`, `ChatBridge`, `ViewerConfig` | [Mission viewer](usage/viewer.md) |
| `grasping/` | where to grasp, how far fingers close | `GraspPlanner`, `GripperModel`, `ParallelGripperModel`, `RotaryJawGripperModel`, `FingerClosureTable` | |
| `config/` | YAML task configs | `YamlTaskLoader`, `BaseTaskConfig` | |
| `visualization/` | graph diagrams, frames, video, 3D scene process | `SceneProcess`, `MissionViewer`, `VideoRecorder` | |
| `logging/` | run log for script-style tasks, `logging` setup | `RunLogger`, `load_run_log` | [Logging](features/logging-verbosity.md) |
| `utils/`, `cli/` | transforms, terminal menus, shared CLI arguments | | |
| `examples/behaviortree/` (C++) | BT.CPP nodes, plugin, Python bridge, host | `long_tamp_bt_nodes`, `PythonTaskSession`, `agimus_taskplan_bt` | [BT.CPP](usage/behaviortree-integration.md) |

## Reference missions and validation

Missions live in `script/`, built from the library, never inside it.

- **Screw assembly** (`script/screw_assembly/`): the reference track. `task_screw_assembly.py`
  runs it (`--backend none|mock|playback|mujoco`, `--replan`, `--inject-failure`,
  `--resume`, `--chat`, `--instruction … --supervise`); `mission_ui.py` opens the mission UI
  (instruction, plan card, Start/Reset, live plan monitor) through the generic launcher,
  `python -m long_tamp.mission ui --domain …` (`long_tamp.mission.ui`, #133). Gates: `summarize.py --gate`
  (V3 batch), `scenarios.py` and `kill_resume.py` (V4), `concurrency_ab.py`,
  `autonomy_batch.py` (M6). Robot models come from Universal Robots' and PickNik's official
  descriptions.
- **Template domain** (`long_tamp.mission.template`): two objects, one arm, three regions,
  kinematic, no HPP. The whole mission loop on a scene you can read in one sitting; the
  [tutorial](usage/write-your-own-mission.md) walks through it.
- **RoboCerebra** (`script/robocerebra/`, `long_tamp.eval`): a long-horizon benchmark in
  MuJoCo; its scenes are exported to URDF/SRDF and planned with HPP (M11).
- **TWIN lift ball** (`script/twin/`), **IKEA LACK table** (`script/ikea_table_prototype/`),
  **grasp planning** (`script/grasp_planning/`), and `script/templates/` for a new task.

Validation levels V0–V4 and which change needs which: [Validation](development/validation.md).

## Extension points

- **A mission:** a `MissionDomain` (subclass `BaseMissionDomain`): its capabilities,
  `step_for` (a plan step → a grasp sequence to refine), `setup` (the scene), and
  optionally backends, a live scene, scenarios and repair rules. `check_domain` lists what
  is missing; `python -m long_tamp.mission ui --domain module:factory` runs it. See
  [Write your own mission](usage/write-your-own-mission.md). Register a factory in
  `host.py` to run it from a C++ host.
- **A task planner:** anything that returns a `TaskPlan` document (the `TaskPlanner`
  protocol for PDDL-style planners). It passes the same validation.
- **An execution backend:** `start` / `poll` / `cancel` from `execution.contract`; wrap it
  in `ProcessBackend` to run it in its own process, or serve it with `serve_backend` and
  drive it through `RemoteBackend` when it runs on another host (ADR-0007). A backend with a
  hand adds `open_gripper` / `close_gripper` / `gripper_state` and reports `holds` or
  `grasp_missed` after each close (ADR-0009).
- **A gripper:** a `GripperModel` (a `ParallelGripperModel` or `RotaryJawGripperModel` with
  its stroke table, or your own class), added with `register_gripper` (ADR-0008).
- **A skill:** a `SkillSpec` on the capability, and a controller in a backend that runs it.
- **A model role:** input, JSON output schema with a parser, a checker and a fallback, run
  through `ai.roles.refine`.
- **An HPP binding:** implement `BackendBase`, register it in `backends/__init__.py`.

## Out of scope

long_tamp has no ROS 2 nodes, topics or launch files. A ROS 2 stack uses it through
BT.CPP nodes with a ROS-backed `TaskSession`, or an execution backend that is an action
client; that integration lives in the consumer. `CMakeLists.txt` and `package.xml` exist to
install next to an HPP source build and to build the BT.CPP examples; they add no ROS 2
runtime dependency to the Python code.
