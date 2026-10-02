# Roadmap

Where `long_tamp` is going after 0.1.0, and how progress is tracked. The design
behind it is in the ADRs ([adr/](../adr/README.md)); the process is in
[development/workflow.md](../development/workflow.md).

**Tracker:** each milestone below is a GitHub milestone, each item a GitHub issue
([issues](https://github.com/thanhndv212/long-tamp/issues),
[milestones](https://github.com/thanhndv212/long-tamp/milestones)). This page is the
overview; issue status on GitHub is authoritative. Tick an item here when its issue
closes.

## Direction

Today the task level is written by hand and `task_planning/` compiles a hand-written plan
into a BehaviorTree.CPP tree. The target splits the stack into three pluggable roles
([ADR-0001](../adr/0001-planner-refiner-executor.md)):

- a **task planner** decides which discrete actions to take, in what order (hand-written, a
  classical PDDL planner, PDDLStream, or a language model producing the *goal*);
- a **refiner**, long_tamp's core, binds them to continuous grasps, configurations and
  paths on HPP's manipulation constraint graph;
- an **executor** runs the result (Python by default, BehaviorTree.CPP as an export,
  robot or simulator backends), with effects checked against the world so missions skip
  what's already done and resume after a restart
  ([ADR-0002](../adr/0002-effects-as-runtime-guards.md)).

Every milestone is delivered and proven on the screw-assembly mission first
([ADR-0003](../adr/0003-screw-assembly-reference-track.md)). Each milestone gate must
keep the V3 batch gate green ([validation](../development/validation.md)).

## M1 — State model and effect-based resume (0.2.0)

**Released as 0.2.0 on 2026-09-28.**

[Milestone 1](https://github.com/thanhndv212/long-tamp/milestone/1)

Capabilities declare what they need and what they achieve; plans are checked for
feasibility before any geometry runs; missions skip steps whose effect already holds.

- [x] [#2](https://github.com/thanhndv212/long-tamp/issues/2) Typed preconditions and effects on `CapabilityDescriptor`; `TaskPlan.from_dict`
      simulates them and rejects infeasible plans; `TaskStepReady` checks preconditions
- [x] [#3](https://github.com/thanhndv212/long-tamp/issues/3) World-state provider: observed predicates (grasps, poses) and recorded facts
      (written only by completed execution, e.g. `screwed(part, hole)`)
- [x] [#4](https://github.com/thanhndv212/long-tamp/issues/4) Runtime effect guard: a transaction is skipped when its effect already holds
      (replaces the in-memory completed set)
- [x] [#5](https://github.com/thanhndv212/long-tamp/issues/5) Screw-assembly domain and mission expressed as a TaskPlan (`plan_block`,
      `home_move` capabilities)
- [x] [#6](https://github.com/thanhndv212/long-tamp/issues/6) Initial-state scenario harness (V4), ≥4 screw-assembly scenarios, TWIN regrasp
      from ≥3 initial grasp states
- [x] [#7](https://github.com/thanhndv212/long-tamp/issues/7) Static plan diagram: `to_mermaid()` / `to_dot()` for TaskPlans

**Exit test:** the screw-assembly TaskPlan succeeds from ≥4 initial states with no plan
change; the 10-seed batch gate passes.

## M2 — Executor contract and refiner interface (0.3.0)

**Released as 0.3.0 on 2026-09-28.**

[Milestone 2](https://github.com/thanhndv212/long-tamp/milestone/2)

- [x] [#8](https://github.com/thanhndv212/long-tamp/issues/8) Executor contract: statuses `SUCCESS/FAILURE/RUNNING/BUSY`, heartbeats with
      inactivity timeouts, duration-scaled execution timeouts, pause/resume/breakpoints
- [x] [#9](https://github.com/thanhndv212/long-tamp/issues/9) Python executor for TaskPlans (default), with mock and trajectory-playback backends
- [x] [#10](https://github.com/thanhndv212/long-tamp/issues/10) `Refiner` interface over `run_block_with_recovery()` and the phase-target lookahead
- [x] [#11](https://github.com/thanhndv212/long-tamp/issues/11) Structured JSONL event stream from the executor; the BehaviorTree.CPP host emits
      the same stream (IR ids stamped on compiled nodes)
- [x] [#12](https://github.com/thanhndv212/long-tamp/issues/12) Resume from world state after the process is killed mid-mission

**Exit test:** batch gate passes on the Python executor; a mission killed mid-run
resumes from world state, not from a block index.

## M3 — Automatic task planning (0.4.0)

**Released as 0.4.0 on 2026-09-29.**

[Milestone 3](https://github.com/thanhndv212/long-tamp/milestone/3)

- [x] [#13](https://github.com/thanhndv212/long-tamp/issues/13) PDDL domain/problem export from the capability registry and a goal
- [x] [#14](https://github.com/thanhndv212/long-tamp/issues/14) Unified Planning adapter (Fast Downward) producing plan skeletons → TaskPlan
- [x] [#15](https://github.com/thanhndv212/long-tamp/issues/15) Structured refiner failure facts (`CanNotReach`, `IKUnreachable`,
      `ReleaseEdgeInfeasible`, `Blocks`) and a block → replan loop
- [x] [#16](https://github.com/thanhndv212/long-tamp/issues/16) Screw-assembly domain with real choices: clamp slot, part order, driving arm

**Exit test:** an N-part screw-assembly goal is planned automatically; an injected
`CanNotReach` triggers a re-clamp and the mission completes.

## M4 — Execution in simulation and controllers (0.5.0)

**Released as 0.5.0 on 2026-09-30.**

[Milestone 4](https://github.com/thanhndv212/long-tamp/milestone/4)

- [x] [#17](https://github.com/thanhndv212/long-tamp/issues/17) MuJoCo scene export (MJCF from the example's URDFs)
- [x] [#18](https://github.com/thanhndv212/long-tamp/issues/18) MuJoCo execution backend: trajectory tracking controller, gripper actuation
- [x] [#19](https://github.com/thanhndv212/long-tamp/issues/19) Skill capabilities (pre/post conditions, start/end poses) and a screwing skill stub
- [x] [#20](https://github.com/thanhndv212/long-tamp/issues/20) Drift check before executing a cached plan; replan from observed state;
      plan-ahead in a planner process
- [x] [#67](https://github.com/thanhndv212/long-tamp/issues/67) The UR10 from Universal Robots' own description package
- [x] [#71](https://github.com/thanhndv212/long-tamp/issues/71) The Robotiq 2F-85 from PickNik's ros2_robotiq_gripper, with contact grasps in MuJoCo

**Exit test:** a full two-part mission executes in simulation with tracking control;
injected drift triggers a replan and the mission completes.

## M5 — Multi-arm partial-order execution

**Released as 0.6.0 on 2026-10-01.**

[Milestone 5](https://github.com/thanhndv212/long-tamp/milestone/5)

- [x] [#21](https://github.com/thanhndv212/long-tamp/issues/21) Partial-order plan skeletons (precedence constraints); concurrent arm execution
      where the domain allows; BehaviorTree.CPP export lowers to `Parallel` + sync

**Exit test:** the parallel mission beats the sequential baseline's wall-clock time.
Revisit against ScheduleStream before starting.

## M6 — AI model integration

**Released as 0.7.0 on 2026-10-01.**

[Milestone 6](https://github.com/thanhndv212/long-tamp/milestone/6) · Design:
[ADR-0006](../adr/0006-ai-model-integration.md)

Any model, through one gateway; models act through typed, checked roles and gated tools;
interactive and autonomous operation. A model writes the problem, never the plan (ADR-0001).

- [x] [#22](https://github.com/thanhndv212/long-tamp/issues/22) A model writes a mission's
      goal from an instruction, through the same validation gate (PR #85)
- [x] [#86](https://github.com/thanhndv212/long-tamp/issues/86) One model gateway for
      Anthropic- and OpenAI-compatible APIs (`long_tamp.ai`)
- [x] [#87](https://github.com/thanhndv212/long-tamp/issues/87) Typed model roles (contracts,
      checkers, fallbacks); grounder and plan reviewer
- [x] [#88](https://github.com/thanhndv212/long-tamp/issues/88) Execution supervisor role with
      autonomy limits and escalation
- [x] [#89](https://github.com/thanhndv212/long-tamp/issues/89) Interactive mission session:
      gated tools and a terminal chat
- [x] [#91](https://github.com/thanhndv212/long-tamp/issues/91) Exit test

**Exit test:** from one instruction, the screw-assembly mission runs in MuJoCo to the end with
no human input, through an injected failure that needs a goal-level decision, or it stops
within its limits with an escalation report.

## 0.8.0: the mission UI (from the backlog)

**Released as 0.8.0 on 2026-10-02.** Picked from the backlog after M6:

- [x] [#24](https://github.com/thanhndv212/long-tamp/issues/24) Web mission viewer on the event stream (`long_tamp.viewer`)
- [x] [#90](https://github.com/thanhndv212/long-tamp/issues/90) Chat panel in the viewer (`ChatBridge`)
- [x] [#104](https://github.com/thanhndv212/long-tamp/issues/104) Mission UI: launcher, plan card, Start button, one-screen layout
- [x] [#105](https://github.com/thanhndv212/long-tamp/issues/105) Plan monitor: the plan as a live behavior tree
- [x] [#106](https://github.com/thanhndv212/long-tamp/issues/106) The 3D scene in its own process
- [x] [#108](https://github.com/thanhndv212/long-tamp/issues/108) Progress from a step's planning; skip/abort mid-step
- [x] [#109](https://github.com/thanhndv212/long-tamp/issues/109) Step watchdog (model or rule)
- [x] [#114](https://github.com/thanhndv212/long-tamp/issues/114) Reset mission button

## Backlog (not scheduled)

- [#23](https://github.com/thanhndv212/long-tamp/issues/23) Optional PDDLStream adapter
  (`long-tamp[pddlstream]`), also an independent cross-check (moved out of M6)
- [#98](https://github.com/thanhndv212/long-tamp/issues/98) One live Claude call through the
  gateway (needs API credit; follow-up of #86)
- [#25](https://github.com/thanhndv212/long-tamp/issues/25) BehaviorTree.CPP node plugin library for ROS 2 / Nav2 users: done, `long_tamp_bt_nodes_plugin`
- [#26](https://github.com/thanhndv212/long-tamp/issues/26) Re-enable the spline path optimizer once hpp-core caps the QP solve
  ([bug 6](../bugs/hpp-core-unbounded-planning-loops.md))
- Cross-step backtracking in the refiner, when an example needs it
- ROS 2 bridge example (JTC / MPC execution backends)
