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

Capabilities declare what they need and what they achieve; plans are checked for
feasibility before any geometry runs; missions skip steps whose effect already holds.

- [ ] Typed preconditions and effects on `CapabilityDescriptor`; `TaskPlan.from_dict`
      simulates them and rejects infeasible plans; `TaskStepReady` checks preconditions
- [ ] World-state provider: observed predicates (grasps, poses) and recorded facts
      (written only by completed execution, e.g. `screwed(part, hole)`)
- [ ] Runtime effect guard: a transaction is skipped when its effect already holds
      (replaces the in-memory completed set)
- [ ] Screw-assembly domain and mission expressed as a TaskPlan (`plan_block`,
      `home_move` capabilities)
- [ ] Initial-state scenario harness (V4), ≥4 screw-assembly scenarios, TWIN regrasp
      from ≥3 initial grasp states
- [ ] Static plan diagram: `to_mermaid()` / `to_dot()` for TaskPlans

**Exit test:** the screw-assembly TaskPlan succeeds from ≥4 initial states with no plan
change; the 10-seed batch gate passes.

## M2 — Executor contract and refiner interface (0.3.0)

- [ ] Executor contract: statuses `SUCCESS/FAILURE/RUNNING/BUSY`, heartbeats with
      inactivity timeouts, duration-scaled execution timeouts, pause/resume/breakpoints
- [ ] Python executor for TaskPlans (default), with mock and trajectory-playback backends
- [ ] `Refiner` interface over `run_block_with_recovery()` and the phase-target lookahead
- [ ] Structured JSONL event stream from the executor; the BehaviorTree.CPP host emits
      the same stream (IR ids stamped on compiled nodes)
- [ ] Resume from world state after the process is killed mid-mission

**Exit test:** batch gate passes on the Python executor; a mission killed mid-run
resumes from world state, not from a block index.

## M3 — Automatic task planning (0.4.0)

- [ ] PDDL domain/problem export from the capability registry and a goal
- [ ] Unified Planning adapter (Fast Downward) producing plan skeletons → TaskPlan
- [ ] Structured refiner failure facts (`CanNotReach`, `IKUnreachable`,
      `ReleaseEdgeInfeasible`, `Blocks`) and a block → replan loop
- [ ] Screw-assembly domain with real choices: clamp slot, part order, driving arm

**Exit test:** an N-part screw-assembly goal is planned automatically; an injected
`CanNotReach` triggers a re-clamp and the mission completes.

## M4 — Execution in simulation and controllers (0.5.0)

- [ ] MuJoCo scene export (MJCF from the example's URDFs)
- [ ] MuJoCo execution backend: trajectory tracking controller, gripper actuation
- [ ] Skill capabilities (pre/post conditions, start/end poses) and a screwing skill stub
- [ ] Drift check before executing a cached plan; replan from observed state;
      plan-ahead in a planner process

**Exit test:** a full two-part mission executes in simulation with tracking control;
injected drift triggers a replan and the mission completes.

## M5 — Multi-arm partial-order execution

- [ ] Partial-order plan skeletons (precedence constraints); concurrent arm execution
      where the domain allows; BehaviorTree.CPP export lowers to `Parallel` + sync

**Exit test:** the parallel mission beats the sequential baseline's wall-clock time.
Revisit against ScheduleStream before starting.

## M6 — Language front end and PDDLStream adapter

- [ ] Language/vision model that produces the planning *problem* (goal + initial
      state), never the plan, through the same validation gate
- [ ] Optional PDDLStream adapter (`long-tamp[pddlstream]`), also used as an
      independent cross-check

## Backlog (not scheduled)

- HTML mission viewer (replay/live) on the event stream
- BehaviorTree.CPP node plugin library for ROS 2 / Nav2 users
- Re-enable the spline path optimizer once hpp-core caps the QP solve
  ([bug 6](../bugs/hpp-core-unbounded-planning-loops.md))
- Cross-step backtracking in the refiner, when an example needs it
- ROS 2 bridge example (JTC / MPC execution backends)
