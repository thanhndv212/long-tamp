# 0002. Capability effects are checked at plan time and at run time

- Status: Accepted
- Date: 2026-09-27

## Context

`CapabilityDescriptor.effects` is declared but unused. Plan validation checks types,
not feasibility: a plan that grasps with an occupied gripper passes and fails minutes
later inside the motion planner. At run time, "already done" is an in-memory set of
completed step ids, so a mission cannot resume after a restart, and cannot start from
a state where part of the work is already done.

Mission executives that attach a "still needed?" check to every action (goal-regression
or teleo-reactive style) handle arbitrary initial states and restarts well, but only
when those checks read the world. Checks that read bookkeeping flags ("plan computed")
silently skip steps whose effect was never achieved.

## Decision

Each capability declares typed preconditions and effects over a small predicate
vocabulary. The same predicates are used twice:

- **Plan time:** `TaskPlan.from_dict` simulates the plan symbolically and rejects it if a
  precondition can't hold. `TaskStepReady` checks preconditions for real.
- **Run time:** before a transaction runs, its effect is evaluated against the world;
  if it already holds, the step is skipped.

Predicates come in two kinds, and nothing else may feed them:

- **observed**: read from the world model (grasp tracker, object poses, joint states);
- **recorded**: facts no sensor shows afterwards (e.g. `screwed(part, hole)`), written
  only by a completed execution into the mission's run log, never by planning.

Planner and executor bookkeeping (plans computed, attempt counts) is never a predicate.

## Consequences

- Resume after a restart and robustness to initial states come from one definition,
  without per-mission hand-written checks.
- Capabilities need a world-state provider, and the example missions must expose their
  state through it.
- The run log becomes part of the world state for recorded facts, so it must be
  written atomically (as `MissionCheckpoint` already does).
