# 0001. Split task planner, refiner and executor

- Status: Accepted
- Date: 2026-09-27

## Context

The task level is written by hand in three different ways today: a `(gripper, handle)`
list for `GraspSequencePlanner.plan_sequence()`, a Python mission loop over
`run_block_with_recovery()` (screw assembly), and a TaskPlan IR compiled to
BehaviorTree.CPP (TWIN only). The BT drives *planning*, so the tree structure limits
recovery to retrying the same step from the same configuration, while BT strengths
(reactivity, halting, parallel branches) go unused. There is no automated task
planner, and no way to swap one in.

Published LLM + TAMP systems that work on real robots (e.g. ViLaIn-TAMP, arXiv
2506.03270) use the same split: a symbolic planner produces the action sequence, a
motion layer binds it to geometry, and motion failures return to the planner as facts.

## Decision

Separate three roles behind small interfaces:

- **Task planner** (pluggable): produces a plan skeleton (grounded capability calls)
  from a symbolic problem and a set of blocked facts. Implementations: hand-written,
  classical PDDL via Unified Planning, PDDLStream (optional extra), a language model
  that writes the *problem*, not the plan.
- **Refiner** (long_tamp's core): binds a skeleton to grasps, configurations and paths
  on HPP's constraint graph, with recovery; on failure it returns structured facts.
- **Executor** (pluggable): runs a refined plan. The Python executor is the default;
  BehaviorTree.CPP is an export target; simulator and robot backends plug in below it.

The TaskPlan IR stays the validated, fingerprinted orchestration format between them.
Every planner's output passes the same validation gate.

## Consequences

- The BT no longer runs planning; the refiner can search across steps when needed.
- Optional heavy dependencies (PDDLStream, a simulator) stay extras; the base install
  and `pip install long-tamp` story are unchanged.
- Interfaces are frozen only once two real implementations exist per role, to avoid
  abstracting too early.
- A repair policy decides the level of recovery per failure: retry the step, re-refine,
  replan the task with a new fact, or replan from the observed state.
