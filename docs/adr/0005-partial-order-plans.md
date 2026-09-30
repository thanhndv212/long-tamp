# 0005. Partial-order plans and concurrent arms

- Status: Accepted
- Date: 2026-09-30

## Context

Plans are total orders, so two arms never move at once, even when one arm's step has
nothing to do with the other's (the right arm going home while the left arm releases a
part). M5 (#21) asks for partial-order skeletons and concurrent execution. Planning is
most of a mission's time (a 4-part screw assembly: ~750 s of planning against ~240 s of
motion), and HPP plans one path at a time. ScheduleStream (arXiv 2511.04758) schedules
arms together with sampling; this ADR stops at running independent steps' motions
concurrently.

## Decision

- **IR:** a `parallel` node whose lanes are steps or sequences of steps. At load time,
  no step of one lane may depend on a step of another lane.
- **Independence is symbolic and conservative.** A later step depends on an earlier one
  when they share a resource (a capability's `resources` name the parameters it holds;
  `ur10_left` and `ur10_left/gripper` are the same arm), when their literals interfere,
  or when either declares neither resources nor literals.
- **`parallelize(document, registry)`** turns runs of consecutive steps in a sequence
  into `parallel` nodes. Conditions, fallbacks and retries stay in place as barriers.
- **Planning stays sequential.** The runner plans lanes one after another; the result is
  the same because the lanes are independent. Executors get `on_group` at the start and
  end of a group.
- **Execution merges motions** (a follow-up PR). A group's lanes are combined joint by
  joint, since each lane moves joints and objects the others don't. The merged motion is
  checked for collisions in HPP before it runs. If the check fails, or if a lane contains
  a skill, the lanes run one after another.
- **BT.CPP export:** `parallel` lowers to `Parallel` (`success_count` = the number of
  lanes, `failure_count` = 1).

## Consequences

- The gain is bounded by motion time. Planning the lanes in parallel would need one
  planner process per lane; that is left for later.
- Symbolic independence doesn't guarantee that the motions don't collide. The geometric
  check at execution keeps concurrency safe; when it fails, the lanes run sequentially.
- Capabilities must declare `resources` to take part.
