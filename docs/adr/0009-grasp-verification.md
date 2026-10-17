# 0009. Grasp verification: a hand interface, measured openings, facts

- Status: Accepted
- Date: 2026-10-12
- Issues/PRs: #127

## Context

A grasp is assumed to succeed:

- the grasp tracker adds `holds(g, h)` when the planned grasp transition runs, whether or not
  the object is in the hand;
- the MuJoCo backend snaps the object into the planned grasp when it is within
  `grasp_tolerance` of it, and otherwise fails with a message that nothing downstream can
  read.

On hardware a missed grasp then turns into a place with an empty hand. The SO-101 showcase
(M7) needs the miss to be detected and named, so that a repair (#128) can open the hand,
look again and replan the pick.

The hand already says what a grasp should look like. `GraspPlanner` (ADR-0008) computes the
opening at which the pads touch the object, and commands that opening minus
`default_squeeze`. A hand that holds the object stops near the commanded opening. A hand
that closes on nothing keeps closing.

Forces:

- **Backends differ.** The MuJoCo backend measures finger joints. A servo hand reports a
  position and a load. A remote robot answers over the wire (ADR-0007). The check must use
  only what every hand reports: its opening.
- **Thresholds belong to the hand.** A 2F-85 squeezes 2 mm and the SO-101's jaw 5 mm. A fixed
  millimetre threshold fits one of them.
- **Failure facts already have a path.** Skill controllers report `screw_misaligned` and the
  like as `Feedback.facts`, and `run_command` keeps them in `ExecutionResult.facts`.

## Decision

1. **`GripperActuation` protocol** (`long_tamp.execution.gripper`, `runtime_checkable`). A
   backend that drives a hand implements:
   - `open_gripper(gripper)`;
   - `close_gripper(gripper, width=None, effort=None)`, which closes to `width` (default:
     fully) or until the fingers push with `effort`, and returns once they have settled;
   - `gripper_state(gripper) -> GripperState(width, effort)`, where `effort` is `None` when
     the hand doesn't measure it.

   `gripper` is the HPP gripper name. The methods are named `*_gripper`, not `open` and
   `close`, because backends already have a `close()` that ends a connection or a process.
   `ProcessBackend` and `RemoteBackend` forward the three methods when the served backend
   has them, and `GripperState` has its own tag in the JSON codec.

2. **`ExpectedGrasp(gripper, obj, model, contact_width, width, prefix)`** is what a close
   should measure. `FingerClosureTable.expected_grasp(gripper, handle)` builds one from the
   grasp planner's evaluation.
   - `threshold` = `width - MISS_MARGIN * model.default_squeeze`, with `MISS_MARGIN = 2`,
     floored at `model.min_width + 0.5 mm`.
   - `held(state)` is `state.width >= threshold`.
   - `verify(state)` returns `holds(<gripper>, <object>)` or
     `grasp_missed(<gripper>, <object>)`.

   Effort is reported but not part of the decision: its units and scale differ from hand to
   hand, and no model carries an expected effort yet.

3. **The facts.** A backend reports the outcome of each close in `Feedback.facts`:
   - a hold: `holds(g, o)`, and the command goes on;
   - a miss: `grasp_missed(g, o)`, and the command fails.

   The arguments are the HPP gripper (`ur10_left/gripper`) and the object (`part1`), as the
   predicate language takes them (ADR-0002). `PlanExecutor` appends a failed command's facts
   to the step's failure message (`... [grasp_missed(ur10_left/gripper, part1)]`), and they
   stay in `run.executions[i].result.facts` for a repair to read.

4. **MuJoCo.** `MuJoCoBackend(expect=..., hands=...)`:
   - `expect(object body, carrier body)` returns the `ExpectedGrasp` (`GripTable.expect`).
   - With contact grasps, a new grasp whose object is beyond `grasp_tolerance` no longer
     fails before the fingers move. The fingers close on whatever is there, and the object
     stays welded where it is. After `grip_time`, the opening is measured from the driven
     finger joints and judged as above.
   - A missed weld grasp has no fingers to measure. It fails as before, now with the
     `grasp_missed` fact.
   - `hands` (`GripperBinding`s) are what `open_gripper` / `close_gripper` /
     `gripper_state` act on, between commands. Opening a hand that holds an object welds the
     object where it is.

   Without `expect`, nothing changes.

## Consequences

- A missed grasp is a named, machine-readable failure, so #128 can repair it: open the hand,
  look again, replan the pick.
- On the screw-assembly scene, a 2F-85 holding a part measures 26.9 mm and one closing on
  nothing measures 14.0 mm. The threshold is 24.0 mm. The test removes the part before the
  grasp and gets `grasp_missed`.
- A hand closing on an object thinner than `MISS_MARGIN` squeezes can only report a miss
  when it closes fully. Such an object leaves the hand no room to tell the two apart.
- The grasp tracker still adds `holds` at planning time, and this ADR does not change that.
  The repair works from the fact, and the tracker's atom is rolled back by replanning from
  the observed state.
- A hardware backend must measure the opening with the same model (a servo position through
  `driving_value` and `width_at`). An uncalibrated hand reports misses that didn't happen.
