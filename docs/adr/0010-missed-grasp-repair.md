# 0010. Repairing a missed grasp: retry from the observed world, then escalate

- Status: Accepted
- Date: 2026-10-12
- Issues/PRs: #128

## Context

ADR-0009 makes a missed grasp a fact: the step fails with `grasp_missed(g, o)`. The mission
repair loop (`plan_execute_repair`, ADR-0001) answers a failure by blocking the failed
step's binding and replanning. That is right when the binding can't work (a clamp that
can't reach a seat). It is wrong for a miss. The binding was fine, but the world changed
under it: the object moved after it was seen. Blocking `part1`'s grasp would plan the
mission without the part, or find no plan.

A miss can also repeat. The object may be out of reach, out of view, or not there at all.
The robot must then stop and say so, and a supervisor (ADR-0006) or a human needs more
than the last failure to decide what to do.

## Decision

1. **Retry, don't block.** `plan_execute_repair(..., retry=, max_retries=2)`: when
   `retry(failure)` is true, the loop replans without blocking anything, and does so at most
   `max_retries` times per step. Each retry counts as a round. `retry_missed_grasp` is the
   predicate for `grasp_missed`. Past the limit, the loop stops with
   `"<step>: still failing after N retries (<facts>)"`.
2. **The executor recovers before the retry.** In the screw-assembly example,
   `recover_missed_grasp`:
   - opens the hand (`open_gripper`, ADR-0009);
   - drops the grasps the tracker assumed for the failed step;
   - replans from the *observed* configuration, where the object is now.

   The default repair is therefore: open, re-observe, replan the pick.
3. **The fact history goes up.** `RepairOutcome.failures` lists every round's failure,
   oldest first. `Situation.failures` passes that list to the supervisor's prompt (when
   there is more than one failure), and to `escalation.json` / `escalation.md` ("Fact
   history").
4. **A miss can be injected.** `MuJoCoBackend.displace(obj, delta)` moves a free object that
   isn't gripped. `ProcessBackend` forwards it. `--inject-miss LABEL:OBJECT:DX,DY,DZ` calls
   it just before the step's motion starts.

## Consequences

- A displaced part is picked again where it lies, and the mission completes (test and V2
  smoke). Completed steps are not redone, as before.
- Other failures keep the blocking policy. A failure is retried only when `retry` says it
  is transient.
- Each retry uses a round, so `--replan` must allow for the retries
  (`--grasp-retries`, default 2).
- "Re-observe" is the backend's configuration. A real robot needs a perception update of
  the object poses here. Until there is one, a retry on hardware replans against the pose
  the object had last time.
