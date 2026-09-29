# Mission events

A mission writes one JSON object per line (JSONL) for every status change of a plan node.
The Python executor and the BehaviorTree.CPP host write the same schema, so a mission can
be traced, replayed or visualized the same way whichever ran it.

- Python: `run_plan(session, on_event=sink)` or `PlanExecutor(..., on_event=sink)`, with
  `sink = JsonlEventWriter("events.jsonl")` (from `long_tamp.tasks.task_planning.events`).
- C++ host: `agimus_taskplan_bt --factory <name> --events events.jsonl`.
- Screw assembly writes `<run folder>/events.jsonl`, appended to on `--resume`.

Events are flushed one by one, so a killed mission keeps every event up to the kill.

## Schema `long-tamp.events/1`

| Field | Type | Meaning |
|---|---|---|
| `schema` | string | `"long-tamp.events/1"` |
| `t` | number | Unix time, seconds |
| `source` | string | `"python"` or `"bt"` |
| `ir_id` | string | id of the TaskPlan IR node |
| `role` | string | the part of that node that changed status (below) |
| `name` | string | the BT element's name (the same in both sources) |
| `status` | string | `RUNNING`, `SUCCESS`, `FAILURE` or `SKIPPED` |
| `previous` | string | status before the change: `IDLE` or `RUNNING` |
| `message` | string, optional | why: a failure message, or why a step counts as complete |
| `metrics` | object, optional | numbers about the transition (below) |

Roles:

| Role | For | BT element |
|---|---|---|
| `sequence`, `fallback`, `retry`, `condition`, `operation` | the IR node of that type | `Sequence`, `Fallback`, `RetryUntilSuccessful`, `TaskCapabilityCondition`, `ExecuteTaskStep` |
| `transaction` | a transaction | its `Fallback` |
| `complete` | "is its effect already there?" (`SUCCESS`: skipped) | `TaskStepComplete` |
| `ready` | preconditions, then the attempts | `Sequence` |
| `precondition` | its preconditions | `TaskStepReady` |
| `attempts` | its retry budget | `RetryUntilSuccessful` |
| `execute` | one attempt at the step | `ExecuteTaskStep` |
| `motion` | a command executed on a backend (Python executor only) | none |

Transitions follow BehaviorTree.CPP: composites go `RUNNING`, then `SUCCESS` or `FAILURE`;
leaves go straight to their result; resets to `IDLE` are not events. Motion comes before
the `execute` result it belongs to.

Metrics, when present: `execute` has `attempt` and `seconds`; `motion` has `seconds`,
`duration` (the command's), `feedback_count`, `busy_retries` and, on failure, `reason`
(see [Execution backends](execution.md)), plus whatever the backend measured
(`Feedback.metrics`; the MuJoCo backend reports tracking error and drift, see
[Simulation](simulation-mujoco.md)).

## Example

A transaction whose first attempt fails and whose second succeeds:

```json
{"ir_id": "move-b", "role": "transaction", "name": "Move b transaction", "status": "RUNNING", "previous": "IDLE", ...}
{"ir_id": "move-b", "role": "complete", "name": "Move b complete", "status": "FAILURE", "previous": "IDLE", "message": "not_completed", ...}
{"ir_id": "move-b", "role": "ready", "name": "Move b ready", "status": "RUNNING", "previous": "IDLE", ...}
{"ir_id": "move-b", "role": "precondition", "name": "Move b precondition", "status": "SUCCESS", "previous": "IDLE", ...}
{"ir_id": "move-b", "role": "attempts", "name": "Move b retry", "status": "RUNNING", "previous": "IDLE", ...}
{"ir_id": "move-b", "role": "execute", "name": "Move b", "status": "FAILURE", "previous": "IDLE", "message": "no path", "metrics": {"attempt": 1, "seconds": 12.4}, ...}
{"ir_id": "move-b", "role": "execute", "name": "Move b", "status": "SUCCESS", "previous": "IDLE", "metrics": {"attempt": 2, "seconds": 9.1}, ...}
{"ir_id": "move-b", "role": "attempts", "name": "Move b retry", "status": "SUCCESS", "previous": "RUNNING", ...}
{"ir_id": "move-b", "role": "ready", "name": "Move b ready", "status": "SUCCESS", "previous": "RUNNING", ...}
{"ir_id": "move-b", "role": "transaction", "name": "Move b transaction", "status": "SUCCESS", "previous": "RUNNING", ...}
```

The C++ host doesn't measure `metrics` or report `message`s yet: its events carry the
transitions only.
