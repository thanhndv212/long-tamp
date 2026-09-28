# Execution backends and the execution contract

`long_tamp.execution` defines how planned motion is run on a robot, a simulator or a
viewer, without depending on ROS. Design: [ADR-0004](../adr/0004-execution-contract.md).

## Writing a backend

A backend runs one command at a time and is driven by polling:

```python
from long_tamp.execution import ExecutionCommand, ExecutionStatus, Feedback


class MyBackend:
    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        # Send command.payload (e.g. a trajectory) to the controller.
        # RUNNING if accepted; BUSY if it can't take a command right now
        # (it will be retried, it is not a failure); FAILURE if rejected.
        ...

    def poll(self) -> tuple[ExecutionStatus, Feedback | None]:
        # RUNNING / SUCCESS / FAILURE, plus a Feedback heartbeat whenever
        # there was progress since the last poll.
        ...

    def cancel(self) -> None:
        ...
```

## Running a command

```python
from long_tamp.execution import ExecutionCommand, ExecutionPolicy, run_command

policy = ExecutionPolicy(
    inactivity_timeout=30.0,    # no heartbeat for this long: stuck, cancel
    min_real_time_factor=0.5,   # a slow simulator may run at half real time
    deadline_margin=10.0,       # deadline = duration / 0.5 + 10 s
    busy_retries=10,
    busy_backoff=1.0,
)
result = run_command(backend, ExecutionCommand("b03", duration=37.0, payload=traj), policy)
if result.status != "success":
    print(result.reason, result.message)  # failed | busy | inactivity | deadline | stopped
```

## Pause, resume, stop, breakpoints

```python
from long_tamp.execution import ExecutionControl

control = ExecutionControl()
control.add_breakpoint("b03", "before")  # pause before step b03 starts

# In the executor, around each step:
if not control.checkpoint(step_id, "before"):
    ...  # stopped: don't start the step
result = run_command(backend, command, policy, control=control)
control.checkpoint(step_id, "after")

# From another thread (a UI, a service):
control.pause(); control.resume(); control.stop()
```

A pause takes effect at the next step boundary; the running command finishes first.
`stop()` releases a paused executor (its checkpoint returns `False`) and cancels a running
command.

`MockBackend` (scriptable: busy starts, failures, stalls, slow real-time factor) is there
for tests and demos.

## Running a TaskPlan: `PlanExecutor`

`PlanExecutor(session, backend, policy, control).run()` walks the plan with the compiled
BehaviorTree's semantics (`task_planning.runner.run_plan`). For each step that runs, it
checks the pause/stop control, lets the step's capability plan it, then executes the motion
the capability submitted, command by command, on the backend under `run_command`:

```python
from long_tamp.execution import ExecutionCommand, PathPlaybackBackend, PlanExecutor

executor = PlanExecutor(session, backend=PathPlaybackBackend(display=viewer))

def grasp_impl(parameters):          # a capability
    result = planner.grasp(...)      # plans the step, updates the world model
    for path in result["paths"]:
        executor.submit(ExecutionCommand(step_id="grasp", duration=path.length(), payload=path))
    return {}

run = executor.run()                 # PlanRun: success, skipped, failed_step, executions
```

- Motion goes through `submit`, not through the session's JSON responses (those exist for
  the C++ host and can't carry path objects). Commands from a failed planning attempt are
  discarded.
- A failed execution (backend failure, BUSY exhausted, heartbeat silence, deadline) fails
  its step with the reason, and the plan stops there.
- Without a backend, steps are only planned; submitted motion is dropped.
- Execution follows planning, so the world model already reflects a step when its motion
  runs; a failed execution stops the mission rather than rolling that back.

`PathPlaybackBackend` plays a time-parameterized path (anything with `length()` and
`eval(t)`, like an HPP path, or an id resolved by `get_path`) in scaled real time, sending
each configuration to `display` (a viewer) or playing headless. The screw-assembly example
takes `--backend none|mock|playback`.
