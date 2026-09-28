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
