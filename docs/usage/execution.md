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

### Inside a step that is planning: progress, skip, abort

Planning one step can take minutes, for example a lookahead search drawing candidate
targets, then phase after phase. The executor marks the step being planned, and planning
code reports what it is doing and offers safe points to stop
(`long_tamp.execution.activity`):

```python
from long_tamp.execution import activity

with activity.searching("a clamp pose that leaves both holes reachable"):
    for i in range(max_candidates):
        activity.checkpoint("search")        # a skip ends the search here
        activity.progress(f"{i} rejected so far", rejected=i)
        ...
activity.checkpoint("step")                  # an abort ends the step here
```

- **Progress:** each `progress(...)` call is a `progress` event on the step (see
  [Mission events](events.md)). The viewer shows the latest one on the running step.
- **Requests:** `control.request("skip")` or `control.request("abort_step")` come from a UI
  or a watchdog, and act at the step's next checkpoint.
  - **skip** abandons the current search. `GraspSequencePlanner`'s lookahead then plans the
    block without a hint.
  - **abort_step** fails the step and stops the run. The block's grasps are rolled back.
  - A request made while no step is planning is dropped.
- **Interruptions:** they derive from `BaseException`, like `KeyboardInterrupt`, so they get
  through the planner's `except Exception` retries. The executor catches them at the step
  boundary.
- **Where the planner reports and checks:** each lookahead round and candidate, each
  phase, and each replan.

### A watchdog for steps that plan too long

`StepWatchdog` (`long_tamp.execution.watchdog`) follows the event stream. When the step being
planned passes `soft` seconds, it picks from a fixed menu:

- `wait` for longer, but never past `hard`;
- `skip` the search the step is in;
- `abort_step`.

A model decides when a client is given. This is a checked role (ADR-0006): a decision that
fails its check, or a model that can't be reached, falls back to the rule. Without a
client, the rule decides: skip while in a search, otherwise wait. At `hard`, the step is
aborted, whatever was decided before.

```python
from long_tamp.execution.watchdog import StepWatchdog, model_decider

watchdog = StepWatchdog(control, soft=300, hard=900,
                        decide=model_decider(client),   # or None: the rule
                        sink=events).start()
executor = PlanExecutor(session, backend, control=control,
                        on_event=lambda e: (events(e), watchdog.observe(e)))
```

Every decision is a `watchdog` event, and the viewer and the chat show it. An operator's
pending request always comes first, and so does a pause. In the screw assembly, use
`--watchdog 300,900`. `mission_ui.py` turns it on by default.

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

## Drift: replanning from the observed state

A plan is computed before its motion runs, from where the previous step's plan ended. If the
robot isn't there when the motion starts (a tracking error, a bump, a step planned ahead from
an expected state), the cached plan is stale. Before running a step's motion, `PlanExecutor`
asks the backend how far the robot is from where the plan starts:

```python
executor = PlanExecutor(
    session,
    backend=backend,                                  # with start_error / observed_config
    policy=ExecutionPolicy(max_start_drift=0.05),     # rad; None: no check
    on_drift=replan,                                  # (node, observe) -> new commands
)
```

- `backend.start_error(command)`: the largest joint error [rad] between the robot and the
  command's start, or `None` if unknown. The MuJoCo backend implements it.
- Beyond `max_start_drift`, the executor emits a `drift` event (`FAILURE`, with
  `start_drift`) and calls `on_drift(node, observe)`. `observe(like)` returns the robot's
  observed configuration (`backend.observed_config`). The hook plans the step again from
  there and returns its commands, which run instead.
- Without `on_drift`, the step fails with reason `drift`.

The screw assembly replans the block from the grasps it started with and the observed joint
positions, clipped to the planner's joint bounds, keeping the objects where the planner put
them. This assumes the grasps still hold. A 0.2 rad bump of an arm whose gripper holds a
clamped part would break the grasp in the real world, so the planner can't start from that
configuration either. `--inject-drift
"LABEL:JOINT:RAD"` bumps a joint just before step LABEL's motion, to exercise this.

## Planning ahead

With `plan_ahead=True`, the executor plans step k+1 while step k's motion executes in a
worker thread:

1. Step k is planned, and its motion starts in the worker.
2. Step k+1 is planned against the *expected* world. Its preconditions see step k's recorded
   effects, which are pending until k's motion has executed.
3. At the handoff, the executor waits for step k's motion. It commits step k's recorded
   effects, checks drift for step k+1 (and replans if needed), then starts step k+1's motion.

HPP paths may re-project onto their constraints through state they share with the planner,
so the worker never evaluates them. Paths are sampled into arrays (`SampledPath`) in the
planning thread first. A motion that fails stops the plan at the next step boundary.

`PlanRun.timing` reports:

| Key | Meaning |
|---|---|
| `wall` | seconds from start to end |
| `execution` | seconds of motion |
| `idle_between_motions` | seconds the robot waited between two motions, which is what planning ahead removes |
| `waited_for_motion` | seconds planning waited for motion (plan-ahead only) |
| `drift_replans` | steps replanned because of drift |

The screw assembly takes `--plan-ahead`, `--max-drift RAD` and, with `--backend mujoco`,
`--sim-speed 1` (real time: without it motion takes no wall time, so there is nothing to
overlap).
