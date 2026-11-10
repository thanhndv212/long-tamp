# Run long_tamp on your robot

This guide takes a mission that runs in simulation and runs it on a real arm. You write one
class on the robot's side, the **backend**, and serve it over TCP. The planner then drives
it like the MuJoCo backend. The worked example is the SO-101: the planner runs in the HPP
container, and the arm's servos are on the host's USB port. The real SO-101 driver lives in
[soarm_tamp](https://github.com/thanhndv212/soarm_tamp).

What you need:

- A task that plans in simulation. See [standalone usage](standalone-usage.md).
- A way to command your arm's joints and read them back: a servo bus, a vendor SDK, or a
  ROS 2 controller.
- For grasps, a model of your hand. See [registering a gripper model](#4-register-a-gripper-model).

The pieces, from the planner to the motors:

```text
planner (container)                         robot side (host)
PlanExecutor ── RemoteBackend ──TCP/JSON──▶ serve_backend ── YourBackend ── servos
                 (client)                    (server)          start/poll/cancel
```

## 1. The execution contract

The executor talks to a backend through three methods
([ADR-0004](../adr/0004-execution-contract.md), [execution backends](execution.md)):

| Method | Returns | Meaning |
| --- | --- | --- |
| `start(command)` | `RUNNING` / `BUSY` / `FAILURE` | Accept one `ExecutionCommand` (`step_id`, `duration`, `payload`). `BUSY` means "not now" and is retried, it is not a failure. |
| `poll()` | `(status, Feedback \| None)` | `RUNNING`, `SUCCESS` or `FAILURE`, plus a `Feedback` whenever there was progress since the last poll. |
| `cancel()` | — | Stop and hold where you are. It must be safe to call at any time: twice, or with nothing running. |

The executor supervises every command under an `ExecutionPolicy`. Tune it to your arm:

| Field | Default | Fails the command when |
| --- | --- | --- |
| `inactivity_timeout` | 30 s | no `Feedback` heartbeat arrives for this long |
| `min_real_time_factor` | 0.5 | progress runs slower than half real time |
| `deadline_margin` | 10 s | the command runs past `duration` by this much |
| `busy_retries`, `busy_backoff` | 10, 1 s | the backend still answers `BUSY` after this many retries |
| `poll_interval` | 0.1 s | (how often `poll` is called) |
| `max_start_drift` | `None` | the robot is this far [rad] from the path's start (see [the drift check](#5-the-drift-check)) |

A failed command's `ExecutionResult.reason` is one of `failed`, `busy`, `inactivity`,
`deadline` or `stopped`. A remote backend adds `disconnected`.

## 2. Write the backend

Over the wire, a command's `payload` is a `SampledPath`: the planned path sampled every 5 ms.
`payload.eval(t)` returns `(q, ok)`, the planner configuration at time `t`, and
`payload.length()` is the duration. A planner configuration holds every joint of the
planning problem, including object poses. Map it to your joints by name with the robot's
`rankInConfiguration`, once, on the planner's side. Then pass the indices to the backend.

A sketch for the SO-101. `Servos` stands for your driver: soarm_tamp's, or a vendor SDK.

```python
import threading
import time

import numpy as np

from long_tamp.execution import ExecutionCommand, ExecutionStatus, Feedback

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class SO101Backend:
    def __init__(self, port: str, ranks: dict[str, int], rate: float = 100.0):
        self.servos = Servos(port)  # your driver: read/write joint positions [rad]
        self.index = [ranks[f"so101/{j}"] for j in JOINTS]
        self.period = 1.0 / rate
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._status = ExecutionStatus.SUCCESS
        self._progress = 0.0
        self._message = ""

    def start(self, command: ExecutionCommand) -> ExecutionStatus:
        if self._thread is not None and self._thread.is_alive():
            return ExecutionStatus.BUSY
        path = command.payload
        self._stop.clear()
        self._status, self._progress = ExecutionStatus.RUNNING, 0.0
        self._thread = threading.Thread(target=self._run, args=(path,), daemon=True)
        self._thread.start()
        return ExecutionStatus.RUNNING

    def _run(self, path) -> None:
        length, t0 = path.length(), time.monotonic()
        while not self._stop.is_set():
            t = min(time.monotonic() - t0, length)
            q, ok = path.eval(t)
            if not ok:
                return self._finish(ExecutionStatus.FAILURE, f"path eval failed at {t:.2f}s")
            self.servos.write_goal(np.asarray(q)[self.index])
            with self._lock:
                self._progress = t / length if length > 0 else 1.0
            if t >= length:
                return self._finish(ExecutionStatus.SUCCESS, "done")
            time.sleep(self.period)
        self._finish(ExecutionStatus.FAILURE, "cancelled")

    def _finish(self, status, message) -> None:
        with self._lock:
            self._status, self._message = status, message

    def poll(self):
        with self._lock:
            return self._status, Feedback(progress=self._progress, message=self._message)

    def cancel(self) -> None:
        self._stop.set()
        self.servos.hold()  # goal := measured position, torque stays on
```

Points that matter on hardware:

- **Stream, don't block.** `start` returns at once. A thread (or the servo controller's
  own trajectory interface) runs the motion, and `poll` only reads its state.
- **Progress is the heartbeat.** The executor treats a stalled `progress` as a hung
  controller after `inactivity_timeout`. Report the time actually reached, not the time
  commanded.
- **Hold, don't drop.** `cancel` keeps torque on and sets the goal to where the arm is. An
  arm that goes limp when a command is cancelled drops what it holds.
- **Park before torque.** Read the arm's position and set it as the goal *before* you
  enable torque. A servo enabled with a stale goal jumps to it.

Before serving it, check it in-process with `run_command(backend, command, policy)` on a
short, slow path.

## 3. Serve it

On the robot's side, serve the backend with `serve_backend`, or from the command line:

```bash
python -m long_tamp.execution.remote my_robot:make_backend --kwargs '{"port": "/dev/ttyACM0"}' --port 7070
```

`my_robot:make_backend` is any `module:callable` that returns a backend. `--kwargs` is JSON.
The same in Python:

```python
from long_tamp.execution import serve_backend

serve_backend(make_backend(port="/dev/ttyACM0"), host="127.0.0.1", port=7070, timeout=2.0)
```

On the planner's side, use a `RemoteBackend` wherever a backend is taken:

```python
from long_tamp.execution import ExecutionPolicy, PlanExecutor, RemoteBackend

backend = RemoteBackend("127.0.0.1", 7070)
executor = PlanExecutor(session, backend=backend, policy=ExecutionPolicy(max_start_drift=0.05))
```

`RemoteBackend` forwards `start`, `poll` and `cancel`, and the optional methods the served
backend has: `start_error`, `observed_config`, `disturb`, `open_gripper`, `close_gripper`
and `gripper_state`. The wire is length-prefixed JSON, never pickle. Paths travel as
sampled arrays, and anything the codec doesn't know is refused
([ADR-0007](../adr/0007-remote-execution.md)).

**Container to host (the SO-101 setup).** Run the server on the host, where the USB port is.
From a Docker Desktop container, `host.docker.internal` reaches the host:
`RemoteBackend("host.docker.internal", 7070)`. The server must then listen where the
container reaches it, not on `127.0.0.1` only. Bind it to the Docker bridge address rather
than `0.0.0.0` on an untrusted network.

**Robot on another machine.** Keep the server on localhost and forward the port over SSH:

```bash
ssh -N -L 7070:127.0.0.1:7070 robot-pc
```

## 4. Register a gripper model

The planner chooses a finger closure for each grasp from a model of your hand
([ADR-0008](../adr/0008-gripper-models.md)). `PRESETS` has `robotiq_2f85`, `panda_hand` and
`so101_jaw` (`SO101_JAW`, a `RotaryJawGripperModel`: one jaw swinging against a fixed one).
For another hand, describe it as a `ParallelGripperModel` and register it:

```python
from long_tamp.grasping import ParallelGripperModel, calibrate_from_urdf, register_gripper

stroke, inner_front = calibrate_from_urdf(
    "my_hand.urdf",
    tcp_frame="tcp",
    pad_frames=("left_pad", "right_pad"),
    joints={"finger_joint": 1.0},
    q_values=[0.0, 0.2, 0.4, 0.6, 0.8],
)
register_gripper(ParallelGripperModel(name="my_hand", joints={"finger_joint": 1.0}, stroke=stroke, ...))
```

`calibrate_from_urdf` measures the stroke table (driving value, gap, pad position) by
forward kinematics. It needs pinocchio. Then fill in the pads, fingers and palm from the
hand's drawings. Set `default_squeeze` to how far past contact the hand closes to grip: a
position-controlled servo grips by stalling past contact, which is about 5 mm for the
SO-101.

## 5. The drift check

A plan starts where the previous step's plan ended. If the arm isn't there (tracking error,
a bump, a plan made ahead of time), the cached plan is stale. Give the backend two optional
methods and set `max_start_drift`:

```python
    def start_error(self, command) -> float | None:
        """Largest joint error [rad] between the arm and the command's start."""
        q, ok = command.payload.eval(0.0)
        if not ok:
            return None
        return float(np.max(np.abs(self.servos.read() - np.asarray(q)[self.index])))

    def observed_config(self, like, objects=False):
        """``like`` (a planner configuration) with the arm's joints as measured."""
        q = np.array(like, dtype=float)
        q[self.index] = self.servos.read()
        return q
```

Beyond `max_start_drift` the executor emits a `drift` event and calls `on_drift(node,
observe)`, which plans the step again from `observe(like)`. Without `on_drift` the step
fails with reason `drift`. See [drift](execution.md#drift-replanning-from-the-observed-state).

`observed_config` only knows the arm's joints, so it keeps the objects from `like`. Until
perception reports object poses, a moved object is caught by grasp verification, not here.

## 6. Grippers and grasp verification

A backend that drives a hand adds `open_gripper(g)`, `close_gripper(g, width=None,
effort=None)` and `gripper_state(g) -> GripperState(width, effort)`
([ADR-0009](../adr/0009-grasp-verification.md)). `g` is the HPP gripper name, such as
`"so101/gripper"`. Convert between the jaw angle and the opening with the model:
`model.q_for_width(width)` and the stroke table.

After the hand closes on a planned grasp, compare the measured opening with what the grasp
should measure. `FingerClosureTable.expected_grasp(gripper, handle)` builds an
`ExpectedGrasp` on the planner's side, and `expected.verify(state)` returns
`holds(g, obj)` or `grasp_missed(g, obj)`. An opening more than two `default_squeeze`
below the commanded one means the hand closed on nothing. Report the fact in
`Feedback.facts` and fail the command on a miss. The executor then
[retries from the observed world](../adr/0010-missed-grasp-repair.md) instead of carrying on with an empty
hand.

`ExpectedGrasp` holds a gripper model, so it doesn't cross the wire. On the robot's side,
rebuild it from the preset (`PRESETS["so101_jaw"]`) and the numbers the planner computed
(`contact_width`, `width`), passed as factory `--kwargs`.

## 7. Safety: what the robot's side must do

The library stops commands. It does not make the arm safe. The robot's side owns the rest:

- **Stop on disconnect.** `serve_backend` calls `cancel` when the client disconnects, or
  sends nothing for `timeout` seconds (2 s by default) while a command runs. The client's
  polls are the heartbeat, so keep `timeout` well above `poll_interval`, and below how long
  the arm may move unwatched. The planner sees a `FAILURE` whose message starts with
  `disconnected`.
- **A `cancel` that always works.** It is called from the server's thread on disconnect,
  possibly while `start` or `poll` runs. Make it idempotent and thread-safe.
- **One client at a time.** The server refuses a second client. There is no
  authentication: bind to localhost and use an SSH tunnel, or a network you trust.
- **Limits on the robot's side.** Clamp goals to the joint limits and cap velocity in your
  driver, whatever the path says. A planner bug or a corrupted frame must not reach the
  motors unchecked.
- **An emergency stop the software doesn't own.** A hardware e-stop or a torque-off switch
  within reach. `cancel` is not an e-stop.
- **Start slow.** Run the first missions with the arm's velocity limit scaled down. Then
  watch `tracking_error` against the plan before you trust the arm at full speed.

## Checklist

1. The backend passes `run_command` in-process on a short path, with `cancel` mid-motion.
2. `python -m long_tamp.execution.remote ...` serves it, and `RemoteBackend` reaches it
   from where the planner runs.
3. Unplugging the client (or killing the planner) stops the arm within `timeout`.
4. `start_error` reads near zero at the start of each step, and a hand bump past
   `max_start_drift` triggers a replan.
5. A grasp on nothing reports `grasp_missed` and the mission retries.
