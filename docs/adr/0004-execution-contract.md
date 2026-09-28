# 0004. The execution contract: polled backends, heartbeats, duration-scaled deadlines

- Status: Accepted
- Date: 2026-09-28
- Issues/PRs: #8

## Context

ADR-0001 makes the executor pluggable, with robot, simulator and viewer backends
underneath. Each integration so far rediscovered the same execution problems:

- **A busy backend is not a failed command.** A mission executive that treated a
  rejected goal ("planner still busy") as a failure retried immediately and piled up
  redundant goals (a retry storm).
- **Wall-clock timeouts cancel healthy commands.** A trajectory controller under a slow
  simulator (real-time factor well below 1) took over a minute for a 37 s trajectory; a
  flat 60 s deadline cancelled it mid-motion and left the robot far from both ends.
- **Silence is the real signal of a stuck backend,** not elapsed time: a long command
  that keeps reporting progress is healthy.
- **Pausing mid-command is unsafe.** Operators need pause/resume and breakpoints, but at
  step boundaries.

## Decision

`long_tamp.execution` (ROS-free, pure Python):

- A backend implements a **polled** protocol: `start(command)` returns `RUNNING`, `BUSY`
  or `FAILURE`; `poll()` returns the status and an optional `Feedback` heartbeat;
  `cancel()` stops the command. Polling keeps backends free of threads and fits ROS
  action clients, simulator loops and stubs alike.
- `run_command(backend, command, policy)` supervises one command:
  - retries `BUSY` starts with a backoff, up to a limit;
  - cancels when no heartbeat arrives for `inactivity_timeout`;
  - cancels past `duration / min_real_time_factor + deadline_margin`, with no deadline
    when the duration is unknown;
  - reports the reason: `failed`, `busy`, `inactivity`, `deadline` or `stopped`.
- `ExecutionControl` pauses, resumes and stops at **step boundaries**
  (`checkpoint(step, "before"|"after")`), with breakpoints on boundaries. A pause lets the
  running command finish; `stop()` also cancels it.
- Clock and sleep are injected, so timeout behaviour is tested deterministically.

## Consequences

- Backends stay small: a JTC action client, a MuJoCo backend (M4) or a viewer playback
  only map their own status onto four values and send heartbeats.
- The executor (#9) and its event stream (#11) build on `run_command` and
  `ExecutionControl` rather than re-implementing timeouts per backend.
- Backends must send heartbeats while making progress. One that can't report progress
  needs a generous `inactivity_timeout`, which weakens stuck detection for it.
