# 0007. Remote execution: backends served over TCP, JSON frames, stop on disconnect

- Status: Accepted
- Date: 2026-10-12
- Issues/PRs: #125

## Context

The planner and the robot rarely share a process. HPP runs in a Linux container while the
SO-101's servos sit on the host's USB port (`soarm_tamp` bridges them with a manifest file,
which cannot do closed-loop execution). Other robots have their own controller PC, and
RoboCerebra's simulator needs a Python 3.10 environment that long_tamp's cannot share (#156).
`ProcessBackend` (ADR-0004) puts a backend in a child process, but over a pickled pipe on one
machine.

Forces:

- **Safety.** A planner that crashes, or a network that drops, must not leave the arm
  following a stale command. The side that owns the hardware has to notice and stop.
- **Trust.** Unpickling bytes from a socket runs arbitrary code. The wire format must be data.
- **No new dependency.** ROS 2, gRPC or ZeroMQ would each bind every user to one stack. The
  library must integrate into any stack, so the transport has to be the lowest common
  denominator.

## Decision

`long_tamp.execution.remote`, standard library only:

- **Server**, `serve_backend(backend, host, port, timeout=2.0)`, runs next to the hardware.
  **Client**, `RemoteBackend(host, port)`, runs next to the planner. The client is an
  ordinary ADR-0004 backend, so `run_command` and `PlanExecutor` are unchanged. It also
  forwards the optional `start_error`, `observed_config` and `disturb` methods when the
  served backend has them.
- **Wire: length-prefixed JSON frames, never pickle.**
  - Each frame is a 4-byte big-endian length followed by UTF-8 JSON, capped at 64 MB.
  - Requests are `{op, args}`; replies are `{ok, value | error}`.
  - Non-JSON values are tagged one-key objects:
    - `$nd`: a numpy array, as dtype, shape and base64 data.
    - `$path`: a `SampledPath`, as sample times and configurations.
    - `$skill`: a `SkillCommand`.
    - `$command`: an `ExecutionCommand`.
    - `$feedback`: a `Feedback`.
    - `$status`: an `ExecutionStatus`.
    - `$tuple`: a tuple.
  - Anything else is refused with `TypeError`, not guessed. Paths are sampled in the
    caller's thread before sending, as with `ProcessBackend`.
- **Versioning.** The first request is `hello` with `PROTOCOL_VERSION` (1). A mismatch is
  refused. The reply lists the optional methods the backend has. A new tag or operation
  bumps the version; old peers refuse instead of misreading.
- **Stop on disconnect.**
  - The server cancels the running command (`backend.cancel()`, so the robot holds its
    pose) when the client closes the connection, or when it sends nothing for `timeout`
    seconds **while a command runs**. The client's polls are the heartbeat. Silence
    cancels the command but keeps the connection, so a slow client sees the cancel on its
    next poll.
  - An idle client may be silent. The first smoke run showed why: HPP planning in the
    client process starved its keepalive thread for 13 s, and an idle robot has nothing
    to stop. The client still pings every `keepalive` (0.5 s) when it can.
  - On the client side, a lost connection is `FAILURE` with a message starting with
    `disconnected`, so `run_command` reports reason `failed` and the executor stops or
    replans as for any other failure.
- **One client at a time.** A second client is refused with a `busy` error. One robot, one
  commander: arbitration between planners is out of scope.
- **No authentication.** The default bind is `127.0.0.1`. A remote robot is reached through
  an SSH tunnel or a trusted network, as the module docs say.
- Helpers:
  - `RemoteBackend.spawn(factory, ...)` serves a backend from a local child process.
  - `python -m long_tamp.execution.remote module:factory --port N` serves one on a robot
    host.

## Consequences

- Any language can implement a server: it is a TCP socket and JSON. A C++ or ROS 2 node can
  stand in without Python on the robot side.
- The SO-101 can run closed-loop from the container: soarm_tamp serves its streamer, and
  the planner observes state (`observed_config`) and replans on drift (#127, #128, #130).
- RoboCerebra's environment server (#156) reuses the protocol.
- Payloads are limited to what the codec knows. A backend that needs a new payload type
  needs a codec tag and a protocol bump; arbitrary objects no longer cross a process
  boundary.
- Paths cross as samples (5 ms by default), not as HPP paths. A backend that wanted to
  re-project onto constraints cannot. None does: `ProcessBackend` already made this trade.
- The heartbeat timeout bounds how long the robot keeps moving after the planner dies:
  about `timeout` seconds for a silent client, and immediately for a closed socket. A
  backend's `cancel` must therefore be safe to call at any time.
- A planner that polls from a thread while it plans the next step (`plan_ahead`) must keep
  polling within `timeout`. If a planner holds the interpreter lock longer than that, the
  command is cancelled and the executor replans. Raise the server's `timeout` for such
  setups rather than disabling it.
- Once disconnected, `start_error` answers `None` (drift unknown) instead of raising, so
  the step's `start` fails cleanly with `disconnected`.
