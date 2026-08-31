# Using `agimus_spacelab` with BehaviorTree.CPP (ROS-free)

A third way to drive `agimus_spacelab`, alongside the plain-Python API
([`standalone-usage.md`](standalone-usage.md)) and the ROS 2 / DBT stack
([`dbt-integration.md`](dbt-integration.md)): a standalone C++ executable that compiles a
versioned task-plan IR into a BehaviorTree.CPP tree and drives HPP through an in-process
CPython bridge. No ROS, no network boundary, no `libDBT.so`. It targets the same eventual
consumer as the DBT layer — a mission executive — but keeps the whole call chain in one
process and one language boundary (C++ ↔ embedded CPython) instead of crossing ROS 2.

This is also the intended landing spot for a **future** VLM/LLM planner: the IR, capability
registry, validator, and compiler are the extension points a model would target. That
integration does not exist yet — model output would be untrusted proposal data, never
executed as generated code.

For full status, verification results, and the current real-mission gap, see
[`../report/behaviortree-screwdriving-report.md`](../report/behaviortree-screwdriving-report.md)
and [`../plans/behaviortree-screwdriving-taskplan.md`](../plans/behaviortree-screwdriving-taskplan.md)
— this page only covers how to build, run, and extend it.

## 1. The big picture

```
Hand-authored (or future model-proposed) TaskPlan IR
    │  CapabilityRegistry-checked validation (model.py)
    ▼
Deterministic IR-to-BT compiler (compiler.py) ──► BehaviorTree.CPP XML + source map
    │
    ▼
C++ standalone executable (examples/behaviortree/)
    │  BT::BehaviorTreeFactory + generic task-planning nodes
    ▼
Embedded CPython bridge (PythonSession)
    │  in-process call, no RPC
    ▼
Python TaskPlanningSession / HostSession (session.py, host.py)
    │
    ▼
SpaceLab screwdriving adapter (script/spacelab/screwdriving_plan.py, screwdriving_session.py) ──► agimus_spacelab / PyHPP
```

The same IR, validator, compiler, C++ nodes, and bridge run two adapters: a deterministic
non-SpaceLab `create_fake_session` (used for CTest conformance/fault-path coverage, no PyHPP)
and the real `create_screwdriving_session`. All SpaceLab-specific geometry, phase lists,
frozen-arm policy, and lookahead stay isolated in the example adapter — the generic layer
knows nothing about screws, robots, or grippers.

## 2. Component map

| File | Role |
|---|---|
| `src/agimus_spacelab/tasks/task_planning/model.py` | `TaskPlan` — schema/semantic validation, canonical JSON, SHA-256 `plan_fingerprint` binding the plan *and* the capability registry snapshot |
| `src/agimus_spacelab/tasks/task_planning/capabilities.py` | `CapabilityDescriptor` (policy: required params, `max_attempts`, `max_timeout`, `restartable`) and `CapabilityRegistry` (binds descriptors to trusted callables; `freeze()`s before execution) |
| `src/agimus_spacelab/tasks/task_planning/compiler.py` | `compile_behavior_tree()` — deterministic, allowlisted-element IR→XML compiler; output carries its own `artifact_fingerprint` |
| `src/agimus_spacelab/tasks/task_planning/session.py` | `TaskPlanningSession` — dispatches transactions/conditions through the frozen registry; freezes the registry on construction |
| `src/agimus_spacelab/tasks/task_planning/host.py` | Allowlisted session factories the C++ host is permitted to call: `create_fake_session`, `create_screwdriving_session` |
| `script/spacelab/screwdriving_plan.py` | The mission IR: transactions, capability descriptors (`create_screwdriving_registry`, `build_screwdriving_plan`) — kept out of `src/` so the generic layer stays SpaceLab-agnostic |
| `script/spacelab/screwdriving_session.py` | The SpaceLab adapter: `ScrewdrivingPlanningSession`, checkpointing, `PathRecorder` capture — loaded dynamically by `host.py` |
| `examples/behaviortree/` | C++ host: `main.cpp` (CLI + allowlist), `python_session.{hpp,cpp}` (CPython bridge), `task_nodes.{hpp,cpp}` (generic BT node types) |
| `script/spacelab/run_taskplan_bt_supervised.py` | Process supervisor: attempt/total timeouts, process-group kill, bounded restart backoff |
| `script/spacelab/replay_captured_paths.py` | Validates a `PathRecorder` capture (continuity, seam checks) without re-planning |

## 3. Task Plan IR contract

A plan is a JSON document: `schema_version` (must be `"1.0"`), `mission_id`, `scene`,
`provenance`, and a `root` node tree of `sequence` / `fallback` / `retry` / `condition` /
`operation` / `transaction` nodes. Every node id must match `^[A-Za-z][A-Za-z0-9_.-]{0,127}$`
and be unique across the whole tree.

- **`transaction`** — exactly one `operation` child; requires `restart_state` (a list of
  session-state keys it restarts from); the child's capability **must** be registered
  `restartable=True` or validation rejects the plan.
- **`retry`** — one child, `max_attempts` capped at the child's effective attempt budget
  (can't loosen a capability's own limit by wrapping it in `retry`).
- **`operation` / `condition`** — reference a `capability` id that must already be in the
  registry; `parameters` are type-checked against `descriptor.required_parameters`;
  `constraints.max_attempts` / `max_timeout` are capped at the descriptor's own limits, never
  raised.

`TaskPlan.from_dict(document, registry)` normalizes (NFC, sorted keys, finite numbers only),
validates, and returns a frozen `TaskPlan` whose `.document` property is a **defensive deep
copy** — callers can't mutate the validated IR in place. `plan_fingerprint` hashes the
normalized plan together with the registry's own snapshot, so the same plan JSON compiled
against a different capability registry produces a different fingerprint.

## 4. Compiler → BT XML mapping

`compile_behavior_tree()` wraps the whole tree in `SetupTaskPlan` / `FinalizeTaskPlan` and
maps IR nodes deterministically:

| IR node | Compiled BT shape |
|---|---|
| `transaction` | `Fallback[ TaskStepComplete, Sequence[ TaskStepReady, RetryUntilSuccessful(num_attempts=effective)[ ExecuteTaskStep ] ] ]` — already-complete short-circuits, not-ready fails without consuming a retry |
| `retry` | `RetryUntilSuccessful(num_attempts=effective)` wrapping the compiled child |
| `operation` | `ExecuteTaskStep` |
| `condition` | `TaskCapabilityCondition` |
| `sequence` / `fallback` | `Sequence` / `Fallback` |

Every compiled node keeps a `source_map` entry back to its IR path. The resulting
`CompiledBehaviorTree.artifact_fingerprint` is a SHA-256 over `{plan_fingerprint,
compiler_version, xml, source_map}` — the C++ side and any stored checkpoint can both assert
they're looking at the exact plan+compiler combination that produced a given run.

## 5. C++ host and the CPython bridge

`examples/behaviortree/src/main.cpp` takes `--factory <name>` (checked against a hardcoded
allowlist: `create_fake_session`, `create_screwdriving_session` — an unlisted name is a
non-retryable exit code `2`, never dispatched to Python) and `--options <json>`, constructs a
`PythonSession`, registers the five generic node types
(`RegisterTaskPlanningNodes`, `task_nodes.cpp`), builds the tree from
`session->call("get_behavior_tree_xml")`, and ticks it to completion with a `BT::TreeObserver`
attached.

**Threading**: every `PythonSession::call()` must run on the BT/interpreter thread. An
earlier version dispatched PyHPP calls from a `std::async` worker and crashed on the
cross-thread GIL/HPP-state violation; there is no thread pool in the current design.

**Exit codes** (also the contract `run_taskplan_bt_supervised.py` interprets):

| Code | Meaning | Retried by the supervisor? |
|---|---|---|
| `0` | All transactions committed | — |
| `1` | Deterministic BT `FAILURE` (a capability raised for real, not a crash) | No — checkpoint left as-is for inspection |
| `2` | Non-retryable configuration/contract error (unknown `--factory`, malformed bridge contract) | No |
| `124` | Total wall-clock timeout | Preserved; re-invoke to continue |
| `125` | Max restart count exceeded | Preserved |
| signal / attempt timeout | Native crash or a single attempt exceeding `--attempt-timeout` | Yes, exponential backoff, from the preserved checkpoint |

## 6. Building

```bash
cmake -S . -B build-bt \
  -DBUILD_BEHAVIORTREE_EXAMPLES=ON \
  -DBUILD_TESTING=ON
cmake --build build-bt --parallel --target agimus_taskplan_bt
```

`BUILD_BEHAVIORTREE_EXAMPLES` (default `OFF`) gates `add_subdirectory(examples/behaviortree)`
in the top-level `CMakeLists.txt`, so it never affects a normal library build.
`examples/behaviortree/CMakeLists.txt` fetches BehaviorTree.CPP via `FetchContent` pinned to a
known commit unless `BEHAVIORTREE_CPP_SOURCE_DIR` is already defined (vendored/cached
checkout), and always builds it with its own examples/tools/Groot/SQLite logging disabled.

## 7. Running

Bounded conformance tests (no real PyHPP; safe anywhere, including CI):

```bash
ctest --test-dir build-bt --output-on-failure \
  -R 'taskplan_bt_(fake|screwdriving_mock)'
```

Real PyHPP mission, supervised (resumes from whatever checkpoint already exists in
`--checkpoint-dir`):

```bash
python script/spacelab/run_taskplan_bt_supervised.py \
  --executable "$PWD/build-bt/examples/behaviortree/agimus_taskplan_bt" \
  --checkpoint-dir /tmp/agimus_bt_full_checkpoint \
  --capture-dir /tmp/agimus_bt_full_paths \
  --attempt-timeout 600 \
  --total-timeout 3600
```

Validate a capture without re-planning:

```bash
python script/spacelab/replay_captured_paths.py /tmp/agimus_bt_full_paths --check
```

Run the host directly (bypasses the supervisor — useful for a single attempt or debugging a
specific `--options` payload):

```bash
./build-bt/examples/behaviortree/agimus_taskplan_bt \
  --factory create_screwdriving_session \
  --options '{"backend":"pyhpp","no_viz":true,"checkpoint_dir":"/tmp/agimus_bt_full_checkpoint"}'
```

`create_screwdriving_session` options: `backend` (default `pyhpp`), `no_viz` (default
`true`), `log_level` (default `INFO`), `mock_motion` (skip real PyHPP planning — what the
`taskplan_bt_screwdriving_mock` CTest uses), `checkpoint_dir`, `capture_dir`,
`migrate_legacy_checkpoint`.

## 8. Checkpointing and path capture

`checkpoint_dir` enables atomic checkpoint replacement to
`<checkpoint_dir>/taskplan_checkpoint.json`, validated against the compiled artifact's
fingerprint on load — a checkpoint written by a different plan/compiler combination is
rejected rather than silently resumed against. `capture_dir` enables `PathRecorder`, which
writes a `manifest.json` plus one `seg_*.json` per captured path segment; this is what
`replay_captured_paths.py --check` re-validates (continuity, quaternion-aware seam checks)
without touching HPP at all.

## 9. Adding a capability or a new mission

1. Write a `CapabilityDescriptor` (id, version, `required_parameters`, `effects`,
   `max_attempts`, `max_timeout`, and `restartable=True` if it will back a `transaction`) and
   a plain callable `dict -> dict` implementation.
2. `registry.register(descriptor, implementation)` into a fresh `CapabilityRegistry` — do
   this before constructing any `TaskPlanningSession`/`HostSession`; the registry freezes
   (`RuntimeError` on further `register`/`bind`) the moment a session is constructed.
3. Author the mission as a `TaskPlan` document (see
   `script/spacelab/screwdriving_plan.py` for the reference SpaceLab example) and validate it
   with `TaskPlan.from_dict(document, registry)`.
4. Expose it through a new factory function in `host.py`, then add that factory's name to the
   `allowed_factories` set in `examples/behaviortree/src/main.cpp` — a factory not on both
   allowlists (Python import path *and* C++ set) can never be reached from the host.

## 10. Relationship to the DBT/ROS integration

Both eventually call the same `agimus_spacelab` planning primitives, but the boundaries and
guarantees differ:

| | BehaviorTree.CPP (this doc) | DBT / ROS 2 |
|---|---|---|
| Process boundary | None — one process, embedded CPython | ROS 2 services/actions across processes |
| Mission format | Versioned, validated JSON IR compiled deterministically to BT XML | Hand-authored Petri-net `<Mission>` XML wired to C++ callbacks |
| Capability contract | Explicit `CapabilityDescriptor` registry, frozen before execution | Implicit — whatever the ROS service/action schema exposes |
| Checkpointing | Built-in atomic checkpoint + fingerprint validation | None at this layer (`resume_sequence` is in-memory, process-lifetime only) |
| Physical execution | None — plans/advances simulated HPP configuration only | Drives Gazebo/`ros2_control` through the mock hardware stack |
| Model/LLM extension point | Yes (deferred) — IR/registry/validator are the intended target | Not designed for this |

If you need physical robot execution today, that's the DBT/ROS 2 stack
([`dbt-integration.md`](dbt-integration.md)), not this one.
