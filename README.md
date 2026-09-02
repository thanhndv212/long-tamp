# Long-TAMP - Long-Horizon Task-and-Motion Planning

Long-horizon, multi-arm task-and-motion planning (TAMP) for manipulation, built on HPP (Humanoid Path Planner).

`long_tamp` plans **long, multi-step manipulation sequences** for **several robot arms** on **many movable objects** in a shared scene. **Motion planning** is HPP's constraint-graph planner, driven through a plain-Python task API. **Task planning** is a declarative task plan compiled to **BehaviorTree.CPP**, for standalone, ROS-free mission execution (see *Task Planning* below).

## Capabilities

- **Long-horizon sequence planning.** `GraspSequencePlanner` chains an arbitrary number of grasp/place/hand-over phases, each with its own minimal phase-local constraint graph, so planning cost grows **linearly (O(N))** with the number of grasps instead of combinatorially (**O(N!)**).
- **Multiple robots & objects.** The scene composes several arms into one shared, mutually-collision-aware planning model — able to act independently, cooperate, or hand off objects — alongside any number of free-flying objects/tools, with grasp legality data-driven via `VALID_PAIRS` so adding one is a config change.
- **Reproducibility, introspection & crash recovery.** Crash-safe JSONL run logging plus a path-capture mechanism (`PathRecorder`) record every phase/edge/path so a run can be replayed, continuity-checked, and checkpointed/resumed rather than replanned from scratch.
- **Declarative task planning → BehaviorTree.CPP.** A versioned, capability-checked task plan (`tasks/task_planning/`) compiles deterministically to a BehaviorTree.CPP tree, run by a standalone C++ host through an embedded-CPython bridge with no ROS and no network hop (see *Task Planning* below).
- **Scene visualization.** Interactive 3D viewers: browser-based **viser** (default, no X11) or **gepetto-viewer** (Qt).

## Installation

`long_tamp` has two dependency tiers: a pure-Python tier installable from PyPI, and the HPP
native bindings (`hpp-python`, `hpp-toppra`, `hpp-gepetto-viewer`), which are C++ extension
modules **not on PyPI** and must come from robotpkg, conda-forge, or a source build/container.
Install the native bindings first, then the package:

```bash
# Step 1: HPP native bindings — robotpkg or the hpp-agimus source-built container
# (the default PyHPP backend needs the source build today; see docs/INSTALL.md)

# Step 2: the long_tamp package
pip install -e .
```

Full instructions — robotpkg vs. source build, the CMake install path, optional extras
(`toppra`), the NumPy/pinocchio ABI pitfall, and runtime backend detection — are in
[`docs/INSTALL.md`](docs/INSTALL.md).

## Usage

Writing a task means implementing `ManipulationTask`'s lifecycle contract (`get_objects()`,
`create_constraints()`, `create_graph()`, `build_initial_config()`,
`generate_configurations()`, then `setup()` / `run()`) — either by hand, or, for new tasks,
via a declarative YAML config (recommended). Full, runnable examples live in
[`docs/usage/standalone-usage.md`](docs/usage/standalone-usage.md) §§4–6 rather than
duplicated here, alongside multi-phase sequences, resume/replay/checkpoints, and backend
selection.

- **Start from a template**: `script/templates/task_config_template.yaml` +
  `task_my_task.py` — copy, fill in the `<PLACEHOLDER>`s, run.
- **Read a real, minimal example**: `script/twin/task_lift_ball.py` (bimanual scene).

## Package structure & architecture

`tasks/` orchestrates `planning/`, which is backend-agnostic and depends only on `backends/`
(the one place HPP-specific bindings are imported); `config/`, `logging/`, `visualization/`,
`utils/`, and `cli/` are horizontal support layers used from `tasks/` and `script/`.

```mermaid
flowchart TB
    script["script/<br/>end-user task scripts<br/>(one per robot/mission)"]
    tasks["tasks/<br/>ManipulationTask, GraspSequencePlanner,<br/>InteractiveGraspSequenceBuilder"]
    planning["planning/<br/>SceneBuilder, ConstraintBuilder, GraphBuilder,<br/>ConfigGenerator, GraspStateTracker,<br/>SequentialConstraintGraphFactory,<br/>SequentialGraspFilter, path_io,<br/>path_recorder, path_replay"]
    backends["backends/<br/>BackendBase (ABC) → PyHPPBackend<br/>only layer importing pyhpp.*"]

    config["config/<br/>BaseTaskConfig, YamlTaskLoader"]
    logging_["logging/<br/>RunLogger, JSONL event schema"]
    viz["visualization/<br/>graph diagrams, frame display, video"]
    utils["utils/<br/>transforms, interactive menus"]
    cli["cli/<br/>argparse helpers, interactive pickers"]

    script --> tasks
    tasks --> planning
    planning --> backends

    tasks -.uses.-> config
    tasks -.uses.-> logging_
    tasks -.uses.-> viz
    script -.uses.-> cli
    cli -.uses.-> utils
    config -.uses.-> utils

    style backends fill:#4c566a,stroke:#2e3440,color:#fff
    style planning fill:#5e81ac,stroke:#2e3440,color:#fff
    style tasks fill:#81a1c1,stroke:#2e3440,color:#fff
    style script fill:#88c0d0,stroke:#2e3440,color:#000
```

Per-phase planning data flow, the loop every mission ultimately runs through:

```mermaid
flowchart TD
    yaml["YAML config"] -->|YamlTaskLoader| loaded["file_paths, joint_bounds_class, task_config"]
    loaded --> setup["ManipulationTask.setup"]

    setup --> scene["SceneBuilder<br/>load robots, env, objects"]
    setup --> constraints["ConstraintBuilder /<br/>FactoryConstraintRegistry"]
    setup --> gbuild["GraphBuilder<br/>factory or manual"]

    scene --> plan["GraspSequencePlanner.plan_sequence"]
    constraints --> plan
    gbuild --> plan

    plan --> p1["1. build_phase_graph<br/>GraphBuilder plus SequentialConstraintGraphFactory"]
    p1 --> p2["2. GraspStateTracker picks the edge name"]
    p2 --> p3["3. ConfigGenerator.generate_via_edge builds target config"]
    p3 --> p4["4. backend.solve builds the path, then optimize and time-parameterize"]
    p4 --> p5["5. RunLogger.log phase_end, optional auto-save of path"]
    p5 -->|next phase| p1
    p5 --> result["concatenated multi-phase path, O of N planning cost"]
```

Both diagrams are copied from **[`ARCHITECTURE.md`](ARCHITECTURE.md)**, which is the
maintained source — it's dated at the top and covers dependency direction and what each
class does in more depth than fits here. If the two ever disagree, trust `ARCHITECTURE.md`
and update this copy to match.

## Task Planning (BehaviorTree.CPP)

Alongside the plain-Python `ManipulationTask` API, `tasks/task_planning/` is a second,
declarative entry point: a versioned JSON **TaskPlan IR** (`sequence` / `fallback` / `retry` /
`condition` / `operation` / `transaction` nodes), validated against a `CapabilityRegistry`,
compiles deterministically to a **BehaviorTree.CPP** tree. A standalone C++ host
(`examples/behaviortree/`) runs that tree via an in-process, embedded-CPython bridge — one
process, no ROS, no network hop. It ships with built-in checkpoint/resume and path-capture
validation for long, restartable missions, and is the intended integration point for a future
model-proposed (VLM/LLM) plan.

| Stage | File |
|-------|------|
| IR validation & fingerprinting | `tasks/task_planning/model.py` (`TaskPlan`) |
| Capability policy | `tasks/task_planning/capabilities.py` (`CapabilityRegistry`) |
| IR → BT XML compiler | `tasks/task_planning/compiler.py` |
| C++ host + CPython bridge | `examples/behaviortree/` |

Build with `-DBUILD_BEHAVIORTREE_EXAMPLES=ON`. Full IR schema, the compiler's node mapping,
build/run steps, checkpointing, and how to add a mission or capability:
[`docs/usage/behaviortree-integration.md`](docs/usage/behaviortree-integration.md).

## Run Logging

`long_tamp` includes a structured run logger that writes a crash-safe JSONL event
stream for every planning run — one event per phase/edge attempt, plus a JSON snapshot and a
replay-ready YAML on close. Use it to replay configurations, debug failures, and audit
results.

Logging is **on by default** for every `ManipulationTask` (`log_dir="auto"` creates
`/tmp/long_tamp/<task_slug>_<timestamp>/`; pass an explicit path to redirect it, or
`None` to disable). `RunLogger` also works standalone, independent of `ManipulationTask`.

| Event | When emitted |
|-------|-------------|
| `run_start` | `ManipulationTask.__init__` (with `log_dir`) |
| `config_snapshot` | `setup()` — full `BaseTaskConfig` + setup params |
| `sequence_start` | Start of `plan_sequence()` — all call params + `q_init` |
| `phase_start` | Before each grasp phase — `gripper`, `handle`, `q_start` |
| `edge_start` | Before each transition edge attempt |
| `edge_end` | After each edge — `success`, timing, `q_to` or `error` |
| `phase_end` | After each phase — timing, `state_after`, saved files |
| `run_end` | On normal return or `KeyboardInterrupt` |

For runnable examples — standalone use, inspecting a log afterward
(`print_run_summary`/`load_run_log`/`get_replay_config`), and configuring the underlying
Python `logging` hierarchy — see [`docs/usage/standalone-usage.md`](docs/usage/standalone-usage.md) §10.

## Documentation

- **Installation**: [`docs/INSTALL.md`](docs/INSTALL.md) — robotpkg vs. source build, pip/CMake install paths, optional extras, backend detection.
- **Architecture**: [`ARCHITECTURE.md`](ARCHITECTURE.md) — module layering, dependency direction, data flow. Dated at the top; check it before trusting a claim about what exists.
- **Usage guide (living reference)**: [`docs/usage/standalone-usage.md`](docs/usage/standalone-usage.md) — writing a task, multi-phase sequences, resume/replay/checkpoints, backends, example scripts.
- **Development report**: [`docs/legacy/report/development-report.md`](docs/legacy/report/development-report.md) — *why* the framework is built this way: architecture decisions vs. bare HPP, measured before/after numbers, project timeline, and a bugs-found appendix. A point-in-time report, not a living reference.
- **Design rationale for specific mechanisms**: [`docs/features/`](docs/features/); **upstream HPP defects worked around here**: [`docs/bugs/`](docs/bugs/).
- **API Reference**: See docstrings in source files.
- **ROS-free BehaviorTree.CPP integration**: [`docs/usage/behaviortree-integration.md`](docs/usage/behaviortree-integration.md).

## License

MIT - See [LICENSE](LICENSE) file


---

**Last Updated**: 2026-09-02
