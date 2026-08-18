# Using `agimus_spacelab` as a standalone library

How to write, run, resume, and replay a manipulation-planning task with `agimus_spacelab`
directly — no ROS 2, no DBT. For the ROS 2 / DBT integration, see
[`dbt-integration.md`](dbt-integration.md).

This doc describes the **current** API (branch `main`, commit `e1844c9`). The package was
heavily refactored in 2026-08; if you find older examples elsewhere in the repo that
contradict this doc, trust this doc and the root [`README.md`](../../README.md) /
[`ARCHITECTURE.md`](../../ARCHITECTURE.md).

> **Stale references to ignore.** `src/agimus_spacelab/__init__.py`'s docstring mentions
> `TaskOrchestrator`, `TaskBuilder`, `PlanningBridge` — these classes don't exist.
> `script/spacelab/README.md` still describes CORBA as the default backend and lists files
> that no longer exist. Use *this* doc, the root README, and `script/templates/README.md`
> instead.

---

## 1. What it is

`agimus_spacelab` is a pure-Python planning library (with C++ HPP bindings underneath) that
turns a high-level assembly goal — *grip this, move that, hand it over, place it* — into one
concatenated, collision-free motion for a multi-robot, multi-object scene. Its headline
feature is **linear-cost sequence planning**: `GraspSequencePlanner` chains N grasp/place
phases into one plan, each phase getting a *minimal* phase-local constraint graph, so cost
grows **O(N)** instead of **O(N!)**.

Nothing under `src/agimus_spacelab/` imports `rclpy` — this library has no ROS 2 dependency.
`ros2_ws_agimusxads` is a downstream *consumer* (see [`dbt-integration.md`](dbt-integration.md)).

## 2. Layered architecture

```
script/            example / entry-point tasks (what you write)
   │
tasks/             ManipulationTask, GraspSequencePlanner, InteractiveGraspSequenceBuilder
   │
planning/          SceneBuilder, ConstraintBuilder, GraphBuilder, ConfigGenerator,
   │                GraspStateTracker, SequentialConstraintGraphFactory/GraspFilter, path_io
   │
backends/          BackendBase (ABC) → PyHPPBackend (default) | CorbaBackend (deprecated)
```
Horizontal, used from any layer: `config/` (YAML + dataclass config), `logging/` (RunLogger),
`visualization/` (viser/gepetto), `utils/`. Only `backends/` imports `pyhpp.*` /
`hpp.corbaserver.*` — everything above it is backend-agnostic.

## 3. Install (summary)

Full instructions with troubleshooting are in the root [`README.md`](../../README.md). Short
version:

1. **HPP native bindings first** (not on PyPI). The default PyHPP backend needs the
   **source-built** HPP — the `hpp-agimus` Docker container (see
   `/home/dvtnguyen/devel/CLAUDE.md`) — because the stable robotpkg `hpp-python` 6.1.0 binary
   is missing symbols this library needs (`RSTimeParameterization`, `GraphRandomShortcut`,
   `SplineGradientBased_bezier{1,3,5}`, …). Inside the container:
   ```bash
   docker exec -it hpp-agimus bash
   source ~/devel/ros2_ws_agimusxads/scripts/source.sh
   ```
2. **Then the package**: `cd hpp/src/agimus_spacelab && pip install -e .` (or the CMake path:
   `cd build && make install` — required after every Python change if you built via CMake).
3. Verify: `python -c "from agimus_spacelab import get_available_backends; print(get_available_backends())"`.

## 4. Core concepts

| Concept | Class | Role |
|---|---|---|
| Scene | `planning.scene.SceneBuilder` | loads robot/environment/object models, sets joint bounds |
| Constraints | `planning.constraints.ConstraintBuilder` | grasp / placement / locked-joint constraints |
| Graph | `planning.graph.GraphBuilder` | builds the HPP constraint graph (states + edges) |
| Config generation | `planning.config.ConfigGenerator` | samples/solves robot configurations |
| Backend | `backends.PyHPPBackend` / `CorbaBackend` | the actual HPP solver, behind a uniform `BackendBase` interface |
| Task | `tasks.ManipulationTask` | glues the four above together for one task |
| Sequence planner | `tasks.grasp_sequence.GraspSequencePlanner` | plans a *sequence* of grasp/release phases as one linear-cost plan |

You will almost always work at the **Task** / **GraspSequencePlanner** level; the layers below
are what those two orchestrate.

## 5. Quickstart — minimal task

Two ways to define a task; **YAML is recommended** for all new work.

### 5a. Copy a template

```
script/templates/task_config_template.yaml  → script/config/<robot>_config.yaml
script/templates/task_my_task.py            → script/<robot>/task_<name>.py
```

In `task_my_task.py`, edit five `# <-- EDIT` sections:

```python
TASK_NAME               = "My Robot: Pick and Place"
_YAML_PATH              = Path(...) / "config" / "my_robot_config.yaml"
GRASP_GOALS              = ["my_robot/gripper grasps my_object/handle"]
GRASP_SEQUENCE           = [("my_robot/gripper", "my_object/handle")]
FREEZE_JOINT_SUBSTRINGS  = []
COLLISION_EXCLUSIONS     = []
```

In the YAML, replace every `<PLACEHOLDER>` (robot name, package, environment name, object
names, gripper/handle frame names). Not sure of frame names? Run with `--show-joints` first.

Run it:
```bash
python script/<robot>/task_<name>.py --show-joints     # discover joint/frame names
python script/<robot>/task_<name>.py --backend pyhpp
```

### 5b. Minimal working reference

`script/spacelab/task_grasp_FG_yaml.py` is a real, runnable example for the multi-arm
SpaceLab scene (UR10 grasps `frame_gripper`, VISPA/VISPA2 frozen) — read it before writing
a new task from the template.

### 5c. What a task looks like, conceptually

```python
class MyTask(ManipulationTask):
    def get_objects(self): ...            # what's in the scene
    def create_constraints(self): ...      # grasp / placement constraints
    def create_graph(self): ...            # constraint graph (skip if using GraspSequencePlanner)
    def build_initial_config(self): ...
    def generate_configurations(self, q_init): ...

task = MyTask(backend="pyhpp", log_dir="auto", log_level="INFO")
task.setup(skip_graph=True)   # skip_graph=True if a GraspSequencePlanner will build phase graphs
task.run(visualize=True, solve=False)
```
`ManipulationTask.__init__` also takes `joint_bounds`, `FILE_PATHS`, `task_name`, `viewer_type`.
`setup()` additionally accepts `validation_step`, `projector_step`, `freeze_joint_substrings`.
`run()` additionally accepts `preferred_configs`, `max_iterations`, `solve_mode`,
`transition_edges`, `record`, `output_dir`, `video_name`, `framerate`.

## 6. Multi-phase grasp sequences (`GraspSequencePlanner`)

This is the workhorse for anything beyond a single grasp.

```python
planner = GraspSequencePlanner(
    graph_builder, config_gen, planner, task_config,
    backend="pyhpp", run_logger=logger,
)
result = planner.plan_sequence(
    grasp_sequence=[("ur10/gripper", "RS1/handle"), ("ur10/gripper", None), ...],  # (gripper, None) = explicit release
    q_init=q0,
    frozen_arms_mode="auto",       # or "manual" + per_phase_frozen_arms
    timeout_per_edge=60.0,
    phase_q_hints=None,            # warm-start hints, see §7
)
# result = {"success", "paths", "phase_results", "final_config", "grasp_tracker"}
```

Internally, each phase calls `GraphBuilder.build_phase_graph()` to build a **minimal** graph
(exactly 2 states + waypoints per phase by default, via `SequentialGraspFilter`) instead of
the full combinatorial graph, then solves just that edge.

> **Phase-local index invariant.** `build_phase_graph()` must always be followed by
> `grasp_tracker.set_phase_indices(phase_grippers, phase_handles)` — this keeps edge-name
> generation in sync with the reduced phase graph. `GraspSequencePlanner` does this for you
> internally; only worry about it if you're calling `build_phase_graph()` yourself in a custom
> script (a past bug from a missing call is documented in
> [`docs/features/transit-edge-robustness.md`](../features/transit-edge-robustness.md)).

## 7. Phase-target lookahead (warm-starting)

`find_feasible_phase_target(phase_n, phase_n1, q_current, q_scene_init, frozen_arms_n,
frozen_arms_n1)` probes — on a throwaway copy of the grasp tracker, never mutating the real
one — that phase N+1 stays reachable *before* committing phase N's randomized target. This
avoids the "2000+ failed random draws" pathology seen on tightly-constrained targets (e.g.
screw holes). The whole per-edge config chain it finds is fed back as
`phase_q_hints={phase_idx: [q0, q1, ...]}` into `plan_sequence()`/`resume_sequence()` to warm-
start config generation. See
[`docs/features/phase-target-lookahead.md`](../features/phase-target-lookahead.md) for the
full mechanism and measured costs.

## 8. Resuming a failed sequence

Two independent mechanisms — don't confuse them.

### 8a. In-process resume (same Python session)

```python
if planner.get_resumable_state():         # None if nothing to resume
    result = planner.resume_sequence(retry_from_edge=0, phase_q_hints=hints)
```
`resume_sequence()` restarts the **failed phase from where the call began** (`_q_call_start`),
not from wherever the failed attempt physically stopped — a search must never leave the robot
mid-motion. `phase_q_hints` here is keyed by absolute index into the *original* sequence.

If a warm-start hint chain breaks mid-block (a collision retry redrew a config), don't resume
forward on a broken guarantee — call `planner.reset_grasp_tracker_to_call_start()` and replan
the block from its entry instead.

This mechanism only survives within one `GraspSequencePlanner` instance / one process — it is
**not** how you recover from a killed process. For that, see §9.

### 8b. Cross-process checkpoints (diagnostic re-runs)

Long scripts (e.g. `test_screwdriving_sequence.py`) dump `(q_current, held_grasps)` to
`$AGIMUS_CHECKPOINT_DIR/phase_{NN:02d}.json`. `script/spacelab/repro_phase_range.py` loads one
and calls `plan_sequence()` directly on a phase sub-range — turns a 20+ minute re-run into
seconds when debugging one failing phase:
```bash
python repro_phase_range.py --start 5 --end 7 --checkpoint-dir /tmp/checkpoints --backend pyhpp
```

## 9. Capturing and replaying a whole run

The newest and most durable mechanism — samples every planned/executed path to disk as it
happens, so a run can be replayed **without rebuilding any constraint graph**, in a separate
process, after the original run exited (crashed, was killed, or finished normally).

```python
from agimus_spacelab.planning.path_recorder import PathRecorder

recorder = PathRecorder(output_dir, planner=backend, dt=0.05)
recorder.begin_step(step_idx, "grasp RS1")
recorder.record_path(path_or_stored_id, kind="grasp", edge_name="ur10>RS1")
recorder.record_phase_results(result["phase_results"])   # call after every plan_sequence()/resume_sequence()
recorder.close()
```
Writes `manifest.json` (atomically rewritten after every segment — crash-safe) plus
`seg_{index:04d}_{kind}[_{edge}].json` waypoint files. `mark()`/`rollback(mark)` drop segments
from an abandoned block replan (motion that never actually happened).

Replay from the CLI, no scene/HPP session needed for a pure continuity check:
```bash
python script/spacelab/replay_captured_paths.py /tmp/run/paths --check     # continuity only, seconds
python script/spacelab/replay_captured_paths.py /tmp/run/paths             # visual replay (viser/gepetto)
python script/spacelab/replay_captured_paths.py /tmp/run/paths --only RS1 --speed 2
```
Or in Python:
```python
from agimus_spacelab.planning.path_replay import load_manifest, validate
m = load_manifest(directory)
report = validate(m)          # independently re-derives seam continuity from the files on disk
assert report.ok
```
To play a whole captured run as one continuous motion in-process, join already-stored per-edge
paths with `PyHPPBackend.concatenate_paths(path_ids)` — note this does **not** check continuity
itself and does not re-optimize velocity across the joins (each sub-path was time-parameterized
independently, so velocity is zero at each junction). Feed it seam-checked ids from a manifest.

## 10. Run logging (structured JSONL, separate from path capture)

`RunLogger` is wired automatically (`ManipulationTask(log_dir="auto")` → default
`/tmp/agimus_spacelab/<task_slug>_<timestamp>/`) and emits events (`run_start`,
`config_snapshot`, `sequence_start`, `phase_start`, `edge_start`, `edge_end`, `phase_end`,
`run_end`) during `plan_sequence()`/`resume_sequence()`. Inspect afterward:
```python
from agimus_spacelab.logging import print_run_summary, load_run_log, get_replay_config
print_run_summary("/tmp/agimus_spacelab/.../run_....jsonl")
```
`get_replay_config()` returns `{backend, task_name, task_config, setup_params, sequence}` —
enough to reproduce a run without re-deriving it by hand. This is a log of *what happened*, not
motion data — pair it with §9's path capture for full auditability.

## 11. Backends

| | `PyHPPBackend` (default) | `CorbaBackend` (deprecated) |
|---|---|---|
| Process | in-process bindings, no server | external `hppcorbaserver` (port 13331) |
| Select | `backend="pyhpp"` | `backend="corba"` |
| Status | actively developed, default everywhere | frozen; kept for compatibility only |

Both implement the same `BackendBase` contract (`load_robot/environment/object`,
`create_state/edge`, `solve()`, `get_path()`, `play_path()`, constraint factories) — task code
should never need backend-specific branches. `create_planner(backend=...)` /
`check_backend(backend)` in `planning/planner.py` are the usual entry points.

## 12. Example scripts (read these before writing your own)

| Script | Use it to learn |
|---|---|
| `script/spacelab/task_grasp_FG_yaml.py` | one grasp, multi-arm scene, YAML config |
| `script/spacelab/interactive_planning.py -i` | menu-driven exploration: enumerate feasible goals from `VALID_PAIRS`, solve interactively |
| `script/spacelab/test_full_sequence.py` | full 13-phase assembly, non-stop auto-resume-on-failure mode |
| `script/spacelab/test_screwdriving_sequence.py` | everything together: multi-phase sequence + lookahead hints + path capture + end-of-run replay menu (2300+ lines — the flagship reference) |
| `script/spacelab/repro_phase_range.py` | fast checkpoint-based re-run of one phase range |
| `script/spacelab/replay_captured_paths.py` | CLI replay of a `PathRecorder` manifest |
| `script/spacelab/benchmark_optimizer_phases.py` | compare path-quality across optimizer settings |

## 13. The screwdriving pattern (SpaceLab-specific, not a framework feature)

Nothing in `src/agimus_spacelab/` knows about "screws" or "holes" — this is entirely a
**script-level pattern** in `test_screwdriving_sequence.py`, riding on `GraspSequencePlanner`
+ lookahead + `PathRecorder`. Vocabulary if you're reading that script:
- **Interfaces**: adjacent RS-part pairs screwed together once both are placed
  (RS1–RS6, RS6–RS5, RS2–RS1, RS2–RS3, RS3–RS4).
- **Holes**: handle names `CON0..CON3` per part.
- `SCREW_PLAN_RS`: a hand-written table mapping each of the six placement turns to
  `(target_part, (hole_names...))`.
- Each turn is split into an **A-block** (`A0`: frame-gripper grasp alone) and **A-REST**
  (workbench grasp + grasp/release pairs per hole) — the lookahead's workbench-grasp probe
  needs to run from the part's real in-place pose, not its staging pose.

See [`docs/features/screwdriving-sequence-assembly-order.md`](../features/screwdriving-sequence-assembly-order.md)
if you're extending this pattern to a new assembly order.

## 14. Config format reference

YAML top-level keys: `robots`, `environments`, `joint_groups`, `objects` (handles, contact
surfaces, initial pose), `grippers`, `valid_pairs` (gripper → allowed handles — this is what
drives grasp legality), `arm_groups`, `freeze_joints`, `environment_contacts`, `paths`
(URDF/SRDF locations), `planning` (validation/projector steps, iteration limits),
`optimization` (shortcut loops, TOPPRA params), `freeflyer_bounds`. Loaded via
`config.yaml_loader.YamlTaskLoader(yaml_path)`, exposing `.file_paths`, `.joint_bounds_class`,
`.task_config` (dynamic config object), `.build_initial_config()`. Real example:
`script/spacelab/config/spacelab_config.yaml` (complex, multi-arm). Template:
`script/templates/task_config_template.yaml`.

The older dataclass style (`config/base_config.py` → `BaseTaskConfig`) still works but isn't
recommended for new tasks — YAML needs no Python changes to add a robot/object.

## 15. Running tests

```bash
cd build && make install     # if you edited src/ and built via CMake
python -m pytest tests/ -v   # 118 pass, 0 fail, 16 skip (baseline) — inside the hpp-agimus container
```
Most illustrative for learning the API by example:
- `tests/test_grasp_sequence_resume_state.py` — exact resume-state-restoration contract (§8a).
- `tests/test_path_replay.py` — `PathRecorder` → `load_manifest` → `validate` round-trip (§9).
- `tests/test_replay_menu.py` — the end-of-run replay-menu keybindings (`<index>`, `s<N>`, `a`).
- `tests/test_grasp_state_copy.py` — the tracker-copy isolation invariant behind lookahead (§7).

## 16. Known gotchas

- **Missing `set_phase_indices()` sync** after a manual `build_phase_graph()` call silently
  falls back to global gripper/handle indices → wrong edge names. See §6.
- **`ConstraintGraphFactory` reference cycle**: each `build_phase_graph()` call leaves ~235MB
  of unreachable C++ graph memory until a gen-2 GC runs, unless the internal
  `_break_factory_cycle()` step runs — already handled inside `GraphBuilder`, only relevant if
  you bypass it.
- **`concatenate_paths()` doesn't validate continuity** — pass only ids known to join (e.g. a
  seam-checked manifest's path ids), not arbitrary ones.
- **CORBA backend deprecation warning** is expected and intentional — don't "fix" it by
  suppressing it; migrate to `pyhpp` instead.
