# Agimus Spacelab - Manipulation Planning Framework

Multi-arm collaborative manipulation planning for SpaceLab assembly tasks using HPP (Humanoid Path Planner).

## Features

- **Modular architecture**: Reusable tools for scene setup, constraints, and configuration
- **Scene visualization**: Interactive 3D viewer with gepetto-viewer
- **Dual backend support**: CORBA (hpp-manipulation-corba) and PyHPP (hpp-python)
- **Task examples**: Grasp, assembly, and collaborative manipulation tasks


## Quick Start

```bash
cd script/spacelab
./task_display_states.py -i
```

### Installation

```bash
# Via pip (editable mode)
pip install -e .

# Or via CMake in HPP workspace
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR
make install
```

## Usage

### Create a Manipulation Task

```python
from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import SceneBuilder

class MyTask(ManipulationTask):
    def get_objects(self):
        return ["frame_gripper"]
        
    def create_constraints(self):
        # Define grasp/placement constraints
        pass
        
    def create_graph(self):
        # Build constraint graph
        pass
        
    def generate_configurations(self, q_init):
        # Generate waypoint configs
        pass

# Run task
task = MyTask()
task.setup()
task.run(visualize=True, solve=False)
```



## Package Structure

The package is organized into logical modules:

```
src/agimus_spacelab/
├── __init__.py                  # Main exports
├── backends/                    # Backend implementations
│   ├── __init__.py
│   ├── base.py                  # Backend base class
│   ├── corba.py                 # CORBA backend (hpp-manipulation-corba)
│   └── pyhpp.py                 # PyHPP backend (hpp-python)
├── planning/                    # Planning tools
│   ├── __init__.py
│   ├── planner.py               # create_planner() factory function
│   ├── scene.py                 # SceneBuilder
│   ├── constraints.py           # ConstraintBuilder
│   ├── graph.py                 # GraphBuilder
│   └── config_generator.py      # ConfigGenerator
├── tasks/                       # Task management
│   ├── __init__.py
│   ├── base.py                  # ManipulationTask base class
│   └── grasp_sequence.py       # GraspSequencePlanner
├── visualization/               # Visualization tools
│   ├── __init__.py
│   └── viz.py                   # Graph visualization, frame display
├── config/                      # Configuration classes
│   ├── __init__.py
│   └── rules.py                 # RuleGenerator, SpaceLabScenario
└── utils/                       # Utilities
    ├── __init__.py
    └── transforms.py            # Transform helpers (xyzrpy_to_se3, etc.)
```

## Architecture

Multi-layer design for scalable manipulation planning:

```
Assembly Mission
      ↓
Behavior Tree Layer (planned by external BT planner)
      ↓
Task Orchestration Layer (planned by external BT planner)
      ↓
Atomic Task Layer (implemented)
      ↓
Motion Planning Layer (HPP)
```

**Key Components:**
- `SceneBuilder`: Fluent API for scene setup
- `ConstraintBuilder`: Helper for constraint creation
- `ConfigGenerator`: Waypoint generation
- `ManipulationTask`: Base class for tasks
- `GraspSequencePlanner`: Multi-phase grasp sequence planning
- `create_planner()`: Factory for backend-specific planners

## Run Logging

`agimus_spacelab` includes a structured run logger that writes a crash-safe JSONL event stream for every planning run. Use it to replay configurations, debug failures, and audit results.

### Enable via `ManipulationTask`

Logging is **on by default**. `log_dir` defaults to `"auto"`, which creates a directory under `/tmp/agimus_spacelab/<task_slug>_<YYYYMMDD_HHMMSS>/`. Pass an explicit path to override, or `None` to disable.

```python
# Default: auto-creates /tmp/agimus_spacelab/my_task_20260415_143022/
task = MyTask(backend="pyhpp")

# Custom directory
task = MyTask(backend="pyhpp", log_dir="/data/runs/experiment_01")

# Disable logging
task = MyTask(backend="pyhpp", log_dir=None)

task.setup()
task.run()
# Writes: <log_dir>/run_20260415_143022_<id>.jsonl
#         <log_dir>/run_20260415_143022_<id>.json     (snapshot on close)
#         <log_dir>/run_20260415_143022_<id>_replay.yaml
```

### Enable standalone

```python
from agimus_spacelab.logging import RunLogger

logger = RunLogger("/tmp/runs")
planner = GraspSequencePlanner(..., run_logger=logger)
planner.plan_sequence(q_init, ...)
```

### Inspect logs after a run

```python
from agimus_spacelab.logging import print_run_summary, load_run_log, get_replay_config

# Human-readable summary to stdout
print_run_summary("/tmp/runs/run_20260415_143022_abc12345.jsonl")

# Structured dict: run_id, events, phase_results, one key per event type
data = load_run_log("/tmp/runs/run_20260415_143022_abc12345.jsonl")

# Reproduce the run: returns backend, task_name, task_config, setup_params, sequence
cfg = get_replay_config("/tmp/runs/run_20260415_143022_abc12345.jsonl")
```

### Configure Python logging

```python
from agimus_spacelab.logging import configure_logging

# Console + file handler under the "agimus_spacelab" logger hierarchy
configure_logging(level="DEBUG", log_dir="/tmp/runs", console=True)
```

### Event types

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

### Package location

```
src/agimus_spacelab/
└── logging/
    ├── __init__.py       # Public API: RunLogger, configure_logging, get_logger,
    │                     #   load_run_log, iter_events, get_replay_config,
    │                     #   print_run_summary
    ├── run_logger.py     # RunLogger — crash-safe JSONL writer
    ├── schema.py         # TypedDict definitions for all event shapes
    ├── setup.py          # Python logging module integration
    └── log_loader.py     # Log inspection utilities
```

## Documentation

- **Usage Guide**: `script/spacelab/README.md`
- **API Reference**: See docstrings in source files

## License

LGPL-3.0 - See [LICENSE](LICENSE) file


---

**Last Updated**: 07/04/2026
