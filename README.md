# Agimus Spacelab - Manipulation Planning Framework

Multi-arm collaborative manipulation planning for SpaceLab assembly tasks using HPP (Humanoid Path Planner).

`agimus_spacelab` plans **long, multi-step assembly sequences** for **several robot arms working together** on **many movable objects** in a single shared scene. It builds on HPP's constraint-graph manipulation planning and adds a task/orchestration layer that turns a high-level assembly goal (grip this, move that, hand it over, place it) into a single concatenated, collision-free motion for the whole multi-robot system.

## Capabilities

- **Long-horizon sequence planning.** The `GraspSequencePlanner` chains an arbitrary number of grasp/place/hand-over phases into one continuous plan. Each phase gets a *minimal, phase-local* constraint graph and the paths are concatenated across phases, so planning cost grows **linearly O(N)** with the number of grasps instead of combinatorially **O(N!)** — long assembly missions stay tractable.
- **Multiple robots (multi-arm & collaborative).** The scene composes several arms into one planning model (e.g. UR10 + VISPA + VISPA2, a ~70-DOF composite) that plan in a shared, mutually-collision-aware world. Arms can act independently, cooperate on the same object, or hand objects off between each other.
- **Multiple objects.** Any number of free-flying objects and tools (reflector panels, frame gripper, screw driver, …) coexist in the scene. Grasp legality is data-driven via `VALID_PAIRS` (which gripper may grasp which handle), so adding objects/tools is a config change, not a code change.
- **Constraint-graph manipulation planning.** Grasp, placement, and transition constraints are generated from a declarative `ManipulationConfig`; the constraint graph and its edges are built automatically per phase.
- **Reproducibility & introspection.** Structured, crash-safe JSONL run logging captures every phase/edge attempt for replay, debugging, and auditing (see *Run Logging*).
- **Modular architecture.** Reusable building blocks — `SceneBuilder`, `ConstraintBuilder`, `ConfigGenerator`, `ManipulationTask`, `create_planner()` — compose into custom tasks.
- **Scene visualization.** Interactive 3D viewers: browser-based **viser** (default, no X11) or **gepetto-viewer** (Qt/CORBA).
- **PyHPP backend** (default): in-process bindings via `hpp-python`. The CORBA backend (`hpp-manipulation-corba`) is still available but **deprecated**.


## Quick Start

```bash
cd script/spacelab
./interactive_planning.py -i
```

For a full step-by-step guide (writing a task, multi-phase sequences, resume/replay, and using
this library from the ROS 2 / DBT stack), see [`docs/usage/`](docs/usage/).

## Installation

`agimus_spacelab` has two distinct dependency tiers, and this drives how you install it:

| Tier | Packages | Source |
|------|----------|--------|
| **Python (PyPI)** | `numpy`, `pyyaml`, `pinocchio` (`pin`), and the viser viewer stack (`viser`, `trimesh`, `pycollada`) | `pip` |
| **HPP native bindings** | `hpp-python` (pyhpp), `hpp-toppra`, `hpp-gepetto-viewer`, `hpp-manipulation-corba`, `omniORBpy`, … | **robotpkg / conda-forge / source only — NOT on PyPI** |

The HPP native bindings are C++ extension modules and **cannot be installed with pip**. `pip install agimus-spacelab` therefore gives you a working *pure-Python* package (config parsing, planning-graph construction, transforms, run logging, viser viewer), but the planning **backends** must be provided by your environment (the `hpp-agimus` container, robotpkg, or conda-forge). Instantiating a backend without its bindings raises an `ImportError` explaining exactly what is missing.

Install in **two steps, in this order**: first the HPP native bindings that provide the planning backends, then the `agimus_spacelab` package itself. Installing the package first is pointless — it cannot plan until the backends are on the path.

---

### Step 1 — Install the HPP native bindings (do this first)

These C++ extension modules provide the planning backends and **cannot be installed with pip**. Put them in place before installing or running `agimus_spacelab`. There are two ways to obtain them: the robotpkg binary (1a) or a source build / container (1b). **The default PyHPP backend currently requires the source build (1b)**, because the stable robotpkg binary does not yet ship the extra bindings — see the caveat below.

#### 1a. Binary install via robotpkg

The official HPP binaries are distributed as `robotpkg-*` Debian packages, installed under the `/opt/openrobots` prefix. See the [HPP download page](https://humanoid-path-planner.github.io/hpp-doc/download.html) and the [robotpkg APT repository instructions](http://robotpkg.openrobots.org/debian.html) for the authoritative version.

> **⚠️ The current robotpkg binary (`hpp-python` 6.1.0) is NOT sufficient for
> the PyHPP backend.** `agimus_spacelab` targets a customized/source HPP that
> exposes extra `pyhpp` bindings the upstream stable binary does not yet ship —
> notably `RSTimeParameterization`, `SimpleTimeParameterization`,
> `EnforceTransitionSemantic`, `GraphRandomShortcut` / `GraphPartialShortcut`,
> `SplineGradientBased_bezier{1,3,5}`, and `ProgressiveProjector`. With the
> binary, `import pyhpp` succeeds but the backend reports **"PyHPP backend
> unavailable"** because those symbols are missing.
>
> Until a release fills the gap
> ([humanoid-path-planner/hpp-python](https://github.com/humanoid-path-planner/hpp-python)),
> the PyHPP backend requires the **source-built HPP** (the `hpp-agimus`
> container / `DEVEL_HPP_DIR` flow — see *Source build* below). The robotpkg
> binary is still fine for the C++ toolchain and the deprecated CORBA backend.

1. Add the robotpkg APT repository (the stable `pub` repo is sufficient — all
   HPP packages below, including `hpp-python`, are published there):

   ```bash
   sudo mkdir -p /etc/apt/keyrings
   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc
   sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
   deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
   EOF
   sudo apt-get update
   ```

2. Install the packages. Package names are `robotpkg-py<pyver>-<name>`, where `<pyver>` matches your Python (Ubuntu 24.04 → `312`, 22.04 → `310`, 20.04 → `38`). The current HPP release line is **6.1.0**.

   **Required — just two packages.** `hpp-python` transitively pulls the whole
   HPP core stack via apt (pinocchio, eigenpy, coal, hpp-util, hpp-pinocchio,
   hpp-core, hpp-constraints, hpp-manipulation, hpp-manipulation-urdf,
   hpp-corbaserver, omniorbpy), so you do **not** list those individually:

   ```bash
   pyver=312   # adjust to your Python version

   sudo apt-get install \
     robotpkg-py${pyver}-hpp-python \
     robotpkg-py${pyver}-qt5-hpp-gepetto-viewer
   ```

   | Package | Provides | Pulls in (transitively) |
   |---------|----------|-------------------------|
   | `robotpkg-py${pyver}-hpp-python` | PyHPP backend (`pyhpp.*`) | pinocchio, eigenpy, coal, hpp-util, hpp-pinocchio, hpp-core, hpp-constraints, hpp-manipulation, hpp-manipulation-urdf, hpp-corbaserver, omniorbpy |
   | `robotpkg-py${pyver}-qt5-hpp-gepetto-viewer` | Gepetto + `pyhpp_viser` viewers | gepetto-viewer-corba, qgv, qtbase5 |

   **Optional — deprecated CORBA backend.** Only needed if you set
   `backend:=corba`. This is the single extra package; its own dependencies
   (`hpp-corbaserver`, `omniorbpy`, core libs) are already present from the
   required step above:

   ```bash
   sudo apt-get install robotpkg-py${pyver}-hpp-manipulation-corba
   ```

   **Not available as a binary — TOPPRA.** `hpp-toppra` and its `toppra` C++
   dependency are not published in robotpkg (checked: absent from both `pub`
   and `wip`, all distros). To use the TOPPRA optimizer, build `toppra` and
   `hpp-toppra` from source — this is what the `hpp-agimus` container does.

3. Put `/opt/openrobots` on your environment (add to `~/.bashrc`; fix the Python version in `PYTHONPATH`):

   ```bash
   export PATH=/opt/openrobots/bin:$PATH
   export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
   export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:$PYTHONPATH
   export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
   export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
   export ROS_PACKAGE_PATH=/opt/openrobots/share:$ROS_PACKAGE_PATH
   ```

> **Availability (verified against robotpkg, release line 6.1.0):**
> `hpp-python` (6.0.0+), `qt5-hpp-gepetto-viewer`, the full CORBA stack
> (`hpp-manipulation-corba`, `hpp-corbaserver`, `hpp-template-corba`) and the
> C++ core (`hpp-core`, `hpp-constraints`, `hpp-manipulation`,
> `hpp-manipulation-urdf`, `hpp-pinocchio`, `hpp-util`, `hpp-fcl`) are all in
> the stable `pub` repo for `py312` (Ubuntu 24.04) and `py310` (22.04).
> **`hpp-toppra` is the only piece this project uses that is not packaged** —
> build it from source (see *1b. Source build* below).

#### 1b. Source build / container — required for the PyHPP backend today

The recommended way to get a complete, matching HPP stack is the prebuilt Docker image, which compiles HPP from source under `$DEVEL_HPP_DIR` (`~/devel/hpp`). All backends — including the customized `pyhpp` bindings this project relies on — are available inside it without any robotpkg install.

The Docker definitions live in a separate repository: [gitlab.laas.fr/dvtnguyen/dockers](https://gitlab.laas.fr/dvtnguyen/dockers). It provides **two images, one per ROS 2 distribution**:

| Directory | ROS 2 distro | Ubuntu base |
|-----------|--------------|-------------|
| `hpp/` | Jazzy | 24.04 (noble) |
| `hpp-humble/` | Humble | 22.04 (jammy) |

Pick the one matching your target ROS 2 version, build it (`run_docker.sh` in each directory), and work inside the container. To reproduce the build outside Docker, follow the same steps on the host and point `PYTHONPATH` / `LD_LIBRARY_PATH` at your source-install prefix instead of `/opt/openrobots`.

---

### Step 2 — Install the `agimus_spacelab` package

With the HPP native bindings from Step 1 in place, install the package itself — either with pip (standalone / development) or with CMake (inside an HPP workspace).

#### 2a. pip (standalone / development)

```bash
# Editable install with the default viser viewer stack
pip install -e .

# With dev tooling (pytest, black, ruff, sphinx)
pip install -e ".[dev]"

# Standalone WITHOUT an HPP stack — also pulls pinocchio from PyPI:
pip install -e ".[standalone]"
```

This resolves the Python tier only; the planning backends come from Step 1.

> **⚠️ NumPy ABI — do not install `pinocchio` from PyPI on top of robotpkg.**
> The robotpkg/`/opt/openrobots` pinocchio is compiled against **NumPy 1.x**. A
> NumPy 2.x in your user/site path shadows the system numpy and **segfaults**
> the pinocchio C-extension (`_multiarray_umath` ImportError → core dump).
> Therefore:
> - `pinocchio` is **not** a default pip dependency (use `[standalone]` only
>   when no HPP stack is present), and the default numpy is pinned `<2`.
> - In a robotpkg environment, if a stray NumPy 2.x got installed, remove it:
>   `pip uninstall -y numpy pin` (Python then falls back to the system
>   NumPy 1.26 that pinocchio expects). Verify with
>   `python -c "import numpy; print(numpy.__version__, numpy.__file__)"`.
> - Cleanest of all in a robotpkg env: `pip install --no-deps -e .` and let the
>   system/robotpkg provide numpy + pinocchio.

#### 2b. CMake (in an HPP workspace)

This is the source of truth for the native backends. It installs the Python package into `PYTHON_SITELIB` alongside the HPP libraries.

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR
make install
```

Build options (see `CMakeLists.txt`):

| Option | Default | Provides | Native prerequisites |
|--------|:-------:|----------|----------------------|
| `WITH_PYHPP`  | **ON**  | PyHPP backend (default) | `hpp-python` |
| `WITH_TOPPRA` | OFF     | TOPPRA time-parameterization optimizer | `hpp-toppra` (which requires the `toppra` C++ lib ≥0.6.2) |
| `WITH_CORBA`  | **OFF** *(deprecated)* | CORBA backend | `hpp-manipulation-corba`, `hpp-corbaserver`, `omniORBpy` |

`hpp-gepetto-viewer` is picked up whenever `WITH_PYHPP` **or** `WITH_CORBA` is enabled — it provides both the Gepetto (CORBA/Qt) viewer and the `pyhpp_viser` browser viewer.

```bash
# Enable the optional TOPPRA optimizer:
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR -DWITH_TOPPRA=ON

# Re-enable the deprecated CORBA backend (emits a deprecation warning):
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR -DWITH_CORBA=ON
```

### Optional feature extras

The pip extras below carry **no PyPI packages** — they are documented install targets. The listed native packages must come from robotpkg / conda-forge.

| Extra | Command | Native packages to install separately |
|-------|---------|---------------------------------------|
| `toppra` | `pip install "agimus-spacelab[toppra]"` | `hpp-toppra`, `toppra` — **source build only** (not in robotpkg) |
| `corba` *(deprecated)* | `pip install "agimus-spacelab[corba]"` | `hpp-manipulation-corba`, `hpp-corbaserver`, `hpp-gepetto-viewer`, `omniORBpy` |

### Backend availability at runtime

The viser browser viewer ships by default. The other optimizers/viewers are detected at import time and expose `HAS_*` flags in `agimus_spacelab.backends.pyhpp` (`HAS_PYHPP`, `HAS_TOPPRA`, `HAS_VISER`, `HAS_GEPETTO_VIEWER`). A missing backend fails loudly only when you try to construct it, with guidance on how to obtain the bindings.

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
