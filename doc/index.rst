# Agimus Spacelab Documentation

Welcome to the documentation for **Agimus Spacelab**, a generalized manipulation planning framework.

## Overview

Agimus Spacelab provides a unified interface for manipulation planning that supports both:
- **CORBA Server** backend (hpp-manipulation-corba)
- **PyHPP** backend (hpp-python direct bindings)

## Features

- 🔄 **Dual Backend Support**: Switch between CORBA and PyHPP seamlessly
- 🎯 **Unified API**: Same code works with both backends
- 🧩 **Modular Design**: Reusable components for various manipulation tasks
- 📦 **Flexible Installation**: pip or CMake
- 🔧 **Optional Dependencies**: Install only what you need
- ✅ **Well-Tested**: Unit tests for reliability
- 🐳 **Docker Ready**: Complete development environment

## Quick Links

```{toctree}
:maxdepth: 2
:caption: User Guide:

installation
usage
examples
```

```{toctree}
:maxdepth: 2
:caption: API Documentation:

api
```

```{toctree}
:maxdepth: 1
:caption: Additional:

comparison_backends
pyhpp_guide
```

## Quick Example

```python
from agimus_spacelab import ManipulationPlanner

# Create planner (choose backend)
planner = ManipulationPlanner(backend="pyhpp")

# Load robot and environment
planner.load_robot("spacelab", robot_urdf, robot_srdf)
planner.load_environment("ground", env_urdf)

# Setup and solve
planner.set_initial_config(q_init)
planner.add_goal_config(q_goal)
success = planner.solve()

# Visualize
if success:
    planner.visualize()
    planner.play_path()
```

## Installation

Using Docker (recommended):

```bash
cd /path/to/agimus_spacelab
./docker/run_docker.sh build
./docker/run_docker.sh start
./docker/run_docker.sh exec
```

Using pip:

```bash
pip install -e .
# With optional backends
pip install -e ".[corba]"  # CORBA support
pip install -e ".[pyhpp]"  # PyHPP support
pip install -e ".[all]"    # Both backends
```

## Indices and Tables

* {ref}`genindex`
* {ref}`modindex`
* {ref}`search`

