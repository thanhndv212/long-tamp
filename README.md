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

## Documentation

- **Usage Guide**: `script/spacelab/README.md`
- **API Reference**: See docstrings in source files

## License

LGPL-3.0 - See [LICENSE](LICENSE) file


---

**Last Updated**: 07/04/2026
