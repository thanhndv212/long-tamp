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

<!-- ### Multi-Arm Orchestration

```python
from agimus_spacelab.tasks import TaskOrchestrator, TaskBuilder

orchestrator = TaskOrchestrator(max_concurrent_tasks=2)
orchestrator.setup_resources(arms=["UR10", "VISPA"], objects=["RS1", "RS2"])

# Define parallel tasks
t1 = TaskBuilder("t1", "UR10 grasp RS1").requires_arm("UR10").build()
t2 = TaskBuilder("t2", "VISPA grasp RS2").requires_arm("VISPA").build()

orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.run()  # Executes in parallel
``` -->

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
├── tasks/                       # Task orchestration
│   ├── __init__.py
│   ├── base.py                  # ManipulationTask base class
│   ├── orchestration.py         # TaskOrchestrator, TaskBuilder
│   └── bridge.py                # PlanningBridge
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
Behavior Tree Layer (planned)
      ↓
Task Orchestration Layer
      ↓
Atomic Task Layer
      ↓
Motion Planning Layer (HPP)
```

**Key Components:**
- `SceneBuilder`: Fluent API for scene setup
- `ConstraintBuilder`: Helper for constraint creation
- `ConfigGenerator`: Waypoint generation
- `ManipulationTask`: Base class for tasks
- `TaskOrchestrator`: Multi-arm coordination
- `create_planner()`: Factory for backend-specific planners

## Documentation

- **Usage Guide**: `script/spacelab/README.md`
- **API Reference**: See docstrings in source files

## License

LGPL-3.0 - See [LICENSE](LICENSE) file


---

**Last Updated**: January 2026
