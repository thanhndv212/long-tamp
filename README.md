# Agimus Spacelab - Manipulation Planning Framework

Multi-arm collaborative manipulation planning for SpaceLab assembly tasks using HPP (Humanoid Path Planner).

## Features

- **Multi-arm orchestration**: Task dependency graphs with resource management
- **Modular architecture**: Reusable tools for scene setup, constraints, and configuration
- **Scene visualization**: Interactive 3D viewer with gepetto-viewer
- **Dual backend support**: CORBA (hpp-manipulation-corba) and PyHPP (hpp-python)
- **Task examples**: Grasp, assembly, and collaborative manipulation tasks


## Quick Start

### Visualize Scene

```bash
cd script/spacelab
python visualize_scene.py
```

### Run Example Task

```bash
# Grasp frame_gripper (refactored)
python task_grasp_frame_gripper.py

# Multi-arm collaboration (mock execution)
python example_collaborative_assembly.py
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
from spacelab_tools import ManipulationTask, SpaceLabSceneBuilder

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

### Multi-Arm Orchestration

```python
from task_orchestration import TaskOrchestrator, TaskBuilder

orchestrator = TaskOrchestrator(max_concurrent_tasks=2)
orchestrator.setup_resources(arms=["UR10", "VISPA"], objects=["RS1", "RS2"])

# Define parallel tasks
t1 = TaskBuilder("t1", "UR10 grasp RS1").requires_arm("UR10").build()
t2 = TaskBuilder("t2", "VISPA grasp RS2").requires_arm("VISPA").build()

orchestrator.add_task(t1)
orchestrator.add_task(t2)
orchestrator.run()  # Executes in parallel
```

## Project Structure

See detailed documentation in `script/spacelab/README.md`

```
script/spacelab/
├── spacelab_config.py              # Robot/object configurations
├── spacelab_tools.py               # Reusable utilities (~700 lines)
├── task_orchestration.py           # Multi-arm orchestration
├── task_grasp_frame_gripper.py     # Example task (refactored)
├── example_collaborative_assembly.py  # Multi-arm demo
├── visualize_scene.py              # Scene visualization
└── README.md                       # Architecture & usage guide
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
- `SpaceLabSceneBuilder`: Fluent API for scene setup
- `ConstraintBuilder`: Helper for constraint creation
- `ConfigurationGenerator`: Waypoint generation
- `ManipulationTask`: Base class for tasks
- `TaskOrchestrator`: Multi-arm coordination
- `AtomicTask`: Indivisible manipulation actions

## Dependencies

**Required:**
- Python >= 3.8
- NumPy
- Pinocchio
- hpp-pinocchio
- hpp-manipulation-corba (CORBA backend)
- hpp-gepetto-viewer (visualization)

## Documentation

- **Architecture Guide**: `script/spacelab/MULTI_ARM_ARCHITECTURE.md`
- **Usage Guide**: `script/spacelab/README.md`
- **API Reference**: See docstrings in source files

## License

LGPL-3.0 - See [LICENSE](LICENSE) file

## Acknowledgments

Built on [HPP](https://humanoid-path-planner.github.io/hpp-doc/), [Pinocchio](https://stack-of-tasks.github.io/pinocchio/), and [Agimus](https://gepettoweb.laas.fr/doc/agimus/agimus/master/doxygen-html/)

---

**Version**: 0.1.0 | **Last Updated**: December 2025
