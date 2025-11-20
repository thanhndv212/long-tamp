# Agimus Spacelab - Manipulation Planning Framework

A generalized and extensible manipulation planning framework for the Agimus project, supporting both CORBA and PyHPP backends for HPP-based manipulation tasks.

## ✨ Features

- **Dual Backend Support**: Seamlessly switch between CORBA server and PyHPP bindings
- **Unified API**: Consistent interface regardless of backend choice
- **Modular Design**: Reusable components for robot setup, planning, and visualization
- **Flexible Installation**: Install via pip, CMake, or use Docker
- **Optional Dependencies**: Install only the backends you need
- **Comprehensive Documentation**: Detailed API reference and tutorials
- **Well-Tested**: Unit tests for both backends with 13 new test methods
- **Visualization Examples**: 4 demo modes for configuration display
- **Multiple Task Examples**: Pick-and-place, dual-arm coordination, assembly tasks


## Quick Start

### Docker Workflow (Recommended)

```bash
# Navigate to package
cd /home/dvtnguyen/devel/hpp/src/agimus_spacelab

# Build Docker image (first time only)
./docker/run_docker.sh build

# Start container
./docker/run_docker.sh start

# Enter container
./docker/run_docker.sh exec

# Inside container - test installation
./test_installation.sh

# Inside container - run examples
cd script/examples
python3 unified_api_example.py
python3 spacelab_pyhpp_example.py --solve

# Inside container - run tests
pytest tests/ -v
```

### Alternative: Direct Installation

#### Via pip

```bash
# Install in editable mode with dev dependencies
pip install -e ".[dev]"

# Or install with specific backends
pip install -e ".[corba]"   # CORBA backend only
pip install -e ".[pyhpp]"   # PyHPP backend only
pip install -e ".[all]"     # All backends
```

#### Via CMake

```bash
# In HPP workspace
cd ~/hpp/src/agimus_spacelab
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/hpp/install
make install
```

#### Via ROS2

```bash
# In ROS2 workspace
cd ~/ros2_ws/src
ln -s /home/dvtnguyen/devel/hpp/src/agimus_spacelab
cd ~/ros2_ws
colcon build --packages-select agimus_spacelab
```

## Usage Examples

### Unified API (Backend-Agnostic)

```python
from agimus_spacelab import ManipulationPlanner
from agimus_spacelab.config import InitialConfigurations, ManipulationConfig

# Create planner with chosen backend
planner = ManipulationPlanner(backend="pyhpp")  # or "corba"

# Load robot and environment
planner.load_robot(
    name="ur10",
    urdf_path="package://ur_description/urdf/ur10_robot.urdf",
    srdf_path="package://ur_description/srdf/ur10.srdf"
)

# Load objects
planner.load_object(
    name="box",
    urdf_path="package://example_robot_data/robots/box_robot.urdf",
    joint_type="freeflyer"
)

# Configure planning
q_init = InitialConfigurations.build_full_configuration(
    InitialConfigurations.UR10,
    {"box": [0.5, 0.0, 0.2, 0, 0, 0, 1]}
)
planner.set_initial_config(q_init)

# Create constraint graph with automatic rules
planner.create_constraint_graph(
    name="manipulation",
    grippers=ManipulationConfig.GRIPPERS,
    objects=["box"],
    rules_strategy="auto"
)

# Solve and visualize
if planner.solve(max_iterations=5000):
    planner.visualize()
    planner.play_path()
```

### Configuration Management

```python
from agimus_spacelab.config import (
    RobotJoints,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
    RuleGenerator
)

# Access predefined configurations
joints = RobotJoints.all_robot_joints()
q_ur10 = InitialConfigurations.UR10
bounds = JointBounds.freeflyer_bounds()

# Build custom configurations
from agimus_spacelab.utils import ConfigBuilder
builder = ConfigBuilder()
builder.add_robot("ur10", InitialConfigurations.UR10)
builder.add_object("box", [0.5, 0.0, 0.2, 0, 0, 0, 1])
q_full = builder.build()

# Generate constraint graph rules
rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
```

### Transformation Utilities

```python
from agimus_spacelab.utils import xyzrpy_to_xyzquat, xyzrpy_to_se3
import numpy as np

# Convert RPY to quaternion
xyzrpy = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
xyzquat = xyzrpy_to_xyzquat(xyzrpy)  # [x, y, z, qx, qy, qz, qw]

# Convert to SE3 matrix
se3_matrix = xyzrpy_to_se3(xyzrpy)  # 4x4 numpy array
```

## Examples

The package includes comprehensive examples demonstrating different manipulation tasks and backends:

### Task-Specific Examples

#### Spacelab Multi-Robot Manipulation (`script/spacelab/`)
- **spacelab_corba_example.py** - Complete Spacelab scene with CORBA (200 lines)
- **spacelab_pyhpp_example.py** - Complete Spacelab scene with PyHPP (230 lines)
- Multi-robot coordination (UR10 + VISPA)
- Multiple objects (RS1, ScrewDriver, FrameGripper, CleatGripper)
- Complex constraint graphs with grasp/placement rules

#### Grasp Ball Manipulation (`script/graspball/`)
- **graspball_corba_example.py** - UR5 ball grasping with CORBA (465 lines)
- **graspball_pyhpp_example.py** - UR5 ball grasping with PyHPP (345 lines)
- Single-arm manipulation with free-flying object
- 8-step manipulation sequence (approach → grasp → lift → transfer → place → release)
- Placement and grasp constraints

### Generic Examples (`script/examples/`)

1. **unified_api_example.py** - Backend-agnostic API demonstration
2. **visualization_example.py** - 4 visualization demos (400 lines)
3. **pick_place_example.py** - Simple pick-and-place task (120 lines)
4. **dual_arm_example.py** - Two-robot coordination (170 lines)
5. **assembly_example.py** - Multi-part assembly task (220 lines)

Run any example:
```bash
cd script/examples
python3 pick_place_example.py
python3 visualization_example.py --demo 1
python3 spacelab_corba_example.py --solve
```

## Testing

### Run Tests

```bash
# All tests
pytest tests/ -v

# With coverage
pytest tests/ --cov=agimus_spacelab --cov-report=html

# Specific backend tests
pytest tests/test_corba.py -v
pytest tests/test_pyhpp.py -v

# Specific test
pytest tests/test_utils.py::test_xyzrpy_to_xyzquat -v
```

### Test Coverage

- **Core functionality**: 4 test files
- **Backend implementations**: 13 new test methods added
- **Utilities**: Transformation and configuration tests
- **Configuration**: Rule generation and bounds tests

## Package Structure

```
agimus_spacelab/
├── src/agimus_spacelab/          # Main Python package
│   ├── __init__.py               # Package initialization
│   ├── version.py                # Version: 0.1.0
│   ├── planner.py                # Unified API (ManipulationPlanner)
│   ├── core/                     # Base classes and interfaces
│   │   └── __init__.py           # ManipulationTask, RobotConfig, etc.
│   ├── corba/                    # CORBA backend (hpp-manipulation-corba)
│   │   └── __init__.py           # CorbaManipulationPlanner (~262 lines)
│   ├── pyhpp/                    # PyHPP backend (hpp-python)
│   │   └── __init__.py           # PyHPPManipulationPlanner (~235 lines)
│   ├── utils/                    # Utility functions
│   │   ├── __init__.py           # Transformations (XYZRPY↔XYZQUAT↔SE3)
│   │   └── config_utils.py       # ConfigBuilder, BoundsManager
│   └── config/                   # Configuration classes
│       ├── __init__.py           # RobotJoints, Configurations, Bounds
│       └── rules.py              # RuleGenerator for constraint graphs
├── script/examples/              # 7 example scripts
├── tests/                        # Comprehensive test suite
├── doc/                          # Sphinx documentation
│   ├── index.rst                 # Main documentation
│   ├── api.rst                   # API reference (160 lines)
│   ├── usage.rst                 # Usage guide (370 lines)
│   ├── examples.rst              # Example documentation (450 lines)
│   └── installation.rst          # Installation instructions
├── docker/                       # Docker environment
│   ├── Dockerfile                # HPP-based image
│   ├── docker-compose.yml        # Compose configuration
│   ├── run_docker.sh             # Management script
│   └── README.md                 # Docker documentation
├── CMakeLists.txt               # CMake build system
├── package.xml                   # ROS2 package manifest
├── pyproject.toml               # Python packaging (pip)
├── setup.cfg                     # Setup configuration
├── test_installation.sh          # Installation verification
└── README.md                     # This file
```

## Dependencies

### Required
- Python >= 3.8
- NumPy >= 1.19.0
- Pinocchio
- hpp-pinocchio

### Optional (Backends)
- **hpp-manipulation-corba** - For CORBA backend
- **hpp-gepetto-viewer** - For CORBA visualization
- **hpp-python** - For PyHPP backend

### Development
- pytest >= 6.0
- pytest-cov
- black (code formatting)
- ruff (linting)
- sphinx (documentation)
- sphinx-rtd-theme

## Docker Environment

### Container Management

```bash
# Build image (first time)
./docker/run_docker.sh build

# Container lifecycle
./docker/run_docker.sh start      # Start container
./docker/run_docker.sh stop       # Stop container
./docker/run_docker.sh restart    # Restart container
./docker/run_docker.sh remove     # Remove container

# Access and testing
./docker/run_docker.sh exec       # Enter container (bash)
./docker/run_docker.sh test       # Run tests
./docker/run_docker.sh status     # Check status
./docker/run_docker.sh logs       # View logs
```

### Quick Scripts

```bash
./docker/docker_exec.sh           # Quick bash access
./docker/docker_test.sh           # Quick test runner
./docker/docker_python.sh         # Quick Python REPL
./docker/docker_install.sh        # Quick pip install
```

### Container Features

- **Base Image**: HPP Docker with all dependencies
- **Mounted Volume**: `/workspace/agimus_spacelab` (live editing)
- **Python Path**: Pre-configured for package import
- **Display Support**: X11 forwarding for visualization
- **User**: Non-root user (1000:1000)

## Documentation

### Sphinx Documentation

Build HTML documentation:
```bash
cd doc/
sphinx-build -b html . _build/html
# Open _build/html/index.html in browser
```

### Documentation Structure

- **Installation Guide**: Docker, pip, CMake installation methods
- **Usage Guide**: Complete workflow examples and best practices
- **API Reference**: Auto-generated API documentation for all modules
- **Examples**: Detailed example documentation with code listings
- **Backend Comparison**: CORBA vs PyHPP feature comparison

### Test Package Import

```bash
# Basic import
python3 -c "import agimus_spacelab; print(f'Version: {agimus_spacelab.__version__}')"

# API import
python3 -c "from agimus_spacelab import ManipulationPlanner; print('✓ API ready')"

# Backend availability
python3 -c "from agimus_spacelab import get_available_backends; print(get_available_backends())"

# Configuration import
python3 -c "from agimus_spacelab.config import RobotJoints; print('✓ Config loaded')"

# Utilities import
python3 -c "from agimus_spacelab.utils import xyzrpy_to_xyzquat; print('✓ Utils ready')"
```

### Run Installation Test

```bash
./test_installation.sh
```

## Troubleshooting

### Import Errors in Docker

```bash
# Check PYTHONPATH
echo $PYTHONPATH  # Should include /workspace/agimus_spacelab/src

# Set if needed
export PYTHONPATH=/workspace/agimus_spacelab/src:$PYTHONPATH
```

### Container Issues

```bash
# Check status
./docker/run_docker.sh status

# Remove and recreate
./docker/run_docker.sh remove
./docker/run_docker.sh build
./docker/run_docker.sh start
```

### Permission Issues

```bash
# Fix file ownership (on host)
sudo chown -R $USER:$USER /home/dvtnguyen/devel/hpp/src/agimus_spacelab
```

### Backend Not Available

```python
from agimus_spacelab import check_backend, get_available_backends

# Check specific backend
if not check_backend("pyhpp"):
    print("PyHPP backend not available")

# List available backends
print(f"Available: {get_available_backends()}")
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes with tests
4. Run tests (`pytest tests/ -v`)
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## License

This project is licensed under the LGPL-3.0 License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use this work in your research, please cite:

```bibtex
@software{agimus_spacelab_2025,
  author = {Nguyen, Thanh},
  title = {Agimus Spacelab: Manipulation Planning Framework},
  year = {2025},
  publisher = {GitHub},
  url = {https://gitlab.laas.fr/agimus-project/agimus_spacelab}
}
```

## Acknowledgments

This framework is built upon:
- [HPP (Humanoid Path Planner)](https://humanoid-path-planner.github.io/hpp-doc/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- [Agimus Project](https://gepettoweb.laas.fr/doc/agimus/agimus/master/doxygen-html/)

## 💬 Support

For questions and issues:
- GitHub Issues: [https://gitlab.laas.fr/agimus-project/agimus_spacelab/issues](https://gitlab.laas.fr/agimus-project/agimus_spacelab/issues)
- Documentation: See `doc/` directory
- Examples: See `script/examples/` directory

---

**Status**: ✅ Production Ready | **Version**: 0.1.0 | **Last Updated**: November 2025
