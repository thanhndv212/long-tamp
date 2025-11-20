# Installation Guide

## Prerequisites

Agimus Spacelab requires:
- Python >= 3.8
- NumPy >= 1.19.0
- Pinocchio
- hpp-pinocchio

Optional (depending on backend):
- hpp-manipulation-corba (for CORBA backend)
- hpp-gepetto-viewer (for CORBA visualization)
- hpp-python (for PyHPP backend)

## Installation Methods

### Method 1: pip (Python-only)

Install from PyPI (when available):

```bash
pip install agimus-spacelab
```

Or install from source:

```bash
git clone https://gitlab.laas.fr/agimus-project/agimus_spacelab.git
cd agimus-spacelab
pip install -e .
```

### Method 2: CMake (System-wide)

For integration with ROS2 or HPP workspace:

```bash
cd ~/hpp/src  # or your workspace
git clone https://gitlab.laas.fr/agimus-project/agimus_spacelab.git
cd agimus-spacelab
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/hpp/install
make install
```

### Method 3: ROS2 Workspace

```bash
cd ~/ros2_ws/src
git clone https://gitlab.laas.fr/agimus-project/agimus_spacelab.git
cd ~/ros2_ws
colcon build --packages-select agimus_spacelab
source install/setup.bash
```

## Backend Installation

### CORBA Backend

Install via robotpkg:

```bash
sudo apt-get install robotpkg-hpp-manipulation-corba
sudo apt-get install robotpkg-hpp-gepetto-viewer
```

### PyHPP Backend

Install via robotpkg:

```bash
sudo apt-get install robotpkg-hpp-python
```

## Verification

Test your installation:

```python
import agimus_spacelab
print(agimus_spacelab.__version__)
print(agimus_spacelab.get_available_backends())
```

Expected output:
```
0.1.0
['pyhpp']  # or ['corba', 'pyhpp'] if both installed
```

## Troubleshooting

### Import Error: "No module named 'agimus_spacelab'"

Make sure the package is in your PYTHONPATH:

```bash
export PYTHONPATH=$PYTHONPATH:/path/to/install/lib/python3.x/site-packages
```

### Backend Not Available

If a backend doesn't appear in `get_available_backends()`:
1. Verify the backend packages are installed
2. Check that Python can import them:
   ```python
   import hpp.corbaserver  # for CORBA
   import pyhpp  # for PyHPP
   ```

### URDF Files Not Found

Make sure ROS package paths are set:

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/packages
```
