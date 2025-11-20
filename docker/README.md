# Docker Environment for Agimus Spacelab

This directory contains Docker configuration files for developing and testing Agimus Spacelab within an HPP environment.

## Quick Start

### 1. Build the Docker Image

```bash
cd /home/dvtnguyen/devel/hpp/src/agimus_spacelab
./docker/run_docker.sh build
```

### 2. Start the Container

```bash
./docker/run_docker.sh start
```

### 3. Enter the Container

```bash
./docker/run_docker.sh exec
```

Or using direct Docker command:
```bash
docker exec -it agimus_spacelab_dev bash
```

## Available Commands

The `run_docker.sh` script provides the following commands:

- **`build`** - Build the Docker image
- **`start`** - Start the container (creates if doesn't exist)
- **`stop`** - Stop the running container
- **`restart`** - Restart the container
- **`remove`** - Remove the container
- **`exec`** - Enter container with bash shell
- **`test`** - Run tests inside the container
- **`logs`** - Show container logs
- **`status`** - Show current status of image and container

## Development Workflow

### Initial Setup

```bash
# Build the image (first time only)
./docker/run_docker.sh build

# Start the container
./docker/run_docker.sh start

# Enter the container
./docker/run_docker.sh exec
```

### Inside the Container

Once inside the container, you can:

```bash
# Navigate to project
cd /workspace/agimus_spacelab

# Run Python interactively
python3

# Import the package
>>> import agimus_spacelab
>>> from agimus_spacelab import ManipulationPlanner

# Run examples
cd script/examples
python3 spacelab_corba_example.py

# Run tests
pytest tests/

# Run specific test
pytest tests/test_utils.py -v

# Run tests with coverage
pytest --cov=agimus_spacelab --cov-report=html
```

### Working with CORBA Backend

```bash
# Inside container, start CORBA server if needed
# Then run CORBA examples
cd /workspace/agimus_spacelab/script/examples
python3 spacelab_corba_example.py
```

### Working with PyHPP Backend

```bash
# PyHPP examples don't require CORBA server
cd /workspace/agimus_spacelab/script/examples
python3 spacelab_pyhpp_example.py
```

## File Structure

```
docker/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Docker Compose configuration
├── run_docker.sh          # Convenience script for Docker operations
└── README.md              # This file
```

## Volume Mounts

The container mounts the following directories:

- **Project source**: `/workspace/agimus_spacelab` → mounted from host
- **SSH keys**: `~/.ssh` → read-only access for git operations
- **Git config**: `~/.gitconfig` → read-only for git settings
- **X11 socket**: `/tmp/.X11-unix` → for GUI applications (Gepetto viewer)

## Environment Variables

Inside the container:

- `PYTHONPATH=/workspace/agimus_spacelab/src` - Python can import agimus_spacelab
- `DISPLAY` - Set for X11 forwarding (GUI support)
- `QT_X11_NO_MITSHM=1` - Qt compatibility

## Testing Installation

### Test Python Import

```bash
# Enter container
./docker/run_docker.sh exec

# Test imports
python3 -c "import agimus_spacelab; print('✓ Package imported successfully')"
python3 -c "from agimus_spacelab.utils import xyzrpy_to_xyzquat; print('✓ Utils imported')"
python3 -c "from agimus_spacelab.config import RobotJoints; print('✓ Config imported')"
```

### Run Test Suite

```bash
# From host
./docker/run_docker.sh test

# Or inside container
cd /workspace/agimus_spacelab
pytest tests/ -v
```

## Troubleshooting

### Container Won't Start

```bash
# Check status
./docker/run_docker.sh status

# Remove and recreate
./docker/run_docker.sh remove
./docker/run_docker.sh start
```

### GUI Applications Don't Display

```bash
# On host, allow X11 connections
xhost +local:docker

# Verify DISPLAY inside container
docker exec -it agimus_spacelab_dev bash -c 'echo $DISPLAY'
```

### Python Import Errors

```bash
# Inside container, verify PYTHONPATH
echo $PYTHONPATH

# Should include: /workspace/agimus_spacelab/src

# Check if files exist
ls -la /workspace/agimus_spacelab/src/agimus_spacelab/
```

### Permission Issues

```bash
# Files created inside container are owned by root
# To change ownership back on host:
sudo chown -R $USER:$USER /home/dvtnguyen/devel/hpp/src/agimus_spacelab
```

## Using Docker Compose (Alternative)

You can also use docker-compose for managing the container:

```bash
# Start container
cd /home/dvtnguyen/devel/hpp/src/agimus_spacelab
docker-compose -f docker/docker-compose.yml up -d

# Enter container
docker-compose -f docker/docker-compose.yml exec agimus-spacelab bash

# Stop container
docker-compose -f docker/docker-compose.yml down
```

## Integration with Existing HPP Docker

If you already have an HPP Docker container running:

```bash
# Enter your existing HPP container
docker exec -it hpp bash

# Inside the container, mount the agimus_spacelab project
# (Adjust path as needed)
export PYTHONPATH=/path/to/agimus_spacelab/src:$PYTHONPATH

# Test import
python3 -c "import agimus_spacelab; print('Success')"
```

## Next Steps

After setting up the Docker environment:

1. Run the examples in `script/examples/`
2. Modify configurations in `src/agimus_spacelab/config/`
3. Create your own manipulation tasks
4. Run tests to verify changes
5. Contribute back to the project!

## Additional Resources

- [Agimus Spacelab Documentation](../doc/)
- [HPP Documentation](https://humanoid-path-planner.github.io/hpp-doc/)
- [Docker Documentation](https://docs.docker.com/)
