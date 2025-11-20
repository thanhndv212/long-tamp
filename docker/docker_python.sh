#!/bin/bash
# Script to run Python code inside the HPP Docker container
# Usage: ./docker_python.sh script.py [args]
#    or: ./docker_python.sh -c "python code"
#    or: ./docker_python.sh (interactive Python)

CONTAINER_NAME="hpp"
PACKAGE_PATH="/home/dvtnguyen/devel/hpp/src/agimus_spacelab"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Docker container '${CONTAINER_NAME}' is not running."
    echo "Please start the container first with: docker start ${CONTAINER_NAME}"
    exit 1
fi

# Set up Python environment
ENV_SETUP="export PYTHONPATH=${PACKAGE_PATH}/src:\$PYTHONPATH"

if [ $# -eq 0 ]; then
    # Interactive Python
    echo "Starting interactive Python in container '${CONTAINER_NAME}'..."
    docker exec -it ${CONTAINER_NAME} bash -c "${ENV_SETUP} && cd ${PACKAGE_PATH} && python3"
    
elif [ "$1" = "-c" ]; then
    # Execute Python code
    shift
    echo "Executing Python code in container '${CONTAINER_NAME}'..."
    docker exec -it ${CONTAINER_NAME} bash -c "${ENV_SETUP} && cd ${PACKAGE_PATH} && python3 -c '$@'"
    
else
    # Run Python script
    echo "Running Python script in container '${CONTAINER_NAME}': $@"
    docker exec -it ${CONTAINER_NAME} bash -c "${ENV_SETUP} && cd ${PACKAGE_PATH} && python3 $@"
fi
