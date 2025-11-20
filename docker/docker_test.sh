#!/bin/bash
# Script to run tests inside the HPP Docker container
# Usage: ./docker_test.sh [pytest options]

CONTAINER_NAME="hpp"
PACKAGE_PATH="/home/dvtnguyen/devel/hpp/src/agimus_spacelab"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Docker container '${CONTAINER_NAME}' is not running."
    echo "Please start the container first with: docker start ${CONTAINER_NAME}"
    exit 1
fi

echo "Running tests in Docker container '${CONTAINER_NAME}'..."
echo "Package path: ${PACKAGE_PATH}"
echo ""

# Run pytest with optional arguments
docker exec -it ${CONTAINER_NAME} bash -c "cd ${PACKAGE_PATH} && python3 -m pytest $@"
