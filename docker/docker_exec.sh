#!/bin/bash
# Helper script to access the HPP Docker environment
# Usage: ./docker_exec.sh [command]
# If no command is provided, opens an interactive bash shell

CONTAINER_NAME="hpp"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Docker container '${CONTAINER_NAME}' is not running."
    echo "Please start the container first."
    exit 1
fi

# Execute command or open interactive shell
if [ $# -eq 0 ]; then
    echo "Opening interactive bash shell in container '${CONTAINER_NAME}'..."
    docker exec -it ${CONTAINER_NAME} bash
else
    echo "Executing command in container '${CONTAINER_NAME}': $@"
    docker exec -it ${CONTAINER_NAME} bash -c "$@"
fi
