#!/bin/bash
# Script to install agimus_spacelab package inside the HPP Docker container
# Usage: ./docker_install.sh [install_method]
#   install_method: "pip" (default) or "cmake"

CONTAINER_NAME="hpp"
PACKAGE_PATH="/home/dvtnguyen/devel/hpp/src/agimus_spacelab"
INSTALL_METHOD="${1:-pip}"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Docker container '${CONTAINER_NAME}' is not running."
    echo "Please start the container first with: docker start ${CONTAINER_NAME}"
    exit 1
fi

echo "Installing agimus_spacelab in Docker container '${CONTAINER_NAME}'..."
echo "Installation method: ${INSTALL_METHOD}"
echo ""

if [ "${INSTALL_METHOD}" = "pip" ]; then
    echo "Installing via pip in editable mode..."
    docker exec -it ${CONTAINER_NAME} bash -c "cd ${PACKAGE_PATH} && pip3 install -e ."
    
elif [ "${INSTALL_METHOD}" = "cmake" ]; then
    echo "Installing via CMake..."
    docker exec -it ${CONTAINER_NAME} bash -c "
        cd ${PACKAGE_PATH} && \
        mkdir -p build && \
        cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
        make -j\$(nproc) && \
        sudo make install
    "
    
else
    echo "Error: Unknown install method '${INSTALL_METHOD}'"
    echo "Usage: $0 [pip|cmake]"
    exit 1
fi

echo ""
echo "Installation complete!"
echo "You can now run tests with: ./docker_test.sh"
echo "Or access the container with: ./docker_exec.sh"
