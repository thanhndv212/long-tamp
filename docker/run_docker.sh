#!/bin/bash
# Script to run the Agimus Spacelab Docker container
# Usage: ./docker/run_docker.sh [command]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Docker image and container names
IMAGE_NAME="agimus-spacelab:latest"
CONTAINER_NAME="agimus_spacelab_dev"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Agimus Spacelab Docker Environment ===${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed${NC}"
    exit 1
fi

# Function to build the Docker image
build_image() {
    echo -e "${YELLOW}Building Docker image...${NC}"
    cd "$PROJECT_DIR"
    docker build -f docker/Dockerfile -t "$IMAGE_NAME" .
    echo -e "${GREEN}✓ Image built successfully${NC}"
}

# Function to start the container
start_container() {
    # Check if container exists
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        # Container exists, check if it's running
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo -e "${GREEN}✓ Container is already running${NC}"
        else
            echo -e "${YELLOW}Starting existing container...${NC}"
            docker start "$CONTAINER_NAME"
            echo -e "${GREEN}✓ Container started${NC}"
        fi
    else
        # Create and start new container
        echo -e "${YELLOW}Creating new container...${NC}"
        
        # Allow X server access for GUI applications
        xhost +local:docker > /dev/null 2>&1 || true
        
        docker run -d \
            --name "$CONTAINER_NAME" \
            --hostname agimus-dev \
            --network host \
            -e DISPLAY="$DISPLAY" \
            -e QT_X11_NO_MITSHM=1 \
            -e PYTHONPATH=/workspace/agimus_spacelab/src \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$PROJECT_DIR:/workspace/agimus_spacelab" \
            -v "$HOME/.ssh:/root/.ssh:ro" \
            -v "$HOME/.gitconfig:/root/.gitconfig:ro" \
            -w /workspace/agimus_spacelab \
            -it \
            "$IMAGE_NAME" \
            /bin/bash
        
        echo -e "${GREEN}✓ Container created and started${NC}"
    fi
}

# Function to stop the container
stop_container() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Stopping container...${NC}"
        docker stop "$CONTAINER_NAME"
        echo -e "${GREEN}✓ Container stopped${NC}"
    else
        echo -e "${YELLOW}Container is not running${NC}"
    fi
}

# Function to remove the container
remove_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        stop_container
        echo -e "${YELLOW}Removing container...${NC}"
        docker rm "$CONTAINER_NAME"
        echo -e "${GREEN}✓ Container removed${NC}"
    else
        echo -e "${YELLOW}Container does not exist${NC}"
    fi
}

# Function to enter the container (bash shell)
exec_bash() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${GREEN}Entering container...${NC}"
        docker exec -it "$CONTAINER_NAME" /bin/bash
    else
        echo -e "${RED}Error: Container is not running${NC}"
        echo "Use '$0 start' to start the container first"
        exit 1
    fi
}

# Function to run tests inside container
run_tests() {
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Running tests...${NC}"
        docker exec -it "$CONTAINER_NAME" bash -c "cd /workspace/agimus_spacelab && pytest tests/"
    else
        echo -e "${RED}Error: Container is not running${NC}"
        exit 1
    fi
}

# Function to show container logs
show_logs() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        docker logs -f "$CONTAINER_NAME"
    else
        echo -e "${RED}Error: Container does not exist${NC}"
        exit 1
    fi
}

# Function to show status
show_status() {
    echo -e "\n${YELLOW}=== Docker Status ===${NC}"
    
    # Check if image exists
    if docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
        echo -e "Image: ${GREEN}✓ ${IMAGE_NAME}${NC}"
    else
        echo -e "Image: ${RED}✗ Not built${NC}"
    fi
    
    # Check if container exists and its status
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo -e "Container: ${GREEN}✓ Running${NC}"
        else
            echo -e "Container: ${YELLOW}⚠ Stopped${NC}"
        fi
    else
        echo -e "Container: ${RED}✗ Not created${NC}"
    fi
    
    echo ""
}

# Main script logic
case "${1:-}" in
    build)
        build_image
        ;;
    start)
        start_container
        ;;
    stop)
        stop_container
        ;;
    restart)
        stop_container
        start_container
        ;;
    remove|rm)
        remove_container
        ;;
    exec|bash|shell)
        exec_bash
        ;;
    test)
        run_tests
        ;;
    logs)
        show_logs
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {build|start|stop|restart|remove|exec|test|logs|status}"
        echo ""
        echo "Commands:"
        echo "  build      - Build the Docker image"
        echo "  start      - Start the container"
        echo "  stop       - Stop the container"
        echo "  restart    - Restart the container"
        echo "  remove     - Remove the container"
        echo "  exec       - Enter container (bash shell)"
        echo "  test       - Run tests inside container"
        echo "  logs       - Show container logs"
        echo "  status     - Show container status"
        echo ""
        echo "Quick start:"
        echo "  $0 build    # First time only"
        echo "  $0 start    # Start container"
        echo "  $0 exec     # Enter container"
        exit 1
        ;;
esac

exit 0
