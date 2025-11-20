# Dockerfile for Agimus Spacelab Development Environment
# Based on HPP manipulation planning stack

FROM humanoid-path-planner/hpp:latest

# Set maintainer
LABEL maintainer="dvtnguyen@laas.fr"
LABEL description="Agimus Spacelab - Manipulation Planning Framework"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    git \
    vim \
    tmux \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python development tools
RUN pip3 install --no-cache-dir \
    black \
    ruff \
    sphinx \
    sphinx-rtd-theme

# Create workspace directory
RUN mkdir -p /workspace/agimus_spacelab

# Set working directory
WORKDIR /workspace

# Copy agimus_spacelab package (will be mounted as volume in practice)
# COPY . /workspace/agimus_spacelab/

# Set up Python path
ENV PYTHONPATH=/workspace/agimus_spacelab/src:${PYTHONPATH}

# Default command
CMD ["/bin/bash"]
