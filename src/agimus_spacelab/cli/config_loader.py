"""
Configuration loading utilities for agimus_spacelab task scripts.

This module provides utilities for loading task configurations from
the config directory, replacing the repeated sys.path manipulation
pattern found in task scripts.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from typing import Any, Optional


def load_task_config(
    config_dir: Path,
    module_name: str,
    class_name: str,
    init_poses: bool = True,
) -> Any:
    """
    Load a task configuration class from the config directory.

    This function handles the sys.path manipulation needed to import
    configuration modules from the script's config directory.

    Args:
        config_dir: Path to the directory containing config modules.
        module_name: Name of the module to import (e.g., "spacelab_config").
        class_name: Name of the configuration class or attribute to get
                    (e.g., "TaskConfigurations.DisplayAllStates").
        init_poses: If True, call cfg.init_poses() after loading.

    Returns:
        The loaded configuration object.

    Raises:
        ImportError: If the module cannot be imported.
        AttributeError: If the class_name cannot be found.

    Example:
        >>> from pathlib import Path
        >>> config_dir = Path(__file__).parent.parent / "config"
        >>> cfg = load_task_config(
        ...     config_dir,
        ...     "spacelab_config",
        ...     "TaskConfigurations.DisplayAllStates",
        ... )
    """
    # Add config directory to path if not already present
    config_dir_str = str(config_dir.resolve())
    if config_dir_str not in sys.path:
        sys.path.insert(0, config_dir_str)

    # Import the module
    module = importlib.import_module(module_name)

    # Navigate to the class/attribute
    obj = module
    for attr in class_name.split("."):
        obj = getattr(obj, attr)

    # Initialize poses if requested
    if init_poses and hasattr(obj, "init_poses"):
        obj.init_poses()

    return obj


def get_default_config_dir(script_path: Path) -> Path:
    """
    Get the default config directory relative to a script path.

    The convention is that config files are in ../config/ relative
    to the script directory.

    Args:
        script_path: Path to the script file (typically __file__).

    Returns:
        Path to the config directory.

    Example:
        >>> config_dir = get_default_config_dir(Path(__file__))
    """
    return script_path.parent.parent / "config"
