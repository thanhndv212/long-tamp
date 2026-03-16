"""
Path I/O utilities for agimus_spacelab.

This module provides utilities for loading, saving, and replaying
motion planning paths from files.
"""

from __future__ import annotations

import os
import glob
from pathlib import Path
from typing import List, Optional, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from agimus_spacelab.planning.planner import Planner


__all__ = [
    "PathLoadError",
    "load_paths_from_directory",
    "replay_paths",
    "get_path_files",
]


class PathLoadError(Exception):
    """Exception raised when path loading fails.

    Attributes:
        message: Explanation of the error.
        requires_graph: True if the error is due to missing constraint graph.
    """

    def __init__(self, message: str, requires_graph: bool = False):
        self.message = message
        self.requires_graph = requires_graph
        super().__init__(self.message)


def get_path_files(
    directory: str,
    pattern: str = "phase_*.path",
    include_json: bool = True,
) -> dict:
    """
    List available path files in a directory.

    Args:
        directory: Directory to search.
        pattern: Glob pattern for native path files.
        include_json: Also look for .json waypoint files.

    Returns:
        Dictionary with keys 'native' and 'json', each containing
        a sorted list of file paths.

    Example:
        >>> files = get_path_files("/tmp/grasp_sequence_paths")
        >>> print(f"Found {len(files['native'])} native, {len(files['json'])} JSON")
    """
    result = {"native": [], "json": []}

    if not os.path.isdir(directory):
        return result

    result["native"] = sorted(glob.glob(os.path.join(directory, pattern)))

    if include_json:
        json_pattern = pattern.replace(".path", ".json")
        result["json"] = sorted(glob.glob(os.path.join(directory, json_pattern)))

    return result


def load_paths_from_directory(
    planner: Any,
    directory: str,
    pattern: str = "phase_*.path",
    prefer_json: bool = True,
    auto_setup_graph: bool = True,
) -> List[int]:
    """
    Load paths from a directory into the planner.

    Supports both native .path files and portable .json waypoint files.
    JSON files are preferred by default as they work across sessions.

    Args:
        planner: Planner instance with load_path_from_waypoints or
                 load_paths_from_directory method.
        directory: Directory containing path files.
        pattern: Glob pattern for path files.
        prefer_json: If True, prefer JSON waypoint files over native paths.
        auto_setup_graph: For JSON loading, validate graph metadata.

    Returns:
        List of loaded path indices.

    Raises:
        PathLoadError: If loading fails (e.g., directory doesn't exist,
                       graph mismatch for native paths).

    Example:
        >>> indices = load_paths_from_directory(
        ...     planner,
        ...     "/tmp/grasp_sequence_paths",
        ...     prefer_json=True,
        ... )
        >>> print(f"Loaded {len(indices)} paths")
    """
    if not os.path.isdir(directory):
        raise PathLoadError(f"Directory does not exist: {directory}")

    files = get_path_files(directory, pattern)
    native_files = files["native"]
    json_files = files["json"]

    indices = []

    # Try JSON first if preferred and available
    if prefer_json and json_files and hasattr(planner, "load_path_from_waypoints"):
        print(f"\nLoading {len(json_files)} JSON waypoint files...")
        for filepath in json_files:
            try:
                idx = planner.load_path_from_waypoints(
                    filepath,
                    add_to_problem=True,
                    auto_setup_graph=auto_setup_graph,
                )
                indices.append(idx)
                print(f"  ✓ Loaded {os.path.basename(filepath)} -> index {idx}")
            except Exception as e:
                print(f"  ✗ Failed to load {os.path.basename(filepath)}: {e}")
        return indices

    # Try native paths
    if native_files and hasattr(planner, "load_paths_from_directory"):
        try:
            indices = planner.load_paths_from_directory(
                directory,
                pattern=pattern,
                sort=True,
            )
            return indices
        except Exception as e:
            error_msg = str(e)
            if (
                "graph" in error_msg.lower()
                or "deserialize edges" in error_msg.lower()
            ):
                raise PathLoadError(
                    "These paths contain constraint graph edge references. "
                    "They can only be loaded in the same session where they were created, "
                    "or use JSON waypoint files instead.",
                    requires_graph=True,
                ) from e
            raise PathLoadError(f"Error loading paths: {e}") from e

    # No suitable loader
    raise PathLoadError(
        f"No path files found in {directory} or backend does not support path loading"
    )


def replay_paths(
    planner: Any,
    indices: List[int],
    verbose: bool = True,
) -> dict:
    """
    Replay a sequence of paths.

    Args:
        planner: Planner instance with play_path method.
        indices: List of path indices to replay.
        verbose: If True, print progress messages.

    Returns:
        Dictionary with 'success' (bool) and 'failed' (list of failed indices).

    Example:
        >>> result = replay_paths(planner, [0, 1, 2])
        >>> if result["success"]:
        ...     print("All paths replayed successfully")
    """
    failed = []

    for idx in indices:
        if verbose:
            print(f"  Playing path {idx}...")
        try:
            planner.play_path(idx)
        except Exception as e:
            if verbose:
                print(f"    Failed: {e}")
            failed.append(idx)

    return {
        "success": len(failed) == 0,
        "failed": failed,
        "played": len(indices) - len(failed),
    }


def get_num_paths(planner: Any) -> int:
    """
    Get the number of available stored paths for replay.

    Handles different backend implementations transparently.

    Args:
        planner: Planner instance.

    Returns:
        Number of available paths.
    """
    stored = getattr(planner, "_stored_paths", None)
    if isinstance(stored, list):
        return len(stored)

    # CORBA backend exposes ProblemSolver via planner.ps
    ps = getattr(planner, "ps", None)
    if ps is not None and hasattr(ps, "numberPaths"):
        try:
            return int(ps.numberPaths())
        except Exception:
            return 0

    # PyHPP backend typically stores a single path in planner.path
    path = getattr(planner, "path", None)
    return 1 if path is not None else 0
