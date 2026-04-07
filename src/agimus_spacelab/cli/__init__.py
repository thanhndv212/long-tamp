"""
CLI utilities for agimus_spacelab task scripts.

This module provides common argument parsing patterns and configuration loading
utilities used across task scripts.

Example usage:
    >>> import argparse
    >>> from agimus_spacelab.cli import add_common_arguments, add_task_arguments
    >>>
    >>> parser = argparse.ArgumentParser(description="My task")
    >>> add_common_arguments(parser)
    >>> add_task_arguments(parser)
    >>> args = parser.parse_args()
"""

from __future__ import annotations

import argparse
from typing import Optional, List

from .config_loader import load_task_config

__all__ = [
    "add_common_arguments",
    "add_task_arguments",
    "add_advanced_arguments",
    "add_grasp_sequence_arguments",
    "load_task_config",
]


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    """
    Add common arguments used by all task scripts.

    Adds the following arguments:
        --backend: Backend to use (corba or pyhpp)
        --no-viz: Disable visualization
        --solve: Attempt to solve planning problem

    Args:
        parser: ArgumentParser instance to add arguments to.
    """
    parser.add_argument(
        "--backend",
        type=str,
        default="pyhpp",
        choices=["corba", "pyhpp"],
        help="Backend to use (default: pyhpp)",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization",
    )
    parser.add_argument(
        "--solve",
        action="store_true",
        help="Attempt to solve planning problem",
    )


def add_task_arguments(parser: argparse.ArgumentParser) -> None:
    """
    Add task-specific arguments commonly used by task scripts.

    Adds the following arguments:
        --factory: Use ConstraintGraphFactory instead of manual graph
        --show-joints: Print joint information

    Args:
        parser: ArgumentParser instance to add arguments to.
    """
    parser.add_argument(
        "--factory",
        action="store_true",
        help="Use ConstraintGraphFactory instead of manual graph construction",
    )
    parser.add_argument(
        "--show-joints",
        action="store_true",
        help="Print joint information",
    )


def add_advanced_arguments(parser: argparse.ArgumentParser) -> None:
    """
    Add advanced arguments for complex task scripts like task_display_states.

    Adds the following arguments:
        --goal: Target grasp state(s) to include in graph (repeatable)
        --pair: Gripper-handle pair to include (repeatable)
        --show: Configuration names to visualize (repeatable)
        --print-goals: Print feasible goal states and exit
        --limit: Limit printed goal states
        -i/--interactive: Enable interactive mode
        --graph-viz: Visualize constraint graph as PNG
        --solve-mode: Planning algorithm to use

    Args:
        parser: ArgumentParser instance to add arguments to.
    """
    parser.add_argument(
        "--goal",
        action="append",
        default=[],
        metavar="STATE",
        help=(
            "Target grasp state(s) to include in graph, e.g. "
            "'spacelab/g_ur10_tool grasps frame_gripper/h_FG_tool'. "
            "Reduces graph size by filtering VALID_PAIRS. Repeatable."
        ),
    )
    parser.add_argument(
        "--pair",
        action="append",
        default=[],
        metavar="GRIPPER:HANDLE",
        help=(
            "Add a gripper-handle pair to VALID_PAIRS, e.g. "
            "'spacelab/g_ur10_tool:frame_gripper/h_FG_tool'. Repeatable. "
            "If specified, only these pairs are included in the graph."
        ),
    )
    parser.add_argument(
        "--show",
        action="append",
        default=[],
        metavar="CONFIG",
        help=(
            "After running, visualize these configs interactively, e.g. "
            "'q_init', 'q_goal'. Repeatable. Use 'all' to list available."
        ),
    )
    parser.add_argument(
        "--print-goals",
        action="store_true",
        help="Print feasible goal-state strings and exit",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=50,
        help="Limit printed goal states (with --print-goals)",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help=(
            "Interactive mode: use arrow-key menus to select pairs and "
            "browse configurations"
        ),
    )
    parser.add_argument(
        "--graph-viz",
        nargs="?",
        const="/tmp/constraint_graph",
        metavar="PATH",
        help=(
            "Visualize constraint graph as PNG. Optionally specify output "
            "path (default: /tmp/constraint_graph). Requires matplotlib."
        ),
    )
    parser.add_argument(
        "--solve-mode",
        type=str,
        default="manipulation-planner",
        choices=["manipulation-planner", "transition-planner"],
        help="Planning algorithm to use when --solve is set",
    )


def add_grasp_sequence_arguments(parser: argparse.ArgumentParser) -> None:
    """
    Add arguments for grasp sequence planning mode.

    Adds the following arguments:
        --grasp-sequence: Plan a sequence of grasps
        --auto-save-dir: Directory to auto-save paths
        --load-paths: Load and replay saved paths

    Args:
        parser: ArgumentParser instance to add arguments to.
    """
    parser.add_argument(
        "--grasp-sequence",
        type=str,
        metavar="SEQUENCE",
        help=(
            "Plan a sequence of grasps using incremental phase graphs. "
            "Format: GRIPPER1:HANDLE1,GRIPPER2:HANDLE2,... "
            "Example: 'spacelab/g_ur10_tool:frame_gripper/h_FG_tool,"
            "spacelab/g_vispa:panel/h_panel'. Uses TransitionPlanner for "
            "each phase with minimal graph regeneration."
        ),
    )
    parser.add_argument(
        "--auto-save-dir",
        type=str,
        metavar="DIR",
        help=(
            "Directory to auto-save paths after each successful phase. "
            "Use with --grasp-sequence. Files are named phase_NN_edge_MM.path."
        ),
    )
    parser.add_argument(
        "--load-paths",
        type=str,
        metavar="DIR",
        help=(
            "Load and replay paths from a directory. "
            "Looks for files matching phase_*.path pattern."
        ),
    )


def parse_grasp_sequence(sequence_str: str) -> List[tuple]:
    """
    Parse a grasp sequence string into a list of (gripper, handle) tuples.

    Args:
        sequence_str: Comma-separated GRIPPER:HANDLE pairs.

    Returns:
        List of (gripper, handle) tuples.

    Example:
        >>> parse_grasp_sequence("g1:h1,g2:h2")
        [('g1', 'h1'), ('g2', 'h2')]
    """
    grasp_sequence = []
    for pair_str in sequence_str.split(","):
        pair_str = pair_str.strip()
        if ":" not in pair_str:
            print(
                f"Warning: Invalid pair format '{pair_str}', "
                "expected 'GRIPPER:HANDLE'"
            )
            continue
        gripper, handle = pair_str.split(":", 1)
        grasp_sequence.append((gripper.strip(), handle.strip()))
    return grasp_sequence


def parse_goal_pairs(pairs: List[str]) -> List[str]:
    """
    Convert --pair arguments to goal state strings.

    Args:
        pairs: List of "GRIPPER:HANDLE" strings.

    Returns:
        List of goal state strings like "gripper grasps handle".
    """
    goals = []
    for pair in pairs:
        if ":" in pair:
            gripper, handle = pair.split(":", 1)
            goals.append(f"{gripper.strip()} grasps {handle.strip()}")
        else:
            print(
                f"Warning: Invalid --pair format '{pair}', "
                "expected 'GRIPPER:HANDLE'"
            )
    return goals
