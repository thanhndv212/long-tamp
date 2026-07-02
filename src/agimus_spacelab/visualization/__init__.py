"""
Visualization utilities for agimus_spacelab manipulation tasks.

This module provides visualization functions:
- print_joint_info: Display joint information
- visualize_all_handles: Add handle frame visualization
- visualize_constraint_graph: Generate static graph diagrams
- visualize_constraint_graph_interactive: Create live interactive graph windows
- VideoRecorder: Record path playback as video
- record_path_playback: Convenience function for video recording

Usage:
    from agimus_spacelab.visualization import (
        visualize_constraint_graph,
        visualize_constraint_graph_interactive,
        visualize_all_handles,
        print_joint_info,
        VideoRecorder,
        record_path_playback,
    )
"""

from .video_recorder import (
    VideoRecorder,
    record_path_playback,
)
from .viz import (
    clear_all_visualizations,
    clear_gripper_visualizations,
    clear_handle_visualizations,
    displayGripper,
    displayGripperApproach,
    displayHandle,
    displayHandleApproach,
    print_gripper_info,
    print_handle_info,
    print_joint_info,
    visualize_all_grippers,
    visualize_all_handles,
    visualize_constraint_graph,
    visualize_constraint_graph_interactive,
)

__all__ = [
    "VideoRecorder",
    "clear_all_visualizations",
    "clear_all_visualizations",
    "clear_gripper_visualizations",
    "clear_handle_visualizations",
    "displayGripper",
    "displayGripperApproach",
    "displayHandle",
    "displayHandleApproach",
    "print_gripper_info",
    "print_handle_info",
    "print_joint_info",
    "record_path_playback",
    "visualize_all_grippers",
    "visualize_all_handles",
    "visualize_constraint_graph",
    "visualize_constraint_graph_interactive",
]
