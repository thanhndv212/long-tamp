"""
Visualization utilities for agimus_spacelab manipulation tasks.

This module provides visualization functions:
- print_joint_info: Display joint information
- visualize_handle_frames: Add handle frame visualization
- visualize_constraint_graph: Generate static graph diagrams
- visualize_constraint_graph_interactive: Create live interactive graph windows
- VideoRecorder: Record path playback as video
- record_path_playback: Convenience function for video recording

Usage:
    from agimus_spacelab.visualization import (
        visualize_constraint_graph,
        visualize_constraint_graph_interactive,
        visualize_handle_frames,
        print_joint_info,
        VideoRecorder,
        record_path_playback,
    )
"""

from .viz import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_constraint_graph_interactive,
    displayHandleApproach,
    displayGripperApproach,
    displayHandle,
    displayGripper,
    visualize_all_handles,
    visualize_all_grippers,
    print_handle_info,
    print_gripper_info,
    clear_handle_visualizations,
    clear_gripper_visualizations,
    clear_all_visualizations,
)

from .video_recorder import (
    VideoRecorder,
    record_path_playback,
)


__all__ = [
    "print_joint_info",
    "visualize_constraint_graph",
    "visualize_constraint_graph_interactive",
    "displayHandleApproach",
    "displayGripperApproach",
    "displayHandle",
    "displayGripper",
    "visualize_all_handles",
    "visualize_all_grippers",
    "print_handle_info",
    "print_gripper_info",
    "clear_handle_visualizations",
    "clear_gripper_visualizations",
    "clear_all_visualizations",
    "VideoRecorder",
    "record_path_playback",
    "clear_all_visualizations",
]
