"""
Domain-specific interactive pickers for agimus_spacelab tasks.

This module provides interactive selection utilities for common task
operations like selecting grasp pairs, frozen arms, and browsing
configurations.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Any

from agimus_spacelab.utils.interactive import interactive_menu


__all__ = [
    "select_grasp_pairs",
    "select_frozen_arms",
    "browse_configurations",
]


def select_grasp_pairs(cfg: Any) -> List[str]:
    """
    Interactively select gripper-handle pairs from a task configuration.

    Uses arrow-key navigation to allow multi-selection of grasp pairs.
    Includes a "Select All" option at the top.

    Args:
        cfg: Task configuration with feasible_grasp_goal_states() method.

    Returns:
        List of selected goal state strings. Empty list if no selection
        or user quits.

    Example:
        >>> from spacelab_config import TaskConfigurations
        >>> cfg = TaskConfigurations.DisplayAllStates
        >>> selected_goals = select_grasp_pairs(cfg)
        >>> print(f"Selected {len(selected_goals)} pairs")
    """
    all_goals = list(cfg.feasible_grasp_goal_states())
    if not all_goals:
        print("No valid pairs available.")
        return []

    # Add "Select All" option at the top
    menu_options = ["[Select All]"] + all_goals

    selected = interactive_menu(
        "Select grasp pair(s) to include in graph:",
        menu_options,
        multi_select=True,
    )

    if not selected:
        print("No pairs selected, using all available pairs.")
        return []

    # Check if "Select All" (index 0) was selected
    if 0 in selected:
        print(f"Selected all {len(all_goals)} pairs.")
        return all_goals

    # Map back to actual goals (subtract 1 for the "Select All" option)
    return [all_goals[i - 1] for i in selected if i > 0]


def select_frozen_arms(
    default_substrings: List[str],
    arm_options: Optional[List[str]] = None,
) -> List[str]:
    """
    Interactively select joint-name substrings to freeze during planning.

    Options map directly to substrings searched in joint names.

    Args:
        default_substrings: Substrings to use if user quits without selecting.
        arm_options: Available arm substrings. Defaults to common SpaceLab arms.

    Returns:
        List of selected substrings. Returns default_substrings if user quits.

    Example:
        >>> frozen = select_frozen_arms(["vispa_"], arm_options=["vispa_", "ur10"])
        >>> print(f"Will freeze joints containing: {frozen}")
    """
    if arm_options is None:
        arm_options = ["vispa_", "vispa2", "ur10"]

    initial_selected = [
        i for i, opt in enumerate(arm_options) if opt in set(default_substrings)
    ]

    selected = interactive_menu(
        "Select arm(s) to lock (freeze joints):",
        arm_options,
        multi_select=True,
        selected=initial_selected,
    )

    if not selected:
        return list(default_substrings)

    return [arm_options[i] for i in selected if 0 <= i < len(arm_options)]


def browse_configurations(
    task: Any,
    configs: Dict[str, List[float]],
) -> None:
    """
    Interactively browse and visualize configurations.

    Presents a menu of available configurations and visualizes the
    selected one using the task's planner.

    Args:
        task: Task instance with planner.visualize() method.
        configs: Dictionary mapping config names to configuration vectors.

    Example:
        >>> configs = {"q_init": [...], "q_goal": [...]}
        >>> browse_configurations(task, configs)
    """
    if not configs:
        print("No configurations available.")
        return

    config_names = sorted(configs.keys())

    while True:
        selected = interactive_menu(
            "Select configuration to visualize (q to quit):",
            config_names + ["[Exit]"],
            multi_select=False,
        )

        if not selected or selected[0] >= len(config_names):
            break

        name = config_names[selected[0]]
        print(f"\nVisualizing '{name}'...")
        task.planner.visualize(configs[name])
        input("Press Enter to continue...")


def select_skip_phases(
    grasp_sequence: List[tuple],
) -> tuple:
    """
    Interactively select phases to skip during grasp sequence planning.

    Args:
        grasp_sequence: List of (gripper, handle) tuples representing the sequence.

    Returns:
        Tuple of (skip_phases: set, skip_all: bool).
        skip_phases contains indices of phases to skip.
        skip_all is True if all phases should be skipped (config generation only).
    """
    if len(grasp_sequence) < 1:
        return set(), False

    print("\n=== Skip Phase Configuration (Optional) ===")
    print("Skip motion planning for selected phases.")
    print("Config generation will still execute; useful for testing.")

    skip_phase_options = (
        [
            "[Skip ALL phases - Config generation only]",
        ]
        + [
            f"Phase {i+1}: {gripper} → {handle}"
            for i, (gripper, handle) in enumerate(grasp_sequence)
        ]
        + ["[No phases to skip]"]
    )

    skip_selected = interactive_menu(
        "Select phases to skip motion planning:",
        skip_phase_options,
        multi_select=True,
    )

    # Check if "Skip ALL phases" (index 0) was selected
    if skip_selected and 0 in skip_selected:
        skip_phases = set(range(len(grasp_sequence)))
        print(
            f"\nSkipping ALL {len(grasp_sequence)} phases - Config generation only"
        )
        return skip_phases, True

    # Collect individual phase indices (adjust for "Skip ALL" option at index 0)
    if skip_selected:
        last_option_idx = len(skip_phase_options) - 1
        skip_phases = set(
            [
                idx - 1  # Adjust for "Skip ALL" option at index 0
                for idx in skip_selected
                if 0 < idx < last_option_idx  # Between "Skip ALL" and "No phases"
            ]
        )
        if skip_phases:
            print(f"\nWill skip motion planning for {len(skip_phases)} phase(s):")
            for idx in sorted(skip_phases):
                gripper, handle = grasp_sequence[idx]
                print(f"  Phase {idx+1}: {gripper} → {handle}")
        else:
            print("\nNo phases selected to skip.")
        return skip_phases, False

    print("\nNo phases will be skipped.")
    return set(), False


def select_frozen_arms_mode(
    grasp_sequence: Optional[List[tuple]] = None,
) -> tuple:
    """
    Interactively select the frozen arms mode for planning.

    Args:
        grasp_sequence: Optional grasp sequence for manual mode configuration.

    Returns:
        Tuple of (mode: str, per_phase_frozen_arms: Optional[dict]).
        Mode is one of: "auto", "manual", "interactive", "global", "none".
        per_phase_frozen_arms is populated only for "manual" mode.
    """
    frozen_mode_options = [
        "auto - Freeze all arms except active gripper's arm",
        "manual - Specify which arms to freeze per phase",
        "interactive - Prompt per phase which arms to freeze",
        "global - Use task's global locked joint constraints",
        "none - No locked joint constraints",
    ]

    print("\nSelect locked joint constraint mode:")
    mode_selected = interactive_menu(
        "Choose mode:",
        frozen_mode_options,
        multi_select=False,
    )

    if not mode_selected:
        return "auto", None

    mode_map = ["auto", "manual", "interactive", "global", "none"]
    frozen_arms_mode = mode_map[mode_selected[0]]
    per_phase_frozen_arms = None

    # Manual mode: collect per-phase specifications
    if frozen_arms_mode == "manual" and grasp_sequence:
        per_phase_frozen_arms = {}
        arm_keywords = ["ur10", "vispa_", "vispa2"]

        for phase_idx, (gripper, handle) in enumerate(grasp_sequence):
            print(f"\nPhase {phase_idx + 1}: {gripper} → {handle}")
            print("Select arm(s) to freeze:")

            selected_arms = interactive_menu(
                "Select arms:",
                arm_keywords + ["[None - No Locking]"],
                multi_select=True,
            )

            arm_count = len(arm_keywords)
            if selected_arms and selected_arms[0] < arm_count:
                frozen = [
                    arm_keywords[i] for i in selected_arms if i < arm_count
                ]
                per_phase_frozen_arms[phase_idx] = frozen
                print(f"  Freezing: {frozen}")
            else:
                print("  No locked joints for this phase")

    return frozen_arms_mode, per_phase_frozen_arms


def select_auto_save_directory() -> Optional[str]:
    """
    Interactively select auto-save directory for paths.

    Returns:
        Directory path string, or None if auto-save disabled.
    """
    print("\n=== Path Auto-Save Configuration ===")
    save_options = [
        "No auto-save",
        "Auto-save to default directory (/tmp/grasp_sequence_paths)",
        "Auto-save to custom directory",
    ]
    save_selected = interactive_menu(
        "Save paths after each successful phase?",
        save_options,
        multi_select=False,
    )

    if save_selected and save_selected[0] == 1:
        auto_save_dir = "/tmp/grasp_sequence_paths"
        print(f"Auto-save enabled: {auto_save_dir}")
        return auto_save_dir
    elif save_selected and save_selected[0] == 2:
        custom_dir = input("Enter directory path: ").strip()
        if custom_dir:
            print(f"Auto-save enabled: {custom_dir}")
            return custom_dir
        print("No directory specified, auto-save disabled")

    return None
