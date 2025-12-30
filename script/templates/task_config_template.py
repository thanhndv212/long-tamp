"""Task config template for agimus_spacelab scripts.

Copy this file into `script/config/<your_task>_config.py` and adapt.

Design goals:
- Canonical config lives in `src/agimus_spacelab/config/spacelab_config.py`.
- Script-level task configs inherit canonical defaults via
    `SpacelabTaskDefaults`.
- Avoid name collisions by using *_INFO aliases when needed.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

from agimus_spacelab.config import ManipulationConfig


class SpacelabTaskDefaults(ManipulationConfig):
    """Shared defaults for script task configs."""


# Alias canonical dicts to avoid collisions with task-local names.
GRIPPERS_INFO = ManipulationConfig.GRIPPERS
OBJECTS_INFO = ManipulationConfig.OBJECTS
VALID_PAIRS_INFO = ManipulationConfig.VALID_PAIRS


def _extract_single_gripper_frame_and_joint(
    gripper_group: str,
) -> Tuple[str, str]:
    """Return (gripper_frame, gripper_joint) from canonical nested GRIPPERS.

    Canonical schema:
    - GRIPPERS[group_key] = {gripper_frame: gripper_joint}

    If you later support multiple gripper frames per group, adapt this helper.
    """
    mapping = GRIPPERS_INFO[gripper_group]
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError(
            f"GRIPPERS['{gripper_group}'] must be a non-empty dict"
        )
    gripper_frame, gripper_joint = next(iter(mapping.items()))
    return gripper_frame, gripper_joint


class TaskConfigurations:
    """Container for task configs used by scripts."""

    class MyTask(SpacelabTaskDefaults):
        """Rename this class to match your task (e.g., PickPlace, ...)."""

        # --- Task identity
        TASK_ID = "<your_task_id>"

        # --- Scene selection
        # Match the script-level task configs in
        # `script/config/spacelab_config.py`.
        ROBOTS = ["UR10"]
        OBJECTS = ["part"]

        # --- Grippers
        _GRIPPER_FRAME_TO_JOINT = GRIPPERS_INFO["ur10"]
        GRIPPER_NAME, GRIPPER_JOINT = _extract_single_gripper_frame_and_joint(
            "ur10"
        )
        GRIPPERS = [GRIPPER_NAME]

        # --- Object metadata (derive from canonical OBJECTS schema)
        HANDLES_PER_OBJECT = [OBJECTS_INFO[obj]["handles"] for obj in OBJECTS]
        CONTACT_SURFACES_PER_OBJECT = [
            OBJECTS_INFO[obj]["contact_surfaces"] for obj in OBJECTS
        ]

        # --- Planning constraints
        VALID_PAIRS: Dict[str, List[str]] = VALID_PAIRS_INFO

        # --- Optional execution parameters
        PATH_VALIDATION_STEP = 0.01
        PATH_PROJECTOR_STEP = 0.1
        MAX_RANDOM_ATTEMPTS = 200
