# Script templates

This folder holds copy/paste templates for creating new tasks and task configs.

## Create a new task

1) Copy the config template:

- From: `script/templates/task_config_template.py`
- To: `script/config/<your_task>_config.py`

2) Copy the task template:

- From: `script/templates/task_template.py`
- To: `script/spacelab/task_<your_task>.py` (or another package under `script/`)

3) Wire the task to the config

In your task file, update `initialize_task_config()` to import your config module
and select the right `TaskConfigurations.<YourClass>` (the template uses `MyTask`).

## Notes on grippers

Canonical config uses a nested schema:

- `ManipulationConfig.GRIPPERS[group_key] = {gripper_frame: gripper_joint}`

The config template shows how to extract `(GRIPPER_NAME, GRIPPER_JOINT)` while keeping `GRIPPERS = [GRIPPER_NAME]` for downstream usage.
