# Script templates

This folder holds copy/paste templates for creating new manipulation tasks.

There are two styles — **YAML-driven** (recommended for all new work) and the
older **Python-config** style (kept for reference).

---

## YAML-driven templates (recommended)

The YAML style is fully robot-agnostic.  No Python file in the framework needs
to know your robot name, joint names, or file paths.

### Quick-start

```
script/
  config/
    my_robot_config.yaml    ← copy from task_config_template.yaml
  my_robot/
    task_my_task.py         ← copy from task_yaml_template.py
```

**Step 1 — Write a YAML config**

Copy `task_config_template.yaml` to `script/config/<your_robot>_config.yaml`.
Every `<PLACEHOLDER>` must be replaced:

| Placeholder | What to put there |
|---|---|
| `<robot_name>` | Name used in the composite URDF (e.g. `ur5`) |
| `<pkg>` | ROS package containing the URDF/SRDF files |
| `<env_name>` | Name of the static environment link |
| `<object_name>` | Name of each manipulated freeflyer object |
| `<robot_name>/gripper` | Gripper frame name from the robot SRDF |
| `<object_name>/handle` | Handle frame name from the object SRDF |

Not sure what joint/gripper names to use? Run the task with `--show-joints`
after filling in the URDF paths (see Step 3).

**Step 2 — Write a task script**

Copy `task_my_task.py` to `script/<your_robot>/task_<name>.py`.
This file is a concrete, ready-to-run template — all five editable sections
are clearly marked with `# <-- EDIT`.  Alternatively, `task_yaml_template.py`
is a more abstract version with `<PLACEHOLDER>` markers and longer comments.

Edit the five `# <-- EDIT` sections at the top of the file:

```python
TASK_NAME        = "My Robot: Pick and Place"
_YAML_PATH       = Path(...) / "config" / "my_robot_config.yaml"
GRASP_GOALS      = ["my_robot/gripper grasps my_object/handle"]
GRASP_SEQUENCE   = [("my_robot/gripper", "my_object/handle")]
FREEZE_JOINT_SUBSTRINGS  = []
COLLISION_EXCLUSIONS     = []
```

The rest of the file (setup, planning, replay menu) works without modification.

**Step 3 — Run**

```bash
# Print joint names to find correct values for joint_groups in the YAML:
python script/my_robot/task_my_task.py --show-joints

# Plan with the PyHPP backend (no CORBA server needed):
python script/my_robot/task_my_task.py --backend pyhpp

# Plan with the CORBA backend:
python script/my_robot/task_my_task.py --backend corba
```

### Real-world examples

| Config | Task script | Description |
|---|---|---|
| `script/config/spacelab_config.yaml` | `script/spacelab/task_grasp_FG_yaml.py` | SpaceLab multi-arm — complex reference |

---

## Python-config templates (legacy)

The older Python-based approach requires a `spacelab_config.py` to exist.
Use these only when maintaining existing SpaceLab-specific scripts.

**Files**

- `task_config_template.py` — Python config class template (SpaceLab-specific imports)
- `task_template.py` — Python task script template (imports `ManipulationConfig`)

**Steps**

1. Copy `task_config_template.py` → `script/config/<your_task>_config.py`
2. Copy `task_template.py` → `script/spacelab/task_<your_task>.py`
3. In the task file update `initialize_task_config()` to import your config
   module and select `TaskConfigurations.<YourClass>`.

### Notes on grippers (Python config style)

Canonical config uses a nested schema:

- `ManipulationConfig.GRIPPERS[group_key] = {gripper_frame: gripper_joint}`

The config template shows how to extract `(GRIPPER_NAME, GRIPPER_JOINT)` while
keeping `GRIPPERS = [GRIPPER_NAME]` for downstream usage.
