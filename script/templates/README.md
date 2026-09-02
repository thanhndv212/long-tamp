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

# Plan with the PyHPP backend:
python script/my_robot/task_my_task.py --backend pyhpp
```

### Real-world examples

| Config | Task script | Description |
|---|---|---|
| `script/twin/config/twin_lift_ball_config.yaml` | `script/twin/task_lift_ball.py` | Bimanual multi-arm — complex reference |

### Notes on grippers

Canonical config uses a nested schema:

- `ManipulationConfig.GRIPPERS[group_key] = {gripper_frame: gripper_joint}`

Extract `(GRIPPER_NAME, GRIPPER_JOINT)` from that nested form while keeping
`GRIPPERS = [GRIPPER_NAME]` for downstream usage.
