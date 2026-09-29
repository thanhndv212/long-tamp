# Skills

A motion-planned capability moves the robot along collision-free paths. Some steps can't be
planned to the millimetre: driving a screw, inserting a peg, pressing a button. A **skill**
splits such a step in two:

1. **Planned motion** brings the robot to the skill's *start* pose, for example the driver's
   tip on the hole's axis, a couple of centimetres out. The planner does this, as for any
   capability.
2. **The skill** runs from there on the robot's controller (compliant approach, force and
   torque thresholds) to its *end* pose. It reports either its **postconditions**, as facts
   that now hold, or a **failure fact** that the task planner can plan around.

Design: [ADR-0001](../adr/0001-planner-refiner-executor.md).

## Declaring a skill

```python
from long_tamp.tasks.task_planning.skills import SkillPose, SkillSpec

SCREW = SkillSpec(
    name="screw",
    parameters=("tool", "part", "hole"),
    postconditions=("screwed(?part, ?hole)",),
    failures=("screw_misaligned(?tool, ?hole)", "screw_no_contact(?tool, ?hole)"),
    start=SkillPose("?tool", "?hole", offset=-0.02),   # 2 cm before the hole
    end=SkillPose("?tool", "?hole"),                   # at the hole
)
```

| Field | Meaning |
|---|---|
| `parameters` | the skill's arguments; conditions may only use these |
| `preconditions` / `postconditions` | literals, the same syntax as capability preconditions and effects |
| `failures` | failure-fact templates the skill may report instead of its postconditions |
| `start` / `end` | where planned motion leaves the robot, and where the skill does: `frame` relative to `target`, `offset` metres along the target's approach axis |

`SCREW.descriptor()` turns the skill into a `CapabilityDescriptor` (its preconditions, its
postconditions as effects), so a TaskPlan or the PDDL export can use it like any capability.

## Running a skill

The executor sends a backend a `SkillCommand` as the payload of an `ExecutionCommand`. The
command holds the spec, the grounded parameters, and `approach`, the planned path from the
start pose to the end pose.

```python
from long_tamp.execution import ExecutionCommand
from long_tamp.tasks.task_planning.skills import SkillCommand

command = SkillCommand(SCREW, {"tool": "driver", "part": "part1", "hole": "part1/h_hole1"},
                       approach=insertion_path)
executor.submit(ExecutionCommand("part1 A", duration=insertion_path.length(), payload=command))
```

- A backend that runs the skill reports its outcome in `Feedback.facts`: the grounded
  postconditions on success, or the failure fact on failure. It also reports its own metrics.
- Any other backend executes `approach` as ordinary motion: a `SkillCommand` is also its path
  (`length()`, `eval(t)`). So the same mission runs with `--backend playback` or `mock`.
- With a backend, a step's recorded effects (such as `screwed`) are written only once its
  motion, skills included, has executed. A step whose skill failed records nothing.
- A failed skill's fact comes back in the mission's `failure.facts`, where the repair loop
  reads planning failures ([Automatic task planning](task-planning-pddl.md#replanning-around-failures)).

## In MuJoCo: `ScrewDriving`

`long_tamp.sim.ScrewDriving` is the screwing stub for the [MuJoCo backend](simulation-mujoco.md):

```python
backend = MuJoCoBackend(export, to_qpos, skills={"screw": ScrewDriving()})
```

Grasps are welds and the screw isn't geometry, so the screw is *virtual*. The arm and the
controller are real:

1. **Approach.** The driver follows the planned approach at `feed` (2 cm/s). The tracking
   controller's stiffness is lowered to `bandwidth` (4 Hz), which makes it compliant.
2. **Touch.** `thread_length` (8 mm) before the end pose, the screw meets the hole. If the
   driver is more than `align_tolerance` (2 mm) off the hole's axis, the skill fails with
   `screw_misaligned(tool, hole)`.
3. **Drive.** The screw pushes back along the axis (`contact_force`, 15 N) and twists the
   driver's housing (reaction torque growing with depth). The arm feeds forward the thrust and
   holding torque it expects (`Jᵀ w`), which is hybrid force/position control.
4. **Seated.** At `torque_threshold` (2 N m), the skill reports `screwed(part, hole)`. If the
   driver never reaches the touch point, it fails with `screw_no_contact(tool, hole)`.

`hole_error` places the real hole away from the planned one, to model a perception error. The
screw assembly exposes it as `--hole-error X Y Z` (mm).

Metrics in the `motion` event: `screw_depth`, `screw_torque`, `lateral_error`,
`contact_force`, plus the backend's tracking metrics.

```bash
python task_screw_assembly.py --seed 1 --no-viewer --backend mujoco        # screws driven
python task_screw_assembly.py --seed 1 --no-viewer --backend mujoco --hole-error 0 5 0
# => screw_misaligned(driver, part1/h_hole1): 5.0 mm off the hole's axis
```

## Not yet

- Contact-based screwing, with the screw as geometry and finger grasps as contacts, needs the
  vendor gripper models (#67).
- The planner doesn't yet use a skill's `start` offset: in the screw assembly, HPP's pregrasp
  plays that role.
