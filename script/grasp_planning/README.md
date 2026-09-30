# Grasp planning

`long_tamp.grasping` is the grasp planner: a module of its own, next to the motion
planner rather than inside it. The motion planner (HPP) moves the gripper frame onto a
handle frame and treats the grasp as a rigid constraint, so the finger joints stay frozen
open in every planned path. The grasp planner answers the two questions around that:

1. **Which handle?** It samples antipodal grasps on an object's collision primitives,
   rejects those that don't fit the stroke, sit off-centre, or collide, and ranks the rest.
   A planned grasp exports as an SRDF `<handle>` the motion planner uses unchanged.
2. **How far do the fingers close?** At any grasp pose, including hand-written handles, it
   closes the fingers on the object and returns the finger joint values: for the viewer,
   a trajectory, or a gripper controller.

A behaviour tree (or any orchestrator) runs it as a step of its own: `plan_grasp` before
motion planning, `close_gripper` / `open_gripper` after (see
`long_tamp/tasks/task_planning/grasp_capability.py`).

## Scripts

```bash
# rank grasps on an object and check its existing handles
python3 plan_grasps.py ../screw_assembly/generated/driver.urdf \
    --srdf ../screw_assembly/generated/driver.srdf
python3 plan_grasps.py ../ikea_table_prototype/generated/leg1.urdf --top 3 --emit-srdf
python3 plan_grasps.py ../twin/assets/pokeball_bimanual.urdf --gripper panda_hand
python3 plan_grasps.py <urdf> --prefer 0 0 -1        # favour top-down approaches

# check closures against the real Robotiq meshes (pinocchio + coal)
python3 validate_closure.py --plan driver --top 10
```

## Library

```python
from long_tamp.grasping import ROBOTIQ_2F85, GraspableObject, GraspPlanner, FingerClosureTable

drill = GraspableObject.from_urdf("driver", "driver.urdf", "driver.srdf")
planner = GraspPlanner(ROBOTIQ_2F85)
best = planner.plan(drill.primitives)[0]          # ranked GraspCandidate
print(best.srdf_handle("h_new"))                  # paste into the SRDF
ev = planner.evaluate_handle(drill.handle_pose("h_grip"), drill.primitives)
ev.width, ev.q, ev.feasible                       # 0.036, 0.484, True

# every (gripper, handle) pair of a task, for playback or a controller
closures = FingerClosureTable.from_task_yaml(
    "config.yaml", {"ur10_left/gripper": ROBOTIQ_2F85, "ur10_right/gripper": ROBOTIQ_2F85})
closures.closed_values("ur10_right/gripper", "driver/h_grip")  # {"ur10_right/robotiq_85_left_knuckle_joint": 0.484, ...}
```

## How it works, and how far to trust it

- **Hand model.** Canonical frame: +X approach, +Y closing, origin at the TCP.
  `ROBOTIQ_2F85`'s stroke table (driving value → pad gap and pad depth) and the envelope
  of the knuckles between the pads come from forward kinematics of
  `../ikea_table_prototype/generated/ur10_robotiq.urdf` (`calibrate_from_urdf`). Palm, pads
  and linkage are boxes. `PANDA_HAND`'s joint mapping is exact, its sizes nominal.
- **Closing.** A grid of rays across each pad, along the closing axis. The TCP is held
  rigidly at the handle, so both fingers close symmetrically and stop when the first one
  touches: an off-centre grasp leaves the other finger short (`offset`), and is rejected
  past 4 mm. The command is 2 mm past contact, for grip force.
- **Geometry.** URDF collision boxes, cylinders and spheres, composed through fixed
  joints. A mesh collision is replaced by its oriented bounding box (coarse, but on the
  safe side).
- **Checked against the real meshes.** `validate_closure.py` places the actual 2F-85
  collision meshes at each grasp. At the planned closure each pad is 1.0 mm into the
  surface, as commanded, on every object tried. Over every accepted grasp on the drill,
  a part, an IKEA leg, the table top and the pokeball (478 grasps), none collided
  anywhere else. The box hull is conservative: it also rejects some grasps the real hand
  would clear.
- **Not modelled.** Friction and grasp-wrench quality (a centroid-distance score stands
  in), the fingers' arc while closing (the pads are cast at their final depth), and
  anything outside the object and the given `obstacles`. HPP still checks the arm's
  path; this module only decides the grasp and the fingers.
