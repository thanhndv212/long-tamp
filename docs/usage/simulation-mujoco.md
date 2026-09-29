# Simulation (MuJoCo)

`long_tamp.sim.mjcf` exports a task's planning scene to MuJoCo. It reads the same YAML
task config the planner loads and writes one MJCF file whose bodies and joints keep HPP's
names, so a configuration planned by HPP maps straight onto the simulation. The MuJoCo execution
backend (below) runs planned paths in it under tracking control.

```bash
pip install "long-tamp[sim]"
python -m long_tamp.sim.mjcf script/screw_assembly/config/screw_assembly_config.yaml -o mjcf/
# wrote mjcf/screw_assembly.xml: 62 bodies, 29 joints (5 free), 10 mimic equalities, 49 meshes
python -m mujoco.viewer --mjcf=mjcf/screw_assembly.xml
```

```python
from long_tamp.sim.mjcf import export_mjcf, fk_mismatch, qpos_from_pinocchio

export = export_mjcf("config.yaml", "mjcf/", object_poses={"part1": [0.4, 0, 0.41, 0, 0, 0, 1]})
model = export.load()                                    # mujoco.MjModel
qpos = qpos_from_pinocchio(model, hpp_model, q)          # an HPP configuration, as MuJoCo qpos
errors = fk_mismatch(model, hpp_model, q)                # {body: pose error}
```

## What the export does

| In the task config | In MJCF |
|---|---|
| a robot or environment URDF | its links as bodies, fixed at the configured `pose` (identity by default), under `<name>/` |
| an object URDF | the same, with a free joint `<name>/root_joint` at its `initial_pose_xyzquat` (or `object_poses`) |
| `<mimic>` joints (the Robotiq fingers) | joint equality constraints |
| a link named `world` | `world_link` (MuJoCo reserves `world`, and would drop every transform above it) |
| mesh paths | resolved like the planner resolves them, COLLADA converted to STL, copied to `meshes/` next to the MJCF (the folder can move) |
| visual geometry | kept in geom group 1, without contacts; collision geometry is group 0 |

Quaternions are `x, y, z, qx, qy, qz, qw` in the config and in HPP, and `w, x, y, z` in
MuJoCo; `qpos_from_pinocchio` reorders them.

## Kinematics match HPP

`fk_mismatch` sets the same configuration in MuJoCo and in HPP's Pinocchio model and
compares the pose of every body both have. On the screw-assembly cell (60 bodies: both
arms and grippers, the fixtures, every part and the driver) the largest error is below
1e-7 m / rad, at the start configuration and at random ones
(`tests/test_sim_mjcf.py`).

## Executing plans in MuJoCo: `MuJoCoBackend`

`long_tamp.sim.MuJoCoBackend` is an execution backend ([Execution backends](execution.md))
that runs each planned path in the exported scene with a controller in the loop:

```python
from long_tamp.sim import MuJoCoBackend, QposMap, export_mjcf

export = export_mjcf("config.yaml", "run/mjcf")
backend = MuJoCoBackend(export, QposMap(export.load(), hpp_model), speed=math.inf)
executor = PlanExecutor(session, backend=backend)      # or run_command(backend, command)
```

The screw assembly runs on it with `--backend mujoco`:

```bash
python task_screw_assembly.py --seed 1 --no-viewer --backend mujoco --run-dir runs/sim
```

| | |
|---|---|
| Tracking control | a torque motor per driven joint (mimic followers excluded): PD on the path's position and velocity, gains scheduled on the joint's apparent inertia for a critically damped response at `bandwidth` (10 Hz), gravity and Coriolis compensated, torques limited to the URDF effort; damping runs through MuJoCo's implicit integrator |
| Grippers | the planner's finger joint is tracked like any joint; the Robotiq linkage follows through its mimic equalities |
| Grasps | welds, not contacts. At each command's start every object is welded to what carries it in the plan (a robot link it moves rigidly with, else the world). A new grasp snaps the object into the planner's grasp if it is within `grasp_tolerance` (2 cm); otherwise the command fails, "grasp missed". A release leaves the object where the simulation put it |
| Timing | planned paths keep their geometry but not always a followable timing (they may start at full speed, or turn corners instantly): each is retimed along its own parameter, rest to rest, never faster than planned, within the URDF velocity limits and `max_acceleration` (3 rad/s²), slowing through corners, then stretched until inverse dynamics needs at most `torque_margin` (80 %) of each joint's effort |
| Contacts | off by default (`contacts=True`): paths are collision-free in the planner and grasps are welds |
| Speed | `speed` simulated seconds per wall-clock second; `math.inf` runs each command in one poll |

The simulation starts at the first command's start configuration and then runs
continuously, so each command starts wherever the previous one left the robot and the
objects.

### Metrics

Each command's `motion` event ([Mission events](events.md)) carries what the backend
measured:

| Metric | Meaning |
|---|---|
| `tracking_error` | largest driven-joint error along the command [rad] |
| `drift` | joint error once the command has settled [rad] |
| `start_drift` | how far the robot was from the path's start when the command began [rad] |
| `object_drift` | largest object position error against the plan, at the end [m] |
| `grasp_error` | how far a newly grasped object was from its planned grasp [m] |
| `time_scale` | retimed duration over the planned one |
| `sim_seconds` | simulated time |
| `kinematic_objects` | objects that moved with nothing rigidly and were driven along their plan |

The screw assembly's `--summary` adds an `execution` block with the worst of each.

## Not yet

- Contact-rich grasping and screwing: grasps are welds. Contact grasps, and a screwing skill
  with a torque threshold, are #19.
- The arm and gripper models are the planner's URDFs; sourcing them, with their dynamics,
  from the vendors' packages is #67.
- Textures are not carried over (the drill's `.obj` keeps its shape, not its image).
