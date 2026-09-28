# Simulation (MuJoCo)

`long_tamp.sim.mjcf` exports a task's planning scene to MuJoCo. It reads the same YAML
task config the planner loads and writes one MJCF file whose bodies and joints keep HPP's
names, so a configuration planned by HPP maps straight onto the simulation. This is the
first step of M4 (execution in simulation); the MuJoCo execution backend builds on it.

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

## Not yet

The export has no actuators, controllers or tuned contact parameters: that is the MuJoCo
execution backend (#18). Textures are not carried over (the drill's `.obj` keeps its shape,
not its image).
