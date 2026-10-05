# TWIN bimanual examples

long_tamp ports of TWIN/PerAct2 bimanual benchmark tasks onto the HPP-based
grasp/motion planner (see `research-vault/papers/twin-benchmark.md`).

## Lift Ball

Two independent Franka Panda arms grasp a ball at two handles, then lift it
together.

```bash
python script/twin/task_lift_ball.py --backend pyhpp
python script/twin/task_lift_ball.py --show-joints
```

Needs only the PyHPP backend: every asset is vendored under `assets/`, and no
external robot-description package (`package://...`) has to resolve.

- `config/twin_lift_ball_config.yaml` holds the scene and planning
  configuration. Its asset paths are placeholders, and `task_lift_ball.py`
  overrides them with absolute paths at runtime.

## Assets

| File | Origin | Notes |
|---|---|---|
| `panda_bimanual.urdf.template`, `panda/meshes/` | Gepetto/example-robot-data `panda_description` (upstream frankaemika/franka_description, Apache 2.0) | Panda replaces the task's original UR5 + rigid `tool0` for its real 2-DOF parallel gripper. The template's mesh-dir placeholder is filled in at runtime, because pinocchio resolves relative mesh paths against the CWD. |
| `panda_bimanual.srdf` | authored here | HPP `<gripper>` semantics (upstream's SRDF is MoveIt-style). Reuses its `disable_collisions` pairs. |
| `pokeball_bimanual.urdf` / `.srdf` | hpp-practicals `ur_benchmark/pokeball` (LGPL v2, LAAS-CNRS) | Same 0.025 m sphere. The visual is a primitive (a pyhpp_viser mesh-scale bug squashes `pokeball.dae`), and a second handle is added for the dual-arm hold. |
| `ground_bimanual.urdf` | hpp-practicals `ur_benchmark/ground` (LGPL v2, LAAS-CNRS) | Same flat box, enlarged and re-centered to cover both arm bases. |

## Tests

- `tests/test_twin_examples.py` covers multi-robot loading, using two of the
  repo's vendored UR10s.
- `tests/test_grasp_release_use_case_twin.py` runs the full lift-ball scene.

Both skip without PyHPP.
