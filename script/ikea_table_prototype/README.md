# IKEA table assembly — long-horizon example

A long-horizon, multi-arm assembly example: two UR10+Robotiq-2F85 arms picking up and
docking four IKEA LACK table legs into their sockets on the tabletop. This is the
proven, multi-phase example `long_tamp` needs (see `docs/usage/behaviortree-integration.md`
§11) — built with real mesh geometry rather than placeholder primitives, sourced from
[clvrai/furniture](https://github.com/clvrai/furniture) (MIT-licensed; see
`research-vault/agimus-spacelab/long-tamp-example-assets-and-vlm-tamp-research.md` for the
full sourcing/licensing reasoning).

## What's here

- `assets/meshes/{leg1,leg2,leg3,leg4,table}.stl` — the LACK table meshes, vendored as-is
  from `clvrai/furniture`'s `furniture/env/models/assets/objects/table_lack_0825/`.
- `assets/textures/light-wood.png` — matching texture.
- `assets/reference_table_lack_0825.mjcf.xml` — the original MJCF definition (5 bodies: 4
  legs + tabletop), kept as a reference for connection-site geometry and collision box
  dimensions when authoring the URDF+SRDF version below.
- `build_assets.py` — the whole asset-generation pipeline (mesh rescaling, xacro
  expansion, URDF/SRDF authoring, arm assembly), run stage-by-stage or all at once;
  writes to `generated/`. See its own docstring for the stage list and run order.
- `config/ikea_table_config.yaml` — the task config (scene, joints, grasps, contacts),
  loaded via `YamlTaskLoader`.
- `task_assemble_table.py` — the actual grasp-sequence planning task.
- `debug_view_frames.py` — viser scene viewer with handle/gripper frames and an
  FK placement check; run inside the hpp-agimus-arm64 container.

## Status

Phase 1's pregrasp target (`ur10_right/gripper > leg1/handle`) converges reliably; the
remaining 11 phases (dock leg1's peg, release, then legs 2-4) are still being worked
through — see `task_assemble_table.py`'s module docstring for current status and
`docs/usage/behaviortree-integration.md` §11 for how this fits the broader roadmap.
