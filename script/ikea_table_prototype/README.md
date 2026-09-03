# IKEA table assembly — local prototype (NOT for public release)

**This directory exists only on the `local/ikea-furniture-prototype` branch, which must
never be pushed to `origin` or merged into `main`.** It vendors real mesh assets derived
from IKEA's LACK table product line, sourced from
[clvrai/furniture](https://github.com/clvrai/furniture) (MIT-licensed code; the 3D assets
themselves are real product replicas, not covered by that license in the same way — see
`research-vault/agimus-spacelab/long-tamp-example-assets-and-vlm-tamp-research.md` for the
full reasoning). Using them here is a deliberate, scoped exception for private local
prototyping only, to validate the new long-horizon example's task structure against a real
reference before authoring fresh, safe-to-publish geometry.

## What's here

- `assets/meshes/{leg1,leg2,leg3,leg4,table}.stl` — the actual LACK table meshes, vendored
  as-is from `clvrai/furniture`'s `furniture/env/models/assets/objects/table_lack_0825/`.
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

## Before this ever goes near `main` or a public release

Do not merge this branch. The path to a publishable version is a **separate**, freshly
authored box-primitive table (same part count and connection topology — 4 legs + 1
tabletop, same collision-box dimensions already extracted from the MJCF above) with no
IKEA mesh files and no IKEA product naming — see the vault note for the box-primitive
option that was the alternative to this branch.
