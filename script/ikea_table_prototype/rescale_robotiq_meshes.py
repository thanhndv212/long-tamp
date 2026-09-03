#!/usr/bin/env python3
"""Bake Robotiq's 0.001 mesh scale into the mesh vertices themselves.

Local prototype only — see README.md. The vendored xacro sources declare
<mesh scale="0.001 0.001 0.001"/> (their DAE/STL files are authored in
millimeters). pyhpp_viser's Viewer._update_geometry_frames() has a known
bug (see script/twin/assets/pokeball_bimanual.urdf's header comment):
rendered position = world_translation * meshScale, when meshScale should
only scale local vertices. Any non-unit <mesh scale> collapses that
geometry down near the world origin regardless of its real joint pose —
confirmed live: Robotiq's parts (scale 0.001) all render within ~1mm of
origin while UR10's own parts (scale 1, unaffected) render correctly.

Fix (same one the pokeball example already applied): eliminate the
non-unit scale by rescaling the mesh data itself via trimesh, so the URDF
can reference it with no <mesh scale> attribute at all — same convention
UR10's meshes already use.

Run generate_robotiq_urdf.sh again after this to rebuild
robotiq_2f85_generated.urdf pointing at the rescaled meshes (this script
rewrites the *_model_macro.xacro / *.xacro sources' scale/filename
in-place, same way patch step in generate_robotiq_urdf.sh's docstring
already documents editing them).
"""

from pathlib import Path

import trimesh

HERE = Path(__file__).parent
MESH_DIR = HERE / "assets" / "robotiq_2f85" / "meshes"
SCALED_DIR = HERE / "assets" / "robotiq_2f85" / "meshes_scaled"
SCALE = 0.001


def rescale_one(src: Path, dst: Path) -> None:
    mesh = trimesh.load(src, force="mesh")
    mesh.apply_scale(SCALE)
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(dst)
    print(f"rescaled {src.name} -> {dst.relative_to(HERE.parent.parent)}")


# robotiq_arg2f_base_link.stl is the one file the source xacro references
# with NO <mesh scale> — already authored in meters, unlike every other
# Robotiq mesh here (all in millimeters). Must not be rescaled again.
ALREADY_METERS = {"robotiq_arg2f_base_link.stl"}


def main() -> None:
    for sub in ("collision", "visual"):
        for src in sorted((MESH_DIR / sub).iterdir()):
            dst = SCALED_DIR / sub / src.name
            if src.name in ALREADY_METERS:
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(src.read_bytes())
                print(f"copied (already meters) {src.name}")
                continue
            rescale_one(src, dst)


if __name__ == "__main__":
    main()
