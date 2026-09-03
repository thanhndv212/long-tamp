#!/usr/bin/env python3
"""Bake the table/leg mesh scale into the mesh vertices themselves.

Local prototype only — see README.md. Same bug, third occurrence — see
rescale_robotiq_meshes.py's docstring for the first two (Robotiq meshes,
then ur10_right's pedestal via a different mechanism). generate_urdf_srdf.py
gives table.stl / leg{1..4}.stl non-unit <mesh scale> (0.015/0.015/0.02 for
the table, 0.02/0.02/0.02 for the legs — the raw IKEA STLs aren't authored
in meters), which hits pyhpp_viser's same "rendered position =
world_translation * meshScale" bug in _update_geometry_frames — confirmed
live: table/leg cached positions were each exactly target_position *
mesh_scale (e.g. table target (0.75, 0, 0.5) cached as
(0.01125, 0, 0.01) = (0.75*0.015, 0, 0.5*0.02)).

Fix: rescale the mesh vertices via trimesh (same as Robotiq), write to
meshes_scaled/, so generate_urdf_srdf.py can reference them with no
<mesh scale> attribute at all.
"""

from pathlib import Path

import trimesh

HERE = Path(__file__).parent
MESH_DIR = HERE / "assets" / "meshes"
SCALED_DIR = HERE / "assets" / "meshes_scaled"

# (source file, scale) — matches generate_urdf_srdf.py's current
# mesh_scale= arguments exactly.
FILES = {
    "table.stl": (0.015, 0.015, 0.02),
    "leg1.stl": (0.02, 0.02, 0.02),
    "leg2.stl": (0.02, 0.02, 0.02),
    "leg3.stl": (0.02, 0.02, 0.02),
    "leg4.stl": (0.02, 0.02, 0.02),
}


def main() -> None:
    SCALED_DIR.mkdir(exist_ok=True)
    for name, scale in FILES.items():
        src = MESH_DIR / name
        dst = SCALED_DIR / name
        mesh = trimesh.load(src, force="mesh")
        mesh.apply_scale(scale)
        mesh.export(dst)
        print(f"rescaled {name} (x{scale}) -> {dst.relative_to(HERE.parent.parent)}")


if __name__ == "__main__":
    main()
