#!/usr/bin/env python3
"""Generate a box-with-holes table mesh and a box-with-peg leg mesh.

Local prototype only — see README.md. Fresh, programmatically-generated
geometry (box boolean-differenced/unioned with cylinders via trimesh) —
not derived from or copying the real vendored IKEA STL meshes in any way,
just built from the same box dimensions/socket positions already used for
collision (generate_urdf_srdf.py's LEG_HALF_EXTENT / TABLE_HALF_EXTENT /
LEG_SOCKET_XY).

Visual only. Collision stays the plain convex box in generate_urdf_srdf.py
— HPP-FCL's narrow-phase collision checking wants convex shapes, and a
box-with-holes is non-convex; visual/collision geometry not matching
exactly is standard practice (detailed visual mesh, simple convex
collision proxy), same pattern already used for e.g. UR10's own meshes
(separate visual .dae / collision .stl, different tessellation).

Requires a trimesh boolean backend (this repo's HPP env didn't ship one
by default): pip install manifold3d.
"""

from pathlib import Path

import trimesh

HERE = Path(__file__).parent
OUT_DIR = HERE / "assets" / "meshes_generated"

TABLE_HALF_EXTENT = (0.32, 0.12, 0.02)
LEG_HALF_EXTENT = (0.015, 0.015, 0.13125)
LEG_SOCKET_XY = [
    (-0.305, -0.095),
    (-0.305, 0.095),
    (0.305, -0.095),
    (0.305, 0.095),
]

HOLE_RADIUS = 0.012
HOLE_DEPTH = 0.015  # blind hole, out of the table's 0.04 total thickness
PEG_RADIUS = 0.010
PEG_HEIGHT = 0.020  # protrudes above the leg's top face


def build_table() -> trimesh.Trimesh:
    tx, ty, tz = TABLE_HALF_EXTENT
    box = trimesh.creation.box(extents=(2 * tx, 2 * ty, 2 * tz))

    cutters = []
    for cx, cy in LEG_SOCKET_XY:
        cyl = trimesh.creation.cylinder(radius=HOLE_RADIUS, height=0.06)
        # Cylinder is centered at its own origin; place it so it starts
        # below the table's bottom face (-tz) and reaches HOLE_DEPTH up
        # into the material — the portion below -tz has no box material
        # to subtract, so only the overlapping part actually carves in.
        cyl.apply_translation((cx, cy, -tz + HOLE_DEPTH - 0.03))
        cutters.append(cyl)

    result = trimesh.boolean.difference([box, *cutters])
    if not result.is_watertight:
        raise RuntimeError("table-with-holes mesh is not watertight")
    return result


def build_leg_with_peg() -> trimesh.Trimesh:
    lx, ly, lz = LEG_HALF_EXTENT
    box = trimesh.creation.box(extents=(2 * lx, 2 * ly, 2 * lz))
    peg = trimesh.creation.cylinder(radius=PEG_RADIUS, height=PEG_HEIGHT)
    peg.apply_translation((0, 0, lz + PEG_HEIGHT / 2))
    result = trimesh.boolean.union([box, peg])
    if not result.is_watertight:
        raise RuntimeError("leg-with-peg mesh is not watertight")
    return result


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)

    table = build_table()
    table_path = OUT_DIR / "table_with_holes.stl"
    table.export(table_path)
    print(f"wrote {table_path.relative_to(HERE.parent.parent)} "
          f"({len(table.vertices)} vertices, watertight={table.is_watertight})")

    leg = build_leg_with_peg()
    leg_path = OUT_DIR / "leg_with_peg.stl"
    leg.export(leg_path)
    print(f"wrote {leg_path.relative_to(HERE.parent.parent)} "
          f"({len(leg.vertices)} vertices, watertight={leg.is_watertight})")


if __name__ == "__main__":
    main()
