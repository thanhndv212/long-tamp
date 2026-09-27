#!/usr/bin/env python3
"""Plan parallel-jaw grasps on an object and check its existing handles.

    python3 plan_grasps.py ../screw_assembly/generated/driver.urdf \\
        --srdf ../screw_assembly/generated/driver.srdf        # rank + check handles
    python3 plan_grasps.py ../ikea_table_prototype/generated/leg1.urdf --top 3 --emit-srdf
    python3 plan_grasps.py ../twin/assets/pokeball_bimanual.urdf --gripper panda_hand
    python3 plan_grasps.py <urdf> --prefer 0 0 -1 --json out.json   # top-down first

For each existing ``<handle>`` of ``--srdf`` it prints how far the fingers
close there and whether the grasp holds; then the best new grasps, ranked.
``--emit-srdf`` prints them as ``<handle>`` elements ready to paste into the
object's SRDF. No HPP needed: only the object's URDF collision geometry.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from long_tamp.grasping import (
    PRESETS,
    GraspableObject,
    GraspPlanner,
    GraspPlannerParams,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--srdf", type=Path, help="check this SRDF's handles too")
    ap.add_argument("--gripper", default="robotiq_2f85", choices=sorted(PRESETS))
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument(
        "--prefer",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="preferred approach direction, object frame",
    )
    ap.add_argument("--emit-srdf", action="store_true", help="print <handle> elements")
    ap.add_argument("--json", type=Path, help="write handles + candidates here")
    args = ap.parse_args()

    model = PRESETS[args.gripper]
    params = GraspPlannerParams(
        preferred_approach=tuple(args.prefer) if args.prefer else None
    )
    planner = GraspPlanner(model, params)
    name = args.urdf.stem
    obj = GraspableObject.from_urdf(name, args.urdf, args.srdf)
    print(
        f"{name}: {len(obj.primitives)} collision primitive(s); "
        f"{model.name} stroke {model.min_width * 1000:.0f}-{model.max_width * 1000:.0f} mm"
    )

    report: dict = {
        "object": name,
        "gripper": model.name,
        "handles": {},
        "candidates": [],
    }
    if obj.handles:
        print("\nexisting handles:")
        for handle, pose in obj.handles.items():
            ev = planner.evaluate_handle(pose, obj.primitives)
            report["handles"][handle] = ev.to_dict()
            if ev.reasons == ["fingers close on nothing"]:
                print(
                    f"  {handle:14s} -- not a {model.name} grasp (nothing between the pads)"
                )
                continue
            status = "ok" if ev.feasible else "NO: " + "; ".join(ev.reasons)
            print(
                f"  {handle:14s} width {ev.contact_width * 1000:5.1f} mm -> command "
                f"{ev.width * 1000:5.1f} mm (q {ev.q:.3f}), contact {ev.contact_ratio:.0%}, "
                f"offset {ev.offset * 1000:+.1f} mm  {status}"
            )

    candidates = planner.plan(obj.primitives, max_candidates=args.top)
    print(f"\nbest {len(candidates)} planned grasp(s):")
    for i, c in enumerate(candidates, 1):
        ev = c.evaluation
        xyz = ", ".join(f"{v:+.3f}" for v in c.handle_pose[:3, 3])
        print(
            f"  #{i:<2d} score {c.score:.2f}  width {ev.contact_width * 1000:5.1f} mm "
            f"(q {ev.q:.3f})  at ({xyz})  {c.source}"
        )
        report["candidates"].append(c.to_dict())
    if args.emit_srdf:
        print()
        for i, c in enumerate(candidates, 1):
            print(c.srdf_handle(f"h_planned{i}"), end="")
    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
        print(f"\nwrote {args.json}")
    return 0 if candidates else 1


if __name__ == "__main__":
    sys.exit(main())
