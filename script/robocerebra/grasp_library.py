#!/usr/bin/env python3
"""Export RoboCerebra scenes with generated grasp handles (M11 R3, #158).

Runs in the HPP environment and starts the benchmark server in its Python
3.10 environment. For every requested case, it exports all movable objects,
generates object-relative Panda handles, and writes an inventory report.
Handles blocked by scene clutter are marked ``requires_scene_planning`` and
must pass HPP path planning before execution. The report fails if any object
class in the selected cases has no generated handle.

    python script/robocerebra/grasp_library.py --cases-file /tmp/r3-cases.txt
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from long_tamp.eval import RoboCerebraClient, add_grasp_handles, object_class, write_export  # noqa: E402
from replay_demo import server_command  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("cases", nargs="*", help="benchmark cases; default: every case")
    parser.add_argument("--cases-file", help="one case per line")
    parser.add_argument("--env-sh", default="~/devel/hpp/robocerebra/env.sh")
    parser.add_argument("--bench", default=None)
    parser.add_argument("--out", default="/tmp/robocerebra-grasp-scenes")
    parser.add_argument("--max-handles", type=int, default=6)
    parser.add_argument(
        "--results",
        default=str(HERE / "results" / "2026-10-25-r3-grasp-handles.json"),
    )
    args = parser.parse_args(argv)

    cases = list(args.cases)
    if args.cases_file:
        cases.extend(
            line.strip()
            for line in Path(args.cases_file).read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
    cases = list(dict.fromkeys(cases))

    command = server_command(args.env_sh, args.bench, no_render=True)
    root = Path(args.out)
    result: dict[str, Any] = {"cases": {}, "classes": {}, "errors": []}
    by_class: dict[str, list[dict[str, Any]]] = defaultdict(list)
    expected_classes: set[str] = set()
    with RoboCerebraClient.spawn(command) as env:
        cases = cases or env.cases()
        for case in cases:
            try:
                env.load(case)
                env.reset(step=0)
                scene = write_export(env.export_scene(), root / case)
                expected_classes.update(object_class(name) for name in scene.objects)
                handles = add_grasp_handles(scene, max_handles=args.max_handles)
                result["cases"][case] = {
                    "objects": len(handles),
                    "handles": sum(len(v["handles"]) for v in handles.values()),
                }
                for instance, row in handles.items():
                    by_class[row["class"]].append(
                        {
                            "case": case,
                            "instance": instance,
                            "handles": row["handles"],
                        }
                    )
                print(
                    f"{case}: {len(handles)} objects, "
                    f"{result['cases'][case]['handles']} handles",
                    flush=True,
                )
            except Exception as error:
                result["errors"].append(
                    {"case": case, "error": f"{type(error).__name__}: {error}"}
                )
                print(f"{case}: ERROR {type(error).__name__}: {error}", flush=True)

    result["classes"] = {
        name: {
            "instances": len(rows),
            "example": rows[0],
        }
        for name, rows in sorted(by_class.items())
    }
    result["summary"] = {
        "cases": len(result["cases"]),
        "classes": len(result["classes"]),
        "objects": sum(row["objects"] for row in result["cases"].values()),
        "errors": len(result["errors"]),
        "classes_without_handles": sorted(expected_classes - set(by_class)),
    }
    out_path = Path(args.results)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=1))
    print(json.dumps(result["summary"], indent=1))
    return 1 if result["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
