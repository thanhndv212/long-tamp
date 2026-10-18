#!/usr/bin/env python3
"""Start the screw assembly's mission UI (#104): one browser page with the 3D
scene, the operator chat and the live plan monitor.

    python mission_ui.py                    # MuJoCo, chat with the default model
    python mission_ui.py --backend playback --seed 4 --port 8090

The library's generic launcher (``python -m long_tamp.mission ui``, #133) on
this folder's domain (``screw_mission.py:domain``), with MuJoCo as the
default backend; the same as

    python -m long_tamp.mission ui \\
        --domain script/screw_assembly/screw_mission.py:domain --backend mujoco

Type an instruction in the chat ("assemble parts 1 and 2, part 2 first"); the
model sets the goal and the task planner builds the plan, shown as a card
with a Start mission button. Start runs it: the plan tree and the timeline
follow it live, the scene plays the motion, and pause/resume/stop act at
step boundaries. Reset mission starts over (a fresh mission, robot and parts
back at the start). Ctrl-C here ends everything. ``--help`` lists the
options; arguments after ``--`` go to ``serve`` as they are (e.g.
``-- --backend-option speed=1.0``).
"""

from __future__ import annotations

import sys
from pathlib import Path

from long_tamp.mission.ui import launch

HERE = Path(__file__).resolve().parent


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    domain = f"{HERE / 'screw_mission.py'}:domain"
    return launch(["--domain", domain, "--backend", "mujoco", *argv], runs=HERE / "runs")


if __name__ == "__main__":
    raise SystemExit(main())
