#!/usr/bin/env python3
"""Replay a recorded MuJoCo mission in MuJoCo's viewer.

Record it with ``task_screw_assembly.py --backend mujoco --sim-record``: the
run folder then holds the scene (``mjcf/``) and the simulation (``sim/``).
The viewer only needs ``mujoco``, so the replay can run outside the planning
container. On macOS, MuJoCo's viewer needs ``mjpython``:

    mjpython view_mujoco.py runs/seed1_...               # real time
    mjpython view_mujoco.py runs/seed1_... --speed 4     # 4x
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


def load(run: Path):
    scenes = sorted((run / "mjcf").glob("*.xml"))
    if not scenes:
        raise SystemExit(
            f"no scene in {run / 'mjcf'}: was it run with --backend mujoco?"
        )
    chunks = sorted((run / "sim").glob("chunk_*.npz"))
    if not chunks:
        raise SystemExit(f"no recording in {run / 'sim'}: run with --sim-record")
    times, qpos = [], []
    for chunk in chunks:
        with np.load(chunk) as data:
            times.append(data["time"])
            qpos.append(data["qpos"])
    return scenes[0], np.concatenate(times), np.concatenate(qpos)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path, help="the run folder")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed")
    ap.add_argument("--loop", action="store_true", help="replay until closed")
    args = ap.parse_args()

    import mujoco
    import mujoco.viewer

    scene, times, qpos = load(args.run)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    print(
        f"{scene.name}: {len(times)} frames, {times[-1] - times[0]:.0f} simulated s "
        f"({(times[-1] - times[0]) / args.speed:.0f} s at {args.speed:g}x)"
    )
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.monotonic()
            for t, q in zip(times - times[0], qpos):
                if not viewer.is_running():
                    break
                wait = t / args.speed - (time.monotonic() - start)
                if wait > 0:
                    time.sleep(wait)
                data.qpos[:] = q
                mujoco.mj_forward(model, data)
                viewer.sync()
            if not args.loop:
                while viewer.is_running():
                    time.sleep(0.1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
