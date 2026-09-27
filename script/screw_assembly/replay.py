#!/usr/bin/env python3
"""Replay a recorded screw-assembly mission in viser.

    python3 task_screw_assembly.py --seed 10 --trajectory traj.json   # record
    python3 replay.py traj.json --port 8081 --loop                     # replay

Loads the scene only (no planning), serves viser on 0.0.0.0:PORT and plays
the recorded frames at their recorded timing (``--speed`` scales it). HPP's
paths keep the Robotiq fingers open (a grasp is a rigid TCP constraint), so
the fingers are closed and opened here as an overlay: closed from each arm
grasp until that arm's release, to the width the grasp planner computes for
that handle (``task_screw_assembly.finger_closures()``).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np



def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true", help="replay forever")
    args = ap.parse_args()

    data = json.loads(args.trajectory.read_text())
    sys.argv = [sys.argv[0]]
    import task_screw_assembly as T
    from pyhpp_viser import Viewer

    task, _ = T.setup()
    closures = T.finger_closures()
    viewer = Viewer(task.planner.device, task.planner.problem)
    viewer.start(host="0.0.0.0", port=args.port, open=False)
    server = viewer.viewer

    @server.on_client_connect
    def _aim(client):
        client.camera.position = (0.75, -1.55, 1.45)
        client.camera.look_at = (0.78, 0.0, 0.45)
        client.camera.up_direction = (0.0, 0.0, 1.0)

    rank = task.robot.rankInConfiguration

    def overlay(q, fingers):
        for values in fingers.values():
            for joint, value in values.items():
                q[rank[joint]] = value
        return q

    def animate_fingers(q, fingers, gripper, target, steps=12):
        start = fingers.get(gripper) or closures.open_values(gripper)
        for i in range(steps + 1):
            s = i / steps
            fingers[gripper] = {j: start[j] + (v - start[j]) * s for j, v in target.items()}
            viewer(overlay(q.copy(), fingers))
            time.sleep(0.02)

    frame_dt = data.get("dt", 0.05) / max(args.speed, 1e-3)
    segments = data["segments"]
    n_frames = sum(len(s["configs"]) for s in segments)
    print(
        f"replaying {n_frames} frames, {len(segments)} segments "
        f"(~{n_frames * frame_dt:.0f} s) at http://0.0.0.0:{args.port}",
        flush=True,
    )
    time.sleep(3.0)  # let a browser connect before the first frame
    while True:
        fingers: dict[str, dict[str, float]] = {}
        block = None
        for seg in segments:
            if seg["block"] != block:
                block = seg["block"]
                print(f"  {block}", flush=True)
            gripper = seg["gripper"]
            hand = closures.has(gripper)
            first = np.asarray(seg["configs"][0], dtype=float)
            # A release opens the fingers; a home move (also handle None)
            # carries its object, so its fingers stay closed.
            release = seg["handle"] is None and "home" not in seg["block"]
            if hand and release and gripper in fingers:
                animate_fingers(first, fingers, gripper, closures.open_values(gripper))
                fingers.pop(gripper)
            for q in seg["configs"]:
                viewer(overlay(np.asarray(q, dtype=float), fingers))
                time.sleep(frame_dt)
            if hand and seg["handle"] is not None:
                last = np.asarray(seg["configs"][-1], dtype=float)
                target = closures.closed_values(gripper, seg["handle"])
                animate_fingers(last, fingers, gripper, target)
        if not args.loop:
            break
        time.sleep(2.0)
    print("replay done; viewer stays up (Ctrl+C to stop)", flush=True)
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())
