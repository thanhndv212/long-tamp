#!/usr/bin/env python3
"""Rerun the R3 grasp-lift per class, rendering every control step (20 Hz) for video.

Frames go to out_video/<class>/frames/NNNN.jpg: agentview | wrist side by side,
with the step number burned in. Encoding to 30 fps happens on the host.
"""

import json, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "script" / "robocerebra"))
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
import grasp_lift as gl  # noqa: E402
from replay_demo import server_command  # noqa: E402
from long_tamp.eval import RoboCerebraClient  # noqa: E402
from long_tamp.eval.robocerebra_policy import HppPolicy  # noqa: E402

OUT = Path(__file__).resolve().parent / "out_video"
SIZE, LIMIT = 384, 150
CAMS = ["agentview", "robot0_eye_in_hand"]


class Recorder:
    def __init__(self, env):
        self._env, self.dir, self.n = env, None, 0

    def __getattr__(self, name):
        return getattr(self._env, name)

    def _shot(self):
        imgs = self._env.render(cameras=CAMS, size=SIZE)
        tile = Image.new("RGB", (SIZE * 2, SIZE))
        for i, cam in enumerate(CAMS):
            tile.paste(
                Image.fromarray(np.asarray(imgs[cam], dtype=np.uint8)), (i * SIZE, 0)
            )
        d = ImageDraw.Draw(tile)
        d.rectangle((0, 0, 92, 18), fill=(0, 0, 0))
        d.text((5, 3), f"step {self.n:3d}", fill=(255, 255, 255))
        tile.save(self.dir / f"{self.n:04d}.jpg", quality=90)

    def reset(self, *a, **k):
        r = self._env.reset(*a, **k)
        self.n = 0
        self._shot()
        return r

    def step(self, action):
        r = self._env.step(action)
        self.n += 1
        self._shot()
        return r


def main():
    recorded = json.loads(
        (REPO / "script/robocerebra/results/2026-10-26-r3-grasp-lift.json").read_text()
    )["classes"]
    only = set(sys.argv[1:])
    OUT.mkdir(parents=True, exist_ok=True)
    res_path = OUT / "rerun.json"
    rows = json.loads(res_path.read_text()) if res_path.exists() else {}
    by_case = {}
    for name, row in sorted(recorded.items()):
        if (only and name not in only) or name in rows:
            continue
        by_case.setdefault(row["case"], []).append((name, row["instance"]))
    cmd = server_command("~/devel/hpp/robocerebra/env.sh", None, no_render=False)
    with RoboCerebraClient.spawn(cmd) as client:
        env = Recorder(client)
        for case, items in sorted(by_case.items()):
            client.load(case)
            for name, instance in items:
                t = time.time()
                env.dir = OUT / name / "frames"
                env.dir.mkdir(parents=True, exist_ok=True)
                for old in env.dir.glob("*.jpg"):
                    old.unlink()
                policy = HppPolicy(
                    work_dir=Path("/tmp/grasp-gallery-video") / case / instance
                )
                try:
                    row = gl.grasp_lift(env, policy, case, instance, 0, LIMIT)
                    row.pop("policy", None)
                except Exception as e:
                    row = {
                        "case": case,
                        "instance": instance,
                        "success": False,
                        "error": f"{type(e).__name__}: {e}",
                    }
                row["frames"] = len(list(env.dir.glob("*.jpg")))
                row["wall_seconds"] = round(time.time() - t, 1)
                rows[name] = row
                res_path.write_text(json.dumps(rows, indent=1))
                print(
                    f"{name:32s} {'OK' if row['success'] else 'FAIL'} lifted={row.get('lifted')} "
                    f"frames={row['frames']} {row.get('error', '')}",
                    flush=True,
                )


if __name__ == "__main__":
    main()
