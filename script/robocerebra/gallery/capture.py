#!/usr/bin/env python3
"""Rerun the R3 grasp-lift per class with rendering on, saving camera frames.

Reuses grasp_lift.grasp_lift() unchanged; a proxy around the env client renders
agentview at reset, every EVERY steps and at the last step of each rollout.
"""

import json, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "script" / "robocerebra"))
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import grasp_lift as gl  # noqa: E402
from replay_demo import server_command  # noqa: E402
from long_tamp.eval import RoboCerebraClient  # noqa: E402
from long_tamp.eval.robocerebra_policy import HppPolicy  # noqa: E402

OUT = Path(__file__).resolve().parent / "out"
EVERY, SIZE, LIMIT = 30, 384, 150
LIMITS = {"plate": 800}  # the edge pinch pushes before it grasps (#188)
CAMS = ["agentview", "robot0_eye_in_hand"]


class Recorder:
    def __init__(self, env):
        self._env, self.frames, self.n = env, [], 0
        self.limit = LIMIT

    def __getattr__(self, name):
        return getattr(self._env, name)

    def _shot(self, label):
        imgs = self._env.render(cameras=CAMS, size=SIZE)
        self.frames.append(
            (label, {k: np.asarray(v, dtype=np.uint8) for k, v in imgs.items()})
        )

    def reset(self, *a, **k):
        r = self._env.reset(*a, **k)
        self.frames, self.n = [], 0
        self._shot("start")
        return r

    def step(self, action):
        r = self._env.step(action)
        self.n += 1
        if r.get("all_done") or self.n >= self.limit:
            self._shot(f"end (step {self.n})")
        elif self.n % EVERY == 0:
            self._shot(f"step {self.n}")
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
                policy = HppPolicy(
                    work_dir=Path("/tmp/grasp-gallery") / case / instance
                )
                try:
                    env.limit = LIMITS.get(name, LIMIT)
                    row = gl.grasp_lift(env, policy, case, instance, 0, env.limit)
                    row.pop("policy", None)
                except Exception as e:  # keep going; record the error
                    row = {
                        "case": case,
                        "instance": instance,
                        "success": False,
                        "error": f"{type(e).__name__}: {e}",
                    }
                d = OUT / name
                d.mkdir(exist_ok=True)
                files = []
                for i, (label, imgs) in enumerate(env.frames):
                    for cam, img in imgs.items():
                        f = f"{i:02d}_{cam}.jpg"
                        Image.fromarray(img).save(d / f, quality=88)
                        files.append(
                            {"label": label, "camera": cam, "file": f"{name}/{f}"}
                        )
                row["frames"] = files
                row["wall_seconds"] = round(time.time() - t, 1)
                rows[name] = row
                res_path.write_text(json.dumps(rows, indent=1))
                print(
                    f"{name:32s} {'OK' if row['success'] else 'FAIL'} lifted={row.get('lifted')} "
                    f"frames={len(files)} {row.get('error','')}",
                    flush=True,
                )


if __name__ == "__main__":
    main()
