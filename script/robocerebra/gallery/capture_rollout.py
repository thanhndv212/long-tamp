#!/usr/bin/env python3
"""Rerun whole RoboCerebra rollouts (row C or row D), rendering every control step.

    python script/robocerebra/gallery/capture_rollout.py C Ideal/case2 Ideal/case7
    python script/robocerebra/gallery/capture_rollout.py D Ideal/case2 --seed 0

Frames go to out_rollout/<row>/<case>/frames/NNNN.jpg (agentview | wrist, the
step and the last completed subtask burned in), with the rollout's record and
the step at which each subtask completed in rollout.json. Encoding to 30 fps
happens on the host (encode.sh out_rollout). The rollouts are the full runs'
own, at one seed: same policy, same budgets, rendering only added.
"""

import argparse, json, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "script" / "robocerebra"))
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
from replay_demo import server_command  # noqa: E402
from long_tamp.eval import RoboCerebraClient  # noqa: E402

OUT = Path(__file__).resolve().parent / "out_rollout"
SIZE = 320
CAMS = ["agentview", "robot0_eye_in_hand"]


class Recorder:
    """The env, rendering after every reset and step."""

    def __init__(self, env):
        self._env, self.dir, self.n, self.events, self.label = env, None, 0, [], ""

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
        text = f"step {self.n:4d}  {self.label}"
        d.rectangle((0, 0, 8 + 6 * len(text), 18), fill=(0, 0, 0))
        d.text((5, 3), text, fill=(255, 255, 255))
        tile.save(self.dir / f"{self.n:04d}.jpg", quality=88)

    def reset(self, *a, **k):
        r = self._env.reset(*a, **k)
        self.n, self.events, self.label = 0, [], ""
        self._shot()
        return r

    def step(self, action):
        r = self._env.step(action)
        self.n += 1
        for message in r.get("monitor", []):
            if "Completed subtask" in message:
                done = message.split("`")[1] if "`" in message else message
                self.events.append({"step": self.n, "subtask": done})
                self.label = "done: " + done.replace("\u2192", "->")
        self._shot()
        return r


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("row", choices=["C", "D"])
    parser.add_argument("cases", nargs="+")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    cmd = server_command("~/devel/hpp/robocerebra/env.sh", None, no_render=False)
    with RoboCerebraClient.spawn(cmd) as client:
        env = Recorder(client)
        for case in args.cases:
            t = time.time()
            out = OUT / args.row / case.replace("/", "_")
            env.dir = out / "frames"
            env.dir.mkdir(parents=True, exist_ok=True)
            for old in env.dir.glob("*.jpg"):
                old.unlink()
            if args.row == "C":
                from long_tamp.eval.robocerebra_policy import hpp_policy_continuous
                from long_tamp.eval.robocerebra_runner import evaluate_case

                row = evaluate_case(
                    env, case, hpp_policy_continuous, seeds=[args.seed], resume=False
                )
            else:
                from long_tamp.eval.robocerebra_mission import evaluate_mission_case
                from long_tamp.eval.robocerebra_policy import HppPolicy

                row = evaluate_mission_case(
                    env, case, seeds=[args.seed], policy=HppPolicy()
                )
            rollout = row["rollouts"][0]
            record = {
                "row": args.row,
                "case": case,
                "seed": args.seed,
                "completed": rollout["total_completed"],
                "total": rollout["total"],
                "success": rollout["success"],
                "frames": env.n + 1,
                "events": env.events,
                "wall_seconds": round(time.time() - t, 1),
                "rollout": rollout,
            }
            (out / "rollout.json").write_text(json.dumps(record, indent=1, default=str))
            print(
                f"{args.row} {case:24s} {record['completed']}/{record['total']} "
                f"frames={record['frames']} {record['wall_seconds']}s",
                flush=True,
            )


if __name__ == "__main__":
    main()
