#!/usr/bin/env python3
"""Build a page of recorded RoboCerebra rollouts (capture_rollout.py, encode.sh).

    python3 script/robocerebra/gallery/build_rollout_html.py \
      script/robocerebra/gallery/out_rollout script/robocerebra/results/rollout-gallery

Each rollout gets its video and a keyframe at the start, at every subtask that
completed, and at the last step.
"""

import html, json, shutil, sys
from pathlib import Path

src, out = (Path(a) for a in sys.argv[1:3])
if not src.is_dir():
    sys.exit(f"no recorded rollouts in {src}")
for sub in ("img", "video"):
    if (out / sub).exists():
        shutil.rmtree(out / sub)
    (out / sub).mkdir(parents=True)

ROWS = {
    "C": "Row C: continuous, one policy call a segment, no resume(k)",
    "D": "Row D: one long_tamp mission over the whole goal",
}


def keyframes(record):
    last = record["frames"] - 1
    shots = [(0, "start")]
    shots += [(e["step"], e["subtask"]) for e in record["events"]]
    shots.append((last, "last step"))
    return shots


sections = []
for row in sorted(p.name for p in src.iterdir() if p.is_dir()):
    cards = []
    for case_dir in sorted((src / row).iterdir()):
        path = case_dir / "rollout.json"
        if not path.exists():
            continue
        record = json.loads(path.read_text())
        tag = f"{row}_{case_dir.name}"
        video = case_dir / f"{case_dir.name}.mp4"
        if video.exists():
            shutil.copy(video, out / "video" / f"{tag}.mp4")
        strip = []
        for step, label in keyframes(record):
            frame = case_dir / "frames" / f"{step:04d}.jpg"
            if not frame.exists():
                continue
            name = f"{tag}_{step:04d}.jpg"
            shutil.copy(frame, out / "img" / name)
            strip.append(
                f'<figure><img src="img/{name}" loading="lazy" alt="step {step}">'
                f"<figcaption>step {step}: {html.escape(label)}</figcaption></figure>"
            )
        rollout = record["rollout"]
        notes = []
        mission = rollout.get("mission") or {}
        if mission.get("given_up"):
            notes.append("given up: " + ", ".join(mission["given_up"]))
        for seg in rollout.get("segments", []):
            if not seg.get("segment_success"):
                why = (seg.get("policy") or {}).get("error") or "not registered"
                notes.append(f"step {seg['segment']} missed: {why}")
        state = "done" if record["success"] else "partial"
        video_html = (
            f'<video controls preload="metadata" src="video/{tag}.mp4"></video>'
            if video.exists()
            else "<p>(no video)</p>"
        )
        cards.append(
            f'<section class="card {state}"><h3>{html.escape(record["case"])}, '
            f'seed {record["seed"]}: {record["completed"]}/{record["total"]} subtasks'
            f'</h3>{video_html}<div class="strip">{"".join(strip)}</div>'
            + (
                "<ul>" + "".join(f"<li>{html.escape(n)}</li>" for n in notes) + "</ul>"
                if notes
                else ""
            )
            + "</section>"
        )
    sections.append(f"<h2>{html.escape(ROWS.get(row, row))}</h2>" + "".join(cards))

page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>RoboCerebra rollouts, rows C and D</title>
<style>
body {{ font: 14px/1.4 system-ui, sans-serif; margin: 2rem auto; max-width: 1400px; padding: 0 1rem; }}
.card {{ border: 1px solid #ccc; border-left: 6px solid #f9a825; padding: .5rem 1rem; margin: 1rem 0; }}
.card.done {{ border-left-color: #2e7d32; }}
video {{ width: 640px; max-width: 100%; }}
.strip {{ display: flex; gap: .5rem; overflow-x: auto; }}
figure {{ margin: 0; flex: 0 0 320px; }}
figure img {{ width: 320px; }}
figcaption {{ font-size: 12px; }}
</style></head><body>
<h1>RoboCerebra rollouts, rows C and D</h1>
<p>Single rollouts rerun with rendering on, the same policy and budgets as the full runs.
Each frame is agentview | wrist camera, with the step and the last completed subtask.
Keyframes are taken at the step each subtask completed. Green: every subtask done.
A row D rollout is not reproducible at a fixed seed, so it can differ from the full run's.</p>
{"".join(sections)}
</body></html>
"""
(out / "index.html").write_text(page)
print(f"wrote {out / 'index.html'}")
