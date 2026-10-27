# R3 grasp-lift gallery (work in progress)

Visual evidence for #158 AC2: one pick per object class, rerun with rendering on.
The page is `../results/r3-grasp-gallery/index.html`; it is updated as the grasping work
moves on, so its numbers are those of the latest rerun.

Run the captures in the HPP container (the repo is bind-mounted there), with the
benchmark env from `grasp_lift.py`:

```bash
python script/robocerebra/gallery/capture.py          # screenshots: start, every 30 steps, last step
python script/robocerebra/gallery/capture_video.py    # a frame at every control step (20 Hz)
python script/robocerebra/gallery/capture.py white_bowl   # or only some classes
```

Both reuse `grasp_lift.grasp_lift()` unchanged through a proxy that renders the
agentview and wrist cameras, and skip classes already in their `rerun.json`
(delete `out*/rerun.json` to redo them). Then on the host:

```bash
script/robocerebra/gallery/encode.sh   # 20 Hz frames -> 30 fps real-time H.264 (ffmpeg)
python3 script/robocerebra/gallery/build_html.py \
  script/robocerebra/results/2026-10-26-r3-grasp-lift.json \
  script/robocerebra/gallery/out script/robocerebra/results/r3-grasp-gallery \
  script/robocerebra/gallery/out_video
```

The page compares each class's recorded verdict with the rerun's and flags a
difference. Failure causes are in `CAUSE` in `build_html.py`; keep them in step
with the README's AC2 table.
