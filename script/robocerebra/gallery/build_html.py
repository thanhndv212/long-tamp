#!/usr/bin/env python3
"""Build a side-by-side gallery of the R3 grasp-lift rerun frames, 25 classes."""

import html, json, shutil, sys
from pathlib import Path

recorded_path, frames_dir, out_dir = (Path(a) for a in sys.argv[1:4])
video_dir = Path(sys.argv[4]) if len(sys.argv) > 4 else None
recorded = json.loads(recorded_path.read_text())["classes"]
recorded_date = recorded_path.name[:10]  # results/YYYY-MM-DD-....json
rerun = (
    json.loads((frames_dir / "rerun.json").read_text())
    if (frames_dir / "rerun.json").exists()
    else {}
)
out_dir.mkdir(parents=True, exist_ok=True)
for sub in ("img", "video"):
    if (out_dir / sub).exists():
        shutil.rmtree(out_dir / sub)
n_video = 0

CAUSE = {
    "black_book": "No handle plans in the scene. Since #158 a tilted grasp, leaning toward the robot, picks it",
    "plate": "No handle plans in the scene (near-flat rim; a top pinch puts the hand in the table). Since #188 it is pushed to the table's edge and pinched there, past 150 steps (the rerun's limit is 800)",
    "rack": "No handle plans at Ideal/case2, 0.85 m from the base. Recorded at Memory_Execution/case1, 0.75 m out (the rack is only ever a place target)",
    "glazed_rim_porcelain_ramekin": "Every grasp needs a joint at its limit under OSC_POSE (20 cm in front of the base). Since #158 a tilted grasp, leaning toward the robot, picks it",
    "macaroni_and_cheese": "Every grasp needs a joint at its limit under OSC_POSE (20 cm in front of the base); knocked over. Since #158 a tilted grasp, leaning toward the robot, holds it (it starts in no region)",
    "moka_pot": "No handle plans: 21 cm in front of the base, its top-down grasps need joint 4 at its stop and its tilted ones do not plan either. Never a pick goal in the benchmark; also unregistrable",
    "white_bowl": "Its 2 mm rim slipped out of the pinch: the pads overhung the rim by half their length, and the lift began while the fingers still closed. Since #158 the pads sink down the wall and the close waits for the fingers to stop",
    "ketchup": "Starts in no region (origin above the table band)",
    "milk": "Starts in no region (origin above the table band)",
    "orange_juice": "Starts in no region (origin above the table band)",
    "basket": "Carries its own contain region",
}


def verdict(r):
    if r.get("success"):
        return "picked", "Picked"
    if r.get("held"):
        return "held", "Held, unregistrable"
    return "failed", "Failed"


def fmt(v, unit=""):
    return "–" if v is None else f"{v}{unit}"


groups = {"picked": [], "held": [], "failed": []}
for name in sorted(recorded):
    groups[verdict(recorded[name])[0]].append(name)

cards = []
for key, title in (
    ("picked", "Picked by the checker"),
    ("held", "Held, but the checker can't register it"),
    ("failed", "Failed"),
):
    cards.append(
        f'<h2 id="{key}">{title} <span class="count">{len(groups[key])}</span></h2>'
    )
    for name in groups[key]:
        rec = recorded[name]
        new = rerun.get(name)
        kind, label = verdict(rec)
        strip = {"agentview": [], "robot0_eye_in_hand": []}
        if new:
            for f in new.get("frames", []):
                src = frames_dir / f["file"]
                dst = out_dir / "img" / f["file"]
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
                strip[f["camera"]].append(
                    f'<figure><img loading="lazy" src="img/{html.escape(f["file"])}" alt="{name} {f["label"]} {f["camera"]}">'
                    f'<figcaption>{html.escape(f["label"])}</figcaption></figure>'
                )
        if new is None:
            body = '<p class="meta">Rerun pending.</p>'
            rerun_cell = "<td>pending</td>"
        else:
            nk, nl = verdict(new)
            same = "" if nk == kind else ' <span class="diff">differs</span>'
            err = (
                f'<p class="meta">Rerun error: {html.escape(new["error"])}</p>'
                if new.get("error")
                else ""
            )
            body = (
                err
                + f'<div class="strip cam-agentview">{"".join(strip["agentview"])}</div>'
                f'<div class="strip cam-robot0_eye_in_hand">{"".join(strip["robot0_eye_in_hand"])}</div>'
            )
            rerun_cell = (
                f'<td><span class="chip {nk}">{nl}</span>{same}</td><td>{fmt(new.get("lifted"), " m")}</td>'
                f'<td>{fmt(new.get("rotation"))}</td><td>{fmt(new.get("steps_used"))}</td>'
            )
        video = ""
        mp4 = video_dir / name / f"{name}.mp4" if video_dir else None
        if mp4 and mp4.exists():
            (out_dir / "video").mkdir(exist_ok=True)
            shutil.copy2(mp4, out_dir / "video" / mp4.name)
            n_video += 1
            video = (
                f'<video class="clip" controls muted loop playsinline preload="metadata" '
                f'src="video/{mp4.name}"></video>'
                '<p class="meta vcap">Left: agent view · right: wrist · every control step (20 Hz), 30 fps, real time</p>'
            )
        body = video + body
        cause = (
            f'<p class="cause">{html.escape(CAUSE[name])}</p>' if name in CAUSE else ""
        )
        cards.append(f"""
<section class="card" data-kind="{kind}" id="c-{name}">
  <header><h3>{name}</h3><span class="chip {kind}">{label}</span>
  <span class="meta">{html.escape(rec["case"])} · {html.escape(rec["instance"])}</span></header>
  {cause}
  <table><thead><tr><th></th><th>verdict</th><th>lifted</th><th>rotation</th><th>steps</th></tr></thead><tbody>
  <tr><th>recorded ({recorded_date})</th><td><span class="chip {kind}">{label}</span></td><td>{fmt(rec.get("lifted"), " m")}</td><td>{fmt(rec.get("rotation"))}</td><td>{fmt(rec.get("steps_used"))}</td></tr>
  <tr><th>this rerun</th>{rerun_cell}</tr></tbody></table>
  {body}
</section>""")

n_done = len(rerun)
page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RoboCerebra R3: grasp and lift, all 25 classes</title>
<style>
.viz-root {{ color-scheme: light; --page:#f9f9f7; --surface-1:#fcfcfb; --text-primary:#0b0b0b; --text-secondary:#52514e;
  --text-muted:#898781; --border:rgba(11,11,11,.10); --good:#0ca30c; --held:#2a78d6; --critical:#d03b3b; --warn:#eb6834; }}
@media (prefers-color-scheme: dark) {{ .viz-root {{ color-scheme: dark; --page:#0d0d0d; --surface-1:#1a1a19; --text-primary:#f0efec;
  --text-secondary:#c3c2b7; --text-muted:#898781; --border:rgba(255,255,255,.10); --held:#3987e5; --warn:#d95926; }} }}
* {{ box-sizing:border-box; }} body {{ margin:0; }}
.viz-root {{ background:var(--page); color:var(--text-primary); font:15px/1.55 system-ui,-apple-system,"Segoe UI",sans-serif; min-height:100vh; padding:40px 20px 80px; }}
main {{ max-width:1180px; margin:0 auto; }}
h1 {{ font-size:28px; line-height:1.2; margin:0 0 8px; font-weight:650; }}
h2 {{ font-size:20px; margin:44px 0 4px; font-weight:620; }} h3 {{ font-size:16px; margin:0; font-weight:600; }}
p {{ color:var(--text-secondary); max-width:80ch; }} .meta {{ color:var(--text-muted); font-size:13px; }}
.count {{ color:var(--text-muted); font-weight:500; }}
.toolbar {{ position:sticky; top:0; z-index:2; background:var(--page); padding:10px 0; display:flex; gap:8px; flex-wrap:wrap; border-bottom:1px solid var(--border); }}
.toolbar button {{ font:inherit; font-size:13px; padding:5px 12px; border-radius:999px; border:1px solid var(--border); background:var(--surface-1); color:var(--text-primary); cursor:pointer; }}
.toolbar button[aria-pressed="true"] {{ background:var(--text-primary); color:var(--page); }}
.card {{ background:var(--surface-1); border:1px solid var(--border); border-radius:12px; padding:16px 20px; margin-top:14px; }}
.card header {{ display:flex; gap:10px; align-items:baseline; flex-wrap:wrap; }}
.chip {{ font-size:12px; font-weight:600; padding:1px 9px; border-radius:999px; border:1.5px solid currentColor; white-space:nowrap; }}
.chip.picked {{ color:var(--good); }} .chip.held {{ color:var(--held); }} .chip.failed {{ color:var(--critical); }}
.diff {{ font-size:12px; color:var(--warn); font-weight:600; margin-left:6px; }}
.cause {{ margin:6px 0 0; color:var(--text-primary); }}
table {{ border-collapse:collapse; margin:10px 0; font-size:13px; font-variant-numeric:tabular-nums; }}
th, td {{ text-align:left; padding:3px 14px 3px 0; }} thead th {{ color:var(--text-muted); font-weight:500; }}
tbody th {{ color:var(--text-secondary); font-weight:500; }}
.strip {{ display:grid; grid-template-columns:repeat(6, 1fr); gap:6px; }}
.clip {{ width:100%; max-width:768px; display:block; border-radius:8px; border:1px solid var(--border); background:#000; }}
.vcap {{ margin:2px 0 10px; }}
figure {{ margin:0; }} figure img {{ width:100%; display:block; border-radius:6px; border:1px solid var(--border); cursor:zoom-in; }}
figcaption {{ font-size:12px; color:var(--text-muted); text-align:center; }}
.viz-root[data-cam="agentview"] .cam-robot0_eye_in_hand, .viz-root[data-cam="robot0_eye_in_hand"] .cam-agentview {{ display:none; }}
.viz-root[data-cam="both"] .strip + .strip {{ margin-top:6px; }}
dialog {{ border:none; padding:0; background:transparent; }} dialog::backdrop {{ background:rgba(0,0,0,.8); }}
dialog img {{ max-width:92vw; max-height:88vh; image-rendering:auto; display:block; }}
dialog p {{ color:#eee; text-align:center; margin:6px 0 0; }}
@media (max-width:760px) {{ .strip {{ grid-template-columns:repeat(3,1fr); }} }}
</style></head>
<body><div class="viz-root" data-cam="both" data-filter="all"><main>
<h1>RoboCerebra R3: grasp and lift, all 25 classes</h1>
<p class="lede">One pick of each object class (<code>pick &lt;instance&gt;</code>, seed 0, 150-step budget, <code>OSC_POSE</code>), planned by the HPP segment policy.
Recorded verdicts are from <code>results/{html.escape(recorded_path.name)}</code>: <strong>{len(groups["picked"])} picked, {len(groups["held"])} held but unregistrable, {len(groups["failed"])} failed</strong>.
The frames come from a rerun of the same protocol with rendering on; where its verdict differs from the recorded one, the card says so.</p>
<p class="meta">Videos: {n_video}/25. Rerun: {n_done}/25 classes. Frames: start, every 30 steps, and the last step. Click a frame to enlarge.</p>
<div class="toolbar" role="toolbar">
  <span class="meta" style="align-self:center">Show</span>
  <button data-filter="all" aria-pressed="true">All 25</button>
  <button data-filter="picked" aria-pressed="false">Picked {len(groups["picked"])}</button>
  <button data-filter="held" aria-pressed="false">Held {len(groups["held"])}</button>
  <button data-filter="failed" aria-pressed="false">Failed {len(groups["failed"])}</button>
  <span class="meta" style="align-self:center;margin-left:16px">Camera</span>
  <button data-cam="both" aria-pressed="true">Both</button>
  <button data-cam="agentview" aria-pressed="false">Agent view</button>
  <button data-cam="robot0_eye_in_hand" aria-pressed="false">Wrist</button>
  <button id="playall" style="margin-left:16px">Play all visible</button>
</div>
{"".join(cards)}
</main>
<dialog id="zoom"><img alt=""><p></p></dialog>
<script>
const root = document.querySelector('.viz-root');
document.querySelectorAll('[data-filter]').forEach(b => b.tagName === 'BUTTON' && b.addEventListener('click', () => {{
  document.querySelectorAll('button[data-filter]').forEach(x => x.setAttribute('aria-pressed', x === b));
  const f = b.dataset.filter;
  document.querySelectorAll('.card').forEach(c => c.hidden = f !== 'all' && c.dataset.kind !== f);
  document.querySelectorAll('h2').forEach(h => h.hidden = f !== 'all' && h.id !== f);
}}));
document.querySelectorAll('button[data-cam]').forEach(b => b.addEventListener('click', () => {{
  document.querySelectorAll('button[data-cam]').forEach(x => x.setAttribute('aria-pressed', x === b));
  root.dataset.cam = b.dataset.cam;
}}));
const dlg = document.getElementById('zoom');
document.querySelectorAll('figure img').forEach(i => i.addEventListener('click', () => {{
  dlg.querySelector('img').src = i.src; dlg.querySelector('p').textContent = i.alt; dlg.showModal();
}}));
dlg.addEventListener('click', () => dlg.close());
document.getElementById('playall').addEventListener('click', () =>
  document.querySelectorAll('.card:not([hidden]) video').forEach(v => {{ v.currentTime = 0; v.play(); }}));
const h = location.hash.slice(1);
const b0 = document.querySelector(`button[data-filter="${{h}}"]`);
if (b0) b0.click();
</script></div></body></html>"""
(out_dir / "index.html").write_text(page)
print(f"wrote {out_dir / 'index.html'} ({n_done}/25 rerun)")
