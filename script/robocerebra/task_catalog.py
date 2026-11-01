"""The RoboCerebra test cases' tasks, steps and goal predicates, and how far
each round of results has got on them.

``dump`` reads every case from the benchmark (in the HPP container, with the
env server of ``replay_demo.py``) into ``results/case-catalog.json``:

    python script/robocerebra/task_catalog.py dump

``build`` needs only that file and the round results next to it, and writes
``docs/plans/robocerebra-tasks.md``; rerun it after adding a results file
(add the file to ``ROUNDS`` first):

    python3 script/robocerebra/task_catalog.py build
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
CATALOG = RESULTS / "case-catalog.json"
DOC = HERE.parents[1] / "docs" / "plans" / "robocerebra-tasks.md"
BLOB = "https://github.com/thanhndv212/long-tamp/blob/main"

# The rounds with per-step results, oldest first: (label, results file, row,
# what changed). Rows: A per segment from the official resume(k), C straight
# through without resets, D one mission over the whole goal.
ROUNDS = [
    ("R4", "2026-10-25-r4-ideal-hpp.json", "A", "HPP segment policy, pick/in/on"),
    ("R5", "2026-10-29-r5-ideal-hpp-pour.json", "A", "adds pour"),
    ("R8", "2026-10-29-r8-ideal-hpp-continuous.json", "C", "continuous"),
    ("R9", "2026-10-30-r9-ideal-hpp-mission.json", "D", "one mission a rollout"),
    ("R10", "2026-10-31-r10-ideal-hpp-continuous.json", "C", "place onto an object"),
    ("R11", "2026-10-31-r11-ideal-hpp-mission.json", "D", "place onto an object"),
]
REPLAY = "2026-10-17-r1-replay-ideal-demo.json"
EXPORT = "2026-10-18-r2-scene-export.json"
GRASP_LIFT = "2026-11-01-r3-thin-rims.json"
HANDLES = "2026-10-25-r3-grasp-handles.json"
# The issue behind each round; R10 and R11 rerun R8 and R9 after #187.
ISSUES = {
    "R1": 156,
    "R2": 157,
    "R3": 158,
    "R4": 159,
    "R5": 160,
    "R6": 161,
    "R7": 162,
    "R8": 163,
    "R9": 186,
    "R10": 187,
    "R11": 187,
}
# Rounds on the roadmap with no results yet: what they add, and what waits on them.
NOT_RUN = [
    (
        "R6",
        "A, C, D",
        "articulated objects: drawers, doors, knobs",
        "the {articulated} steps checked as `open`/`close`/`turnon` ({none} in the tables)",
    ),
    (
        "R7",
        "A, B",
        (
            "Random_Disturbance, Observation_Mismatching and Mix; row B"
            " (camera poses #151, goals from the step text #145) for the Memory conditions"
        ),
        "planner results on the {other} cases outside Ideal",
    ),
]
ISSUE_URL = "https://github.com/thanhndv212/long-tamp/issues"
ARTICULATED = {"open", "close", "turnon"}
GALLERY = RESULTS / "rollout-gallery"
GRASP_GALLERY = RESULTS / "r3-grasp-gallery"
# The mission runner's step verbs for each goal predicate.
NONE = "\u2013"  # en dash: a step not checked, or not attempted
VERBS = {"pick": "pick", "on": "place", "in": "place", "pour": "pour"}


def dump(out: Path) -> None:
    sys.path.insert(0, str(HERE))
    from replay_demo import server_command

    from long_tamp.eval import RoboCerebraClient

    cases = sorted(json.loads((RESULTS / EXPORT).read_text())["cases"], key=_order)
    catalog = {}
    command = server_command("~/devel/hpp/robocerebra/env.sh", None, no_render=True)
    with RoboCerebraClient.spawn(command) as env:
        for case in cases:
            info = env.load(case)
            catalog[case] = {
                "task": info.get("task"),
                "steps": list(info["steps"]),
                "goal": info["goal"],
                "goal_steps": info.get("goal_steps", {}),
            }
            print(case, len(info["steps"]), "steps", flush=True)
    out.write_text(json.dumps(catalog, indent=1) + "\n")


def _order(case: str) -> tuple[str, int]:
    kind, number = case.split("/case")
    return kind, int(number)


def step_specs(entry: dict) -> list[list[list[str]]]:
    """Each step's goal predicates (none for a step the benchmark does not
    check)."""
    specs: list[list[list[str]]] = [[] for _ in entry["steps"]]
    for obj, steps in entry["goal_steps"].items():
        for index, step in enumerate(steps):
            if step is not None and 0 <= step < len(specs):
                specs[step].append(list(entry["goal"][obj][index]))
    return specs


def _label(spec: list[str]) -> str:
    """A predicate as the mission runner names its step."""
    verb = VERBS.get(spec[0], spec[0])
    return " ".join([verb, *spec[1:]])


def step_tally(record: dict, row: str, entry: dict) -> list:
    """Per step, the rollouts that got it done and the rollouts run, or
    ``None`` where the round did not attempt it (unsupported). Row A: the
    segment's own check, from ``resume(k)``. Row C: the benchmark's
    ``Completed subtask`` messages, wherever in the rollout they came.
    Row D records no such messages: the mission's execution of the step
    succeeded (it may not have registered; the case total is the
    benchmark's)."""
    specs = step_specs(entry)
    rollouts = record["rollouts"]
    tally = []
    for index, step in enumerate(specs):
        done = tried = 0
        for rollout in rollouts:
            if not step:
                continue
            if row == "D":
                tried += 1
                executions = rollout.get("mission", {}).get("executions", [])
                succeeded = {e["step"] for e in executions if e["status"] == "success"}
                done += all(_label(spec) in succeeded for spec in step)
                continue
            segments = rollout.get("segments", [])
            if index >= len(segments):
                continue
            if segments[index].get("policy", {}).get("status") == "unsupported":
                continue
            tried += 1
            if row == "C":
                done += index in _completed(rollout, entry)
            else:
                done += bool(segments[index].get("segment_success"))
        tally.append((done, tried) if tried else None)
    return tally


COMPLETED = re.compile(r"(\S+): Completed subtask .*\((\d+)/\d+\)")


def _completed(rollout: dict, entry: dict) -> set[int]:
    """The steps the benchmark registered over a rollout, from its monitor
    messages (an object's ``k``-th predicate is its ``k``-th step)."""
    steps = set()
    for segment in rollout.get("segments", []):
        for message in segment.get("monitor", []):
            match = COMPLETED.search(message)
            if match:
                obj, k = match.group(1), int(match.group(2))
                annotated = entry["goal_steps"].get(obj, [])
                if k - 1 < len(annotated) and annotated[k - 1] is not None:
                    steps.add(annotated[k - 1])
    return steps


def _cell(entry) -> str:
    if entry is None:
        return NONE
    done, tried = entry
    mark = "✅" if done == tried else ("🟡" if done else "❌")
    return f"{mark} {done}/{tried}"


def replay_steps(record: dict, entry: dict) -> list:
    """R1 per step: whether the demo's actions got it done straight through
    and from the official ``resume(k)``, or ``None`` for a step not checked."""
    straight = set()
    for item in record["continuous"]["completed_at"]:
        steps = entry["goal_steps"].get(item["object"], [])
        if item["subtask"] < len(steps) and steps[item["subtask"]] is not None:
            straight.add(steps[item["subtask"]])
    resumed = {
        s["step"]: s["completed_expected"]
        for s in record["resumed"]["benchmark_resume"]
    }
    return [
        None if not spec else (index in straight, bool(resumed.get(index)))
        for index, spec in enumerate(step_specs(entry))
    ]


def _mark(ok: bool | None) -> str:
    return NONE if ok is None else ("✅" if ok else "❌")


def export_ok(record: dict) -> bool:
    """R2: the case's scene exported whole, matching the simulator."""
    return (
        not record.get("missing_shapes")
        and not record.get("missing_surfaces")
        and not record.get("contacts", {}).get("disagree")
        and record.get("plan", {"ok": True}).get("ok", False)
    )


def export_line(record: dict) -> str:
    """R2 for one case, in a sentence."""
    shape = record["shape_error"]
    contacts = record["contacts"]
    parts = [
        f"{shape['n']} collision shapes (worst {shape['pos']:.0e} m off),",
        f"{record['contact_surfaces']} contact surfaces,",
        f"{contacts['resting']} resting contacts as MuJoCo has them",
        f"({len(contacts['disagree'])} disagree),",
        f"robot clear by {record['robot_clearance'] * 100:.1f} cm",
    ]
    if "plan" in record:
        plan = record["plan"]
        parts[-1] += ","
        parts.append(
            f"a test plan for its layout {'found' if plan['ok'] else 'failed'}"
            f" in {plan['time']:.2f} s"
        )
    line = "Scene export (R2): " + " ".join(parts) + "."
    skipped = record.get("regions_skipped") or []
    if skipped:
        line += " Regions not exported: " + ", ".join(f"`{r}`" for r in skipped) + "."
    return line


def _video(row: str, case: str) -> str | None:
    name = f"{row}_{case.replace('/', '_')}.mp4"
    return name if (GALLERY / "video" / name).exists() else None


def build(out: Path) -> None:
    catalog = json.loads(CATALOG.read_text())
    rounds = []
    for label, name, row, change in ROUNDS:
        data = json.loads((RESULTS / name).read_text())
        rounds.append((label, name, row, change, data))
    replay = json.loads((RESULTS / REPLAY).read_text())["cases"]
    lift_data = json.loads((RESULTS / GRASP_LIFT).read_text())
    lift = lift_data["classes"]
    export = json.loads((RESULTS / EXPORT).read_text())
    handles = json.loads((RESULTS / HANDLES).read_text())
    CAUSE = _causes(HERE / "gallery" / "build_html.py")

    def rel(path: Path) -> str:
        # The docs site only resolves links between its own pages.
        return f"{BLOB}/{path.relative_to(HERE.parents[1]).as_posix()}"

    lines = [
        "# RoboCerebra tasks and progress",
        "",
        "<!-- Generated by script/robocerebra/task_catalog.py build; do not edit by hand. -->",
        "",
        "Every RoboCerebra test case's task as the benchmark gives it (`env.load(case)`:",
        "the `task` sentence, its `steps`, and the `goal` predicates the simulator checks),",
        "with each round's results per step. The README",
        f"([`script/robocerebra/README.md`]({rel(HERE / 'README.md')})) has the protocol",
        "and the causes behind the numbers.",
        "",
        "## How to read it",
        "",
        "- **Step** is the annotation; **Checked as** is the predicate that scores it. They",
        '  can disagree: "Place … into the bowl" is scored as a `pour` in some cases, and a',
        f"  step with no predicate ({NONE}) cannot fail and is not counted.",
        "- **Rows:** A runs each step from the official `resume(k)` state, so a step's",
        "  result does not depend on the ones before it. C runs the steps straight through",
        "  with no resets (a dropped object stays dropped). D hands the whole goal to the",
        "  mission runner, which orders and repairs the steps itself. B is A with the",
        "  object poses from the simulated camera and the goals written from the step",
        "  text, not from the simulator; no round has run it yet (R7).",
        "- **Cells** are rollouts that got the step done over rollouts that tried it",
        f"  (✅ all, 🟡 some, ❌ none, {NONE} not attempted: the predicate is unsupported, e.g.",
        "  `open`/`turnon` before articulated objects, #161). Row C counts a step done when",
        "  the benchmark registered it anywhere in the rollout. Row D's per-step cells",
        "  count the mission's execution of the step succeeding, which can be ahead of",
        "  what registered; its case totals are the benchmark's.",
        "- **R1** replays the benchmark's recorded demo actions, straight through and",
        "  from `resume(k)` per step: what the demonstration itself achieves, the bar",
        "  the planner rounds compare against. **R2** exports each case's scene to HPP",
        "  and checks it against MuJoCo. **R3** generates grasp handles for every object",
        "  class and tests each class with one pick and lift. R1-R3 build the pipeline;",
        "  R4 on are planner rounds.",
        "",
        "## Rounds",
        "",
        "| Round | Issue | Row | What changed | Results | Cases | Headline |",
        "|---|---|---|---|---|---|---|",
    ]
    straight = resumed = steps = 0
    for item in json.loads((RESULTS / REPLAY).read_text())["summary"]:
        done, total = map(int, item["continuous"].split("/"))
        straight, steps = straight + done, steps + total
        resumed += int(item["benchmark_resume"].split("/")[0])
    summary = export["summary"]
    early = [
        (
            "R1",
            REPLAY,
            "demo",
            "env server: the recorded demo replayed",
            len(replay),
            f"demo: {straight}/{steps} steps straight through, {resumed}/{steps} from resume(k)",
        ),
        (
            "R2",
            EXPORT,
            NONE,
            "scene export to HPP",
            summary["cases"],
            (
                f"{summary['cases']} cases, {summary['layouts']} layouts, {summary['load_errors']} load errors;"
                f" test plans {summary['plans']}"
            ),
        ),
        (
            "R3",
            HANDLES,
            NONE,
            "grasp handles per object class",
            handles["summary"]["cases"],
            (
                f"{handles['summary']['classes']} classes over {handles['summary']['objects']} objects,"
                f" {len(handles['summary']['classes_without_handles'])} without handles"
            ),
        ),
        (
            "R3",
            GRASP_LIFT,
            NONE,
            "one pick and lift per class (AC2)",
            len({r["case"] for r in lift.values()}),
            f"{lift_data['summary']['picked']}/{lift_data['summary']['classes']} classes lifted",
        ),
    ]
    table = [
        (label, row, change, f"[`{name}`]({rel(RESULTS / name)})", cases, headline)
        for label, name, row, change, cases, headline in early
    ]
    for label, name, row, change, data in rounds:
        summary = data.get("summary", {})
        if row == "A":
            headline = f"{summary.get('supported_segment_success_rate', 0):.0%} of supported steps done"
        else:
            headline = (
                f"{summary.get('completed_share', 0):.0%} of each rollout's goal done"
            )
        if label in ("R10", "R11"):
            change = f"{'R8' if row == 'C' else 'R9'} rerun: {change}"
        path = rel(RESULTS / name)
        table.append(
            (label, row, change, f"[`{name}`]({path})", len(data["cases"]), headline)
        )
    articulated = sum(
        any(spec[0] in ARTICULATED for spec in step)
        for entry in catalog.values()
        for step in step_specs(entry)
    )
    other = sum(not case.startswith("Ideal/") for case in catalog)
    for label, row, change, waits in NOT_RUN:
        waits = waits.format(articulated=articulated, other=other, none=NONE)
        table.append((label, row, change, "not run yet", NONE, f"waits on it: {waits}"))
    table.sort(key=lambda r: int(r[0][1:]))
    for label, row, change, results, cases, headline in table:
        issue = f"[#{ISSUES[label]}]({ISSUE_URL}/{ISSUES[label]})"
        lines.append(
            f"| {label} | {issue} | {row} | {change} | {results} | {cases} | {headline} |"
        )
    lines += [
        "",
        (
            "Details of R1-R3: [R1 demo replay](#r1-demo-replay),"
            " [R2 scene export](#r2-scene-export),"
            " [R3 grasp handles and grasp-lift](#r3-grasp-handles-and-grasp-lift)."
        ),
        "",
        "## Progress by case",
        "",
    ]
    header = " | ".join(label for label, *_ in rounds)
    lines += [
        f"| Case | Steps | R1 demo | R1 resume(k) | R2 | R3 handles | {header} | Videos |",
        "|---|---|---|---|---|---|" + "---|" * len(rounds) + "---|",
    ]
    for case in sorted(catalog, key=_order):
        entry = catalog[case]
        specs = step_specs(entry)
        cells = []
        for _, _, row, _, data in rounds:
            record = data["cases"].get(case)
            if record is None:
                cells.append("")
                continue
            if row == "A":
                tally = [t for t in step_tally(record, row, entry) if t is not None]
                done = sum(d for d, _ in tally)
                tried = sum(t for _, t in tally)
            else:
                done = sum(r["total_completed"] for r in record["rollouts"])
                tried = sum(r["total"] for r in record["rollouts"])
            cells.append(f"{done}/{tried}" if tried else NONE)
        demo = resume = ""
        if case in replay:
            run = replay_steps(replay[case], entry)
            checked = [r for r in run if r is not None]
            demo = f"{sum(a for a, _ in checked)}/{len(checked)}"
            resume = f"{sum(b for _, b in checked)}/{len(checked)}"
        exported = (
            _mark(export_ok(export["cases"][case])) if case in export["cases"] else ""
        )
        handled = handles["cases"].get(case)
        handled = f"{handled['handles']} on {handled['objects']}" if handled else ""
        videos = ", ".join(
            f"[{row}]({rel(GALLERY / 'video' / video)})"
            for row in ("C", "D")
            if (video := _video(row, case))
        )
        anchor = case.lower().replace("/", "").replace("_", "-")
        lines.append(
            f"| [{case}](#{anchor}) | {len(entry['steps'])} | {demo} | {resume} | {exported} | {handled} | "
            + " | ".join(cells)
            + f" | {videos} |"
        )
    lines += [
        "",
        "Each cell sums the case's steps over its rollouts (10 seeds). Row A: steps done",
        "over steps tried, leaving out unsupported ones. Rows C and D: the benchmark's",
        "own count, every predicate of the goal over every rollout (the paper's Eq. 1",
        "before averaging). R1 counts checked steps, one replay a case. R2: ✅ when the",
        "export matched MuJoCo (no missing shape or surface, no contact disagreeing, the",
        "layout's test plan found). R3 handles: handles generated on that many objects.",
        "A blank cell is a case the round did not run.",
        "",
    ]
    kind = None
    for case in sorted(catalog, key=_order):
        entry = catalog[case]
        if case.split("/")[0] != kind:
            kind = case.split("/")[0]
            lines += [f"## {kind.replace('_', ' ')}", ""]
        specs = step_specs(entry)
        lines += [f"### {case}", "", f"> {entry['task']}", ""]
        ran = [
            (label, row, step_tally(data["cases"][case], row, entry))
            for label, _, row, _, data in rounds
            if case in data["cases"]
        ]
        demo = replay_steps(replay[case], entry) if case in replay else None
        head = "| # | Step | Checked as |"
        if demo:
            head += " R1 demo | R1 resume(k) |"
        head += "".join(f" {label} ({row}) |" for label, row, _ in ran)
        width = 3 + 2 * bool(demo) + len(ran)
        lines += [head, "|" + "---|" * width]
        for index, (step, spec) in enumerate(zip(entry["steps"], specs)):
            checked = "<br>".join(f"`{' '.join(s)}`" for s in spec) or NONE
            cells = ""
            if demo:
                pair = demo[index]
                cells += f" {_mark(pair and pair[0])} | {_mark(pair and pair[1])} |"
            cells += "".join(f" {_cell(t[index])} |" for _, _, t in ran)
            lines.append(f"| {index} | {step} | {checked} |{cells}")
        unchecked = _unchecked(entry, specs)
        if unchecked:
            lines += ["", "Predicates with no step: " + ", ".join(unchecked) + "."]
        videos = [
            f"[row {row}]({rel(GALLERY / 'video' / video)})"
            for row in ("C", "D")
            if (video := _video(row, case))
        ]
        if videos:
            lines += [
                "",
                "Rollout videos (seed 0): "
                + ", ".join(videos)
                + f"; all on the [gallery page]({rel(GALLERY / 'index.html')}).",
            ]
        if case in export["cases"]:
            lines += ["", export_line(export["cases"][case])]
        if case in handles["cases"]:
            record = handles["cases"][case]
            lines += [
                "",
                f"Grasp handles (R3): {record['handles']} on its {record['objects']} objects.",
            ]
        waiting = [
            str(index)
            for index, spec in enumerate(specs)
            if any(s[0] in ARTICULATED for s in spec)
        ]
        if waiting:
            lines += [
                "",
                (
                    f"Step{'s' * (len(waiting) > 1)} {', '.join(waiting)} wait"
                    f"{'s' * (len(waiting) == 1)} on articulated objects (R6,"
                    f" [#161]({ISSUE_URL}/161))."
                ),
            ]
        if not case.startswith("Ideal/"):
            lines += [
                "",
                (
                    f"Not run by a planner round yet: its condition waits on R7"
                    f" ([#162]({ISSUE_URL}/162))."
                ),
            ]
        elif not ran:
            lines += ["", "Not run by a planner round yet."]
        lines.append("")
    lines += [
        "## R1 demo replay",
        "",
        "The benchmark's recorded demonstration replayed through the env server (#156),",
        "one replay a case: straight through, and from each step's official `resume(k)`",
        "state (also from a fresh reset, and a fresh reset restored to the demo's state).",
        "Drift is how far the replay strays from the recorded states, the end effector",
        "and the objects. The demo does not complete every step: that is the dataset's",
        "own ceiling, not a replay error.",
        "",
        "| Case | Recorded | Straight through | resume(k) | Fresh | Fresh, restored | Drift (eef / objects) |",
        "|---|---|---|---|---|---|---|",
    ]
    for item in json.loads((RESULTS / REPLAY).read_text())["summary"]:
        lines.append(
            f"| [{item['case']}](#{item['case'].lower().replace('/', '')}) | {item['recorded']} |"
            f" {item['continuous']} | {item['benchmark_resume']} | {item['fresh']} |"
            f" {item['fresh_restore_demo']} | {item['continuous_max_eef_drift_m'] * 1000:.1f} mm /"
            f" {item['continuous_max_object_drift_m'] * 1000:.1f} mm |"
        )
    lines += [
        "",
        "## R2 scene export",
        "",
        "Every case's scene written out for HPP (#157): the objects' collision shapes,",
        "their contact surfaces and the robot, checked against MuJoCo (shape poses, the",
        "resting contacts, the arm's links) and with one test plan per layout. Cases",
        "share a layout across conditions; the exporter passes on all 60.",
        "",
        "| Layout (fixtures) | Cases | Test plan |",
        "|---|---|---|",
    ]
    for layout, cases in export["layouts"].items():
        planned = [
            export["cases"][c]["plan"] for c in cases if "plan" in export["cases"][c]
        ]
        plan = (
            f"{'✅' if planned[0]['ok'] else '❌'} {planned[0]['time']:.2f} s"
            if planned
            else NONE
        )
        lines.append(f"| {layout} | {', '.join(sorted(cases, key=_order))} | {plan} |")
    lines += [
        "",
        "## R3 grasp handles and grasp-lift",
        "",
        "Grasp handles generated for every object class (#158), in the scenes listed by",
        f"[`{HANDLES}`]({rel(RESULTS / HANDLES)}), then one pick of each class from where",
        "it starts in a scene with it, lifted (`pick <instance>`). Not a task step: it",
        "tests the grasps alone. Videos on the",
        f"[grasp gallery]({rel(GRASP_GALLERY / 'index.html')}).",
        "",
        "| Class | Instances | Handles (feasible) | Grasp-lift scene | Result | Why it failed | Video |",
        "|---|---|---|---|---|---|---|",
    ]
    for name, record in sorted(lift.items()):
        result = "✅ lifted" if record["success"] else "❌"
        video = GRASP_GALLERY / "video" / f"{name}.mp4"
        link = f"[video]({rel(video)})" if video.exists() else ""
        generated = handles["classes"].get(name, {})
        made = generated.get("example", {}).get("handles", [])
        feasible = sum(h.get("evaluation", {}).get("feasible", False) for h in made)
        cause = CAUSE.get(name, "") if not record["success"] else ""
        lines.append(
            f"| {name} | {generated.get('instances', '')} | {len(made)} ({feasible}) |"
            f" {record['case']} | {result} | {cause} | {link} |"
        )
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out}")


def _causes(path: Path) -> dict[str, str]:
    """The grasp gallery's failure causes (``CAUSE`` in ``build_html.py``,
    read without running the script)."""
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", None) == "CAUSE" for t in node.targets
        ):
            return ast.literal_eval(node.value)
    return {}


def _unchecked(entry: dict, specs: list[list[list[str]]]) -> list[str]:
    """Goal predicates no step is annotated with (the benchmark still checks
    them for ``all_done``)."""
    stepped = {json.dumps(s) for step in specs for s in step}
    return [
        f"`{' '.join(spec)}`"
        for predicates in entry["goal"].values()
        for spec in predicates
        if json.dumps(list(spec)) not in stepped
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("dump").add_argument("--out", type=Path, default=CATALOG)
    sub.add_parser("build").add_argument("--out", type=Path, default=DOC)
    args = parser.parse_args()
    (dump if args.command == "dump" else build)(args.out)


if __name__ == "__main__":
    main()
