# Validation with the screw-assembly mission

`script/screw_assembly/` is the reference mission: two UR10 + Robotiq arms fasten N
plates to a jig (tool pickup, clamping, two screws per part, home retreats, tool
return). It is long-horizon, multi-arm and already reliable, so every behavioural
change is measured against a known-good baseline instead of judged by eye.

## Validation levels

| Level | What | Where | Time |
|---|---|---|---|
| **V0** local | `ruff check --select F src`, `black --check src`, tests of the touched modules (`pre-commit run -a` covers the lint part) | your machine | minutes |
| **V1** CI | lint, base-install tests, wheel tests, distribution, docs build, changelog check | GitHub Actions, every push/PR | ~20 min |
| **V2** smoke mission | one-part screw assembly, seed 1, plus the TWIN checks | `gh workflow run pypi.yml --ref <branch>` (the `nightly-*` jobs) | ~20 min |
| **V3** batch gate | 10 seeds × 4 parts, `summarize.py --gate` against the baseline | the baseline's environment (see below) | 1.5–4 h |
| **V4** scenario gate | the same plan from several initial states: screw assembly (`scenarios.py --all`, 4 scenarios) and the TWIN regrasp (3 scenarios) | same as V3 | ~1 h |

## Which change needs which level

| Change touches | Required |
|---|---|
| docs, CI, packaging only | V0, V1 |
| `task_planning/` (IR, compiler, session, host) | V0, V1, V2 (V4 once M1 lands) |
| `planning/`, `tasks/`, `backends/`, `grasping/` in a way that can change what gets planned | V0, V1, V2 |
| planner behaviour or performance (sampling, recovery, lookahead, optimizers, timeouts), or labelled `needs-batch-gate` | V0–V3 |
| closing a milestone, or a release commit | V0–V3, plus V4 from M1 on |

When unsure, run the next level up. A skipped level is stated in the PR's Validation
section, with the reason.

## Running the V3 batch gate

Inside the environment of the baseline you compare against:

```bash
cd script/screw_assembly
python build_scene.py --parts 4
bash run_batch.sh 10 3 batch_<topic>           # 10 seeds, 3 in parallel
python summarize.py batch_<topic> \
    --json results/$(date +%F)-<topic>.json \
    --gate --baseline results/pypi-wheel-batch-2026-09-26.json
```

The gate passes when **all** of these hold:

- every mission completed (10/10);
- replanning trigger rate **< 2 %** (planning blocks with at least one replan);
- recovery rate **> 95 %** (failure episodes that still finished);
- median mission time **≤ 1.25×** the baseline's.

Commit the `results/*.json` file in the PR and paste the gate output into the PR's
Validation section. A failing gate blocks the merge unless the PR explains why the
baseline itself must move (see below).

## Running the V4 scenario gate

Each scenario reaches its start state by planning part of the mission for real (the
*setup*), then runs the **unchanged** plan from there. It passes when the mission
completes, the work the setup did is skipped (effects that hold, parts already done), and
none of it is planned again. Every scenario runs in its own process.

```bash
cd script/screw_assembly
python build_scene.py --parts 2
python scenarios.py --all --seed 1 --json results/$(date +%F)-scenarios.json
cd ../twin
python scenarios.py --all --seed 1
```

| Screw assembly (2 parts) | Start state | Expected skips |
|---|---|---|
| `nominal` | nothing held | none |
| `driver_in_hand` | ur10_right holds the driver | the pickup |
| `part_in_hand` | ur10_left holds part 1 | part 1's grasp |
| `part1_assembled` | part 1 clamped, screwed, released; driver held | the pickup, all of part 1 |

| TWIN regrasp | Start state | Expected skips |
|---|---|---|
| `nominal` | nothing held | none |
| `left_holds_ball` | panda_left holds `ball/handle` | the first grasp |
| `both_hold` | panda_left on `ball/handle`, panda_right on `ball/handle2` | the first grasp; the left arm then releases and regrasps while the right arm holds the ball |

Every scenario's start state is also checked without HPP in the test suite: the same plan
must pass the load-time check from it.

## Baselines are per environment

Timing depends heavily on the environment. The same mission ran with a median of
**687 s** on the stock PyPI HPP wheels (`results/pypi-wheel-batch-2026-09-26.json`)
and **~1220 s** in a source-built HPP container. Only compare a batch against a
baseline from the same environment. The canonical baseline is the **PyPI-wheel
environment** (what CI and users run). If you only have another environment,
first record a baseline there from `dev`, then run your branch, and put both files in
the PR.

Updating the canonical baseline is a deliberate change: its own PR (or a clearly
separated commit), with the reason (e.g. an intentional speed/robustness trade-off)
and the old and new numbers.

## Milestone gate

A milestone closes when its exit test (listed in [plans/roadmap.md](../plans/roadmap.md))
passes on `dev`, with V3 (and V4 from M1 on) result files committed under
`script/screw_assembly/results/` and linked from the milestone's closing notes.

## Seeds and determinism

HPP's configuration shooter is not seeded unless the script seeds it:
`task_screw_assembly.py --seed S` seeds libc `srand` and pinocchio. Never compare
unseeded runs; N unseeded runs are one run repeated N times.
