# Open simulated benchmark for M11

Outcome of [#150](https://github.com/thanhndv212/long-tamp/issues/150), checked 2026-10-11.
**Choice: RoboCerebra**, implemented as #156–#163 under
[#153](https://github.com/thanhndv212/long-tamp/issues/153).

## Comparison

| | RoboCerebra | LIBERO-Long / LIBERO-Plus | RoboTwin 2.0 | ManiSkill3 | RoboCasa365 | TWIN/RLBench |
|---|---|---|---|---|---|---|
| Venue | NeurIPS 2025 D&B (2506.06677) | NeurIPS 2023 / CVPR 2026 | 2025 | 2024 | — | 2024 (PerAct2) |
| License | Apache-2.0 | MIT | MIT | Apache-2.0 | not declared | — |
| Simulator | LIBERO (robosuite 1.4.0, MuJoCo) | robosuite / MuJoCo | SAPIEN | SAPIEN | MuJoCo 3.3.1 | CoppeliaSim |
| Robot | Panda | Panda | dual-arm | many | Panda, GR1, mobile | dual Panda/UR5 |
| Horizon | ~9 subtasks (2–20+) | 2 subgoals | mostly short | mostly short | long, mobile | short |
| Disturbance / mismatch scored | yes (Random Disturbance, Observation Mismatching, Mix) | LIBERO-Plus perturbs the initial scene only | no | no | no | no |
| Goal spec | BDDL + `goal.json` predicates | BDDL | task code | task code | task code | task code |
| Best published | SparkVLA 47.1 SR (own subtasks and stops); GT plan + OpenVLA 25.2; HPE 16.6 | fine-tuned VLAs ~92–99% on Long | — | — | — | — |

## Why RoboCerebra

- It is long-horizon and scores recovery from disturbances and plan–observation mismatch,
  which is long_tamp's replan story.
- Low-level execution is the published bottleneck: a ground-truth plan with OpenVLA reaches
  only 25%. A motion planner addresses exactly that.
- It runs on LIBERO, so the same adapter later covers LIBERO-Long (comparability) and
  LIBERO-Plus (robustness).
- No planner-based entry has been published.

## Task subset and constraints

- **First subset:** the Ideal condition on `pick`/`in`/`on` subtasks, about 68% of the
  557 subtasks in the 60 test cases. `pour` adds 17% (#160). `open`/`close`/`turnon` need
  articulated objects (#161).
- **Version conflict:** robosuite 1.4.0 pins an old MuJoCo and `long_tamp[sim]` needs
  `mujoco>=3.11`. So the benchmark env runs in its own venv behind the remote backend
  (#125, #156).
- **Fairness:** results are reported in three rows:
  - A: privileged state, official protocol
  - B: simulated camera with the VLM goal writer, official protocol, comparable to the
    published rows
  - C: continuous run with no per-step resets; new, with no published baseline
- **Not chosen now:**
  - RoboTwin 2.0 and BiCoord: multi-arm, needs a SAPIEN import. These come later.
  - RoboDojo and BEHAVIOR-1K: they need an NVIDIA GPU for Isaac Sim.
  - ManiSkill3: short-horizon tasks.
  - RoboCasa: no declared license.
  - TWIN: kept as an internal regression test.

Detailed study (protocol, component mapping, risks) and the survey of follow-up work live
in the research vault: `agimus-spacelab/long-tamp-sim-benchmark-selection.md`,
`agimus-spacelab/long-tamp-robocerebra-feasibility.md`, `papers/robocerebra-2506.06677.md`.
