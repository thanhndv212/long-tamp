# RoboCerebra env server

[RoboCerebra](https://github.com/buaa-colalab/RoboCerebra) (NeurIPS 2025 D&B,
arXiv 2506.06677) is the long-horizon benchmark of milestone M11. It runs on a
LIBERO fork on robosuite 1.4, Python 3.10 and `numpy<2`, so it can't share
an interpreter with long_tamp. `env_server.py` serves one benchmark env from its
own Python 3.10 env, over the wire of `long_tamp.execution.remote` (length-prefixed
JSON frames, the tagged codec, no pickle). `long_tamp.eval.RoboCerebraClient` is
long_tamp's side (issue #156).

| File | Runs in | What |
|---|---|---|
| `env_server.py` | benchmark env | the server; numpy + stdlib at import, robosuite/LIBERO on `load` |
| `replay_demo.py` | long_tamp env | replays a case's demo through the client, measures drift |
| `results/` | | replay logs |

## Operations

`cases()`, `load(case, model="bddl")` (builds the env: Panda, `OSC_POSE`, 20 Hz,
as the benchmark's `task_runner.py`), `reset(step=0, settle=None, restore_demo=False)`,
`resume(step)`, `step(actions)` ((7,) or (N, 7)), `check()`, `get_state()`,
`set_state(sim_state, controller=None)`, `render(cameras=None, size=256)`, `demo()`,
`export_scene()`, `set_goal(goal)` (replaces the loaded case's goal, for example
`{"butter_1": [["pick", "butter_1"]]}` as a one-step task: the pick checker judges
only objects in the goal).
See the docstrings in `env_server.py`.

`model="bddl"` is the benchmark's scene, built from the case's BDDL. `"demo"` is the
demo's own recorded MJCF, for replays. Its `*_region` sites are copied from the BDDL
scene, because the fork's `_get_table_z_offset` puts the table regions 0.4 m higher
than in the recorded model, and with the recorded sites no `In` predicate holds.

Episodes follow the benchmark's `episode.py`:

- `reset(0)` starts from the case's init file, then 15 dummy steps settle before
  the success baseline.
- `resume(k)` is the benchmark's mid-episode resume. It sets the demo's state at the
  start of step `k`, marks earlier steps' subtasks done (`resume.py`), and skips the
  pick-quaternion check once. It doesn't reset, so the controller goal, the gripper
  command and the checker's caches carry over from the previous segment.
- `reset(k)` for `k > 0` starts a fresh episode at step `k` instead. It holds the
  current pose, or with `restore_demo=True` it restores the demo's controller and
  gripper commands. Neither is part of the sim state.

Success is the env's own stateful `_check_success(goal)`, called after every step.
The benchmark's segment schedule (150 steps a step, `switch_steps`) is the caller's
job, not the server's.

Two controller details matter for any state jump:

- `OSC_POSE` latches its orientation goal. `set_goal` changes `goal_ori` only when
  the scaled rotation delta is nonzero.
- `env.reset()` updates the controller's pose cache at the reset pose. The first
  action after a `set_state` still sees that stale pose, unless the state is applied
  with a forced controller update.

The demos were recorded with the stale cache at step 0, so `reset(0)` keeps it.
Every other jump syncs the controller.

Cameras render only on `render`, so `step` stays physics-only. With Mesa's llvmpipe,
two 256×256 cameras cost about 0.35 s. Pass `--no-render` where there is no EGL.

## Set up (the `hpp-agimus-arm64` container)

```bash
mkdir -p ~/devel/hpp/robocerebra && cd ~/devel/hpp/robocerebra
git clone https://github.com/buaa-colalab/RoboCerebra.git src
conda create -y -n robocerebra python=3.10
conda activate robocerebra
conda install -y -c conda-forge mesalib libgl-devel libegl-devel libopengl-devel
pip install --no-deps robosuite==1.4.0   # its pynput/evdev deps don't build on arm64
pip install "numpy<2" numba scipy mujoco==3.2.7 Pillow opencv-python-headless termcolor \
    bddl easydict cloudpickle gym "imageio[ffmpeg]" h5py huggingface_hub matplotlib pyyaml
pip install --no-deps -e src/LIBERO --config-settings editable_mode=compat
hf download qiukingballball/RoboCerebraBench --repo-type dataset --local-dir RoboCerebra_Bench
```

Two versions are pinned on purpose:

- **robosuite 1.4.0.** The demos were recorded with it. 1.4.1 adds a
  `robot0_link7_collision` geom to the Panda, so a demo's recorded model fails to
  load, and it changes the gripper's reset. RoboCerebra's evaluation installs
  openvla-oft's LIBERO requirements, which pin 1.4.1, so the published scores ran
  on 1.4.1 against 1.4.0 demos. 1.4.1 is fine for `model="bddl"` runs.
- **MuJoCo 3.2.7.** robosuite 1.4 fails on MuJoCo 3.16: `get_joint_qpos_addr`
  asserts on the joint-type enum.

LIBERO asks for a config path on first import. Write
`$LIBERO_CONFIG_PATH/config.yaml` yourself, pointing `benchmark_root`,
`bddl_files`, `init_states`, `datasets` and `assets` into `src/LIBERO/libero/libero`.

`env.sh` activates the env and sets the rest:

```bash
source /opt/conda/etc/profile.d/conda.sh && conda activate robocerebra
export LIBERO_CONFIG_PATH=$RC_DIR/libero_config RC_BENCH=$RC_DIR/RoboCerebra_Bench
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless
export __EGL_VENDOR_LIBRARY_DIRS=$CONDA_PREFIX/share/glvnd/egl_vendor.d
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

## Run

Serve a case, from the benchmark env:

```bash
python script/robocerebra/env_server.py --bench $RC_BENCH --port 7071
```

Drive it from long_tamp:

```python
from long_tamp.eval import RoboCerebraClient

with RoboCerebraClient(port=7071) as env:
    info = env.load("Ideal/case1")
    env.reset(step=0)
    result = env.step(actions)            # (N, 7) OSC_POSE
    result["total_completed"], result["all_done"]
```

`RoboCerebraClient.spawn(command)` starts the server itself. `replay_demo.py`
shows how, unsetting the HPP env's `PYTHONPATH` and `LD_LIBRARY_PATH` first so
they don't shadow the benchmark env's numpy:

```bash
python script/robocerebra/replay_demo.py Ideal --out script/robocerebra/results/<date>-r1-replay-ideal-demo.json
```

It takes cases (`Ideal/case1`), task types (`Ideal`) or `all`.

## Segmented policy evaluation

`long_tamp.eval.evaluate_cases` and `evaluate.py` provide the rollout harness
for planner policies. A policy callback receives `(env, case_info, segment,
step_limit)` and returns an `(N, 7)` array of normalized `OSC_POSE` actions, or a
controller `controller(obs) -> (7,)` stepped closed loop from each observation
(its `info` dict is recorded with the segment).
The harness seeds each reset, caps each segment at 150 actions by default,
checks the benchmark goal after every action, and enters later segments with
the benchmark's official `resume(k)` operation. It records the seed, action
count and success for each segment and rollout, plus success rate per case.

```bash
python script/robocerebra/evaluate.py Ideal/case1 \
    --policy my_policy:act_segment --rollouts 10 --seed 100 \
    --out results/ideal-policy.json
```

The summary adds segment-level counts next to the per-case rollout success
rate: `resume(k)` marks earlier steps done, so a rollout can "succeed" with
failed segments. `supported_segment_success_rate` counts the segments whose
predicates registered within their budget, leaving out the ones the policy
reported as `unsupported` (`supported_segment_success_by_case` per case);
`mean_steps_used_on_success` is against the budget.

Long runs can be split, one case and seed per process run in parallel, and
joined:

```bash
python script/robocerebra/evaluate.py --merge parts/*.json --out results/ideal-policy.json
```

The callback is the policy boundary: the harness does not substitute recorded
demo actions for a planner. A completed benchmark result must use a callback
that plans and executes the goal predicates through the project MissionRunner
and report the resulting per-case rollout artifact.

## Replay results

`results/2026-10-17-r1-replay-ideal-demo.json`: the 10 Ideal cases, `model="demo"`,
robosuite 1.4.0, arm64 container (llvmpipe, no rendering).

| Case | Recorded | Continuous | Max drift eef / object (m) | Steps/s | Resume: steps done, max end drift eef / object (m) | Fresh | Fresh, demo restored |
|---|---|---|---|---|---|---|---|
| 1 | 4/6 | 4/6 | 0.0017 / 0.0017 | 181 | 3/6, 0.0002 / 0.0083 | 3/6 | 3/6 |
| 2 | 1/4 | 1/4 | 0.0066 / 0.0083 | 134 | 1/4, 0.0032 / 0.0012 | 1/4 | 1/4 |
| 3 | 8/10 | 8/10 | 0.0064 / 0.0059 | 142 | 9/10, 0.0005 / 0.0007 | 8/10 | 9/10 |
| 4 | 4/6 | 4/6 | 0.0005 / 0.0005 | 120 | 4/6, 0.0005 / 0.0005 | 4/6 | 4/6 |
| 5 | 6/6 | 6/6 | 0.0005 / 0.0005 | 163 | 6/6, 0.0026 / 0.0027 | 5/6 | 6/6 |
| 6 | 6/6 | 6/6 | 0.0005 / 0.0005 | 152 | 6/6, 0.0004 / 0.0004 | 6/6 | 6/6 |
| 7 | 3/7 | 3/7 | 0.0146 / 0.0110 | 125 | 4/7, 0.0005 / 0.0005 | 4/7 | 4/7 |
| 8 | 7/7 | 7/7 | 0.0005 / 0.0004 | 104 | 7/7, 0.0005 / 0.0005 | 7/7 | 7/7 |
| 9 | 7/11 | 7/11 | 0.0447 / 0.0035 | 167 | 9/11, 0.0004 / 0.0008 | 7/11 | 9/11 |
| 10 | 10/13 | 10/13 | 0.0026 / 0.0048 | 100 | 10/13, 0.0004 / 0.0004 | 8/13 | 10/13 |
| **All** | **56/76** | **56/76** | | | **59/76** | **53/76** | **59/76** |

"Recorded" and "Continuous" count subtasks of the case's goal. The resume columns
count benchmark steps whose expected subtasks complete within their own span.

- **The replay reproduces the demo.** The continuous replay scores exactly what the
  recorded states score, in every case. Objects stay within 1.1 cm of the recorded
  states. The worst end-effector drift is 4.5 cm, on case 9.
- **Resumes are faithful.** The benchmark's `resume(k)` ends each step within 3.2 mm
  of the demo, and so does a fresh episode that restores the demo's controller and
  gripper. A fresh episode that only holds the pose drifts much further, up to 1.1 m
  for an object on case 2: the latched orientation goal and the gripper command
  matter.
- **The demos don't fully meet their own goals.** They complete only 56 of 76 goal
  subtasks (case 1: popcorn not `In` right_side, butter not `In` left_side, even on
  the recorded states). Some step annotations also lag the demo. On case 1, the
  butter pick of step 4 registers at demo step 2088, inside step 5's span. So
  replaying the demo isn't a 100 % oracle. Compare planners against the 56/76
  ceiling, or against `goal.json` with the dataset caveat.

## Scene export (R2)

`export_scene()` turns the loaded case's compiled MuJoCo model into a scene HPP loads
(`scene_export.py`, #157):

- the static fixtures as one environment model, `environment.urdf`, whose
  `environment.srdf` holds each LIBERO `*_region` site as a contact surface on the geom
  that supports it;
- each movable object as its own freeflyer model, `objects/<name>.urdf|srdf`, with a
  `bottom` contact;
- a manifest: object poses, the robot base pose, links and joints, the regions (with a
  `reason` when one has no support), the articulated joints, every exported shape's
  pose under its pinocchio name, and MuJoCo's contacts.

```python
from long_tamp.eval import RoboCerebraClient, write_export

scene = write_export(client.export_scene(), "out/Ideal_case1")
scene.file_paths()            # SceneBuilder.FILE_PATHS entries, environment + objects
scene.contacts                # the environment contact surfaces
scene.object_config("butter_1")
scene.robot_pose()            # where to put the Panda
```

`export_scene.py` exports cases, loads each in HPP and compares it with MuJoCo:

```bash
python script/robocerebra/export_scene.py all --plan \
    --results script/robocerebra/results/2026-10-18-r2-scene-export.json
```

It runs in the HPP environment and starts the env server itself. `--plan` also plans one
free arm motion per layout.

`results/2026-10-18-r2-scene-export.json`, all 60 cases:

| Check | Result |
|---|---|
| Distinct layouts (sets of fixtures) | 13 |
| Load errors, missing shapes, missing contact surfaces | 0 |
| Shape pose error, max | 8.9e-10 m, 1.4e-5 rad |
| Arm link pose error, max (link0..7) | 1.1e-5 m, 1.1e-4 rad |
| Finger frame offset, max | 6.5 mm |
| Collision disagreements with MuJoCo | 0 |
| Resting contacts (HPP distance ≤ 0) / near pairs (< 1 mm) | 892 / 278 |
| Contact distance error, max | 1.1e-4 m |
| Robot clearance at the start, min | 68 mm |
| Free-space plans | 13/13 |

- **A disagreement** is a pair HPP sees penetrating by more than 1 mm with no MuJoCo
  contact, or a MuJoCo contact HPP sees more than 1 mm apart.
- **Resting objects touch.** MuJoCo settles objects about 0.015 mm into their support,
  so HPP sees them in collision at the start. R3 has to treat these pairs as contacts
  (the object's placement), not as collisions.
- **The fingers** sit 6.5 mm off: robosuite's finger frames and the Panda URDF's
  differ by that offset. The arm links match.
- **Not every region is a support surface.** `white_storage_box_1_left_side|right_side`
  (6 cases each), `bowl_drainer_1_left|right_region` (6 each), `basket_1_contain_region`
  (3) and `wine_rack_1_top_region` (3) are vertical slabs, not horizontal footprints, so
  they have no contact. Case 1's two failed `In` subtasks of the demo are on the storage
  box sides. Placing into these needs a different model than a support contact.
- **Articulated joints** (drawers, doors) are exported in the manifest but locked at their
  current value in the environment model.
- The issue counts 15 layouts. The 60 cases have 13 distinct sets of fixtures.

## Grasp handle coverage (R3, #158)

`grasp_library.py` exports a class-covering set of cases and writes Panda handle
frames into each object's SRDF. The recorded run is
`results/2026-10-25-r3-grasp-handles.json`: 8 cases, 46 objects, 25 classes,
251 handles, and no missing classes. It includes a top-rim sampler for thin
container walls. The report marks handles whose approach was blocked by the
current scene as `requires_scene_planning`; those are object-level candidates,
not proven executable grasps.

`grasp_lift.py` checks them in the benchmark env (AC2): per class, its example
case is reset with the goal `pick <instance>`, the HPP segment policy (below)
plans a grasp and lift, and `OSC_POSE` tracks it until the pick checker passes
or 150 steps run out. The recorded run is
`results/2026-10-26-r3-grasp-lift.json`: **14 of 25 classes picked**
(akita_black_bowl, alphabet_soup, bbq_sauce, butter, chefmate_8_frypan,
chocolate_pudding, cookies, cream_cheese, popcorn, red_coffee_mug,
tomato_sauce, white_yellow_mug, wine_bottle, wooden_tray; 42 to 107 steps),
and **4 more held** (below): 18 of 25 grasped and lifted without failing the
checker's rotation test. Some run at another instance than the inventory's
example (`--at`): akita_black_bowl, butter and white_bowl in Ideal/case7, the
frypan in Ideal/case4 (in case10 none of its handles plans; that attempt is
kept under `attempts`).

Every row records the object's pose from the observations: `lifted`, how much
higher it ended than it started, and `rotation`, the checker's own test (the
change of its quaternion over 40 steps, failing above 0.1) at its worst over
the rollout. The checker counts a pick when the object goes from some region
to none, not by height. Each row records the regions the object starts in
(`start_regions`); one that starts in none, or in its own (the basket's contain
region travels with it), is `unregistrable`: lifting it cannot register. Such
a row is `held` when the object rose at least 5 cm with `rotation` under the
threshold: the grasp and lift worked, as far as the benchmark can tell without
its region test. The bottles' origins sit just above the table region's band
(z < 0.477 m): basket, ketchup, milk and orange_juice are held, lifted 9 to 10
cm with `rotation` at most 0.034. bbq_sauce, 2 mm above the band, registered
while the closing fingers pressed it into the band and out again, before any
lift. The 7 failures:

| Failure | Classes |
| --- | --- |
| no handle plans in the scene | black_book, plate, rack |
| every grasp needs a joint at its limit under OSC | glazed_rim_porcelain_ramekin, macaroni_and_cheese |
| grasped, lift does not plan | moka_pot (also unregistrable) |
| grasped, slips on the lift | white_bowl |

OSC moves the arm toward each hand pose by the shortest joint motion, so it
does not follow HPP's joint path: from the start posture it stays on another
branch of the arm's redundancy, where the same hand pose can need a joint past
its limit. Before, the arm then stalled 4 to 8 cm short with the joint at its
stop (wooden_tray's wrist). The pick now rolls OSC out by IK pose by pose along
the tracked path and takes a handle whose rollout stays 0.05 rad inside the
limits, planning the lift from where the rollout ends. The ramekin and the
macaroni box sit 20 cm in front of the robot's base, where every approach
folds the elbow to its stop on the start branch; HPP reaches them only with
the shoulder and upper arm twisted (j1 and j3 near ±1.5), a branch `OSC_POSE`
actions cannot move the arm to.

The bowls plan because the rim sampler takes walls leaning up to ~20 degrees
(theirs lean 15), and white_bowl because the pick settles a start MuJoCo
left 0.13 mm off HPP's placement (`settled`). The plate's rim is a near-flat
slab no top-down pinch reaches without the hand in the table.

Run it in the HPP environment with the benchmark server available:

```bash
python script/robocerebra/grasp_library.py --cases-file /path/to/r3-cases.txt
```

## HPP segment policy (R4, #159)

`long_tamp.eval.robocerebra_policy:hpp_policy` plans each segment in HPP on the
scene exported at the segment's start and tracks the plan with `OSC_POSE`:

```bash
python script/robocerebra/evaluate.py Ideal/case1 --rollouts 10 \
    --policy long_tamp.eval.robocerebra_policy:hpp_policy --out results/ideal-hpp.json
```

- Goals: `pick o` becomes *grasped* (the first R3 handle that plans, then a
  straight lift); `In o r` / `On o r` become *placed* (carry the held object into
  `r`'s box and release). `pour`, `open`, `turnon` and `On` over an object are
  rejected as unsupported, as is a region the exported scene lacks (`rack_1`);
  the segment then holds still with the gripper closed.
- Each segment is planned in a forked child process (`HppPolicy(isolate=True)`,
  the default): HPP keeps native memory from every problem it builds, and a
  13-segment case10 rollout in one process ran out of memory.
- The held object at a resume is the one both fingers touch (`robot_contacts`).
- `OSC_POSE` tracks TCP poses, not joints, and HPP's configs can sit on another
  IK branch than the robot's, so transits are shortcut along straight TCP lines
  checked by IK seeded from the start. Resting objects sink about 1 mm into their
  support in MuJoCo; a pick raises the target out of it before planning.
- Each segment record has `steps_used` against the 150-step budget, the
  checker's printed lines (`monitor`) and the plan's `policy` info (stages,
  handle, rejected place orientations, planning errors).

### Row A: Ideal, HPP segment policy

`results/2026-10-25-r4-ideal-hpp.json`: the 10 Ideal cases × seeds 0–9, one
(case, seed) per process, three in parallel, merged with `--merge`. Every segment starts from the
official `resume(k)` state and has 150 steps.

| | |
| --- | --- |
| segments | 760 (220 unsupported) |
| **supported segments passed** | **412 / 540 (76.3%)** |
| mean steps used, passed segments | 63.2 of 150 |

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| supported segments passed | 56/60 | 21/30 | 50/70 | 30/40 | 40/40 | 31/40 | 24/40 | 40/40 | 60/80 | 60/100 |
| rollout success | 0.9 | 0 | 0 | 0 | 1.0 | 1.0 | 0 | 0 | 1.0 | 1.0 |

Rollout success, the benchmark's metric, is not a measure of this policy: the
resume marks earlier steps done and puts the object in the hand, so case9 and
case10 succeed on every seed with a quarter or more of their segments failed.
case8 passes every supported segment and still fails every seed, on its
unsupported ones.

The failures are mostly the same segment on every seed:

| Segment | Seeds failed | Cause |
| --- | --- | --- |
| case3 0, pick plate | 10 | no handle of the plate plans |
| case3 1, plate onto the table | 10 | no release pose in the region |
| case4 5, frypan into the cabinet top | 10 | the held start collides (hand and frypan) |
| case7 6, frypan onto the stove | 10 | no release pose in the region |
| case9 0 and 9, pick white_bowl | 10 + 10 | planned, not picked; on the seeds inspected the arm stalls near an elbow singularity (0) and the fingers push the rim and the bowl slides (9) |
| case10 4, 7, 10, carry to the table | 10 each | the carry to the crowded table does not plan |
| case10 5, pick bbq_sauce | 10 | no handle plans |
| case2 2 and case7 5, pick frypan | 9 and 5 | planned, but the 1.1 m transit uses most of the budget |
| case6 1, pick butter | 6 | no handle plans |
| case6 3, pick tomato_sauce | 3 | planned, not picked |

The other 5 failures are case1's butter pick (2 seeds), popcorn pick and
butter place, and case7's pudding pick (1 seed each).

### Published results on RoboCerebra

The paper's metric differs from ours. Its success rate (Eq. 1) is the share of a
task's key object-state transitions that hold, checked by simulator predicates.
It is averaged over 60 tasks × 10 trials, and the paper gives no step budget.
All published methods pair a planner with a learned VLA controller and run each
task in one go, without resuming. Success, % (best Ideal in bold):

| Method | Source | Avg | Mix | Ideal |
| --- | --- | --- | --- | --- |
| OpenVLA, LIBERO-trained | [RoboCerebra] Table 3 | 2.00 | 0.00 | 4.05 |
| OpenVLA fine-tuned on RoboCerebra | Table 3 | 4.57 | 0.00 | 7.84 |
| GPT-4o planner + OpenVLA | Table 3 | 16.04 | 11.48 | 21.92 |
| Hierarchical framework (HPE), the official baseline | Table 3 | 16.55 | 13.21 | 21.10 |
| GPT-4o planner + π0-fast | Table 6 † | 15.15 | 10.18 | 19.47 |
| Ground-truth plan + OpenVLA | Tables 4, 6 † | 25.16 | 19.26 | 31.23 |
| Ground-truth plan + π0-fast | Table 6 † | 23.04 | 26.55 | 26.58 |
| π0.5 alone | [SparkVLA] Table 1 ‡ | 12.29 | 17.71 | 10.53 |
| Mem-0, reproduced | [SparkVLA] Table 1 ‡ | 20.29 | 19.81 | 19.74 |
| HarnessWAM: VLM task graph + world-action model | [HarnessWAM] abstract | – | – | 23.7 |
| SparkVLA: π0.5 planner + π0.5 executor | [SparkVLA] Table 1 ‡ | **47.12** | 48.52 | **46.38** |

† The rows for Tables 4 and 6 come from a machine extraction of the arXiv HTML,
not the page text. Check them against the PDF before citing.
‡ The columns are mapped by SparkVLA's HPE row, which repeats Table 3.

Our Ideal numbers, from row A and the demo replay:

| | Ideal |
| --- | --- |
| HPP policy, supported segments passed | 412 / 540 (76.3%) |
| HPP policy, all segments passed (unsupported counted as failed) | 412 / 760 (54.2%) |
| HPP policy, rollout success (mean over cases) | 49% |
| Demo replay, subtasks completed | 56 / 76 (73.7%) |

These numbers don't compare like-for-like:

- **Inputs.** The HPP policy reads each segment's goal predicates and the exact
  scene state, so it should be compared with the ground-truth-plan rows, not the
  VLM-planner rows. The VLAs work from camera images.
- **Resuming.** Every one of our segments starts from the official `resume(k)`
  state, so an early failure doesn't carry into later segments. The published
  rollouts run each task through without stopping.
- **Unit.** Our unit is a segment, and theirs is a key state transition. The two
  usually coincide, but not always.
- **Coverage.** We have only run Ideal; Mix and the Memory settings are still to
  do.

[RoboCerebra]: https://arxiv.org/abs/2506.06677
[SparkVLA]: https://arxiv.org/abs/2608.16172
[HarnessWAM]: https://arxiv.org/abs/2608.09516
