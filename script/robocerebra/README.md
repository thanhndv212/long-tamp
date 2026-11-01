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
| `task_catalog.py` | both | every case's task, steps and goal predicates (`dump`, benchmark env) and each round's progress on them (`build`, any Python) into [`docs/plans/robocerebra-tasks.md`](../../docs/plans/robocerebra-tasks.md) |

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

`--continuous` runs each rollout through without `resume(k)` (row C), with
`long_tamp.eval.robocerebra_policy:hpp_policy_continuous`; the summary's
`completed_share` is the paper's Eq. 1.

`--mission` (row D) hands each rollout's whole goal to `long_tamp.mission`:
`long_tamp.eval.robocerebra_mission:evaluate_mission_case` describes the case as
a mission domain and runs it with `MissionRunner.run_with_repair` in the env,
with a step budget of 150 × the case's steps shared over the rollout.

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
lift. Screenshots and a real-time video of every class are in
[`results/r3-grasp-gallery/`](results/r3-grasp-gallery/index.html) (work in progress, rebuilt
with [`gallery/`](gallery/README.md)). The 7 failures:

| Failure | Classes |
| --- | --- |
| no handle plans in the scene | black_book, plate (edge pinch since, #188: picked in 630 steps), rack |
| every grasp needs a joint at its limit under OSC | glazed_rim_porcelain_ramekin, macaroni_and_cheese |
| grasped, lift does not plan | moka_pot (also unregistrable) |
| fingers close on air short of the rim; the bowl never moves (in case4 it lifted but tilted 0.25) | white_bowl |

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

The plate picks since #188 by an edge pinch instead
(`long_tamp.eval.robocerebra_push`). When no handle of a flat object (at
most 3 cm thick, upright) plans, the pick pushes it to the edge of its
support region nearest the robot's base, then pinches the part that hangs
over: the hand comes in sideways along the edge, fingers closing vertically
on the rim, and lifts. The push is short strokes, 6 cm each, the closed
fingertips 2.2 cm above the table and re-seated behind the far rim after
each one: on one long push they ride over the plate's few-mm rim. Its goal
is 2.5 cm past the edge, moved along the edge 7, 14 or 21 cm either way
until the sweep clears every object low enough to hit the plate (the moka
pot in case3; its handle passes above) and the hand's way in clears
everything below 15 cm. Both are scripted from the observations, not
planned by HPP. `results/2026-11-01-plate-edge-pinch.json`: Ideal/case3
picked on 10 of 10 seeds, 5 strokes, 513 to 594 steps (seed 3: 748),
and Ideal/case5 (the coffee table) at seed 0 in 630. That is past R3's 150
steps and row A's 150 a segment; the rows sharing a budget over the rollout
(C, D) can afford it. The checker's rotation test passes at the pick
though the plate turns more than that while it is pushed (worst 0.24 to
0.27 over the rollout).

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
  straight lift; a flat object none of whose handles plans is pushed to the
  table's edge and pinched there instead, #188); `In o r` / `On o r` become *placed* (carry the held object into
  `r`'s box and release); `pour o v` becomes *poured* (below). `open`, `close`
  and `turnon` are rejected as unsupported, as is a region or object the
  exported scene lacks; the segment then holds still with the gripper closed.
- `On o x` over an object (#187; all 15 in the benchmark are onto `rack_1`:
  the frypan in case2 of Ideal, Observation_Mismatching and Random_Disturbance,
  the butter and the mug in Memory_Execution 1–3 and Mix 7–9). LIBERO's object
  `check_ontop` passes when the two touch, `o`'s origin is no lower than
  `x`'s and within 3 cm of it across. The place goes to a region made from
  `x`'s collision geometry at the segment start (`object_top_region`): a
  2 cm square about its origin, resting on the highest face under that origin
  (the rack's plate, between its pegs) or, for an object too wide to go
  between, up to its highest face (the pegs' tops). The demo's own place
  never registers: it leaves the frypan on the pegs 4.6 cm off the rack's
  origin. The frypan is held by a rim pinch, fingers above and below, and
  the rack is 0.85 m out: the place tries the hand as it holds the object
  turned about the vertical (the hand on the near side) right after `held`,
  before turning it on its side, and a release with the fingers closing
  vertically after all the others. case2 step 3 then releases at `held/180`
  within 0.5 cm of the origin. In Memory_Execution 1–3 (row A, seed 0; Mix
  7–9 are the same scenes) the butter goes on the plate as held, and the mug,
  with `held` out of reach, upright at `held/90` or `held/180`: 3/3 each.
  Tried after the side approaches, the mug was released on its side and
  rolled off the pegs in two of the three.
- Pour (R5, #160). LIBERO's checker passes a pour when, within its last 40
  steps, the two objects' origins came within 0.3 m and the poured object's
  quaternion moved by more than 0.1 (0.2 in some scene classes); nothing
  needs to fall out. Every pour in the benchmark follows the pick of the
  same object, so the resume starts it in the hand, as a place does. The
  plan carries it upright over the receiver (its center first, then backed
  off towards the start, turned about the vertical where a leaning grasp is
  out of reach unturned), tilts the hand 60, 45 or 30 degrees (2 sin(a/4) =
  0.52 to 0.26 on the checker's measure) and turns it back. The tilt is about
  the hand's x axis first, across the fingers' closing direction, so gravity
  twists the object against both finger pads rather than about the line
  between them. If no carry plans and the object starts within 0.25 m of
  the receiver, it tilts in place. Many goals the instructions call "place
  X into the bowl" are `pour` predicates in `goal.json`.
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

`results/2026-10-29-r5-ideal-hpp-pour.json` (with pour, R5): the 10 Ideal cases
× seeds 0–9, one (case, seed) per process, three in parallel, merged with
`--merge`. Every segment starts from the official `resume(k)` state and has 150
steps. The R4 column is `results/2026-10-25-r4-ideal-hpp.json`, the same
protocol before pour.

| | R4 | R5 (pour) |
| --- | --- | --- |
| segments | 760 (220 unsupported) | 760 (50 unsupported) |
| supported share | 71.1% | 93.4% |
| **supported segments passed** | **412 / 540 (76.3%)** | **603 / 710 (84.9%)** |
| all segments passed | 412 / 760 (54.2%) | 603 / 760 (79.3%) |
| pour segments passed | – | 170 / 170 |
| mean steps used, passed segments | 63.2 of 150 | 59.0 of 150 |

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| supported segments passed, R4 | 56/60 | 21/30 | 50/70 | 30/40 | 40/40 | 31/40 | 24/40 | 40/40 | 60/80 | 60/100 |
| supported segments passed, R5 | 57/60 | 30/30 | 80/100 | 50/60 | 60/60 | 42/50 | 44/60 | 60/60 | 90/100 | 90/130 |
| rollout success, R4 | 0.9 | 0 | 0 | 0 | 1.0 | 1.0 | 0 | 0 | 1.0 | 1.0 |
| rollout success, R5 | 0.9 | 0 | 1.0 | 0 | 1.0 | 1.0 | 0 | 1.0 | 1.0 | 1.0 |

Rollout success, the benchmark's metric, is not a measure of this policy: the
resume marks earlier steps done and puts the object in the hand, so case3 and
case10 succeed on every seed with a fifth or more of their segments failed.
case2 passes every supported segment and still fails every seed, on its
unsupported one. The 50 unsupported segments are `open` (case6, case8, case9),
`turnon` (case7) and case2's `rack_1` region, missing from the export.

Every pour passes, in 42.9 steps on average and 76 at most (the checker's
40-step window fills at 38–39): all tilt about the hand's x axis after a
straight carry. The pour pose clears the receiver's top measured from its
collision geometry; an earlier run that took it from the manifest's bottom
face, mirrored through the origin, aimed bowls and the frypan 3–14 cm low,
and case10's tomato sauce clipped a mug on the way down on 8 seeds.

The failures are the same segment on every seed, mostly:

| Segment | Seeds failed | Cause |
| --- | --- | --- |
| case3 0, pick plate | 10 | no handle of the plate plans (edge pinch since, #188) |
| case3 1, plate onto the table | 10 | no release pose in the region |
| case4 5, frypan into the cabinet top | 10 | the held start collides (hand and frypan) |
| case7 6, frypan onto the stove | 10 | no release pose in the region |
| case9 9, pick white_bowl | 10 | planned, not picked (fails in the lift) |
| case10 4, 7, 10, carry to the table | 10 each | the carry to the crowded table does not plan |
| case10 5, pick bbq_sauce | 10 | planned, not picked (fails in the lift) |
| case6 1, pick butter | 6 | no handle plans |
| case7 5, pick frypan | 6 | planned, but the 1.1 m transit uses most of the budget |
| case6 3, pick tomato_sauce | 2 | planned, not picked |

The other 3 failures are case1's popcorn pick, butter pick and butter place
(1 seed each).

### Row C: Ideal, uninterrupted (continuous variant, #163)

`results/2026-10-29-r8-ideal-hpp-continuous.json`: the 10 Ideal cases × seeds
0–9 run as the published methods run them, each rollout through from
`reset(seed)` without `resume(k)`, so a segment starts from the scene the
previous one left. Same 150 steps a segment and the same goals; scored with the
paper's Eq. 1 (`completed_share`: the case's goal predicates that hold at the
end). The policy is `hpp_policy_continuous`, the row A policy with `recover`:

- A segment counts as passed at the step its predicates register
  (`success_step`), but the controller runs on until its stages are done: a
  place still releases and retreats, or the next pick finds the object in the
  hand.
- Each segment plans from the scene as it is. An object in the hand that the
  goal does not want is released (open, rise 12 cm); an object to place or pour
  that is not in the hand is picked first.
- Replanning is long_tamp's protection for robustness, not a retry of a
  one-shot run: when a controller that moved is done with the segment
  undone, the runner calls the policy again within the same 150 steps, up to
  3 calls a segment (`replans`). Planning time is not budgeted.

| | Row A (resumed, R5) | Row C (uninterrupted) |
| --- | --- | --- |
| **Eq. 1, key state changes completed** | 94.2% (inflated by resume) | **61.4%** |
| supported segments passed | 603 / 710 (84.9%) | 455 / 710 (64.1%) |
| rollouts with every step done | 69% (mean over cases) | 8 / 100 (case1) |
| mean step a passed segment registers at | 59.0 of 150 | 60.4 of 150 |

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Eq. 1, row A | 0.98 | 0.75 | 1.0 | 0.83 | 1.0 | 1.0 | 0.86 | 1.0 | 1.0 | 1.0 |
| Eq. 1, row C | 0.97 | 0.63 | 0.61 | 0.83 | 0.25 | 0.50 | 0.71 | 0.43 | 0.72 | 0.49 |
| supported segments passed, row C | 58/60 | 24/30 | 60/100 | 50/60 | 15/60 | 30/50 | 50/60 | 30/60 | 79/100 | 59/130 |
| rollout success, row C | 0.8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Of the 455 passed segments, 373 passed on their first plan. 82 took a second
policy call, 77 of them a pick that first released what the previous place
left in the hand; about 5 replanned after a plan that executed and missed. The
8 complete rollouts replanned no segment. Planning took 13 CPU-hours over the
100 rollouts, median 3.6 s a segment; 50 segments planned for over 5 minutes
(65% of the time) and 7 of them passed. case10's seeds took 40 minutes to 2
hours each.

Without the resume a failure carries on: an `open` the policy declines leaves
the cabinet shut for the place into it, and a place that fails leaves the
object in the hand. The failures on every seed:

| Segment | Cause |
| --- | --- |
| case2 3, frypan onto the rack | no region `rack_1` in the export (supported since, #187) |
| case3 0–1, pick and place the plate | no handle of the plate plans (edge pinch since, #188) |
| case3 6–7, cookies onto the plate, the table | the held start collides (finger and plate) |
| case4 5, frypan onto the cabinet top | no release pose in the region (passes since, #187) |
| case5 1, pudding onto the plate | planned, fails in the carry; 2–5 fail after it |
| case6 0, case8 0, case9 8, open | unsupported (#161) |
| case6 1, pick butter | planned, fails in the lift |
| case6 2, butter into the shut cabinet | no release pose |
| case7 4, turn on the stove | unsupported (#161) |
| case7 6, frypan onto the stove | no release pose in the region (passes since, #187) |
| case8 1–2, cream cheese into the shut cabinet | planned, fails in the lift and the retreat |
| case8 6, pour the wine | planned, fails in the tilt |
| case9 9–10, white_bowl into the microwave | no handle of the bowl plans |

case10, 13 segments with three pours and carries to a crowded table, loses
more later in the rollout: its carries back to the coffee table do not plan
(7 seeds for the bbq sauce), and plans that do run out of steps in the
transit.

#### With the place onto an object (#187)

`results/2026-10-31-r10-ideal-hpp-continuous.json`: the same run with `On o x`
onto another object supported (all onto `rack_1`) and the new release
orientations (held turned about the vertical, a rim pinch last). Eq. 1
61.4% → 67.7%; supported segments passed 483 / 720 (720: the frypan onto the
rack is now supported); 26 / 100 rollouts with every step done (8 before).

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Eq. 1, row C (R8) | 0.97 | 0.62 | 0.61 | 0.83 | 0.25 | 0.50 | 0.71 | 0.43 | 0.72 | 0.49 |
| Eq. 1, row C | 0.97 | 0.95 | 0.61 | 1.00 | 0.25 | 0.50 | 0.86 | 0.43 | 0.72 | 0.48 |
| supported segments passed, row C | 58/60 | 32/40 | 60/100 | 60/60 | 15/60 | 30/50 | 60/60 | 30/60 | 79/100 | 59/130 |
| rollout success, row C | 0.8 | 0.8 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |

case2's step 3, the frypan onto the rack, passes on 8 of 10 seeds. On the
other two (0 and 9) the place runs through but the frypan ends on the pegs'
tips, 6.6 cm above its release pose, and does not register. case4's frypan
onto the cabinet top and case7's onto the stove, which had no release pose,
pass on every seed: released as held turned about the vertical. The other cases are unchanged (case10 within a segment).

### Row D: Ideal, one mission per rollout (#186)

`results/2026-10-30-r9-ideal-hpp-mission.json`: the 10 Ideal cases × seeds 0–9, each rollout one
`long_tamp.mission` run over the case's whole goal instead of a policy called
segment by segment. `RoboCerebraDomain` turns the case into a symbolic domain:

- capabilities `pick`, `place`, `pour`, `regrasp` and `putdown` over the case's
  objects, with a stage per goal step of each object, so an object's steps
  run in the case's order. The order across objects is the task planner's,
  not the case's;
- the world it plans from is what the env observes: the benchmark's progress
  and the object between the fingers;
- each step is a `Move` that the row C policy (`HppPolicy.plan`) refines in HPP
  from the scene as it is, and the env runs as the backend.

When a step ends with its goal missed, the object dropped or still in the hand,
the runner replans from the observed world (`run_with_repair`, 2 retries a
step). An object that keeps failing, or whose goal is unsupported, is given up
on: put down, and the rest of the goal planned without it.

The step budget differs from rows A and C: 150 env steps × the case's steps,
shared over the rollout rather than 150 a segment, and one step is cut at 150.
A step that finishes early leaves its steps to later ones; retries spend from
the same budget. Planning time is not budgeted.

| | Row C (per segment) | Row D (one mission) |
| --- | --- | --- |
| **Eq. 1, key state changes completed** | 61.4% | **64.3%** |
| rollouts with every step done | 8 / 100 (case1) | 8 / 100 (7 case1, 1 case5) |
| budget used (mean) | – | 48.9%; 1 rollout spent it all |
| steps retried | 82 policy calls | 230 retries |
| objects given up on | – | 107 over 100 rollouts |

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Eq. 1, row C | 0.97 | 0.63 | 0.61 | 0.83 | 0.25 | 0.50 | 0.71 | 0.43 | 0.72 | 0.49 |
| Eq. 1, row D | 0.92 | 0.75 | 0.60 | 0.83 | 0.48 | 0.47 | 0.43 | 0.51 | 0.71 | 0.73 |
| rollout success, row D | 0.7 | 0 | 0 | 0 | 0.1 | 0 | 0 | 0 | 0 | 0 |

Where row D gains, a failure no longer carries on: the object is put down
and the rest of the goal planned without it, and a step that finishes early
leaves its env steps to the later ones. case5 goes from 0.25 to 0.48 and
case10 from 0.49 to 0.73 (in row C its plans ran out of steps in the
transit).

Where it loses, the planner chose the order. case7 (0.71 → 0.43) does the
frypan first: its place on the stove has no release pose, so it is put down
where the hand is, and from there no handle of the pudding plans, on every
seed. case1 (0.97 → 0.92) loses the butter's place into the storage box side
twice, and one rollout spent its whole budget.

Of the 230 retries, 19 were of a step that executed and missed (a grasp that
stalled, mostly) and passed on the retry, 10 of them in case8. Most of the
rest replan a refinement that fails again the same way: no handle, no
release pose. None of the 235 such replans planned, so since this run a step
that does not plan (`refinement_failed`) is not retried; only an execution miss
is. 107 objects were given up on.

The rollouts took 9.4 hours, median 9 s to 70 s a rollout in cases 1–4,
6 and 9, and 44 minutes in case10. Planning time is not budgeted.

The failures on every seed, or nearly:

| Step | Cause |
| --- | --- |
| case2, frypan onto the rack | `rack_1` is an object, not a region (supported since, #187) |
| case3, pick the plate | no handle of the plate plans (edge pinch since, #188) |
| case3, pour the cookies onto the plate | the held start collides, finger and cookies (#189) |
| case4, frypan onto the cabinet top | no release pose in the region |
| case5, pour the pudding onto the plate | does not plan (8 seeds) |
| case5, wine bottle onto the table | no release pose (6 seeds), the grasp stalls (4) |
| case7, frypan onto the stove | no release pose; put down, it blocks the pudding |
| case10, bbq sauce back onto the table | does not plan (8 seeds) |
| case6, 8, 9 open; case7 turn on | unsupported (#161) |

A step cut at 150 env steps in its tracker (#190) counts as a miss and is
retried.

#### Since the run: a pick checked against the step after it

Planned one step at a time, a mission can pick an object whose place
cannot plan from its grasp (case7's frypan above). Each pick is now
checked: once it plans, the place or pour after it is planned too, held
mode, from the scene the lift leaves (the robot's joints and the object's
pose at the top of the lift; the fingers as exported, open). When that does
not plan, the pick fails with `lookahead_failed(object)`, is not retried,
and the object is deferred: the others go first, then it is picked without
the check (the pick counts) and put down when its place fails.
`--no-lookahead` runs the mission as above.

On case7, seed 0: the frypan is deferred for the reason its place gave in
the run ("no release pose ... in flat_stove_1_cook_region"), the butter and
pudding pours run first, and the case goes from 3/7 to 5/7 (0.71, row C's
score) in 367 s instead of 457 s.

#### With the place onto an object (#187)

`results/2026-10-31-r11-ideal-hpp-mission.json`: the rerun, with the
lookahead above, no retry of a step that does not plan, and #187's place onto
an object and release orientations. Eq. 1 64.3% → 67.9%; 26 / 100 rollouts with every
step done (6 case1, 2 case2, 10 case4, 8 case10); 115 retries, 73 objects
given up on (107 before). Budget used 58.5% on mean, 13 rollouts spent it all.
The rollouts took 6.3 hours.

| Case | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Eq. 1, row D (R9) | 0.92 | 0.75 | 0.60 | 0.83 | 0.48 | 0.47 | 0.43 | 0.51 | 0.71 | 0.73 |
| Eq. 1, row D | 0.85 | 0.60 | 0.60 | 1.00 | 0.23 | 0.47 | 0.86 | 0.57 | 0.70 | 0.91 |
| rollout success, row D | 0.6 | 0.2 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 0.8 |

The frypan goes onto the rack on all 10 seeds of case2, onto the cabinet top
in case4, and, deferred by the lookahead, onto the stove in case7. case10
gains from not retrying what does not plan.

The task order is the planner's, and in this run it differs from R9's: it
depends on Python's hash seed (`PYTHONHASHSEED`), so a whole run takes one
order per case. Rerunning case2 at seed 0 under hash seeds 1–4 gives the
cream cheese first (4/4) for two and the frypan first (2/4) for the other two.
The three cases that lose are that order, not a step that newly fails:

- case2 (0.75 → 0.60) does the frypan first on every seed. Its place on the
  rack ends at full reach, the elbow (joint 4) at its limit, and from there
  every grasp of the cream cheese is at a joint limit: OSC stalls in the
  transit, on 8 seeds. With the cream cheese first, the case is 4/4.
- case5 (0.48 → 0.23) pours the pudding first on every seed (R9: the wine
  bottle first on 6). The pudding pour does not plan and the wine bottle's
  pick after it fails, as in R9's rollouts with that order.
- case1 (0.92 → 0.85) places the butter first on every seed; its sweep into
  the storage box's left side stalls on 4 seeds (R9: 2), and seed 6 plans
  nothing after it.

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

Our Ideal numbers, from rows A and C and the demo replay:

| | Ideal |
| --- | --- |
| **HPP policy, uninterrupted, Eq. 1 (row C)** | **61.4%** |
| HPP mission, one per rollout, Eq. 1 (row D) | 64.3% |
| HPP policy, uninterrupted, rollouts with every step done (row C) | 8 / 100 |
| HPP policy, supported segments passed | 603 / 710 (84.9%) |
| HPP policy, all segments passed (unsupported counted as failed) | 603 / 760 (79.3%) |
| HPP policy, rollout success (mean over cases) | 69% |
| Demo replay, subtasks completed | 56 / 76 (73.7%) |

These numbers don't compare like-for-like:

- **Inputs.** The HPP policy reads each segment's goal predicates and the exact
  scene state, so it should be compared with the ground-truth-plan rows, not the
  VLM-planner rows. The VLAs work from camera images.
- **Resuming.** Row A starts every segment from the official `resume(k)`
  state, so an early failure doesn't carry into later segments. Row C runs each
  task through without stopping, as the published rollouts do, and is the
  number to set beside theirs.
- **Planning.** Our policy plans each segment before executing it and, in row
  C, may replan within the segment's step budget; planning time is not
  counted.
- **Unit.** Row A's unit is a segment, and theirs is a key state transition. The
  two usually coincide, but not always. Row C's Eq. 1 counts the benchmark's goal
  predicates.
- **Coverage.** We have only run Ideal; Mix and the Memory settings are still to
  do.

[RoboCerebra]: https://arxiv.org/abs/2506.06677
[SparkVLA]: https://arxiv.org/abs/2608.16172
[HarnessWAM]: https://arxiv.org/abs/2608.09516
