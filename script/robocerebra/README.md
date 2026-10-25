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
not proven executable grasps. All candidates still need an HPP grasp-and-lift
plan before R3 can be considered complete.

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
