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
`set_state(sim_state, controller=None)`, `render(cameras=None, size=256)`, `demo()`.
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
