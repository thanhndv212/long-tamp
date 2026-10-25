"""Serve a RoboCerebra benchmark environment over TCP (issue #156, R1).

RoboCerebra (NeurIPS 2025 D&B) runs on a LIBERO fork with robosuite 1.4,
Python 3.10 and ``numpy<2``; long_tamp needs ``numpy>=1.26`` and HPP. The two
can't share an interpreter, so the benchmark env lives in its own process and
long_tamp drives it over the same wire as ``long_tamp.execution.remote``
(ADR-0007): length-prefixed JSON frames, the tagged ``$nd`` / ``$tuple``
codec and a ``hello`` handshake, never pickle. This file imports only numpy
and the standard library at module level (robosuite and LIBERO are imported
when a case is loaded), so it runs in the benchmark env without long_tamp and
the tests can import it without the benchmark.

The environment is built as RoboCerebra's ``evaluation/task_runner.py`` does
(Panda, ``OSC_POSE``, 20 Hz, ``ignore_done``) and reset and resumed as its
``evaluation/episode.py`` does: an episode starts from the case's init file,
and a resume jumps to the demo's state at the start of step ``k``, with the
subtasks of earlier steps marked done (``evaluation/resume.py``). Success is
the env's own stateful ``_check_success(goal)``, called after every step as
the benchmark does. Cameras are rendered only when asked for (``render``):
on a software renderer two 256x256 cameras cost ~0.35 s a step.

Operations (``{"op": name, "args": [...]}`` → ``{"ok": true, "value": ...}``):

``hello`` (``version``), ``ping``, ``bye``, ``cases()``, ``load(case)``, ``set_goal(goal)``,
``reset(step=0, settle=None, restore_demo=False)``, ``resume(step)``,
``step(actions)`` (with the checker's ``[Monitor]`` lines), ``check()``, ``get_state()``,
``set_state(sim_state, controller=None)``, ``render(cameras=None, size=256)``,
``demo()``, ``export_scene()`` (the scene as URDF/SRDF, ``scene_export.py``).

Usage, in the benchmark env (``source ~/devel/hpp/robocerebra/env.sh``)::

    python script/robocerebra/env_server.py --bench $RC_BENCH --port 7071

There is no authentication: it binds to localhost unless told otherwise.
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import glob
import io
import json
import logging
import os
import pickle
import socket
import struct
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger("robocerebra.env_server")

PROTOCOL_VERSION = 1  # long_tamp.execution.remote.PROTOCOL_VERSION
SERVICE = "robocerebra"
MAX_FRAME = 64 * 1024 * 1024
_HEADER = struct.Struct(">I")

CAMERAS = ("agentview", "robot0_eye_in_hand")
# get_libero_dummy_action(): hold still, gripper open.
DUMMY_ACTION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
# RoboCerebra's evaluation/config.py.
NUM_STEPS_WAIT = 15
SWITCH_STEPS = 150
# Task types that the benchmark runs on the Ideal cases' files
# (evaluation/utils.py: get_task_directories, load_init_state).
USES_IDEAL = ("Ideal", "Observation_Mismatching", "Random_Disturbance")


# -- wire (the subset of long_tamp.execution.remote's codec it needs) --------


def encode(value: Any) -> Any:
    if value is None or isinstance(value, bool | int | float | str):
        return value
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        data = np.ascontiguousarray(value)
        return {
            "$nd": {
                "dtype": data.dtype.str,
                "shape": list(data.shape),
                "data": base64.b64encode(data.tobytes()).decode("ascii"),
            }
        }
    if isinstance(value, tuple):
        return {"$tuple": [encode(v) for v in value]}
    if isinstance(value, list):
        return [encode(v) for v in value]
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            if not isinstance(key, str) or key.startswith("$"):
                raise TypeError(f"cannot send dict key {key!r}")
            out[key] = encode(item)
        return out
    raise TypeError(f"cannot send a {type(value).__name__}")


def decode(value: Any) -> Any:
    if isinstance(value, list):
        return [decode(v) for v in value]
    if not isinstance(value, dict):
        return value
    if len(value) == 1:
        ((tag, body),) = value.items()
        if tag == "$nd":
            data = base64.b64decode(body["data"])
            return (
                np.frombuffer(data, dtype=np.dtype(body["dtype"]))
                .reshape(body["shape"])
                .copy()
            )
        if tag == "$tuple":
            return tuple(decode(v) for v in body)
        if tag.startswith("$"):
            raise ValueError(f"unknown tag {tag!r}")
    return {key: decode(item) for key, item in value.items()}


def send_frame(sock: socket.socket, message: Any) -> None:
    data = json.dumps(message, separators=(",", ":")).encode()
    if len(data) > MAX_FRAME:
        raise ValueError(f"frame of {len(data)} bytes exceeds {MAX_FRAME}")
    sock.sendall(_HEADER.pack(len(data)) + data)


def recv_frame(sock: socket.socket) -> Any:
    (size,) = _HEADER.unpack(_recv_exact(sock, _HEADER.size))
    if size > MAX_FRAME:
        raise ConnectionError(f"frame of {size} bytes exceeds {MAX_FRAME}")
    return json.loads(_recv_exact(sock, size))


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks, remaining = [], size
    while remaining:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise ConnectionError("connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


# -- server -------------------------------------------------------------------

OPS = (
    "cases",
    "load",
    "set_goal",
    "reset",
    "resume",
    "step",
    "check",
    "regions",
    "get_state",
    "set_state",
    "render",
    "demo",
    "export_scene",
)


def serve(
    env: Any,
    host: str = "127.0.0.1",
    port: int = 0,
    *,
    ready: Callable[[int], None] | None = None,
    stop: threading.Event | None = None,
    once: bool = False,
) -> None:
    """Serve ``env`` (anything with the :data:`OPS` methods) on ``host:port``,
    one client at a time, until ``stop`` is set (or, with ``once``, until the
    first client leaves). ``ready(port)`` is called once listening."""
    server = socket.create_server((host, port))
    server.settimeout(0.1)
    if ready is not None:
        ready(server.getsockname()[1])
    log.info("serving %s on %s:%d", SERVICE, host, server.getsockname()[1])
    try:
        while stop is None or not stop.is_set():
            try:
                conn, _addr = server.accept()
            except TimeoutError:
                continue
            with conn:
                conn.settimeout(None)
                _session(env, conn, stop)
            if once:
                return
    finally:
        server.close()


def _session(env: Any, conn: socket.socket, stop: threading.Event | None) -> None:
    # The env steps synchronously: there's nothing to cancel on a disconnect,
    # so (unlike serve_backend) no heartbeat. A short timeout only lets `stop`
    # be noticed between requests.
    conn.settimeout(0.2)
    while stop is None or not stop.is_set():
        try:
            request = recv_frame(conn)
        except TimeoutError:
            continue
        except (OSError, ValueError) as error:
            log.info("client left (%s)", error)
            return
        if request.get("op") == "bye":
            return
        conn.settimeout(None)
        reply = _handle(env, request)
        try:
            send_frame(conn, reply)
        except (OSError, ValueError) as error:
            log.warning("reply failed: %s", error)
            return
        conn.settimeout(0.2)


def _handle(env: Any, request: dict[str, Any]) -> dict[str, Any]:
    op, args = request.get("op"), request.get("args", [])
    kwargs = request.get("kwargs", {})
    if op == "hello":
        if request.get("version") != PROTOCOL_VERSION:
            error = f"protocol {request.get('version')} != {PROTOCOL_VERSION}"
            return {"ok": False, "error": error}
        info = getattr(env, "info", lambda: {})()
        value = {"version": PROTOCOL_VERSION, "service": SERVICE, "ops": list(OPS)}
        return {"ok": True, "value": encode({**value, **info})}
    if op == "ping":
        return {"ok": True, "value": None}
    if op not in OPS:
        return {"ok": False, "error": f"unknown operation {op!r}"}
    try:
        value = getattr(env, op)(*decode(args), **decode(kwargs))
        return {"ok": True, "value": encode(value)}
    except Exception as error:
        log.exception("%s failed", op)
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}


# -- the benchmark env ----------------------------------------------------------


class RoboCerebraEnv:
    """One RoboCerebra case at a time, built and reset as the benchmark does."""

    def __init__(self, bench: str | os.PathLike[str], *, render: bool = True) -> None:
        self.bench = Path(bench)
        self.render_enabled = render
        self.env: Any = None
        self.case: str | None = None
        self.t = 0
        self.monitor: list[str] = []  # the checker's prints since the last step

    def info(self) -> dict[str, Any]:
        import mujoco
        import robosuite

        return {
            "versions": {"robosuite": robosuite.__version__, "mujoco": mujoco.__version__},
            "bench": str(self.bench),
            "render": self.render_enabled,
        }

    # -- cases --

    def cases(self) -> list[str]:
        out = []
        for path in sorted(self.bench.glob("*/case*")):
            if path.is_dir() and path.parent.name != "init_files":
                out.append(f"{path.parent.name}/{path.name}")
        return sorted(out, key=_case_key)

    def _case_dir(self, case: str) -> tuple[str, Path]:
        task_type, name = case.split("/")
        source = "Ideal" if task_type in USES_IDEAL else task_type
        return task_type, self.bench / source / name

    def load(self, case: str, model: str = "bddl") -> dict[str, Any]:
        """Build the env for ``case`` (``"Ideal/case1"``) and load its demo,
        goal and step annotations. The env is not reset: call ``reset``.

        ``model="bddl"`` builds the scene from the case's BDDL as the
        benchmark does: every ``reset`` resamples the fixtures' placement
        (the storage box moves by ~1 cm). ``model="demo"`` loads the MJCF the
        demo was recorded in and resets deterministically: what a replay of
        the demo needs."""
        if model not in ("bddl", "demo"):
            raise ValueError(f"model must be 'bddl' or 'demo', got {model!r}")
        import h5py
        import libero.libero.envs.bddl_utils as bddl_utils
        from libero.libero.envs import TASK_MAPPING
        from robosuite import load_controller_config

        task_type, case_dir = self._case_dir(case)
        bddl = glob.glob(str(case_dir / "*.bddl"))
        if not bddl:
            raise FileNotFoundError(f"no .bddl in {case_dir}")
        if self.env is not None:
            self.env.close()
            self.env = None
        problem = bddl_utils.get_problem_info(bddl[0])["problem_name"]
        self.env = TASK_MAPPING[problem](
            bddl_file_name=bddl[0],
            robots=["Panda"],
            controller_configs=load_controller_config(default_controller="OSC_POSE"),
            has_renderer=False,
            has_offscreen_renderer=self.render_enabled,
            ignore_done=True,
            use_camera_obs=False,
            reward_shaping=True,
            control_freq=20,
        )
        with h5py.File(case_dir / "demo.hdf5", "r") as h5:
            demo = h5["data"]["demo_1"]
            self.demo_states = demo["states"][()]
            self.demo_actions = demo["actions"][()]
            demo_xml = demo.attrs["model_file"]
        if model == "demo":
            # The fork moved the table's region sites after the demos were
            # recorded (coffee table: z 0.02 -> 0.42, _get_table_z_offset),
            # so the recorded MJCF's regions are not the ones the benchmark
            # scores against. Keep the BDDL scene's.
            regions = _region_sites(self.env.sim.model)
            self.env.reset_from_xml_string(_portable_xml(demo_xml))
            _set_region_sites(self.env.sim.model, regions)
            # Keep this model: a reset only resets the sim, no resampling.
            self.env.deterministic_reset = True
        self.goal, self.goal_steps = _load_goal(case_dir / "goal.json")
        self.steps, self.spans, self.task = _parse_description(
            case_dir / "task_description.txt"
        )
        init_dir = "Ideal" if task_type in USES_IDEAL else task_type
        init_file = self.bench / "init_files" / init_dir / f"{case_dir.name}.init"
        self.init_state = None
        if init_file.exists():
            with open(init_file, "rb") as handle:  # the benchmark's own file
                self.init_state = np.asarray(pickle.load(handle), dtype=float)
        distractor = case_dir / "distractor_name.txt"
        self.case = case
        self.model = model
        return {
            "case": case,
            "model": model,
            "problem": problem,
            "task": self.task,
            "steps": self.steps,
            "spans": [list(span) for span in self.spans],
            "goal": self.goal,
            "goal_steps": self.goal_steps,
            "demo_length": len(self.demo_actions),
            "objects": list(self.env.object_names),
            "regions": list(self.env.regions),
            "distractor": distractor.read_text().strip() if distractor.exists() else None,
            "has_init_file": self.init_state is not None,
            "switch_steps": SWITCH_STEPS,
            "num_steps_wait": NUM_STEPS_WAIT,
        }

    def _need(self) -> Any:
        if self.env is None:
            raise RuntimeError("no case loaded: call load(case) first")
        return self.env

    # -- episode --

    def set_goal(self, goal: dict[str, list[list[str]]]) -> dict[str, Any]:
        """Score against ``goal`` (``{obj: [[predicate, obj, ...], ...]}``)
        instead of the case's, from the next ``reset``: e.g. one object's
        ``pick``, to check a grasp on any object (#158). Each predicate is
        its own task step, in order."""
        self._need()
        objects = set(self.env.object_names)
        unknown = sorted(set(goal) - objects)
        if unknown:
            raise ValueError(f"not in the scene: {unknown}")
        self.goal = {obj: [[p[0].lower(), *p[1:]] for p in specs] for obj, specs in goal.items()}
        self.goal_steps = {obj: list(range(len(specs))) for obj, specs in self.goal.items()}
        return {"goal": self.goal, "goal_steps": self.goal_steps}

    def reset(
        self,
        step: int = 0,
        settle: int | None = None,
        restore_demo: bool = False,
        seed: int | None = None,
    ) -> dict[str, Any]:
        """Start an episode at ``step``, after ``env.reset()``.

        Step 0 is the benchmark's episode start, exactly: the case's init
        file (the demo's first state when there is none), then ``settle``
        dummy steps (default 15) before the success baseline. As in the
        benchmark, the controller still holds the reset pose's goal: the
        first actions pull towards it (the demos were recorded that way).

        Step ``k > 0`` starts a fresh episode from the demo's state at the
        start of step ``k``, with earlier steps' subtasks marked done and no
        settling by default. It isn't the benchmark's resume, which jumps
        mid-episode (see :meth:`resume`). The controller holds the current
        pose and the gripper command starts from zero (half closed: a held
        object loosens until the next actions close it). With
        ``restore_demo`` both continue the demo's own: the orientation goal
        its last rotating action set and its accumulated gripper command, so
        a replay goes on exactly as recorded."""
        env = self._need()
        if not 0 <= step < len(self.spans):
            raise ValueError(f"step {step} not in [0, {len(self.spans)})")
        if seed is not None:
            env.seed(int(seed))
        env.reset()
        self.t = 0
        if step == 0:
            state = self.init_state if self.init_state is not None else self.demo_states[0]
            self._apply(state)
        else:
            begin = self.spans[step][0]
            goal_ori = self._demo_orientation_goal(begin) if restore_demo else None
            self._apply(self.demo_states[begin], sync=True)
            controller = env.robots[0].controller
            controller.goal_pos = np.array(controller.ee_pos)
            controller.goal_ori = np.array(controller.ee_ori_mat if goal_ori is None else goal_ori)
            gripper = env.robots[0].gripper
            gripper.current_action = np.zeros(gripper.dof)
            if restore_demo:
                substeps = round(env.control_timestep / env.model_timestep)
                for action in self.demo_actions[:begin]:
                    for _ in range(substeps):
                        gripper.format_action(action[-gripper.dof :])
            env.skip_pick_quat_once = True
        env._state_progress = {obj: 0 for obj in self.goal}
        resumed = self._resume_progress(step)
        if settle is None:
            settle = NUM_STEPS_WAIT if step == 0 else 0
        for _ in range(settle):
            env.step(list(DUMMY_ACTION))
        completion = self.check()
        return {"step": step, "resumed": resumed, **completion, "obs": self._obs()}

    def resume(self, step: int) -> dict[str, Any]:
        """The benchmark's resume (``episode.py``): jump, mid-episode, to the
        demo's state at the start of ``step`` and mark earlier steps'
        subtasks done. Nothing else is reset: the controller's goal, the
        gripper command and the checker's caches carry over from the
        episode so far, as in the benchmark."""
        env = self._need()
        if not 0 < step < len(self.spans):
            raise ValueError(f"step {step} not in [1, {len(self.spans)})")
        self._apply(self.demo_states[self.spans[step][0]], sync=True)
        env.skip_pick_quat_once = True
        resumed = self._resume_progress(step)
        completion = self.check()
        return {"step": step, "resumed": resumed, **completion, "obs": self._obs()}

    def _demo_orientation_goal(self, begin: int) -> np.ndarray | None:
        """The orientation goal the demo's controller held at ``begin``: OSC
        only updates it on an action that rotates, and keeps it otherwise.
        ``None`` if no action before ``begin`` rotated."""
        import math

        controller = self.env.robots[0].controller
        dim = controller.control_dim
        for index in range(begin - 1, -1, -1):
            rotation = controller.scale_action(np.asarray(self.demo_actions[index][:dim]))[3:]
            if any(not math.isclose(x, 0.0) for x in rotation):
                break
        else:
            return None
        self._apply(self.demo_states[index], sync=True)
        controller.set_goal(np.asarray(self.demo_actions[index][:dim]))
        return np.array(controller.goal_ori)

    def _resume_progress(self, step: int) -> list[str]:
        """evaluation/resume.py: subtasks annotated before ``step`` are done."""
        done = []
        for obj, steps in self.goal_steps.items():
            for index, task_step in enumerate(steps):
                if task_step < step and self.env._state_progress[obj] <= index:
                    self.env._state_progress[obj] = index + 1
                    done.append(f"{obj}_{index}")
        return done

    def step(self, actions: Any) -> dict[str, Any]:
        """Apply one ``OSC_POSE`` action (7,) or a sequence (N, 7), checking
        success after each as the benchmark does. Returns the completion
        after the last one and every subtask completed on the way."""
        env = self._need()
        actions = np.asarray(actions, dtype=float)
        if actions.ndim == 1:
            actions = actions[None]
        if actions.ndim != 2 or actions.shape[1] != 7:
            raise ValueError(f"actions must be (7,) or (N, 7), got {actions.shape}")
        before = dict(env._state_progress)
        completion: dict[str, Any] = {}
        self.monitor = []
        for action in actions:
            with contextlib.redirect_stdout(io.StringIO()):
                env.step(action.tolist())
            self.t += 1
            completion = self.check()
        completed = []
        for obj, count in env._state_progress.items():
            for index in range(before.get(obj, 0), count):
                completed.append({"object": obj, "subtask": index, "spec": self.goal[obj][index]})
        return {
            **completion,
            "completed": completed,
            "monitor": self.monitor,
            "t": self.t,
            "obs": self._obs(),
        }

    def check(self) -> dict[str, Any]:
        """The env's ``_check_success(goal)``: per-object completion (%),
        subtasks done so far and whether all are. Stateful, as in the
        benchmark: a subtask counts once reached, in order."""
        env = self._need()
        # The checker prints its verdicts ("[Monitor] ... Pick successful",
        # "... pick suppressed: quaternion change ..."): keep them for the
        # step's reply instead of a stdout nobody drains.
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            completion, total_completed, all_done = env._check_success(self.goal)
        self.monitor.extend(line for line in out.getvalue().splitlines() if line.strip())
        return {
            "completion": {k: float(v) for k, v in completion.items()},
            "total_completed": int(total_completed),
            "total": int(sum(len(v) for v in self.goal.values())),
            "all_done": bool(all_done),
            "progress": {k: int(v) for k, v in env._state_progress.items()},
        }

    def regions(self, obj: str) -> list[str]:
        """The regions ``obj`` is ``in`` by the benchmark's predicate. Its
        pick counts only when this goes from some region to none: an object
        that starts in none (a bottle whose origin is above the table
        region's band) or only in one it carries (a basket's own contain
        region) can be lifted but never picked."""
        env = self._need()
        return [r for r in env.regions if env._eval_predicate(["in", obj, r])]

    # -- state --

    def get_state(self) -> dict[str, Any]:
        """The sim state, plus what it leaves out that the next steps depend
        on: the controller's goal (OSC keeps its orientation goal through
        actions that don't rotate) and the gripper's command."""
        env = self._need()
        robot = env.robots[0]
        return {
            "sim_state": env.sim.get_state().flatten(),
            "controller": {
                "goal_pos": np.array(robot.controller.goal_pos),
                "goal_ori": np.array(robot.controller.goal_ori),
                "gripper": np.array(robot.gripper.current_action),
            },
            "time": float(env.sim.data.time),
            "t": self.t,
            "obs": self._obs(),
        }

    def set_state(self, sim_state: Any, controller: dict | None = None) -> dict[str, Any]:
        """Set the sim state, and the controller's from :meth:`get_state`
        when given (otherwise the controller keeps its goal)."""
        env = self._need()
        self._apply(np.asarray(sim_state, dtype=float), sync=True)
        if controller is not None:
            robot = env.robots[0]
            robot.controller.goal_pos = np.array(controller["goal_pos"], dtype=float)
            robot.controller.goal_ori = np.array(controller["goal_ori"], dtype=float)
            robot.gripper.current_action = np.array(controller["gripper"], dtype=float)
        return {"obs": self._obs()}

    def _apply(self, state: np.ndarray, sync: bool = False) -> None:
        """Set ``state``. ``sync`` refreshes the controller's cached
        end-effector pose too: ``env.reset()`` cached the reset pose and
        cleared its update flag, so the first action after a plain set_state
        is taken from the reset pose. The benchmark's episode start doesn't
        sync (the init state is close to the reset pose); every jump
        elsewhere must."""
        env = self.env
        env.sim.set_state_from_flattened(state)
        env.sim.forward()
        if sync:
            for robot in env.robots:
                robot.controller.update(force=True)
        env._post_process()
        env._update_observables(force=True)

    def _obs(self) -> dict[str, np.ndarray]:
        obs = self.env._get_observations(force_update=True)
        return {k: np.asarray(v) for k, v in obs.items() if not k.endswith("_image")}

    def render(self, cameras: Any = None, size: int = 256) -> dict[str, np.ndarray]:
        """Camera images (uint8, ``size``x``size``x3), as robosuite returns
        them (upside down; LIBERO policies flip them, the client doesn't)."""
        env = self._need()
        if not self.render_enabled:
            raise RuntimeError("rendering is off (--no-render)")
        images = {}
        for name in cameras or CAMERAS:
            images[name] = env.sim.render(camera_name=name, width=size, height=size)[::-1]
        return images

    def demo(self) -> dict[str, np.ndarray]:
        """The case's demonstration: ``states`` (T, nstate) and ``actions``
        (T, 7); ``states[i]`` is the state ``actions[i]`` was applied in."""
        self._need()
        return {"states": self.demo_states, "actions": self.demo_actions}

    def export_scene(self) -> dict[str, Any]:
        """The scene as it stands, as URDF/SRDF file texts and a manifest
        (issue #157; see ``scene_export.py``). Call after ``reset``: the
        fixtures and objects are exported where they are now."""
        env = self._need()
        robot = env.robots[0]
        export = _scene_export().export_scene(
            env.sim.model._model,
            env.sim.data._data,
            objects={name: obj.root_body for name, obj in env.objects_dict.items()},
            regions=_goal_regions(env),
            robot_root=robot.robot_model.root_body,
            robot_joints=list(robot.robot_joints) + list(robot.gripper.joints),
        )
        export["manifest"].update(case=self.case, model=self.model, t=self.t)
        return export


def _goal_regions(env: Any) -> list[str]:
    """The regions a goal can name: the BDDL's, plus the fixtures' and
    objects' own sites (``flat_stove_1_cook_region``,
    ``basket_1_contain_region``) that ``In`` checks the same way; not the
    ``*_init_region`` sites objects are only sampled in."""
    sites = [name for name in getattr(env, "object_sites_dict", {}) if not name.endswith("_init_region")]
    return list(dict.fromkeys([*env.regions, *sites]))


def _scene_export() -> Any:
    # A sibling file, not a package: under ``python -I`` the script's own
    # directory is not on sys.path.
    import importlib.util
    import sys

    if "robocerebra_scene_export" not in sys.modules:
        path = Path(__file__).with_name("scene_export.py")
        spec = importlib.util.spec_from_file_location("robocerebra_scene_export", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
    return sys.modules["robocerebra_scene_export"]


def _region_sites(model: Any) -> dict[str, tuple[np.ndarray, ...]]:
    """The arena's region sites (``*_region``), in their parent's frame."""
    sites = {}
    for i in range(model.nsite):
        name = model.site_id2name(i)
        if name and name.endswith("_region"):
            sites[name] = (
                model.site_pos[i].copy(), model.site_quat[i].copy(), model.site_size[i].copy()
            )
    return sites


def _set_region_sites(model: Any, sites: dict[str, tuple[np.ndarray, ...]]) -> None:
    for name, (pos, quat, size) in sites.items():
        try:
            i = model.site_name2id(name)
        except ValueError:
            continue
        model.site_pos[i], model.site_quat[i], model.site_size[i] = pos, quat, size


def _portable_xml(xml: str) -> str:
    """The demo's MJCF with its asset paths (absolute, on the recording
    machine) pointed at this install's LIBERO and robosuite assets."""
    import xml.etree.ElementTree as ET

    import libero.libero
    import robosuite

    roots = {
        "libero": Path(libero.libero.__file__).parent / "assets",
        "robosuite": Path(robosuite.__file__).parent,
    }
    tree = ET.fromstring(xml)
    for elem in tree.iter():
        old = elem.get("file")
        if elem.tag not in ("mesh", "texture") or old is None:
            continue
        parts = old.split("/")
        # Robosuite first: its path on the recording machine may run through
        # a directory named "libero" (a conda env, say).
        for key in ("robosuite", "libero"):
            anchor = "robosuite" if key == "robosuite" else "assets"
            if key in parts and anchor in parts:
                rest = parts[len(parts) - parts[::-1].index(anchor) :]
                elem.set("file", str(roots[key].joinpath(*rest)))
                break
    return ET.tostring(tree, encoding="unicode")


def _case_key(case: str) -> tuple[str, int]:
    task_type, name = case.split("/")
    return task_type, int(name.removeprefix("case") or 0)


def _load_goal(path: Path) -> tuple[dict[str, list[list[str]]], dict[str, list[int]]]:
    """evaluation/utils.py: load_actions_with_steps."""
    data = json.loads(path.read_text(encoding="utf-8"))
    goal, steps = {}, {}
    for obj, relations in data.items():
        goal[obj], steps[obj] = [], []
        for item in relations:
            if isinstance(item, dict):
                triple, step = item["state_pair"], item["task_step"]
            else:
                triple, step = item, len(goal[obj])
            if len(triple) not in (2, 3):
                continue
            goal[obj].append([triple[0].lower(), *triple[1:]])
            steps[obj].append(step)
    return goal, steps


def _parse_description(path: Path) -> tuple[list[str], list[tuple[int, int]], str]:
    """task_description.txt: ``Task:``, then ``Step: ...`` lines each followed
    by its ``[start, end]`` span of demo steps."""
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    lines = [line for line in lines if line]
    task, steps, spans = "", [], []
    for i, line in enumerate(lines):
        if line.startswith("Task:"):
            task = line.split(":", 1)[1].strip()
        elif line.startswith("Step") and i + 1 < len(lines):
            span = lines[i + 1].strip("[]").split(",")
            if len(span) == 2:
                steps.append(line.split(":", 1)[1].strip())
                spans.append((int(span[0]), int(span[1])))
    return steps, spans, task


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bench", default=os.environ.get("RC_BENCH"), required=not os.environ.get("RC_BENCH"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7071)
    parser.add_argument("--no-render", action="store_true", help="no offscreen renderer (no EGL needed)")
    parser.add_argument("--once", action="store_true", help="exit when the first client leaves")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    env = RoboCerebraEnv(args.bench, render=not args.no_render)

    def ready(port: int) -> None:
        print(f"READY {port}", flush=True)

    serve(env, args.host, args.port, ready=ready, once=args.once)


if __name__ == "__main__":
    main()
