"""The RoboCerebra env server and client (issue #156): the server's codec is
wire-compatible with long_tamp.execution.remote, and the client drives a case
end to end through the real server loop over a stub env (the benchmark
itself needs robosuite and LIBERO, which CI doesn't have)."""

import importlib.util
import socket
import sys
import threading
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest

from long_tamp.eval import RoboCerebraClient
from long_tamp.execution import remote

SERVER = Path(__file__).resolve().parents[1] / "script" / "robocerebra" / "env_server.py"


@pytest.fixture(scope="module")
def server_module():
    spec = importlib.util.spec_from_file_location("rc_env_server", SERVER)
    module = importlib.util.module_from_spec(spec)
    sys.modules["rc_env_server"] = module
    spec.loader.exec_module(module)
    yield module
    sys.modules.pop("rc_env_server", None)


class StubEnv:
    """Two objects, two steps each; a subtask is done once the summed x
    action reaches the next threshold."""

    def __init__(self):
        self.case = None
        self.x = 0.0
        self.t = 0
        self.progress = {}

    def info(self):
        return {"versions": {"stub": "0"}, "render": True}

    def cases(self):
        return ["Ideal/case1", "Mix/case2"]

    def load(self, case, model="bddl"):
        if case not in self.cases():
            raise FileNotFoundError(case)
        self.case = case
        self.goal = {"a_1": [["pick", "a_1"], ["in", "a_1", "box"]], "b_1": [["pick", "b_1"]]}
        return {"case": case, "steps": ["pick a", "place a", "pick b"], "goal": self.goal}

    def reset(self, step=0, settle=None, restore_demo=False, seed=None):
        self.progress = {"a_1": min(step, 2), "b_1": 0}
        self.x, self.t = float(step), 0
        self.restored = restore_demo
        self.seed = seed
        return {"step": step, "seed": seed, **self.check(), "obs": {"x": np.array([self.x])}}

    def resume(self, step):
        # Mid-episode: progress and t carry over, the state jumps.
        self.progress["a_1"] = max(self.progress["a_1"], min(step, 2))
        self.x = float(step)
        return {"step": step, **self.check(), "obs": {"x": np.array([self.x])}}

    def step(self, actions):
        actions = np.atleast_2d(np.asarray(actions, dtype=float))
        before = dict(self.progress)
        for action in actions:
            self.x += action[0]
            self.t += 1
            done = int(self.x)
            self.progress["a_1"] = max(self.progress["a_1"], min(done, 2))
            self.progress["b_1"] = max(self.progress["b_1"], min(done - 2, 1)) if done > 2 else 0
        completed = [
            {"object": k, "subtask": i}
            for k, n in self.progress.items()
            for i in range(before[k], n)
        ]
        return {**self.check(), "completed": completed, "t": self.t, "obs": {"x": np.array([self.x])}}

    def check(self):
        total_completed = sum(self.progress.values())
        return {"total_completed": total_completed, "total": 3, "all_done": total_completed == 3}

    def get_state(self):
        return {
            "sim_state": np.array([self.x, self.t], dtype=float),
            "controller": [{"goal_pos": np.zeros(3), "gripper": np.zeros(1)}],
        }

    def set_state(self, sim_state, controller=None):
        self.x = float(sim_state[0])
        self.controller = controller
        return {"obs": {"x": np.array([self.x])}}

    def render(self, cameras=None, size=256):
        return {name: np.zeros((size, size, 3), np.uint8) for name in cameras or ["agentview"]}

    def demo(self):
        return {"states": np.zeros((4, 2)), "actions": np.full((4, 7), 1.0)}

    def export_scene(self):
        return {
            "files": {"environment.urdf": "<robot/>", "objects/a_1.urdf": "<robot/>"},
            "manifest": {"case": self.case, "objects": {"a_1": {"pose": [0] * 7}}},
        }


@contextmanager
def served(server_module, env):
    stop = threading.Event()
    ports = []
    ready = threading.Event()
    thread = threading.Thread(
        target=server_module.serve,
        args=(env,),
        kwargs={"ready": lambda p: (ports.append(p), ready.set()), "stop": stop},
        daemon=True,
    )
    thread.start()
    assert ready.wait(5)
    try:
        yield ports[0]
    finally:
        stop.set()
        thread.join(5)


@pytest.mark.parametrize(
    "value",
    [
        np.arange(6, dtype=np.float32).reshape(2, 3),
        np.zeros((2, 2, 3), np.uint8),
        (1, "a", [np.int64(3), None]),
        {"a": [1.5, {"b": np.array([True, False])}]},
    ],
)
def test_codec_matches_long_tamp(server_module, value):
    wire = server_module.encode(value)
    assert wire == remote.encode(value)
    for back in (server_module.decode(wire), remote.decode(wire)):
        _assert_same(back, value)


def _assert_same(a, b):
    if isinstance(b, np.ndarray):
        assert a.dtype == b.dtype and np.array_equal(a, b)
    elif isinstance(b, (list, tuple)):
        assert type(a) is type(b) and len(a) == len(b)
        for x, y in zip(a, b):
            _assert_same(x, y)
    elif isinstance(b, dict):
        assert a.keys() == b.keys()
        for key in b:
            _assert_same(a[key], b[key])
    else:
        assert a == b


def test_codec_refuses_unknown(server_module):
    with pytest.raises(TypeError):
        server_module.encode(object())
    with pytest.raises(ValueError):
        server_module.decode({"$pickle": "x"})


def test_drives_a_case_end_to_end(server_module):
    with served(server_module, StubEnv()) as port, RoboCerebraClient(port=port) as env:
        assert env.server_info["service"] == "robocerebra"
        assert "step" in env.server_info["ops"]
        assert env.cases() == ["Ideal/case1", "Mix/case2"]
        info = env.load("Ideal/case1")
        assert info["steps"][0] == "pick a"
        start = env.reset(step=0)
        assert start["total_completed"] == 0
        assert env.reset(step=0, seed=123)["seed"] == 123
        assert env._call("check")["total_completed"] == 0
        assert env._call("reset", step=0, seed=456)["total_completed"] == 0
        result = env.step(np.tile([1.0, 0, 0, 0, 0, 0, -1], (2, 1)))
        assert result["t"] == 2 and result["total_completed"] == 2
        assert [c["subtask"] for c in result["completed"]] == [0, 1]
        result = env.step([1.0, 0, 0, 0, 0, 0, -1])
        assert result["all_done"]
        assert env.render(["agentview"], size=8)["agentview"].shape == (8, 8, 3)
        demo = env.demo()
        assert demo["actions"].shape == (4, 7)
        state = env.get_state()
        env.set_state(state["sim_state"])
        env.set_state(state["sim_state"], controller=state["controller"])


def test_export_scene(server_module, tmp_path):
    with served(server_module, StubEnv()) as port, RoboCerebraClient(port=port) as env:
        env.load("Ideal/case1")
        assert "export_scene" in env.server_info["ops"]
        assert set(env.export_scene()["files"]) == {"environment.urdf", "objects/a_1.urdf"}
        scene = env.export_scene(tmp_path / "case1")
    assert scene.manifest["case"] == "Ideal/case1"
    assert (tmp_path / "case1" / "objects" / "a_1.urdf").read_text() == "<robot/>"
    assert scene.objects == ["a_1"]


def test_resume_reset(server_module):
    with served(server_module, StubEnv()) as port, RoboCerebraClient(port=port) as env:
        env.load("Mix/case2")
        resumed = env.reset(step=2)
        assert resumed["total_completed"] == 2
        assert np.array_equal(resumed["obs"]["x"], [2.0])
        assert env.reset(step=1, restore_demo=True)["total_completed"] == 1


def test_benchmark_resume_keeps_the_episode(server_module):
    with served(server_module, StubEnv()) as port, RoboCerebraClient(port=port) as env:
        env.load("Ideal/case1")
        env.reset(step=0)
        assert env.step([0.5, 0, 0, 0, 0, 0, -1])["t"] == 1
        resumed = env.resume(2)
        assert resumed["total_completed"] == 2
        assert env.step([0.0, 0, 0, 0, 0, 0, -1])["t"] == 2  # no reset


def test_errors_are_reported(server_module):
    with served(server_module, StubEnv()) as port, RoboCerebraClient(port=port) as env:
        with pytest.raises(RuntimeError, match="FileNotFoundError"):
            env.load("Nope/case9")
        with pytest.raises(RuntimeError, match="unknown operation"):
            env._call("start")
        env.ping()  # the connection survives an error


def test_protocol_mismatch(server_module):
    with served(server_module, StubEnv()) as port:
        with socket.create_connection(("127.0.0.1", port)) as sock:
            remote.send_frame(sock, {"op": "hello", "version": 999})
            reply = remote.recv_frame(sock)
    assert not reply["ok"] and "protocol" in reply["error"]


def test_refuses_a_non_robocerebra_server():
    # An execution-backend server answers hello without "service".
    stop = threading.Event()
    ports, ready = [], threading.Event()
    from long_tamp.execution import MockBackend

    thread = threading.Thread(
        target=remote.serve_backend,
        args=(MockBackend(),),
        kwargs={"ready": lambda p: (ports.append(p), ready.set()), "stop": stop},
        daemon=True,
    )
    thread.start()
    assert ready.wait(5)
    try:
        with pytest.raises(ConnectionError, match="not a robocerebra"):
            RoboCerebraClient(port=ports[0])
    finally:
        stop.set()
        thread.join(5)


def test_spawn_runs_a_server_process(server_module, tmp_path):
    # The real entry point, in a child process, over a stub env module.
    stub = tmp_path / "stub_server.py"
    stub.write_text(
        "import importlib.util, sys\n"
        f"spec = importlib.util.spec_from_file_location('s', {str(SERVER)!r})\n"
        "s = importlib.util.module_from_spec(spec); spec.loader.exec_module(s)\n"
        f"sys.path.insert(0, {str(Path(__file__).parent)!r})\n"
        "from test_eval_robocerebra import StubEnv\n"
        "import argparse\n"
        "p = argparse.ArgumentParser(); p.add_argument('--port', type=int); "
        "p.add_argument('--once', action='store_true')\n"
        "a = p.parse_args()\n"
        "s.serve(StubEnv(), port=a.port, once=a.once, "
        "ready=lambda port: print(f'READY {port}', flush=True))\n"
    )
    with RoboCerebraClient.spawn(
        [sys.executable, str(stub)],
        startup_timeout=60,
        env={"PYTHONPATH": str(Path(__file__).resolve().parents[1] / "src")},
    ) as env:
        assert env.load("Ideal/case1")["case"] == "Ideal/case1"
        process = env._process
    assert process is None or process.poll() is not None


def test_environment_contacts_flatten_per_environment():
    from long_tamp.config.yaml_loader import _environment_contacts

    grouped = {"scene": ["scene/table", "scene/box_floor"], "other": ["other/shelf"]}
    assert _environment_contacts(grouped) == ["scene/table", "scene/box_floor", "other/shelf"]
    assert _environment_contacts(["scene/table"]) == ["scene/table"]
