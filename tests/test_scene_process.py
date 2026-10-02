"""The 3D scene in its own process (#106): the client, the player's pacing,
the process end to end, and MissionViewer sending frames to it."""

import json
import pickle
import socket
import socketserver
import threading
import time
import urllib.request

import numpy as np
import pytest

from long_tamp.visualization import scene_server
from long_tamp.visualization.scene import SceneClient


class _Collector(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            message = json.loads(line)
            self.server.messages.append(message)
            if message["cmd"] == "status":
                self.wfile.write(b'{"clients": 1, "queued": 0}\n')


@pytest.fixture
def collector():
    server = socketserver.ThreadingTCPServer(("127.0.0.1", 0), _Collector)
    server.daemon_threads = True
    server.messages = []
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield server
    server.shutdown()
    server.server_close()


def test_the_client_sends_json_lines_and_reads_status(collector):
    client = SceneClient(collector.server_address[1])
    client.show(np.array([1.0, 2.0]))
    client.play([[0.0], [1.0]], 0.5)
    client.play([], 0.5)  # nothing to send
    client.frame(3.25, (4, 5))
    assert client.status() == {"clients": 1, "queued": 0}
    assert [m["cmd"] for m in collector.messages] == ["show", "play", "frame", "status"]
    assert collector.messages[0]["q"] == [1.0, 2.0]
    assert collector.messages[1] == {"cmd": "play", "dt": 0.5, "frames": [[0.0], [1.0]]}
    assert collector.messages[2] == {"cmd": "frame", "t": 3.25, "q": [4.0, 5.0]}


def test_the_client_is_an_address_and_never_raises():
    client = pickle.loads(pickle.dumps(SceneClient(9, "127.0.0.1")))
    assert (client.host, client.port) == ("127.0.0.1", 9)
    with socket.socket() as s:  # a port nobody listens on
        s.bind(("127.0.0.1", 0))
        free = s.getsockname()[1]
    gone = SceneClient(free)
    gone.show([0.0])  # no scene: nothing happens
    assert gone.status() is None


def _player():
    shown = []
    return scene_server.Player(lambda q: shown.append((time.monotonic(), q))), shown


def test_the_player_plays_frames_at_their_pace():
    player, shown = _player()
    player.play([[i] for i in range(10)], 0.05)
    time.sleep(0.8)
    assert [q for _, q in shown] == [[i] for i in range(10)]
    span = shown[-1][0] - shown[0][0]
    assert span == pytest.approx(0.45, abs=0.3)


def test_show_drops_the_queue_and_streams_keep_their_timing():
    player, shown = _player()
    player.play([[i] for i in range(100)], 0.05)
    time.sleep(0.12)
    player.show([-1])
    time.sleep(0.2)
    assert shown[-1][1] == [-1] and len(shown) < 10
    shown.clear()
    for k in range(5):  # simulated time, sent all at once
        player.frame(10.0 + 0.1 * k, [k])
    time.sleep(0.7)
    assert [q for _, q in shown] == [[k] for k in range(5)]
    assert shown[-1][0] - shown[0][0] == pytest.approx(0.4, abs=0.3)


def test_a_long_backlog_is_thinned(monkeypatch):
    monkeypatch.setattr(scene_server, "MAX_LAG", 1.0)
    player, shown = _player()
    player.play([[i] for i in range(400)], 0.01)  # 4 s queued, 1 s allowed
    assert len(player.queue) * 0.01 <= 4.0 and len(player.queue) < 400
    player.clear()


@pytest.fixture
def sample_robot():
    pin = pytest.importorskip("pinocchio")
    pytest.importorskip("pyhpp_viser")
    model = pin.buildSampleModelManipulator()
    visual = pin.buildSampleGeometryModelManipulator(model)

    class Robot:
        def model(self):
            return model

        def visualModel(self):  # noqa: N802
            return visual

        def geomModel(self):  # noqa: N802
            return visual

    return Robot(), model


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_the_scene_process_serves_the_page_and_takes_frames(sample_robot):
    from long_tamp.visualization.scene import SceneProcess

    robot, model = sample_robot
    scene = SceneProcess(robot, port=_free_port(), host="127.0.0.1")
    try:
        with urllib.request.urlopen(scene.url, timeout=10) as r:
            assert r.status == 200
        q = np.zeros(model.nq)
        scene.show(q)
        scene.play([q + 0.01 * i for i in range(30)], 1 / 30)
        status = scene.status()
        assert status["clients"] == 0 and status["queued"] > 0
        time.sleep(1.5)
        assert scene.status()["queued"] == 0
    finally:
        scene.close()
    assert scene._child.poll() is not None


class _FakeScene:
    def __init__(self):
        self.shown, self.played = [], []

    def show(self, q):
        self.shown.append(q)

    def play(self, frames, dt):
        self.played.append((len(frames), dt))


class _Path:
    def length(self):
        return 1.0

    def timeRange(self):  # noqa: N802
        return (0.0, 1.0)

    def eval(self, t):
        return np.array([t, 0.0]), True


def test_mission_viewer_sends_paths_without_waiting():
    from long_tamp.visualization.mission_viewer import MissionViewer

    viewer = MissionViewer.__new__(MissionViewer)  # without starting a scene
    viewer.scene = _FakeScene()
    viewer.closures, viewer.fps, viewer._rank = None, 30.0, {}
    viewer._fingers, viewer._segments, viewer.path_ids = {}, [], []
    stored = {}
    viewer.backend = type(
        "Backend",
        (),
        {
            "viewer": None,
            "store_path": lambda self, p: stored.setdefault(len(stored), p)
            and len(stored) - 1,
            "get_path": lambda self, pid: _Path(),
        },
    )()
    t0 = time.monotonic()
    viewer.completed([{"paths": [_Path(), _Path()]}])
    assert time.monotonic() - t0 < 0.5  # a 2 s motion, sent, not played here
    assert viewer.scene.played == [
        (31, pytest.approx(1 / 30)),
        (31, pytest.approx(1 / 30)),
    ]
    viewer.display([1.0, 2.0])
    assert viewer.scene.shown == [[1.0, 2.0]]
    assert viewer._watched()
