"""Native playback selection and lifecycle without HPP or a browser."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    "mission_viewer",
    Path(__file__).parents[1] / "src/long_tamp/visualization/mission_viewer.py",
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_only_completed_paths_are_played_and_concatenated(monkeypatch):
    calls = []
    backend = SimpleNamespace(
        store_path=lambda path: 7,
        play_path=lambda pid: calls.append(pid),
        concatenate_paths=lambda ids: calls.append(tuple(ids)) or 99,
    )
    viewer = module.MissionViewer.__new__(module.MissionViewer)
    viewer.backend, viewer.path_ids = backend, []
    monkeypatch.setattr(module.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    viewer.completed(
        [
            {"complete": False, "paths": [1]},
            {"skipped": True, "paths": [2]},
            {"paths": [3, None, object()]},
        ]
    )
    viewer.finish()
    assert calls == [3, 7, (3, 7), 99]


def test_close_is_idempotent():
    stopped = []
    viewer = module.MissionViewer.__new__(module.MissionViewer)
    viewer.backend = SimpleNamespace(
        viewer=SimpleNamespace(
            viewer=SimpleNamespace(stop=lambda: stopped.append(True))
        )
    )
    viewer.close()
    viewer.close()
    assert stopped == [True]


class _Path:
    def __init__(self, q):
        self.q = q

    def length(self):
        return 0.1

    def timeRange(self):
        return (0.0, 0.1)

    def eval(self, t):
        return list(self.q), True


class _Closures:
    """Stand-in FingerClosureTable: one finger joint, 0.5 closed on 'obj/h'."""

    def has(self, gripper):
        return gripper == "arm/gripper"

    def closed_values(self, gripper, handle):
        return {"arm/finger": 0.5}

    def open_values(self, gripper):
        return {"arm/finger": 0.0}


def test_fingers_close_on_grasp_stay_closed_while_carried_and_open_on_release(
    monkeypatch,
):
    shown = []
    paths = {}

    def store(path):
        paths[len(paths)] = path
        return len(paths) - 1

    backend = SimpleNamespace(
        store_path=store,
        get_path=lambda pid: paths[pid],
        viewer=lambda q: shown.append(q[1]),
        play_path=lambda pid: (_ for _ in ()).throw(AssertionError("native playback")),
    )
    viewer = module.MissionViewer.__new__(module.MissionViewer)
    viewer.backend, viewer.path_ids = backend, []
    viewer.closures, viewer.fps = _Closures(), 30.0
    viewer._rank, viewer._fingers, viewer._segments = {"arm/finger": 1}, {}, []
    monkeypatch.setattr(module.time, "sleep", lambda s: None)
    monkeypatch.setattr(module.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    q = [0.0, 0.0]
    viewer.completed(
        [
            {
                "gripper": "arm/gripper",
                "handle": "obj/h",
                "paths": [_Path(q)],
                "state_after": "arm/gripper grasps obj/h",
            },
            # carrying move (loop): still holds, fingers stay closed
            {
                "gripper": "arm/gripper",
                "handle": None,
                "paths": [_Path(q)],
                "state_after": "arm/gripper grasps obj/h",
            },
            # virtual gripper: no fingers to move
            {
                "gripper": "tool/tip",
                "handle": "obj/hole",
                "paths": [_Path(q)],
                "state_after": "arm/gripper grasps obj/h : tool/tip grasps obj/hole",
            },
            # release: open first, then the retreat plays with open fingers
            {
                "gripper": "arm/gripper",
                "handle": None,
                "paths": [_Path(q)],
                "state_after": "free",
            },
        ]
    )
    frames = len(shown)
    n = 4  # samples per path at 30 fps over 0.1 s
    assert shown[:n] == [0.0] * n  # approach: open
    assert shown[n : n + 12][-1] == 0.5  # closes at the end of the grasp
    assert set(shown[n + 12 : n + 12 + 2 * n]) == {0.5}  # carried, then tool phase
    assert shown[n + 12 + 2 * n + 11] == 0.0  # opens before the release path
    assert set(shown[-n:]) == {0.0}
    viewer.finish()  # replays the same script from open fingers
    assert shown[frames:] == shown[:frames]


def _fake_viser(monkeypatch, started):
    """Stand-in pyhpp_viser whose Viewer.start records its keyword args."""

    class Viewer:
        def __init__(self, device, problem):
            self.viewer = SimpleNamespace(on_client_connect=lambda fn: fn)

        def start(self, **kwargs):
            started.append(kwargs)

    monkeypatch.setitem(
        module.sys.modules, "pyhpp_viser", SimpleNamespace(Viewer=Viewer)
    )
    return SimpleNamespace(
        planner=SimpleNamespace(
            device=None, problem=None, viewer=None, visualize=lambda q: None
        ),
        robot=None,
    )


def test_viewer_never_waits_for_a_browser_by_default(monkeypatch):
    """#29: open=True makes pyhpp_viser block until a browser connects, which
    never happens on a headless runner, so the mission never started."""
    started = []
    task = _fake_viser(monkeypatch, started)
    module.MissionViewer(task, 8081, [0.0])
    assert started == [{"host": "0.0.0.0", "port": 8081, "open": False}]


def test_opening_a_browser_is_opt_in(monkeypatch):
    started = []
    task = _fake_viser(monkeypatch, started)
    module.MissionViewer(task, 8081, [0.0], open_browser=True)
    assert started[0]["open"] is True
