"""Native playback selection and lifecycle without HPP or a browser."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    'mission_viewer', Path(__file__).parents[1] / 'src/long_tamp/visualization/mission_viewer.py'
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
    monkeypatch.setattr(module.sys, 'stdin', SimpleNamespace(isatty=lambda: False))
    viewer.completed([
        {'complete': False, 'paths': [1]},
        {'skipped': True, 'paths': [2]},
        {'paths': [3, None, object()]},
    ])
    viewer.finish()
    assert calls == [3, 7, (3, 7), 99]


def test_close_is_idempotent():
    stopped = []
    viewer = module.MissionViewer.__new__(module.MissionViewer)
    viewer.backend = SimpleNamespace(viewer=SimpleNamespace(
        viewer=SimpleNamespace(stop=lambda: stopped.append(True))))
    viewer.close()
    viewer.close()
    assert stopped == [True]
