"""Shared test setup.

Some tests import ``pyhpp`` directly. With the PyPI HPP wheels that only
works once long_tamp has loaded the HPP shared libraries (the wheels'
extension modules have no RPATH), so do it before any test is collected.
"""

from long_tamp.backends._hpp_libs import ensure_hpp_libraries

ensure_hpp_libraries()


import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _no_local_ai_config(monkeypatch, tmp_path_factory):
    """Tests never load the machine's AI env file or default model: no keys
    from the developer's setup, the same defaults everywhere."""
    for name in ("LONG_TAMP_AI_ENV", "LONG_TAMP_GOAL_MODEL"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path_factory.mktemp("xdg")))
