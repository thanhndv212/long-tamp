"""
Tests for viser and gepetto-viewer integration with PyHPPBackend.

Tests are structured in three layers:

1. **Unit tests (no HPP robot required)** — verify import guards, flag
   values, `viewer_type` storage, and `setup_viewer()` error paths.
2. **Integration tests (HPP robot required)** — load a minimal pinocchio
   device, call `setup_viewer()`, exercise `visualize()` and `play_path()`.
3. **Viewer-type dispatch tests** — verify that the correct viewer class is
   instantiated for each `viewer_type` value.

Skip markers:
  - ``pyhpp``    : requires hpp-python (`HAS_PYHPP`)
  - ``viser``    : requires pyhpp_viser (`HAS_VISER`)
  - ``gepetto``  : requires pyhpp_gepetto / gepetto-viewer-corba (`HAS_GEPETTO_VIEWER`)
"""

import pytest

# ---------------------------------------------------------------------------
# Guard: skip entire module if PyHPP is not installed
# ---------------------------------------------------------------------------
try:
    from agimus_spacelab.backends.pyhpp import (
        PyHPPBackend,
        HAS_PYHPP,
        HAS_VISER,
        HAS_GEPETTO_VIEWER,
    )
except ImportError:
    HAS_PYHPP = False
    HAS_VISER = False
    HAS_GEPETTO_VIEWER = False

requires_pyhpp = pytest.mark.skipif(not HAS_PYHPP, reason="hpp-python not available")
requires_viser = pytest.mark.skipif(
    not (HAS_PYHPP and HAS_VISER), reason="pyhpp_viser not available"
)
requires_gepetto = pytest.mark.skipif(
    not (HAS_PYHPP and HAS_GEPETTO_VIEWER),
    reason="gepetto-viewer not available",
)


# ===========================================================================
# Helpers
# ===========================================================================

def _make_backend(viewer_type: str = "auto") -> "PyHPPBackend":
    """Return a fresh, unloaded PyHPPBackend."""
    return PyHPPBackend(viewer_type=viewer_type)


def _load_minimal_robot(backend: "PyHPPBackend") -> bool:
    """
    Load the simplest available pinocchio model (example-robot-data UR5).
    Returns True when the robot was loaded successfully.
    """
    try:
        import example_robot_data as erd  # noqa: F401 – availability probe
    except ImportError:
        return False

    try:
        backend.load_robot(
            robot_name="ur5",
            urdf_path="package://example_robot_data/robots/ur_description/urdf/ur5_robot.urdf",
            srdf_path="package://example_robot_data/robots/ur_description/srdf/ur5_robot.srdf",
            root_joint_type="anchor",
        )
        return True
    except Exception:
        return False


# ===========================================================================
# 1. Unit tests — no robot required
# ===========================================================================

@requires_pyhpp
class TestViewerImportFlags:
    """HAS_VISER and HAS_GEPETTO_VIEWER must be booleans."""

    def test_has_viser_is_bool(self):
        assert isinstance(HAS_VISER, bool)

    def test_has_gepetto_viewer_is_bool(self):
        assert isinstance(HAS_GEPETTO_VIEWER, bool)

    def test_at_least_one_viewer_available(self):
        """At least one viewer should be detectable in a full HPP install."""
        # This is an informational assertion — it will only fail if
        # neither library is importable in the current environment.
        # Useful for CI to notice broken installs.
        assert HAS_VISER or HAS_GEPETTO_VIEWER, (
            "Neither pyhpp_viser nor gepetto-viewer could be imported. "
            "Install at least one viewer."
        )


@requires_pyhpp
class TestViewerTypeStorage:
    """viewer_type is stored correctly and does not affect unrelated state."""

    def test_default_viewer_type_is_auto(self):
        b = _make_backend()
        assert b._viewer_type == "auto"

    def test_viser_viewer_type_stored(self):
        b = _make_backend("viser")
        assert b._viewer_type == "viser"

    def test_gepetto_viewer_type_stored(self):
        b = _make_backend("gepetto")
        assert b._viewer_type == "gepetto"

    def test_viewer_is_none_before_setup(self):
        b = _make_backend("viser")
        assert b.viewer is None

    def test_viewer_type_does_not_affect_device(self):
        b = _make_backend("viser")
        assert b.device is None


@requires_pyhpp
class TestSetupViewerWithoutRobot:
    """setup_viewer() must raise RuntimeError when no robot is loaded."""

    def test_raises_without_robot_viser(self):
        b = _make_backend("viser")
        with pytest.raises(RuntimeError, match="load robot"):
            b.setup_viewer()

    def test_raises_without_robot_gepetto(self):
        b = _make_backend("gepetto")
        with pytest.raises(RuntimeError, match="load robot"):
            b.setup_viewer()

    def test_raises_without_robot_auto(self):
        b = _make_backend("auto")
        with pytest.raises(RuntimeError, match="load robot"):
            b.setup_viewer()


@requires_pyhpp
class TestSetupViewerUnavailableLibrary:
    """setup_viewer(viewer_type=X) raises ImportError when X is not installed."""

    def test_viser_import_error_when_unavailable(self, monkeypatch):
        if HAS_VISER:
            pytest.skip("pyhpp_viser is installed — cannot test missing-library path")
        import agimus_spacelab.backends.pyhpp as _mod
        b = _make_backend()
        # Fake a loaded device so the robot-check passes
        b.device = object()
        with pytest.raises(ImportError, match="pyhpp_viser"):
            b.setup_viewer("viser")

    def test_gepetto_import_error_when_unavailable(self, monkeypatch):
        if HAS_GEPETTO_VIEWER:
            pytest.skip("gepetto-viewer is installed — cannot test missing-library path")
        b = _make_backend()
        b.device = object()
        with pytest.raises(ImportError, match="[Gg]epetto"):
            b.setup_viewer("gepetto")


# ===========================================================================
# 2. Integration tests — require a loadable robot
# ===========================================================================

@requires_pyhpp
class TestViewerWithRobot:
    """Integration tests that load a minimal UR5 model."""

    @pytest.fixture(autouse=True)
    def _load_robot(self, request):
        """Skip the whole class if example_robot_data is not available."""
        b = _make_backend()
        loaded = _load_minimal_robot(b)
        if not loaded:
            pytest.skip("example_robot_data not available — skipping integration tests")
        self.backend = b

    def test_setup_viewer_auto_sets_viewer(self):
        """setup_viewer('auto') installs some viewer object."""
        self.backend.setup_viewer("auto")
        assert self.backend.viewer is not None

    @requires_viser
    def test_setup_viewer_viser(self):
        """setup_viewer('viser') installs a pyhpp_viser.Viewer."""
        from pyhpp_viser import Viewer as ViserViewer
        self.backend.setup_viewer("viser")
        assert self.backend.viewer is not None
        assert isinstance(self.backend.viewer, ViserViewer)

    @requires_gepetto
    def test_setup_viewer_gepetto(self):
        """setup_viewer('gepetto') installs a gepetto viewer."""
        from pyhpp.gepetto.viewer import Viewer as GepettoViewer
        self.backend.setup_viewer("gepetto")
        assert self.backend.viewer is not None
        assert isinstance(self.backend.viewer, GepettoViewer)

    @requires_viser
    def test_visualize_viser_no_exception(self):
        """visualize() with viser viewer should not raise."""
        self.backend.setup_viewer("viser")
        import numpy as np
        q = np.zeros(self.backend.device.configSize())
        self.backend.visualize(q)  # must not raise

    @requires_gepetto
    def test_visualize_gepetto_no_exception(self):
        """visualize() with gepetto viewer should not raise."""
        self.backend.setup_viewer("gepetto")
        import numpy as np
        q = np.zeros(self.backend.device.configSize())
        self.backend.visualize(q)  # must not raise

    def test_visualize_without_explicit_setup_no_exception(self):
        """visualize() auto-initialises the viewer silently."""
        # viewer is None; visualize() must not crash
        import numpy as np
        q = np.zeros(self.backend.device.configSize())
        self.backend.visualize(q)  # must not raise

    def test_play_path_without_paths_no_exception(self):
        """play_path() with empty path list must not raise."""
        self.backend.play_path(0)  # must not raise


# ===========================================================================
# 3. Viewer-type dispatch tests
# ===========================================================================

@requires_pyhpp
class TestViewerTypeDispatch:
    """
    Verify that each viewer_type value instantiates the correct class.
    Uses monkeypatching to avoid needing live X11/browser connections.
    """

    @requires_viser
    def test_dispatch_viser_sets_viser_viewer(self, monkeypatch):
        """viewer_type='viser' → _ViserViewer instantiated."""
        from pyhpp_viser import Viewer as ViserViewer
        import agimus_spacelab.backends.pyhpp as _mod

        created = []

        class _FakeViser:
            def __init__(self, *a, **kw):
                created.append(self)

            def start(self, **kw):
                pass

            def display(self, q):
                pass

        monkeypatch.setattr(_mod, "_ViserViewer", _FakeViser)
        monkeypatch.setattr(_mod, "HAS_VISER", True)

        b = _make_backend("viser")
        b.device = object()  # bypass robot-loaded check
        b.setup_viewer("viser")
        assert len(created) == 1
        assert isinstance(b.viewer, _FakeViser)

    @requires_gepetto
    def test_dispatch_gepetto_sets_gepetto_viewer(self, monkeypatch):
        """viewer_type='gepetto' → _GepettoViewer instantiated."""
        import agimus_spacelab.backends.pyhpp as _mod

        created = []

        class _FakeGepetto:
            def __init__(self, *a, **kw):
                created.append(self)

        monkeypatch.setattr(_mod, "_GepettoViewer", _FakeGepetto)
        monkeypatch.setattr(_mod, "HAS_GEPETTO_VIEWER", True)

        b = _make_backend("gepetto")
        b.device = object()
        b.setup_viewer("gepetto")
        assert len(created) == 1
        assert isinstance(b.viewer, _FakeGepetto)

    @requires_viser
    def test_dispatch_auto_prefers_viser(self, monkeypatch):
        """viewer_type='auto' → viser preferred over gepetto when both available."""
        import agimus_spacelab.backends.pyhpp as _mod

        viser_created = []

        class _FakeViser:
            def __init__(self, *a, **kw):
                viser_created.append(self)

            def start(self, **kw):
                pass

        monkeypatch.setattr(_mod, "_ViserViewer", _FakeViser)
        monkeypatch.setattr(_mod, "HAS_VISER", True)
        monkeypatch.setattr(_mod, "HAS_GEPETTO_VIEWER", True)

        b = _make_backend("auto")
        b.device = object()
        b.setup_viewer("auto")
        assert len(viser_created) == 1
        assert isinstance(b.viewer, _FakeViser)

    @requires_pyhpp
    def test_dispatch_auto_falls_back_to_gepetto_when_no_viser(self, monkeypatch):
        """viewer_type='auto' → falls back to gepetto when viser missing."""
        import agimus_spacelab.backends.pyhpp as _mod

        gepetto_created = []

        class _FakeGepetto:
            def __init__(self, *a, **kw):
                gepetto_created.append(self)

        monkeypatch.setattr(_mod, "HAS_VISER", False)
        monkeypatch.setattr(_mod, "_GepettoViewer", _FakeGepetto)
        monkeypatch.setattr(_mod, "HAS_GEPETTO_VIEWER", True)

        b = _make_backend("auto")
        b.device = object()
        b.setup_viewer("auto")
        assert len(gepetto_created) == 1
        assert isinstance(b.viewer, _FakeGepetto)

    @requires_pyhpp
    def test_dispatch_auto_raises_when_neither_available(self, monkeypatch):
        """viewer_type='auto' → ImportError when no viewer is installed."""
        import agimus_spacelab.backends.pyhpp as _mod

        monkeypatch.setattr(_mod, "HAS_VISER", False)
        monkeypatch.setattr(_mod, "HAS_GEPETTO_VIEWER", False)

        b = _make_backend("auto")
        b.device = object()
        with pytest.raises(ImportError):
            b.setup_viewer("auto")


# ===========================================================================
# 4. Visualize dispatch (unit — monkeypatched viewer)
# ===========================================================================

@requires_pyhpp
class TestVisualizeDispatch:
    """visualize() calls viewer.display() for viser, viewer() for gepetto."""

    def _backend_with_fake_viser(self, monkeypatch):
        import agimus_spacelab.backends.pyhpp as _mod

        calls = []

        class _FakeViser:
            def display(self, q):
                calls.append(("display", q))

        b = _make_backend("viser")
        monkeypatch.setattr(_mod, "HAS_VISER", True)
        # inject a pre-built fake viewer directly
        b.viewer = _FakeViser()
        return b, calls

    def _backend_with_fake_gepetto(self, monkeypatch):
        calls = []

        class _FakeGepetto:
            def __call__(self, q):
                calls.append(("call", q))

        b = _make_backend("gepetto")
        b.viewer = _FakeGepetto()
        return b, calls

    def test_viser_calls_display(self, monkeypatch):
        import numpy as np
        b, calls = self._backend_with_fake_viser(monkeypatch)
        q = np.array([1.0, 2.0, 3.0])
        b.visualize(q)
        assert calls == [("display", pytest.approx(q))]

    def test_gepetto_calls_viewer_callable(self, monkeypatch):
        import numpy as np
        b, calls = self._backend_with_fake_gepetto(monkeypatch)
        q = [0.1, 0.2, 0.3]
        b.visualize(q)
        assert len(calls) == 1
        assert calls[0][0] == "call"


# ===========================================================================
# 5. play_path dispatch (unit — monkeypatched viewer)
# ===========================================================================

@requires_pyhpp
class TestPlayPathDispatch:
    """play_path() calls viewer.loadPath() for viser (non-blocking)."""

    def test_viser_calls_load_path(self, monkeypatch):
        import agimus_spacelab.backends.pyhpp as _mod
        import numpy as np

        load_calls = []

        class _FakeViser:
            def loadPath(self, path, name=None):
                load_calls.append((path, name))

            def display(self, q):
                pass

        class _FakePath:
            def length(self):
                return 1.0

            def __call__(self, t):
                return (np.zeros(3), True)

        monkeypatch.setattr(_mod, "HAS_VISER", True)
        monkeypatch.setattr(_mod, "_ViserViewer", _FakeViser)

        b = _make_backend("viser")
        fake_viewer = _FakeViser()
        b.viewer = fake_viewer
        path = _FakePath()
        b._stored_paths = [path]

        b.play_path(0)
        assert len(load_calls) == 1
        assert load_calls[0][0] is path
        assert load_calls[0][1] == "path_0"

    def test_play_path_without_paths_no_raise(self):
        b = _make_backend()
        b.play_path(0)  # must not raise
