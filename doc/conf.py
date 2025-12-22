# Sphinx configuration for agimus_spacelab docs.

from __future__ import annotations

import os
import sys
from datetime import datetime


def _ensure_numpy_stub() -> None:
    """Provide a tiny numpy stub for autodoc when numpy isn't installed.

    Some modules use constants like ``np.pi`` in default arguments at import time.
    A MagicMock-based mock can break numeric operations (e.g. unary ``-``).
    This stub keeps imports working without requiring full numpy.
    """

    try:
        import numpy  # noqa: F401
    except Exception:
        import math
        import types

        numpy_stub = types.ModuleType("numpy")
        numpy_stub.pi = math.pi

        class ndarray:  # noqa: N801
            pass

        numpy_stub.ndarray = ndarray

        def _unavailable(*_args, **_kwargs):
            raise ImportError("numpy is required to execute this function")

        numpy_stub.array = _unavailable
        numpy_stub.concatenate = _unavailable
        sys.modules["numpy"] = numpy_stub

# Make the package importable for autodoc.
DOC_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.abspath(os.path.join(DOC_DIR, ".."))
SRC_DIR = os.path.abspath(os.path.join(PROJECT_ROOT, "src"))
sys.path.insert(0, SRC_DIR)

_ensure_numpy_stub()

project = "Agimus Spacelab"
author = "Agimus Spacelab contributors"

try:
    import agimus_spacelab  # noqa: F401

    version = getattr(agimus_spacelab, "__version__", "0.0.0")
except Exception:
    version = "0.0.0"

release = version

copyright = f"{datetime.now().year}, {author}"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

# Generate autosummary stubs (optional but useful for API pages).
autosummary_generate = True

autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# The project imports optional heavy dependencies (hpp/pyhpp/pinocchio).
# Mock them so docs can build on machines without the HPP stack installed.
autodoc_mock_imports = [
    "hpp",
    "hpp.corbaserver",
    "hpp.corbaserver.manipulation",
    "hpp.gepetto",
    "hpp.gepetto.manipulation",
    "pyhpp",
    "pyhpp.core",
    "pyhpp.manipulation",
    "pyhpp.constraints",
    "pyhpp.gepetto",
    "pinocchio",
    "pinocchio.rpy",
    "numpy",
]

# Parse .rst sources by default.
source_suffix = {
    ".rst": "restructuredtext",
}

master_doc = "index"

exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "*.md",
]

# Note: Markdown files in this repo are embedded via .rst wrapper pages.
# We intentionally do not build .md files as standalone source documents.

# Theme: prefer ReadTheDocs if installed, otherwise fall back to a built-in.
try:
    import sphinx_rtd_theme  # noqa: F401

    html_theme = "sphinx_rtd_theme"
except Exception:
    html_theme = "alabaster"

# Keep the build output deterministic-ish.
html_last_updated_fmt = "%Y-%m-%d"
