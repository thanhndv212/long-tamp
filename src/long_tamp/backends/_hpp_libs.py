"""Make the PyPI (cmeel) HPP wheels importable without LD_LIBRARY_PATH.

The ``hpp-python`` 9.0.2 wheels ship ``pyhpp``'s extension modules and most
``libhpp-*.so`` libraries without an RPATH/RUNPATH, so the dynamic loader
cannot find ``libhpp-manipulation.so`` & co. in the wheels' shared
``cmeel.prefix/lib`` folder, and ``import pyhpp.manipulation`` fails with
"cannot open shared object file" unless ``LD_LIBRARY_PATH`` points there.

``ensure_hpp_libraries()`` works around it: when that exact failure happens,
it loads the HPP libraries from the cmeel prefix by full path, RTLD_GLOBAL,
dependencies first. A library the loader has already mapped satisfies later
lookups by its soname, so the extension modules then import normally.
Robotpkg/source builds (where the import already works) are left untouched.
"""

from __future__ import annotations

import ctypes
import importlib.util
import re
from pathlib import Path

from long_tamp.logging import get_logger

logger = get_logger("backends.hpp_libs")

# Dependency order: each library only needs the ones before it. Load each
# once, by the name the others link against: the wheels ship
# libhpp-util.so and libhpp-util.so.<version> as two separate copies, and
# mapping both puts two instances of one library in the process (heap
# corruption at exit).
_HPP_LIBRARIES = (
    "libhpp-util.so.*",
    "libhpp-pinocchio.so",
    "libhpp-statistics.so",
    "libhpp-constraints.so",
    "libhpp-core.so",
    "libhpp-manipulation.so",
    "libhpp-manipulation-urdf.so",
)
_MISSING = re.compile(r"([^\s:]+\.so[^\s:]*): cannot open shared object file")


def _cmeel_lib_dir() -> Path | None:
    """``cmeel.prefix/lib`` holding the HPP libraries, if pyhpp comes from it."""
    spec = importlib.util.find_spec("pyhpp")
    if spec is None or not spec.submodule_search_locations:
        return None
    # <prefix>/lib/pythonX.Y/site-packages/pyhpp -> <prefix>/lib
    lib = Path(next(iter(spec.submodule_search_locations))).parents[2]
    return lib if (lib / "libhpp-core.so").exists() else None


def _load(path: Path, lib_dir: Path, depth: int = 0) -> None:
    """dlopen ``path``; first load any dependency the loader reports missing
    from ``lib_dir`` (the non-HPP cmeel libraries lack a RUNPATH too)."""
    for _ in range(32):
        try:
            ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
            return
        except OSError as e:
            m = _MISSING.search(str(e))
            dep = lib_dir / m.group(1) if m else None
            if dep is None or not dep.exists() or dep == path or depth > 8:
                raise
            _load(dep, lib_dir, depth + 1)
    raise OSError(f"could not resolve the dependencies of {path}")


def preload_cmeel_hpp_libraries() -> bool:
    """Load the HPP libraries from the cmeel prefix; False if there is none."""
    lib_dir = _cmeel_lib_dir()
    if lib_dir is None:
        return False
    for pattern in _HPP_LIBRARIES:
        matches = sorted(lib_dir.glob(pattern))
        if not matches:
            continue
        # Several versioned matches: the longest name is the full soname.
        _load(max(matches, key=lambda p: len(p.name)), lib_dir)
    logger.debug(f"preloaded the HPP libraries from {lib_dir}")
    return True


def ensure_hpp_libraries() -> None:
    """Import ``pyhpp.manipulation``, preloading the cmeel HPP libraries first
    if the wheels' missing RPATH makes the plain import fail."""
    try:
        import pyhpp.manipulation  # noqa: F401
    except ImportError as e:
        if "cannot open shared object file" not in str(e):
            return  # not installed, or another problem: pyhpp.py reports it
        try:
            preload_cmeel_hpp_libraries()
        except OSError as load_error:
            logger.debug(f"preloading the cmeel HPP libraries failed: {load_error}")
