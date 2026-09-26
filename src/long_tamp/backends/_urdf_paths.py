"""Resolve a URDF's mesh paths so the same file loads on any machine.

URDFs in this repo name their meshes by absolute path on whichever machine
generated them (``/Users/<name>/...`` on a host, ``/home/<name>/...`` in the
hpp-agimus container), which breaks on every other machine. Before a URDF is
handed to HPP, ``resolve_mesh_paths()`` rewrites each ``<mesh filename>``:

- ``package://`` and ``file://`` URIs are left to HPP;
- a relative path is resolved against the URDF's folder;
- an absolute path that exists is kept;
- an absolute path that does not exist is matched by its tail against the
  URDF's parent folders: ``/home/x/long_tamp/script/a/assets/m.stl`` is
  found at ``<checkout>/script/a/assets/m.stl`` when the URDF sits under
  ``<checkout>/script/...``. The longest tail that exists wins.

If nothing changes the original path is returned; otherwise a resolved copy
is written to a cache folder and its path returned.
"""

from __future__ import annotations

import hashlib
import re
import tempfile
from pathlib import Path, PurePosixPath

from long_tamp.logging import get_logger

logger = get_logger("backends.urdf_paths")

_MESH = re.compile(r'(<mesh\b[^>]*?\bfilename=")([^"]+)(")')
_CACHE = Path(tempfile.gettempdir()) / "long_tamp_urdf"


def _find_by_tail(missing: str, bases: list[Path]) -> Path | None:
    parts = PurePosixPath(missing).parts[1:]  # drop the leading "/"
    for start in range(len(parts) - 1):  # longest tail first, at least dir/file
        tail = Path(*parts[start:])
        for base in bases:
            candidate = base / tail
            if candidate.is_file():
                return candidate
    return None


def resolve_mesh_path(filename: str, urdf_dir: Path) -> str:
    """The path HPP should load for one ``<mesh filename>`` value."""
    if "://" in filename:
        return filename
    path = Path(filename)
    if not path.is_absolute():
        return str((urdf_dir / path).resolve())
    if path.exists():
        return filename
    found = _find_by_tail(filename, [urdf_dir, *urdf_dir.parents])
    return str(found) if found is not None else filename


def resolve_mesh_paths(urdf_path: str) -> str:
    """``urdf_path``, or a copy of it with every mesh path made loadable here."""
    source = Path(urdf_path)
    if "://" in urdf_path or not source.is_file():
        return urdf_path
    text = source.read_text()
    urdf_dir = source.resolve().parent
    unresolved: list[str] = []

    def _sub(m: re.Match) -> str:
        new = resolve_mesh_path(m.group(2), urdf_dir)
        if new.startswith("/") and not Path(new).exists():
            unresolved.append(new)
        return m.group(1) + new + m.group(3)

    resolved = _MESH.sub(_sub, text)
    for missing in sorted(set(unresolved)):
        logger.warning(f"{source.name}: mesh not found: {missing}")
    if resolved == text:
        return urdf_path
    digest = hashlib.sha1(
        (str(source.resolve()) + "\0" + resolved).encode()
    ).hexdigest()[:16]
    _CACHE.mkdir(parents=True, exist_ok=True)
    out = _CACHE / f"{source.stem}.{digest}.urdf"
    if not out.exists():
        tmp = out.with_suffix(f".{digest}.tmp")
        tmp.write_text(resolved)
        tmp.replace(out)
    logger.debug(f"{source.name}: mesh paths resolved -> {out}")
    return str(out)
