"""RoboCerebra scenes as long_tamp scenes (issue #157, R2).

The env server's ``export_scene`` op turns the compiled MuJoCo model of a
case into URDF/SRDF texts plus a manifest (``script/robocerebra/
scene_export.py``): the static fixtures as one environment model whose SRDF
holds the LIBERO regions as contact surfaces, each movable object as its own
model (a freeflyer, with a ``bottom`` contact), and where the robot stands.
This side writes them to disk and reads them back as ``SceneBuilder`` file
paths and HPP configurations.

::

    export = env.export_scene()                 # a RoboCerebraClient
    scene = write_export(export, "out/Ideal_case1")
    scene.file_paths()["environment"]           # {"environment": {"urdf", "srdf"}}
    scene.object_config("butter_1")             # [x, y, z, qx, qy, qz, qw]
"""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

MANIFEST = "manifest.json"


def write_export(export: dict[str, Any], out_dir: str | Path) -> ExportedScene:
    """Write an ``export_scene`` result (``files`` and ``manifest``) under
    ``out_dir`` and return it as an :class:`ExportedScene`."""
    root = Path(out_dir)
    for rel, text in export["files"].items():
        if Path(rel).is_absolute() or ".." in Path(rel).parts:
            raise ValueError(f"export file outside the scene directory: {rel!r}")
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    (root / MANIFEST).write_text(json.dumps(export["manifest"], indent=1))
    return ExportedScene(root, export["manifest"])


def load_export(directory: str | Path) -> ExportedScene:
    """An exported scene written by :func:`write_export`."""
    root = Path(directory)
    return ExportedScene(root, json.loads((root / MANIFEST).read_text()))


def held_export(
    scene: ExportedScene,
    target: str,
    joints: dict[str, float],
    pose: list[float],
    out_dir: str | Path,
) -> ExportedScene:
    """A copy of ``scene`` under ``out_dir`` with the robot at ``joints``
    and ``target`` at ``pose``: the scene a pick leaves, before it runs.
    The fingers stay as exported (open, wider than on the object)."""
    root = Path(out_dir)
    shutil.copytree(scene.root, root, dirs_exist_ok=True)
    manifest = json.loads(json.dumps(scene.manifest))
    manifest["robot"]["joints"].update(joints)
    manifest["objects"][target]["pose"] = list(pose)
    (root / MANIFEST).write_text(json.dumps(manifest, indent=1))
    return ExportedScene(root, manifest)


@dataclass
class ExportedScene:
    """One exported case on disk: ``root`` holds ``environment.urdf|srdf``,
    ``objects/<name>.urdf|srdf`` and the ``manifest``."""

    root: Path
    manifest: dict[str, Any]

    @property
    def environment_name(self) -> str:
        return self.manifest["environment"]["name"]

    @property
    def objects(self) -> list[str]:
        return list(self.manifest["objects"])

    @property
    def contacts(self) -> list[str]:
        """The environment contact surfaces (the supported LIBERO regions)."""
        return [r["region"] for r in self.manifest["regions"] if "support_geom" in r]

    def file_paths(self) -> dict[str, Any]:
        """``SceneBuilder.FILE_PATHS`` entries for the environment and the
        objects (the robot is the caller's)."""
        env = self.environment_name
        return {
            "environment": {
                env: {
                    "urdf": str(self.root / f"{env}.urdf"),
                    "srdf": str(self.root / f"{env}.srdf"),
                }
            },
            "objects": {
                name: {
                    "urdf": str(self.root / "objects" / f"{name}.urdf"),
                    "srdf": str(self.root / "objects" / f"{name}.srdf"),
                }
                for name in self.objects
            },
        }

    def object_config(self, name: str) -> list[float]:
        """The object's freeflyer configuration, ``[x, y, z, qx, qy, qz, qw]``."""
        return list(self.manifest["objects"][name]["pose"])

    def robot_pose(self, link: str | None = None) -> list[float]:
        """World pose ``[x, y, z, qx, qy, qz, qw]`` of the robot root body, or
        of one of its ``link``s at the exported joint values."""
        robot = self.manifest["robot"]
        return list(robot["pose"] if link is None else robot["links"][link])

    def robot_joints(self) -> dict[str, float]:
        """The robot's joint values at export, by MuJoCo joint name."""
        return dict(self.manifest["robot"]["joints"])
