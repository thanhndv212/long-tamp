"""The RoboCerebra scene exporter (issue #157): a compiled MjModel to
URDF/SRDF texts and a manifest, checked on a small hand-written model with
LIBERO's shapes (a table region, a drawer region on a rotated site, a wall
slab, a capsule leg, a plane floor, a free object, the robot left out); and
long_tamp's side: writing the export, reading it back as scene file paths,
and loading an environment with an SRDF."""

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from long_tamp.eval import ExportedScene, load_export, write_export
from long_tamp.planning.scene import SceneBuilder

EXPORTER = (
    Path(__file__).resolve().parents[1] / "script" / "robocerebra" / "scene_export.py"
)

MJCF = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05"/>
    <body name="table" pos="0 0 0.4">
      <geom name="table_top" type="box" size="0.5 0.5 0.02"/>
      <geom name="table_leg" type="capsule" size="0.02 0.18" pos="0.4 0.4 -0.2"/>
      <site name="table_region" pos="0 0 0.016" size="0.3 0.3 0.01"/>
    </body>
    <body name="cabinet" pos="0.3 0 0.42">
      <geom name="cabinet_base" type="box" size="0.1 0.1 0.01" pos="0 0 0.01"/>
      <geom name="cabinet_wall" type="box" size="0.1 0.005 0.15" pos="0 0.1 0.15"/>
      <site name="cabinet_side" pos="0 0.09 0.15" size="0.1 0.005 0.1"/>
      <body name="drawer" pos="0 0 0.1">
        <joint name="drawer_slide" type="slide" axis="1 0 0" range="0 0.1"/>
        <geom name="drawer_floor" type="box" size="0.08 0.06 0.005"/>
        <site name="drawer_region" pos="0 0 0.01" quat="0.7071068 0 0 0.7071068"
              size="0.05 0.06 0.01"/>
      </body>
    </body>
    <body name="cube" pos="-0.2 0 0.4499">
      <freejoint/>
      <geom name="cube_g" type="box" size="0.03 0.03 0.03"/>
    </body>
    <body name="robot0_base" pos="-0.5 0 0.4">
      <geom name="robot_geom" type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture(scope="module")
def exporter():
    pytest.importorskip("mujoco")
    spec = importlib.util.spec_from_file_location("rc_scene_export", EXPORTER)
    module = importlib.util.module_from_spec(spec)
    sys.modules["rc_scene_export"] = module
    spec.loader.exec_module(module)
    yield module
    sys.modules.pop("rc_scene_export", None)


@pytest.fixture(scope="module")
def export(exporter):
    import mujoco

    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return exporter.export_scene(
        model, data, regions=["table_region", "drawer_region", "cabinet_side"]
    )


def _normal(points):
    p = np.asarray(points, dtype=float)
    return np.cross(p[1] - p[0], p[2] - p[0])


def _contact(srdf, name):
    for contact in ET.fromstring(srdf).iter("contact"):
        if contact.get("name") == name:
            values = [float(x) for x in contact.find("point").text.split()]
            return contact.find("link").get("name"), np.reshape(values, (-1, 3))
    raise KeyError(name)


def test_files_parse_and_leave_the_robot_out(export):
    files = export["files"]
    assert sorted(files) == [
        "environment.srdf",
        "environment.urdf",
        "objects/cube.srdf",
        "objects/cube.urdf",
    ]
    for text in files.values():
        ET.fromstring(text)
    env = ET.fromstring(files["environment.urdf"])
    links = {link.get("name") for link in env.iter("link")}
    # The floor plane is on MuJoCo's world body.
    assert links == {"environment_root", "world", "table", "cabinet", "drawer"}
    assert "robot_geom" not in files["environment.urdf"]
    assert export["manifest"]["robot"]["pose"][:3] == pytest.approx([-0.5, 0, 0.4])


def test_regions_pick_their_own_support(export):
    regions = {r["region"]: r for r in export["manifest"]["regions"]}
    # The cabinet's base is higher than the table top and under the table
    # region's footprint too: the region's own body wins.
    assert regions["table_region"]["support_geom"] == "table_top"
    assert regions["table_region"]["height"] == pytest.approx(0.42)
    # The drawer's floor, not the cabinet's base under it; the site is
    # rotated, its footprint is its world box.
    assert regions["drawer_region"]["support_geom"] == "drawer_floor"
    assert regions["drawer_region"]["link"] == "drawer"
    assert "vertical" in regions["cabinet_side"]["reason"]
    assert "support_geom" not in regions["cabinet_side"]


def test_region_contacts_face_up_in_their_link(export):
    srdf = export["files"]["environment.srdf"]
    link, points = _contact(srdf, "drawer_region")
    assert link == "drawer"
    # In the drawer's frame: the floor's top, the site's rotated footprint.
    assert points[:, 2] == pytest.approx(np.full(4, 0.005))
    assert np.ptp(points[:, 0]) == pytest.approx(0.12)  # 2 * 0.06, rotated
    assert np.ptp(points[:, 1]) == pytest.approx(0.10)
    assert _normal(points)[2] > 0
    link, points = _contact(srdf, "table_region")
    assert link == "table" and _normal(points)[2] > 0
    with pytest.raises(KeyError):
        _contact(srdf, "cabinet_side")


def test_object_is_its_own_model_with_a_bottom(export):
    obj = export["manifest"]["objects"]["cube"]
    assert obj["body"] == "cube" and obj["geoms"] == ["cube_g"]
    assert obj["pose"] == pytest.approx([-0.2, 0, 0.4499, 0, 0, 0, 1])
    assert obj["bottom_tilt"] == pytest.approx(0, abs=1e-9)
    link, points = _contact(export["files"]["objects/cube.srdf"], "bottom")
    assert link == "base_link"
    assert points[:, 2] == pytest.approx(np.full(4, -0.03))
    assert _normal(points)[2] < 0  # outward, down


def test_shapes_are_named_as_pinocchio_names_them(export):
    shapes = export["manifest"]["shapes"]
    # The capsule leg is a cylinder and two spheres after the table top.
    assert [shapes[f"environment/table_{i}"]["geom"] for i in range(4)] == [
        "table_top", "table_leg", "table_leg", "table_leg",
    ]
    ends = sorted(shapes[f"environment/table_{i}"]["pose"][2] for i in (2, 3))
    assert ends == pytest.approx([0.2 - 0.18, 0.2 + 0.18])
    # The plane is a slab whose top is the plane.
    floor = next(s for s in shapes.values() if s["geom"] == "floor")
    assert floor["pose"][2] < 0
    assert shapes["cube/base_link_0"]["pose"][:3] == pytest.approx([-0.2, 0, 0.4499])
    assert not any(s["geom"] == "robot_geom" for s in shapes.values())


def test_articulations_and_contacts(export):
    manifest = export["manifest"]
    (slide,) = manifest["articulated"]
    assert slide["joint"] == "drawer_slide" and slide["type"] == "slide"
    assert slide["range"] == pytest.approx([0, 0.1])
    assert slide["bodies"] == ["drawer"]
    pairs = {tuple(sorted((c["geom1"], c["geom2"]))) for c in manifest["contacts"]}
    assert ("cube_g", "table_top") in pairs  # the cube sinks 0.1 mm


def test_write_and_load_back(export, tmp_path):
    scene = write_export(export, tmp_path / "case")
    assert isinstance(scene, ExportedScene)
    back = load_export(tmp_path / "case")
    assert back.manifest == scene.manifest
    paths = back.file_paths()
    env = paths["environment"]["environment"]
    assert Path(env["urdf"]).read_text() == export["files"]["environment.urdf"]
    assert Path(env["srdf"]).is_file()
    assert Path(paths["objects"]["cube"]["srdf"]).is_file()
    assert back.objects == ["cube"]
    assert set(back.contacts) == {"table_region", "drawer_region"}
    assert back.object_config("cube") == pytest.approx([-0.2, 0, 0.4499, 0, 0, 0, 1])
    assert back.robot_pose() == pytest.approx([-0.5, 0, 0.4, 0, 0, 0, 1])


@pytest.mark.parametrize("rel", ["../escape.urdf", "/abs.urdf"])
def test_write_refuses_paths_outside(tmp_path, rel):
    with pytest.raises(ValueError, match="outside"):
        write_export({"files": {rel: "x"}, "manifest": {}}, tmp_path)


def test_scene_builder_loads_an_environment_srdf():
    builder = object.__new__(SceneBuilder)
    builder.planner = MagicMock()
    builder.FILE_PATHS = {
        "environment": {"env": {"urdf": "e.urdf", "srdf": "e.srdf"}, "ground": "g.urdf"}
    }
    builder.load_environment(["env", "ground"])
    first, second = builder.planner.load_environment.call_args_list
    assert first.kwargs == {
        "name": "env", "urdf_path": "e.urdf", "pose": None, "srdf_path": "e.srdf"
    }
    assert second.kwargs == {"name": "ground", "urdf_path": "g.urdf", "pose": None}
