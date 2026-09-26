"""resolve_mesh_paths(): URDF mesh paths that load on any machine."""

from pathlib import Path

from long_tamp.backends._urdf_paths import resolve_mesh_path, resolve_mesh_paths


def _urdf(path: Path, filename: str) -> Path:
    path.write_text(
        '<robot name="r"><link name="l"><visual><geometry>'
        f'<mesh filename="{filename}" scale="1 1 1"/>'
        "</geometry></visual></link></robot>"
    )
    return path


def _checkout(tmp_path: Path) -> Path:
    """<tmp>/repo/script/demo/{assets/m.stl, generated/}"""
    demo = tmp_path / "repo" / "script" / "demo"
    (demo / "assets").mkdir(parents=True)
    (demo / "generated").mkdir()
    (demo / "assets" / "m.stl").write_text("solid m\nendsolid m\n")
    return demo


def test_relative_path_resolves_against_the_urdf_folder(tmp_path):
    demo = _checkout(tmp_path)
    src = _urdf(demo / "generated" / "a.urdf", "../assets/m.stl")
    out = Path(resolve_mesh_paths(str(src)))
    assert out != src
    assert f'filename="{demo / "assets" / "m.stl"}"' in out.read_text()


def test_foreign_absolute_path_is_found_by_its_tail(tmp_path):
    demo = _checkout(tmp_path)
    foreign = "/home/someone/devel/long_tamp/script/demo/assets/m.stl"
    src = _urdf(demo / "generated" / "a.urdf", foreign)
    out = Path(resolve_mesh_paths(str(src)))
    assert f'filename="{demo / "assets" / "m.stl"}"' in out.read_text()


def test_existing_absolute_path_and_uris_are_left_alone(tmp_path):
    demo = _checkout(tmp_path)
    mesh = str(demo / "assets" / "m.stl")
    src = _urdf(demo / "generated" / "a.urdf", mesh)
    assert resolve_mesh_paths(str(src)) == str(src)
    assert resolve_mesh_path("package://pkg/m.stl", demo) == "package://pkg/m.stl"


def test_unfindable_path_is_kept(tmp_path):
    demo = _checkout(tmp_path)
    missing = "/nowhere/else/nothing.stl"
    assert resolve_mesh_path(missing, demo / "generated") == missing


def test_resolved_copy_is_reused(tmp_path):
    demo = _checkout(tmp_path)
    src = _urdf(demo / "generated" / "a.urdf", "../assets/m.stl")
    assert resolve_mesh_paths(str(src)) == resolve_mesh_paths(str(src))
