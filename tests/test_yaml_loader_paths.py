"""YamlTaskLoader resolves relative ``paths:`` entries against the YAML's folder.

A config with absolute paths works in one environment only: this repo is
bind-mounted into the HPP container at a different root than on the host,
so example configs used to hardcode the container path. Relative entries
make one config portable; absolute paths and ``package://`` URIs must pass
through untouched.
"""

import textwrap

from long_tamp.config.yaml_loader import YamlTaskLoader


def _loader(tmp_path, paths_block):
    cfg = tmp_path / "cfg" / "task.yaml"
    cfg.parent.mkdir()
    cfg.write_text("task: t\npaths:\n" + textwrap.indent(paths_block, "  "))
    return YamlTaskLoader(cfg), cfg.parent


def test_relative_entries_resolve_against_the_yaml_folder(tmp_path):
    loader, cfg_dir = _loader(
        tmp_path,
        textwrap.dedent("""\
            robot:
              arm: {urdf: ../robots/arm.urdf, srdf: ../robots/arm.srdf}
            environment:
              bench: bench.urdf
            objects:
              part: {urdf: gen/part.urdf, srdf: gen/part.srdf}
              plain: gen/plain.urdf
            """),
    )

    fp = loader.file_paths

    assert fp["robot"]["arm"]["urdf"] == str((tmp_path / "robots/arm.urdf").resolve())
    assert fp["environment"]["bench"] == str((cfg_dir / "bench.urdf").resolve())
    assert fp["objects"]["part"]["srdf"] == str((cfg_dir / "gen/part.srdf").resolve())
    assert fp["objects"]["plain"] == {
        "urdf": str((cfg_dir / "gen/plain.urdf").resolve()),
        "srdf": "",
    }


def test_absolute_package_and_empty_entries_pass_through(tmp_path):
    loader, _ = _loader(
        tmp_path,
        textwrap.dedent("""\
            robot:
              arm: {urdf: /abs/arm.urdf, srdf: "package://pkg/arm.srdf"}
            objects:
              part: {urdf: /abs/part.urdf}
            """),
    )

    fp = loader.file_paths

    assert fp["robot"]["arm"] == {
        "urdf": "/abs/arm.urdf",
        "srdf": "package://pkg/arm.srdf",
    }
    assert fp["objects"]["part"] == {"urdf": "/abs/part.urdf", "srdf": ""}
