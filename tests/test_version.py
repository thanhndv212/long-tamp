"""The version is written in ``version.py``; everything else follows it (#221)."""

import re
import tomllib
from pathlib import Path

import long_tamp
from long_tamp.version import __version__

ROOT = Path(__file__).resolve().parent.parent


def test_the_package_reports_the_version_of_version_py():
    assert long_tamp.__version__ == __version__
    assert re.fullmatch(r"\d+\.\d+\.\d+", __version__)


def test_pyproject_takes_its_version_from_version_py():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    assert "version" not in project["project"]
    assert "version" in project["project"]["dynamic"]
    dynamic = project["tool"]["setuptools"]["dynamic"]["version"]
    assert dynamic == {"attr": "long_tamp.version.__version__"}


def test_package_xml_has_the_same_version():
    # ROS reads package.xml as it is: this copy is bumped by hand.
    declared = re.search(
        r"<version>([^<]+)</version>", (ROOT / "package.xml").read_text()
    )
    assert declared.group(1) == __version__
