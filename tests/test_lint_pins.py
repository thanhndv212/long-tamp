"""CI, pre-commit and the ``dev`` extra run the same black and ruff (#221): a
check that passes on a laptop must not fail in CI on a newer release."""

import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _pre_commit_pins() -> dict[str, str]:
    text = (ROOT / ".pre-commit-config.yaml").read_text()
    pins = {}
    for tool, repo in (("ruff", "ruff-pre-commit"), ("black", "psf/black")):
        rev = re.search(rf"{repo}\s+rev: v?([\d.]+)", text)
        pins[tool] = rev.group(1)
    return pins


def test_the_ci_lint_jobs_install_the_pre_commit_versions():
    pins = _pre_commit_pins()
    workflow = (ROOT / ".github" / "workflows" / "lint.yml").read_text()
    checked = 0
    for line in re.findall(r"pip install[^\n]*", workflow):
        for tool, version in pins.items():
            if re.search(rf"\b{tool}\b", line):
                assert f"{tool}=={version}" in line, line
                checked += 1
    assert checked >= 2  # the lint job installs both


def test_the_dev_extra_installs_the_pre_commit_versions():
    pins = _pre_commit_pins()
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    dev = project["project"]["optional-dependencies"]["dev"]
    for tool, version in pins.items():
        assert f"{tool}=={version}" in dev


def test_ci_and_pre_commit_run_the_whole_rule_set():
    # #226: the tree is clean under [tool.ruff.lint]; a narrower selection in
    # either place would let the debt grow back unseen.
    workflow = (ROOT / ".github" / "workflows" / "lint.yml").read_text()
    assert re.search(r"run: ruff check src\s*$", workflow, re.M)
    assert "continue-on-error" not in workflow.split("test-standalone:")[0]
    hook = (ROOT / ".pre-commit-config.yaml").read_text()
    ruff_hook = hook.split("id: ruff")[1].split("- repo:")[0]
    assert "--select" not in ruff_hook


def test_the_modules_mypy_skips_all_exist():
    # #228: the list of modules whose errors are not reported only shrinks. A
    # module that was renamed or removed must leave it, or the entry would
    # silently cover whatever takes the name next.
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    (override,) = project["tool"]["mypy"]["overrides"]
    assert override["ignore_errors"] is True
    for module in override["module"]:
        assert (ROOT / "src" / (module.replace(".", "/") + ".py")).is_file(), module
    assert len(override["module"]) <= 39  # the count at introduction
