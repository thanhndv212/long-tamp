---
name: dev-maintain-release-workflow
description: Development, maintenance, and PyPI release workflow for long_tamp. Use when committing, branching, versioning, releasing, or deciding whether a change needs to be mirrored to/from the agimus_spacelab sibling repo.
---

# long_tamp: Develop, Maintain, Release

## Overview

`long_tamp` is a long-horizon, multi-arm TAMP library being spun out of `agimus_spacelab`
(the private, SpaceLab-specific repo it was extracted from) as a standalone open-source
project, to be released on PyPI. This skill covers the day-to-day workflow: branching and
commit conventions (carried over from `agimus_spacelab`, which has the longer track
record), what to do while both repos are maintained in parallel, testing (needs the
HPP native stack — see the note below), and the PyPI release process.

On Linux, `pip install -e ".[hpp,dev]"` installs the HPP 9.0.2+ wheels and
development dependencies. See `docs/plans/pypi-install-roadmap.md` for validation
and `docs/INSTALL.md` for the source-build fallback on unsupported platforms.

## When to Use

Any time you're about to commit, branch, cut a release, or are unsure whether a change
belongs in `long_tamp`, `agimus_spacelab`, both, or neither.

## Commit and Branch Conventions

Carried over from `agimus_spacelab`'s established practice — same conventions, same
reasoning, now on GitHub instead of GitLab.

**Commit messages** — Conventional Commits, `type(scope): description`:

```
feat(planning): add ...
fix(backends): stop ...
refactor(tasks)!: remove ...        # "!" = breaking change
docs(usage): rewrite ...
test(grasp_sequence): cover ...
```

Types: `feat`, `fix`, `refactor`, `docs`, `test`, `style`, `chore`. Scope is the module or
area touched (`backends`, `planning`, `tasks`, `docs`, `script/twin`, …). Body explains
*why*, not what — the diff already shows what.

**Branches**: `feature/<short-description>`, `fix/<short-description>`,
`refactor/<short-description>`. Keep them short-lived; `main` stays the always-mergeable
line.

## Testing

Run `python -m pytest tests --timeout=300 --timeout-method=thread` with `pytest-timeout` installed and the
`[hpp,dev]` extra. CI in `.github/workflows/pypi.yml` runs that suite on Python 3.11
against PyPI wheels (excluding the two `slow_planning` checks), loads both example
scenes, and runs those two checks plus a one-part mission nightly
(and on workflow dispatch). It also builds and smoke-tests the distribution.

The source-built `hpp-agimus-arm64` container remains the path for changing HPP itself;
its environment must not be mixed with the wheel dependencies. Use a clean venv for
wheel validation, without source-install `PYTHONPATH` or `LD_LIBRARY_PATH` entries.

## CI

- Enforcing lint: `ruff check --select F src`, `black --check src`.
- Advisory lint: the full `ruff check src` style set remains non-blocking.
- Base-install tests: `[dev]`, without HPP, plus the standalone config smoke.
- Wheel tests, example scene checks, nightly mission and distribution checks:
  `.github/workflows/pypi.yml`.
- Publication: `.github/workflows/release.yml`, after a matching version tag and
  maintainer configuration of the `pypi` environment and PyPI trusted publisher.

## Parallel maintenance with `agimus_spacelab` (through end of September 2026)

**This section has an expiry.** `agimus_spacelab` is maintained only through end of
September 2026, after which this whole dual-maintenance dance stops applying — re-read
the actual state of both repos before trusting this section past that date, don't just
follow it on autopilot.

Until then, both repos are developed in parallel, and most feature/fix work should be
**mirrored or ported to the other repo** — with one explicit exception:

- **`task_planning/` (the TaskPlan IR → compiler → BehaviorTree.CPP path) is
  `long_tamp`-only.** Never port it back to `agimus_spacelab`. It's the new,
  forward-looking architecture that supersedes `agimus_spacelab`'s legacy, proprietary DBT
  mission-executive path (`ros2_ws_agimusxads` / `spacelab_bt_ros` / `libDBT.so`) — porting
  it back would reintroduce exactly the coupling the open-source split exists to avoid.
- **Everything else** (`backends/`, `planning/`, `tasks/` excluding `task_planning/`,
  generic `config/`/`logging/`/`visualization/`/`utils/`/`cli/` fixes) — when you land a
  fix or feature in one repo, check whether the same code exists in the other and port it
  over. The two package trees are structurally near-identical (`long_tamp` is a straight
  rename of `agimus_spacelab`'s tree at the point of the split, with SpaceLab-mission
  content and CORBA removed), so most patches apply directly or with light adaptation.
- **SpaceLab-mission-specific work** (`script/spacelab/`, the screwdriving mission, and
  anything that only makes sense against that proprietary scene) stays in `agimus_spacelab`
  only — it was deliberately excluded from `long_tamp`'s tree and history (see
  `research-vault/agimus-spacelab/agimus-spacelab-opensource-release.md` in the workspace
  vault for the full reasoning) and must not be reintroduced.

### The `spacelab-example` branch — does not exist in `long_tamp`

Earlier notes here (and in the vault) claimed `long_tamp`'s GitHub repo carried a
`spacelab-example` branch with a full working SpaceLab BT.CPP integration, kept private for
validation. **That was wrong, corrected 2026-09-02.** Verified directly:
`git ls-remote origin spacelab-example` against `long_tamp`'s actual GitHub remote returns
nothing — `origin` has only `main`. What existed was a local-only, unpushed branch in one
working copy, pointing at a commit that's genuinely part of `agimus_spacelab`'s own history
(where the real `spacelab-example` branch lives). It's been deleted from the `long_tamp`
working copy; nothing to clean up on GitHub since it was never pushed there.

The from-scratch `script/screw_assembly/` example now provides the long-horizon,
multi-phase validation of `GraspSequencePlanner`, lookahead and checkpointing. See
Phase 4 in `research-vault/agimus-spacelab/agimus-spacelab-opensource-release.md`: a
from-scratch generic long-horizon example (fictional mission, no SpaceLab content) is the
tracked way to close that gap, not porting or referencing anything from `agimus_spacelab`.

## Release process (PyPI)

Not yet done for `long_tamp` — no release has shipped. When cutting one:

1. **Version**: `pyproject.toml`'s `[project] version` follows semver. `0.x` while the API
   is still moving (per the `Development Status :: 3 - Alpha` classifier already in
   `pyproject.toml`); bump to `1.0.0` once the public API (documented in
   `docs/usage/standalone-usage.md` and `ARCHITECTURE.md`) is considered stable.
2. **What ships**: the Python package; example scripts/assets stay in the checkout.
   `pip install "long-tamp[hpp]"` adds the native HPP wheels on supported Linux systems.
   The base install still imports without HPP. Keep source-built environments separate.
3. **Build and check**: `python -m build` (sdist + wheel), then `twine check dist/*`.
   Test in a clean venv: `pip install dist/*.whl` and confirm
   `python -c "from long_tamp import get_available_backends"` imports without pulling in
   any HPP native package.
4. **Tag**: annotated git tag matching the version (`vX.Y.Z`), pushed after the version
   bump commit lands on `main`.
5. **Publish**: the tag triggers `.github/workflows/release.yml`, which checks the
   version, builds and smoke-tests the artifacts, then publishes using PyPI trusted
   publishing. Configure the publisher for owner `thanhndv212`, repo `long-tamp`,
   workflow `release.yml`, environment `pypi` before pushing the tag. Configure any
   required reviewers on that GitHub environment before enabling publication.
6. **Release notes**: move `CHANGELOG.md`'s Unreleased entries into the dated release
   section at release time, and use them for the GitHub release notes.
7. Before the *first* public release specifically: do one more `git log --all --name-only`
   sweep across `long_tamp`'s history (all branches, `git branch -a`) for anything
   SpaceLab-tagged that a future contributor's branch might have reintroduced since the
   original scrub.

## Docs maintenance

- `docs/usage/` is the **living reference** — keep it current as the API changes; other
  docs explicitly defer to it when they disagree (see the "stale references to ignore"
  notes already in those files).
- `docs/legacy/` holds content that's accurate as history but describes removed
  functionality (the CORBA backend, the SpaceLab example, the DBT/ROS 2 integration) —
  add a `> **Legacy.** ...` banner explaining what changed when moving something there
  (see existing files in that folder for the pattern). Don't delete genuinely useful
  historical engineering rationale; archive it instead, unless it's 100% about content
  that must not exist in this repo at all (SpaceLab-mission specifics) — that gets removed
  outright, not archived.
- `mkdocs.yml`'s `nav` must stay in sync with `docs/` — a moved or removed file needs its
  nav entry updated in the same commit, not left dangling (this repo hand-verifies nav
  links resolve; there's no automated check for it yet — consider adding one).

## Common Rationalizations

| Rationalization | Reality |
|---|---|
| "I'll port this to agimus_spacelab later" | Later becomes never once the two trees drift. Port in the same session, or note it explicitly (e.g., a memory/vault entry) if truly deferred. |
| "This task_planning/ fix is small, agimus_spacelab could use it too" | No — task_planning/ is long_tamp-only, full stop, regardless of size. It supersedes the DBT path there; porting it back reintroduces the coupling the split was for. |
| "I'll port `agimus_spacelab`'s `script/spacelab/` (or its `spacelab-example` branch) into `long_tamp`, just scoped to `script/`" | That reintroduces exactly what the filter-repo history rewrite was done to remove — the mission-specific content (real part/gripper/handle names, the actual assembly sequence) isn't confined to config, it's baked into the Python itself. Build a from-scratch generic example instead (see Phase 4 in the vault note); don't port. |
| "Lint is green, so tests pass" | Check the PyPI-wheel test job separately; lint does not exercise planning. |
| "PyPI release is blocked until we sort out HPP distribution" | It isn't — the PyPI package is pure-Python only; HPP is a runtime dependency the user provides, not a packaging blocker. |

## Verification

Before landing a change:
- [ ] Commit message follows `type(scope): description`, explains why
- [ ] `pytest tests/ -q` run in an HPP-enabled environment (not just lint)
- [ ] If the change touches `backends/`, `planning/`, `tasks/` (outside `task_planning/`),
      `config/`, `logging/`, `visualization/`, `utils/`, or `cli/` — considered whether it
      should be ported to `agimus_spacelab` too (through end of Sept 2026)
- [ ] If the change touches `task_planning/` — confirmed it stays `long_tamp`-only
- [ ] No SpaceLab-mission-specific content reintroduced (script paths, part/gripper naming
      tied to the real mission, checkpoint/fixture data) — see
      `research-vault/agimus-spacelab/agimus-spacelab-opensource-release.md` for what that
      content looks like if unsure
- [ ] Docs (`docs/usage/`, `ARCHITECTURE.md`, `README.md`, `mkdocs.yml` nav) updated if the
      change is user-visible
