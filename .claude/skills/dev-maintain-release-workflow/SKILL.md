---
name: dev-maintain-release-workflow
description: Development, maintenance, and PyPI release workflow for long_tamp. Use when committing, branching, opening/reviewing/merging a pull request, versioning, releasing, installing long_tamp for development (pip vs CMake), debugging a stale-install ModuleNotFoundError, or deciding whether a change needs to be mirrored to/from the agimus_spacelab sibling repo.
---

# long_tamp: Develop, Maintain, Release

## Overview

`long_tamp` is a long-horizon, multi-arm TAMP library being spun out of `agimus_spacelab`
(the private, SpaceLab-specific repo it was extracted from) as a standalone open-source
project, released on PyPI. This skill covers the day-to-day workflow: branching and
commit conventions, testing (needs the HPP native stack — see the note below), and the
PyPI release process.

On Linux, `pip install -e ".[hpp,dev]"` installs the HPP 9.0.2+ wheels and
development dependencies. See `docs/plans/pypi-install-roadmap.md` for validation
and `docs/INSTALL.md` for the source-build fallback on unsupported platforms.

## When to Use

Any time you're about to commit, branch, open or merge a pull request, or cut a release.

## Feature lifecycle, tracker and validation

The full process is written for all contributors in `docs/development/workflow.md`
(roadmap → issue → ADR if design-level → branch → implement + tests → validate →
PR with docs + CHANGELOG → merge commit → issue closed, roadmap ticked → milestone gate
→ release). Validation levels V0–V4 and the screw-assembly batch gate are in
`docs/development/validation.md`. Follow those; this skill adds the agent-specific parts.

**Tracker.** GitHub issues + milestones on `thanhndv212/long-tamp`, one milestone per
roadmap milestone (`docs/plans/roadmap.md`). Before starting work, find or open the issue
(`gh issue list --milestone "M1 …"`, `gh issue view N`); put `Closes #N` in the PR body.
Labels: one `type:*`, one or more `area:*`, `needs-batch-gate` when the V3 gate is
required, `skip-changelog` for refactor/test/CI/docs-only PRs. When a PR completes a
roadmap item, tick it in `docs/plans/roadmap.md` in the same PR. Never close a milestone
without its exit test's result files committed under `script/screw_assembly/results/`.

**Validation with screw assembly.** V2 on a branch: `gh workflow run pypi.yml --ref
<branch>` (runs the `nightly-*` jobs). V3 needs hours on the maintainer's machine or
container: don't claim it ran unless it did; if it's required and can't be run in the
session, say so in the PR and leave the PR unmerged until the maintainer posts the
`summarize.py --gate` output. Baselines are per environment (PyPI-wheel vs source-built
differ ~1.8x in time); never compare across environments.

**ADRs** (`docs/adr/`): write one before a new public interface, module boundary,
dependency or plan-IR schema change. Supersede, don't rewrite, an accepted ADR.

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

**Branches**: `feature/<issue>-<short-description>`, `fix/<issue>-<short-description>`,
`refactor/…`, `docs/…`, `chore/…`, cut from `dev` (hotfixes from `main`). Keep them
short-lived. **`dev` is the integration branch** (all roadmap work), **`main` holds
releases** (release PRs `dev → main`, merged with a merge commit, and hotfixes); see
"Branches" in `docs/development/workflow.md`. `dev` stays the always-mergeable
line.

## Pull requests: open → review → merge

Every change reaches `dev` (or, for releases/hotfixes, `main`) through a pull request,
including agent-authored ones; nothing is pushed to `dev` or `main` directly. An agent drives its own PR from opening to merge:

1. **Open.** Push the branch, then open the PR against `dev` with the GitHub CLI or the
   GitHub MCP tools. Title in Conventional Commits form (`feat(grasping): ...`): it heads
   the merge commit's body. Body: **Why** (the problem, with evidence: numbers, logs),
   **What** (by module), **Validation** (tests run and their result, lint, anything checked
   by hand), **Found along the way** (issues noticed but not fixed here), **Not modelled /
   limits**. Report failures as they are, with the test name; a test that is flaky on
   `dev` too is stated as such, with the runs that show it.
2. **Watch.** Subscribe to the PR's activity (CI results, reviews, comments) and keep a
   check-in scheduled until it is merged or closed: events can arrive late or not at all.
3. **Drive to green.** On every event or check-in, look at the whole PR on its current
   head, in this order:
   - *Merge conflict*: merge `dev` into the branch, resolve, re-run the checks, push.
   - *CI red*: find the root cause and push a fix. "Flaky" is not a root cause; a
     failure that is red on `dev` too gets one PR comment naming the check and why it
     isn't this PR's. Never skip, disable or quarantine a test to get green; never push
     an empty commit or close/reopen to re-run CI.
   - *Review comments*: fix small, local asks (nits, renames, an added test) and push;
     reply on each thread and resolve it. Larger asks (multi-file refactors, API changes,
     open-ended design) go to the maintainer as a proposal before any push.
   Before each push, run the same checks CI enforces (`pre-commit run -a`, which runs
   `ruff check --select F src` and `black --check src`; `mkdocs build --strict` if docs
   changed; the tests of the touched modules) and re-read the diff.
4. **Merge** once all of these hold: every required CI check is green on the current
   head (`lint`, `lint-style` (advisory), `test-standalone`, `test-hpp`, `distribution`,
   `docs`, `changelog`, GitGuardian; the `nightly-*` jobs are skipped on PRs), the
   validation evidence the issue asked for is in the PR, the PR is mergeable with no
   conflict, no review thread is left unanswered, and no reviewer has requested changes.
   An agent merges only when the maintainer has asked it to for that PR ("merge when
   green" counts). Merge with a merge commit, `gh pr merge N --merge`: never squash, never
   rebase. `dev` keeps every per-purpose commit of the PR, and its first-parent history
   reads one merge per PR (`git log --first-parent dev`). If a PR was squashed or rebased
   by mistake, say so: the fix is rebuilding `dev` with the original commits and a merge
   commit per PR, then a `--force-with-lease` push, which needs the maintainer's go-ahead. After the merge, close the linked
   issue explicitly (`gh issue close N -c "Done in #PR (on dev)"`): closing keywords only
   fire on merges into the default branch (`main`). Then stop watching the PR and cancel
   any scheduled check-ins for it.
5. **Delete the PR branch** right after the merge, on the remote and locally:
   `git push origin --delete <branch>` (or the PR page's "Delete branch" button), then
   `git branch -D <branch>` after switching to `dev`. The merge commit on `dev` holds
   everything the branch had, so nothing is lost. Confirm it is gone with
   `git ls-remote --heads origin <branch>` (no output). If the agent can't delete it
   (the push is refused, e.g. a 403 from a session's git proxy that only allows pushes
   to its own branch), tell the maintainer the branch name and the PR link instead of
   leaving it silently. A session that must keep working under the same branch name
   resets it to the new `dev` instead (`git checkout -B <branch> origin/dev`, then
   `git push --force-with-lease` to that branch), since it holds only merged history.
6. **After merge.** User-visible changes already carry their `CHANGELOG.md` Unreleased
   entry (it is part of the PR, not a follow-up). A new session continuing the work
   starts a fresh branch from the updated `dev`; a merged PR is never reused.

## Installing `long_tamp` for development: pip vs CMake

Choose one install per environment. Two installs in one environment conflict, and the
CMake copy wins without warning.

| Situation | Use | Why |
|---|---|---|
| Editing `long_tamp` Python (any `src/long_tamp/` change, running `script/` examples, tests) | **`pip install -e .`**. In the source-built container use `pip install --no-deps -e .`: the source-built HPP libraries are compiled and linked against the env's conda-forge `pinocchio`/`eigenpy`/`numpy`, and letting pip pull the PyPI `pin`/`numpy` wheels would load a second, ABI-mismatched copy (converter errors or segfaults) | Editable: the interpreter reads `src/long_tamp/` directly, so edits take effect without reinstalling |
| Clean-venv wheel validation / release checks | `pip install -e ".[hpp,dev]"` (or the built wheel) | Matches CI and what PyPI users get |
| A downstream **CMake/ament** package needs `find_package(long_tamp)` or the installed `share/long_tamp/` data (URDF/SRDF/meshes/scripts), or you're building a frozen deployment snapshot | **CMake** `make install` into `$INSTALL_HPP_DIR` | This is the only path that exports the CMake package and installs the `share/` data |

**The trap (hit 2026-09-27):** CMake's `install(DIRECTORY src/long_tamp DESTINATION
${PYTHON_SITELIB})` *copies* the package into
`$INSTALL_HPP_DIR/lib/python3.11/site-packages/long_tamp`. The container's
`~/devel/hpp/config.sh` puts that directory on `PYTHONPATH`, and `PYTHONPATH` entries
come before the editable install's `.pth`. The copy freezes at the commit you last ran
`make install` from, so modules added afterwards fail with
`ModuleNotFoundError: No module named 'long_tamp.<new module>'` (that time it was
`visualization.mission_viewer`), and edits you make to existing modules are silently
ignored. `agimus_spacelab` avoids this because `config.sh` prepends its `src/` ahead of
the install dir. `long_tamp` gets no such prepend.

Rules:
- For day-to-day Python work in the container, do **not** `make install` long_tamp. If
  you had to (e.g. for a downstream CMake consumer), remove the Python copy afterwards
  with `rm -rf $INSTALL_HPP_DIR/lib/python3.11/site-packages/long_tamp`. The `share/`
  data and CMake config stay, and the editable install takes over again.
- If an import error names a module that exists in `src/`, first check which copy is
  loaded:
  `python -c "import long_tamp; print(long_tamp.__file__)"`. It must print
  `.../src/long_tamp/src/long_tamp/__init__.py`. Any `install/.../site-packages` path is
  a stale copy.

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

## No SpaceLab-mission content

`long_tamp` was extracted from `agimus_spacelab`, a private, SpaceLab-specific repo whose
maintenance ended in September 2026. Nothing is ported between the two any more.
SpaceLab-mission content (`script/spacelab/`, part and gripper names of the real mission,
its checkpoints and fixtures) was removed from this tree and its history on purpose, and
must not come back. `script/screw_assembly/` is the long-horizon reference mission.

## Release process (PyPI)

First release: `0.1.0` (2026-09-27, see `docs/plans/release-0.1.0.md`). When cutting one:

1. **Version**: `src/long_tamp/version.py` (and `package.xml`) follows semver. `0.x` while the API
   is still moving (per the `Development Status :: 3 - Alpha` classifier already in
   `pyproject.toml`); bump to `1.0.0` once the public API (documented in
   `docs/usage/standalone-usage.md` and `docs/architecture.md`) is considered stable.
2. **What ships**: the Python package; example scripts/assets stay in the checkout.
   `pip install "long-tamp[hpp]"` adds the native HPP wheels on supported Linux systems.
   The base install still imports without HPP. Keep source-built environments separate.
3. **Build and check**: `python -m build` (sdist + wheel), then `twine check dist/*`.
   Test in a clean venv: `pip install dist/*.whl` and confirm
   `python -c "from long_tamp import get_available_backends"` imports without pulling in
   any HPP native package.
4. **Tag**: annotated git tag matching the version (`vX.Y.Z`), pushed after the version
   bump reaches `main` through the release PR `dev → main` (merge commit, not squash).
5. **Publish**: the tag triggers `.github/workflows/release.yml`, which checks the
   version, builds and smoke-tests the artifacts, then publishes using PyPI trusted
   publishing. Configure the publisher for owner `thanhndv212`, repo `long-tamp`,
   workflow `release.yml`, environment `pypi` before pushing the tag. Configure any
   required reviewers on that GitHub environment before enabling publication.
6. **Release notes**: move `CHANGELOG.md`'s Unreleased entries into the dated release
   section at release time, and use them for the GitHub release notes.
7. **Back-merge**: open a PR `main → dev` and merge it with a merge commit (it changes no
   files). Without it, `dev` lacks the release merge commit and the branches read "1 ahead,
   1 behind" after every release. See `docs/development/workflow.md` §Release.
8. Before the *first* public release specifically: do one more `git log --all --name-only`
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
  nav entry updated in the same commit. Enforced: the CI `docs` job runs
  `mkdocs build --strict` with `validation.omitted_files: warn`, so a page missing from
  `nav` or a broken link fails the PR. Links from docs to repo-root files (README,
  src/) use absolute GitHub URLs, since mkdocs only sees `docs/`.

## Common Rationalizations

| Rationalization | Reality |
|---|---|
| "I'll port `agimus_spacelab`'s `script/spacelab/` (or its `spacelab-example` branch) into `long_tamp`, just scoped to `script/`" | That reintroduces exactly what the filter-repo history rewrite was done to remove — the mission-specific content (real part/gripper/handle names, the actual assembly sequence) isn't confined to config, it's baked into the Python itself. Build a from-scratch generic example instead (see Phase 4 in the vault note); don't port. |
| "Lint is green, so tests pass" | Check the PyPI-wheel test job separately; lint does not exercise planning. |
| "I'll just `make install` so the container picks up my change" | That puts a frozen copy on `PYTHONPATH` that shadows the editable install. Use `pip install --no-deps -e .` for Python work; CMake install is only for downstream CMake consumers (see "Installing `long_tamp` for development"). |
| "PyPI release is blocked until we sort out HPP distribution" | It isn't — the PyPI package is pure-Python only; HPP is a runtime dependency the user provides, not a packaging blocker. |

## Verification

Before landing a change:
- [ ] Linked issue (`Closes #N`) with acceptance criteria met
- [ ] Commit message follows `type(scope): description`, explains why
- [ ] Validation levels the issue requires were run and recorded (V3/V4 result files
      under `script/screw_assembly/results/`), or the gap is stated in the PR
- [ ] `CHANGELOG.md` entry, or `skip-changelog` label with a reason
- [ ] Roadmap item ticked in `docs/plans/roadmap.md` if this PR completes one; ADR
      added/superseded if a design decision changed
- [ ] `pytest tests/ -q` run in an HPP-enabled environment (not just lint)
- [ ] No SpaceLab-mission-specific content reintroduced (script paths, part/gripper naming
      tied to the real mission, checkpoint/fixture data) — see
      `research-vault/agimus-spacelab/agimus-spacelab-opensource-release.md` for what that
      content looks like if unsure
- [ ] Docs (`docs/usage/`, `docs/architecture.md`, `README.md`, `mkdocs.yml` nav) updated if the
      change is user-visible
- [ ] Landed through a PR that met the merge conditions in "Pull requests: open → review
      → merge" (CI green on the current head, no conflict, every review thread answered)
- [ ] PR branch deleted after the merge (remote and local), or the maintainer told which
      branch is left and why
