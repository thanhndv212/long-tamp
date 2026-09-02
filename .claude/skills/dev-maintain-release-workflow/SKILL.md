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

**How HPP and its native bindings get installed as a dependency is an open question,
deferred for a separate discussion** — this skill covers everything else and treats that
as a known gap, not something to solve here.

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

The real test suite (`pytest tests/`) needs `pyhpp` — HPP's native Python bindings, not on
PyPI. There is no CI runner with the HPP stack yet (see CI below), so **testing before a
commit/PR means running it yourself** in an environment with HPP installed — currently the
`hpp-agimus-arm64` container (source-built HPP; see the container's own docs for how it's
built) is the only verified environment. `python -m pytest tests/ -q` from the repo root,
inside that environment.

Do not skip this because CI will "catch it later" — CI currently only lints (see below).

## CI (current state)

Matches `agimus_spacelab`'s current setup, adapted for GitHub Actions instead of GitLab CI:
**lint-only**, because the real test suite needs the HPP native stack and there is no
published container image a hosted runner can pull yet (same gap noted in
`agimus_spacelab`'s `.gitlab-ci.yml`).

- Enforcing: `ruff check --select F src` (real errors — undefined/unused names, etc.) and
  a formatter check. A failure here blocks the pipeline.
- Advisory: the full `ruff check src` style set, non-blocking (`continue-on-error`) until
  the existing style debt is burned down.

When an HPP-stack container image becomes available somewhere a GitHub Actions runner can
pull from, add a real `pytest tests/` job — mirror the commented-out sketch in
`agimus_spacelab`'s `.gitlab-ci.yml` for the shape of it.

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

### The `spacelab-example` branch

`long_tamp`'s GitHub repo carries a `spacelab-example` branch — the full working SpaceLab
BT.CPP integration (CORBA already removed, `task_planning/`'s screwdriving adapter still
intact), kept for **private validation during current development only**: it's the one
proven long-horizon, multi-phase example, useful for proving out changes to
`GraspSequencePlanner`/lookahead/checkpointing against a real complex mission before the
generic examples under `script/twin/` catch up to that level of complexity.

- It must never be merged into `main`, and never referenced from `main`'s docs as a
  current example (this is exactly what the docs/tests scrub in this repo's history
  removed).
- **It will be deleted, or replaced with a from-scratch generic long-horizon example,
  at release time** (when `long_tamp` goes public / hits PyPI). Do not build anything on
  the assumption that this branch survives past that point.
- The repo itself is currently **private** specifically so this branch can safely exist.
  If `long_tamp` is made public before the branch is dealt with, delete the branch first.

## Release process (PyPI)

Not yet done for `long_tamp` — no release has shipped. When cutting one:

1. **Version**: `pyproject.toml`'s `[project] version` follows semver. `0.x` while the API
   is still moving (per the `Development Status :: 3 - Alpha` classifier already in
   `pyproject.toml`); bump to `1.0.0` once the public API (documented in
   `docs/usage/standalone-usage.md` and `ARCHITECTURE.md`) is considered stable.
2. **What actually ships to PyPI**: the pure-Python package only (per `README.md`'s own
   install-tier table) — `pip install long-tamp` gives config parsing, planning-graph
   construction, transforms, run logging, and the viser viewer. The HPP native bindings
   (the actual planning backend) are never on PyPI and must come from the user's own
   environment; instantiating a backend without them raises an `ImportError` naming what's
   missing. This means a PyPI release is **not** blocked on the still-open "how do we
   distribute HPP" question — that question only affects how a user gets a *working*
   installation, not whether the package can be published.
3. **Build and check**: `python -m build` (sdist + wheel), then `twine check dist/*`.
   Test in a clean venv: `pip install dist/*.whl` and confirm
   `python -c "from long_tamp import get_available_backends"` imports without pulling in
   any HPP native package.
4. **Tag**: annotated git tag matching the version (`vX.Y.Z`), pushed after the version
   bump commit lands on `main`.
5. **Publish**: `twine upload dist/*` (or a GitHub Actions release workflow triggered by
   the tag — not yet set up; add one modeled on the standard PyPA
   `pypa/gh-action-pypi-publish` action, using trusted publishing rather than a stored
   API token, when this is actually set up).
6. **GitHub release notes**: summarize what changed since the last tag — this is also
   where a `CHANGELOG.md` would get updated, if/when one exists (doesn't yet).
7. Before the *first* public release specifically: confirm the `spacelab-example` branch
   has been handled (see above), and do one more `git log --all --name-only` sweep across
   `long_tamp`'s history for anything SpaceLab-tagged that a future contributor's branch
   might have reintroduced since the original scrub.

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
| "I'll just merge spacelab-example into main to save time" | That reintroduces exactly what the filter-repo history rewrite (and the force-push that corrected it) was done to remove. Never merge that branch into main. |
| "CI is green, so tests pass" | CI is lint-only right now. Green CI says nothing about `pytest tests/` — run it yourself in the HPP environment. |
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
