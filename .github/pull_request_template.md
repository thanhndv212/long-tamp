<!-- Base branch: `dev` (feature/fix work) or `main` (release PR from dev, or a hotfix).
     Title: Conventional Commits, e.g. "feat(task_planning): check preconditions in TaskStepReady".
     It becomes the squash commit's subject. -->

Closes #

## Why

<!-- The problem, with evidence (numbers, logs, a failing test). -->

## What

<!-- The change, by module. -->

## Validation

<!-- Levels from docs/development/validation.md. Tick what you ran, paste results.
     A level you skipped: say which and why. -->

- [ ] V0 local: lint + tests of touched modules
- [ ] V1 CI green on the latest commit
- [ ] V2 smoke mission (`gh workflow run pypi.yml --ref <branch>`): run link
- [ ] V3 batch gate (`summarize.py --gate --baseline …`): result file + gate output
- [ ] V4 initial-state scenarios (from milestone M1 on)

## Docs and changelog

- [ ] `CHANGELOG.md` `[Unreleased]` entry, or `skip-changelog` label (reason: …)
- [ ] Docs updated (`docs/usage/`, `ARCHITECTURE.md`, `mkdocs.yml` nav) or not needed
- [ ] ADR added/updated, or no design decision changed

## Found along the way / limits

<!-- Issues noticed but not fixed here (open issues for them), and what this doesn't cover. -->
