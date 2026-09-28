# Contributing to long_tamp

Thanks for your interest. This page is the short version; the full process is in
[docs/development/workflow.md](docs/development/workflow.md) and validation in
[docs/development/validation.md](docs/development/validation.md).

## Set up

```bash
git clone https://github.com/thanhndv212/long-tamp.git && cd long-tamp
python -m venv .venv && source .venv/bin/activate
pip install -e ".[hpp,dev,docs]" pytest-timeout pre-commit   # Linux; see docs/INSTALL.md
pre-commit install
```

The native HPP wheels are Linux-only today. On other platforms, or to change HPP
itself, see [docs/INSTALL.md](docs/INSTALL.md) for the source-built environment. Don't mix
a source-built HPP environment with the PyPI wheels in one venv.

## Find something to work on

- The [roadmap](docs/plans/roadmap.md) lists milestones; each item is a
  [GitHub issue](https://github.com/thanhndv212/long-tamp/issues).
  `good first issue` marks self-contained ones.
- For anything non-trivial, comment on the issue (or open one with the templates)
  before starting, so the design can be agreed first. Design-level changes get an
  [ADR](docs/adr/README.md).

## Make the change

1. Branch from `dev` (the integration branch; `main` holds releases):
   `feature/<issue>-<slug>` (or `fix/…`, `docs/…`, `chore/…`).
2. Write tests with the code. Real-scene planning checks are marked
   `@pytest.mark.slow_planning`.
3. Commit with [Conventional Commits](https://www.conventionalcommits.org/):
   `feat(planning): add …`, `fix(backends): …`. The body explains why.
4. Validate at the level your change needs (V0–V4). Anything that can change what gets
   planned is validated on the screw-assembly mission.
5. Update `CHANGELOG.md` (`[Unreleased]`) for user-visible changes, and the docs
   (`docs/usage/` is the living reference; new pages go into `mkdocs.yml`).

## Open a pull request

Open it **against `dev`**, use the PR template and link the issue (`Closes #N`). A PR
merges when CI is green on its latest commit, it has no conflicts, every review thread is answered, and the
validation evidence the issue asked for is in the PR. PRs are squash-merged with their
Conventional Commits title.

## Checks you can run locally

```bash
pre-commit run -a                                  # ruff (errors) + black
python -m pytest tests -m "not slow_planning" --timeout=300 --timeout-method=thread
mkdocs build --strict                              # docs
```

## Reporting bugs and security issues

Open a bug with the template: the command, the seed, the log and the environment
(`pip freeze`, OS, HPP install type). For security issues, don't open a public issue;
contact the maintainer through the email on their GitHub profile.

## License

By contributing, you agree that your contributions are licensed under the project's
[LICENSE](LICENSE).
