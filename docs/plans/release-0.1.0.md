# 0.1.0 release draft

`long-tamp` 0.1.0 is the first PyPI release. On supported Linux systems,
`python -m pip install "long-tamp[hpp]"` installs the planning library and
prebuilt HPP bindings. The base package can be installed without HPP. The
example scripts and their large robot assets remain in the repository.

## Release notes draft

- Long-horizon manipulation planning with phase-local constraint graphs,
  multi-arm grasp sequences, lookahead, and bounded block recovery.
- A four-part, two-arm screw assembly example with a cordless drill,
  crash-safe checkpoints, recorded trajectories, and Viser replay. A 10-seed
  PyPI-wheel batch completed 10/10 missions with no block replans; see the
  [recorded results](https://github.com/thanhndv212/long-tamp/blob/main/script/screw_assembly/results/pypi-wheel-batch-2026-09-26.json).
- A declarative task-plan IR compiled to BehaviorTree.CPP, with a standalone
  ROS-free host.
- Browser-based Viser visualization and a release CI path using PyPI HPP
  wheels and trusted publishing.

The full change list lives in [CHANGELOG.md](https://github.com/thanhndv212/long-tamp/blob/main/CHANGELOG.md). Move its
Unreleased entries to a dated 0.1.0 section when the release is approved.

## Release gates

- Hosted [PyPI-wheel](https://github.com/thanhndv212/long-tamp/actions/runs/36264650383)
  and [lint](https://github.com/thanhndv212/long-tamp/actions/runs/36264650392) runs
  passed on `main`; `twine check` and a clean-venv wheel install pass locally.
- The two slow TWIN checks are not a release gate: they are sampling-based
  integration tests of an example scene, excluded from push/PR CI, and listed
  under Known issues in the changelog.
- PyPI trusted publisher registered for `thanhndv212/long-tamp`, workflow
  `release.yml`, environment `pypi` (maintainer review required, `v*` tags only).
- Changelog dated; release commit tagged `v0.1.0` on `main`.
