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

- The two real-scene TWIN checks still fail in isolated nightly validation:
  one encounters a finger/ball collision, and the regrasp case can exceed
  the six-minute process limit. Keep these results visible and resolve them
  before tagging. The normal push/PR selection passed 442 tests with 17
  skips and these two checks deselected.
- Inspect the first hosted GitHub Actions run on `main`. Local checks passed:
  source lint and formatting, actionlint, `python -m build`, `twine check`,
  clean wheel installation, both example scene loads, and the ten-seed batch.
- Register `long-tamp` as a pending PyPI trusted publisher for GitHub owner
  `thanhndv212`, repository `long-tamp`, workflow `release.yml`, environment
  `pypi`. Then tag the release commit on `main` as `v0.1.0`.

No PyPI upload or release tag has been made.
