# Roadmap: PyPI as the default install

**Goal**: `pip install "long-tamp[hpp]"` is the whole install on Linux, with no
LD_LIBRARY_PATH, no robotpkg, no source build and no Docker. The source-built
`hpp-agimus-arm64` container stays as the path for changing HPP itself.

**Why it's realistic**: the original cloud run completed the full 4-part screw-assembly
mission (19 blocks, seed 1, 610 s) on stock `hpp-python` 9.0.2 wheels and reported
444 passed, 17 skipped, 0 failed. The local ARM64 rerun below exposes two slow TWIN
failures, so that earlier result is not a blanket reliability claim. The extra bindings
long_tamp needs
(`RSTimeParameterization`, `EnforceTransitionSemantic`, `GraphRandomShortcut`, …) are in
9.0.2.

## Status

| Step | What | Status |
|---|---|---|
| 1 | Remove the manual setup steps | **Done** (below) |
| 2 | CI on GitHub Actions against the PyPI wheels | Implemented; first hosted push run passed on `main` |
| 3 | Publish `long-tamp` 0.1.0 to PyPI | **Done** 2026-09-27: [PyPI](https://pypi.org/project/long-tamp/0.1.0/), [GitHub release](https://github.com/thanhndv212/long-tamp/releases/tag/v0.1.0) |
| 4 | SessionStart hook for Claude Code cloud sessions | Implemented and locally exercised |
| 5 | Check in the source-built container and refresh the batch results | Done: source mission passed; ten-seed wheel batch refreshed below |

## Step 1 (done): remove the manual setup steps

Developed on `feature/screw-assembly-example` and merged to `main`:

- **No `LD_LIBRARY_PATH`.** The 9.0.2 wheels ship `pyhpp`'s extension modules without an
  RPATH. `long_tamp/backends/_hpp_libs.py` preloads the HPP libraries from
  `cmeel.prefix/lib` when the plain import fails on a missing shared library, loading each
  once by soname. The wheels ship `libhpp-util.so` and `libhpp-util.so.9.0.2` as two
  copies; loading both corrupted the heap at exit. `tests/conftest.py` runs the preload
  before tests that import `pyhpp` directly.
- **Examples load from any checkout.** `long_tamp/backends/_urdf_paths.py` resolves URDF
  mesh paths at load time: relative ones against the URDF, absolute ones from another
  machine by their tail under the URDF's parent folders. The IKEA config names its URDFs
  relative to itself, and `screw_assembly/build_scene.py` writes the drill mesh relative.
- **Error message.** The backend's import error says HPP ≥ 9.0.2 works and points to the
  wheels.
- **Tests that hung on the wheels.** They exercise upstream's `_visitedGrasps` memo, which
  9.0.2 lacks (see "Upstream" below). `test_graph_factory_visited_memo.py` skips with that
  reason; `test_pruned_recursion.py` keeps its bounded-growth check and skips only the
  comparison against the unbounded upstream walk.

Verified with no symlink and no `LD_LIBRARY_PATH`: a 1-part screw-assembly mission ran end
to end (seed 2, 418 s), both example scenes load with every mesh found, and the suite
passes.

## Step 2: CI on GitHub Actions

Implemented in [pypi.yml](https://github.com/thanhndv212/long-tamp/blob/main/.github/workflows/pypi.yml), alongside the existing
[lint/base-install workflow](https://github.com/thanhndv212/long-tamp/blob/main/.github/workflows/lint.yml). The removed
`[standalone]` extra was corrected to `[dev]` in the base-install job.
Push/PR jobs cover the fast test suite and both scene loads; nightly/manual runs
add the one-part mission and two isolated TWIN planning jobs. The distribution job builds an sdist and wheel, runs `twine check`, then
installs the wheel into a clean venv outside the checkout. Test reports, resolved
wheel versions, mission logs and distributions are uploaded as artifacts.
Job and process timeouts bound the longer planning checks. Scheduled runs require
this workflow on the default branch. The first hosted push run on `main` passed:
[PyPI wheels](https://github.com/thanhndv212/long-tamp/actions/runs/36264650383)
reported 442 passed, 17 skipped, and two slow checks deselected in 6.54 s of
pytest time; the HPP job took 53 s overall. The distribution job passed in 31 s.
The separate [lint workflow](https://github.com/thanhndv212/long-tamp/actions/runs/36264650392)
also passed. Nightly-only jobs were correctly skipped on this push.


The workflow uses Ubuntu and Python 3.11:

1. `pip install -e ".[hpp,dev]" pytest-timeout`
2. `pytest tests -m "not slow_planning" --timeout=300 --timeout-method=thread`,
   using a watchdog thread instead of Python signal handling. An outer
   20-minute process timeout also bounds native calls that hold the GIL.
3. `python script/screw_assembly/task_screw_assembly.py --check` and
   `python script/ikea_table_prototype/task_assemble_table.py --show-joints`, to confirm
   both example scenes load.
4. Nightly only: a 1-part screw mission (`build_scene.py --parts 1`, then
   `task_screw_assembly.py --seed 1`), about 7 minutes.

The original full-suite cloud run took about 6 minutes, mostly in two TWIN
integration checks. The local ARM64 rerun took 442 s and exposed two failures:

- `test_grasp_release_use_case_twin.py::test_grasp_release_lifecycle`: the second
  grasp starts with the ball outside its joint bounds (y=0.5797, allowed -0.4..0.4).
  The isolated nightly command also fails (89.86 s), on a left-finger/ball collision.
- `test_twin_regrasp_bt_session.py::test_release_is_forced_before_regrasp`: exceeded
  its 300 s timeout. A native stack sample was in `BiRrtStar::improve` /
  `TransitionPlanner::computePath`, not the spline QP.

Both tests are marked `slow_planning` and run in separate nightly matrix jobs,
with no `continue-on-error` or expected-failure marker. The regular suite excludes
only these two checks. These are unresolved planner/scene regressions, not proof
of a fully green release; keep their nightly results visible and investigate them
before release. Reproduce with `python -m pytest tests/<filename>.py -m slow_planning
--timeout=300 --timeout-method=thread` under an external six-minute timeout.
Measure the two slow checks and mission on the scheduled hosted run.

This would have caught both hangs above and the stale `test_grasp_sequence_logging`
fixture, which were all invisible while `pyhpp` couldn't be imported.

## Step 3: publish `long-tamp` 0.1.0 to PyPI

The proposed public release notes and remaining gates are in
[release-0.1.0.md](release-0.1.0.md).

The tag-triggered [release.yml](https://github.com/thanhndv212/long-tamp/blob/main/.github/workflows/release.yml) checks tag/version
agreement, builds and validates the distributions, smoke-tests the installed wheel,
and publishes from a separate job with `id-token: write` and environment `pypi`.
It follows [PyPI trusted publishing](https://docs.pypi.org/trusted-publishers/using-a-publisher/).

Verified 2026-09-26: GitHub reports `thanhndv212/long-tamp` as public and PyPI's
project JSON endpoint returns 404. The GitHub `pypi` environment now requires
review by `thanhndv212` and allows deployment only from tags matching `v*`.
The maintainer's remaining steps are:

1. Register a [pending PyPI publisher](https://docs.pypi.org/trusted-publishers/creating-a-project-through-oidc/)
   for project `long-tamp`, owner `thanhndv212`, repository `long-tamp`, workflow
   `release.yml`, environment `pypi`. No token is stored in the repository.
2. Address the two failing nightly TWIN checks,
   and move Unreleased changelog entries to the dated `0.1.0` section. Once the
   release commit is on `main`, create and push annotated tag `v0.1.0`.

The existing distribution scope is retained: example scripts and assets remain in
the checkout. Both the wheel and sdist were inspected; neither contains example
assets, run logs, build directories or agent configuration. No tag or publication
has been performed. The Unreleased changelog remains open until release time.

After publication, `python -m pip install "long-tamp[hpp]"` becomes available.
Until then, install from the checkout as described in `docs/INSTALL.md`.

## Step 4: SessionStart hook for Claude Code cloud sessions

Implemented in [.claude/settings.json](https://github.com/thanhndv212/long-tamp/blob/main/.claude/settings.json) and
[session-start.sh](https://github.com/thanhndv212/long-tamp/blob/main/.claude/hooks/session-start.sh). It runs on startup/resume,
exits immediately unless `CLAUDE_CODE_REMOTE=true`, and installs `[hpp,dev]` plus
`pytest-timeout` from `CLAUDE_PROJECT_DIR`. The hook has a 600-second timeout and
propagates installation failures. Local sessions do not install anything.

The referenced `session-start-hook` skill was unavailable locally; implementation
follows the [official cloud hook documentation](https://code.claude.com/docs/en/cloud-environments#install-dependencies-with-a-sessionstart-hook).
The local no-op and simulated remote installation paths were exercised. An actual
Claude Code cloud startup remains to be observed after these files are pushed.

## Step 5: check in the source-built container

- Run the seed-1 screw-assembly mission in `hpp-agimus-arm64` to compare planning time
  against the PyPI run (610 s on the cloud container) and confirm step 1 changes nothing
  on a source build: the library preload only runs when the plain import fails.
- Rerun the 10-seed batch (`bash run_batch.sh 10 3`) on the stack you recommend. The
  README's results table predates the drill and the pedestal collision fix (each arm is
  now checked against its own pedestal).

## Upstream

- **hpp-python wheels have no RPATH** on `pyhpp`'s extension modules (only
  `libhpp-manipulation-urdf.so` has one), and they ship two copies of `libhpp-util`. Report
  to `hpp-python` / its cmeel packaging; once fixed, `_hpp_libs.py` can go.
- **`_visitedGrasps` memo is not in the 9.0.2 wheels.** See
  [Constraint-graph factory combinatorial blowup](../bugs/constraint-graph-factory-combinatorial-blowup.md).
  long_tamp's `PrunedRecursionMixin` carries its own memoized `_recurse`, so long_tamp is
  unaffected; direct users of HPP's `ConstraintGraphFactory` on the wheels are not.
- **SplineGradientBased QP hang** (Bug 6 in
  [hpp-core unbounded planning loops](../bugs/hpp-core-unbounded-planning-loops.md)) is fixed
  in hpp-core 9.1.0 (upstream #459). It reaches PyPI users once 9.1.0 wheels are published;
  until then the backend drops the spline optimizer on the 9.0.2 wheels.

## Local validation (2026-09-26)

Validation uses a clean Python 3.11 venv inside `hpp-agimus-arm64`, with stock
`hpp-python` / `hpp-gepetto-viewer` 9.0.2 wheels and no `PYTHONPATH` or
`LD_LIBRARY_PATH`. The minimal container needed system `libgomp1` for Pinocchio;
this runtime is a prerequisite for minimal Linux images. See `docs/INSTALL.md`.

- Base wheel, installed in a separate clean venv: imports outside the checkout;
  standalone config smoke passes; **363 passed, 98 skipped** in the test suite.
- Both example scene-load commands pass against the HPP wheels.
- `python -m build` and `twine check` pass for the wheel and sdist.
- All workflow files pass actionlint 1.7.12. Source lint and formatting pass.
- Full HPP suite: **442 passed, 17 skipped, 2 failed** in 442 s (TWIN failures above).
- Push/PR HPP selection: **442 passed, 17 skipped, 2 deselected** in 4.66 s.
- One-part nightly mission, seed 1: **success in 139.86 s**.
- Installing the built wheel with `[hpp]` into the clean venv exposes `pyhpp` and
  loads/validates the four-part screw scene, without source-package imports.
- Source-built HPP, four parts, seed 1: **success in 913.41 s**, using the current
  source checkout; importing `pyhpp.core` directly succeeded before `long_tamp`.
- Wheel batch seed 1: **success in 567.46 s**. These concurrent ARM64 runs are not
  a controlled benchmark against the earlier 610 s cloud result.
- Ten-seed four-part wheel batch: **10/10 complete**, **0/140 planning blocks
  replanned**, **13/13 failures recovered**, median **687 s** (547–979 s).
  Every process exited 0; results are in
  [pypi-wheel-batch-2026-09-26.json](https://github.com/thanhndv212/long-tamp/blob/main/script/screw_assembly/results/pypi-wheel-batch-2026-09-26.json).
  The example README now reports this local drill run; it was not a hosted
  GitHub Actions job.
