# Roadmap: PyPI as the default install

**Goal**: `pip install "long-tamp[hpp]"` is the whole install on Linux, with no
LD_LIBRARY_PATH, no robotpkg, no source build and no Docker. The source-built
`hpp-agimus-arm64` container stays as the path for changing HPP itself.

**Why it's realistic**: the full 4-part screw-assembly mission (19 blocks, seed 1,
610 s of planning) completed on the stock `hpp-python` 9.0.2 wheels, and the test suite
passes on them (444 passed, 17 skipped, 0 failed). The extra bindings long_tamp needs
(`RSTimeParameterization`, `EnforceTransitionSemantic`, `GraphRandomShortcut`, …) are in
9.0.2.

## Status

| Step | What | Status |
|---|---|---|
| 1 | Remove the manual setup steps | **Done** (below) |
| 2 | CI on GitHub Actions against the PyPI wheels | To do |
| 3 | Publish `long-tamp` 0.1.0 to PyPI | To do (needs the maintainer) |
| 4 | SessionStart hook for Claude Code cloud sessions | To do |
| 5 | Check in the source-built container and refresh the batch results | To do (needs the maintainer) |

## Step 1 (done): remove the manual setup steps

All on `feature/screw-assembly-example`:

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

A workflow on every push and pull request, Ubuntu, Python 3.11:

1. `pip install -e ".[hpp,dev]" pytest-timeout`
2. `pytest tests --timeout=300`, so a hang fails fast instead of blocking the job.
3. `python script/screw_assembly/task_screw_assembly.py --check` and
   `python script/ikea_table_prototype/task_assemble_table.py --show-joints`, to confirm
   both example scenes load.
4. Nightly only: a 1-part screw mission (`build_scene.py --parts 1`, then
   `task_screw_assembly.py --seed 1`), about 7 minutes.

The full suite took about 6 minutes on a cloud container, mostly in two real-scene twin
integration tests (203 s and 136 s,
`test_twin_regrasp_bt_session.py::test_release_is_forced_before_regrasp` and
`test_grasp_release_use_case_twin.py::test_grasp_release_lifecycle`). If that's too slow
per push, move those two to the nightly job. Measure the runtime on GitHub's runners on
the first run.

This would have caught both hangs above and the stale `test_grasp_sequence_logging`
fixture, which were all invisible while `pyhpp` couldn't be imported.

## Step 3: publish `long-tamp` 0.1.0 to PyPI

Follow the `dev-maintain-release-workflow` skill (versioning, tag, CHANGELOG Unreleased →
0.1.0). Decisions for the maintainer:

- Whether 0.1.0 is ready to be public.
- The PyPI account, or a trusted-publisher setup from GitHub Actions (preferred, no token
  to store).
- Whether example assets ship in the package. They would add about 10 MB (the UR10 and
  Robotiq meshes, the drill scan); keeping `script/` in the repo only is simpler.

After this, `docs/INSTALL.md`'s `pip install long-tamp` line becomes literally true.

## Step 4: SessionStart hook for Claude Code cloud sessions

A hook in the repo's `.claude/` settings that runs `pip install -e ".[hpp,dev]"` when a
cloud session starts (the `session-start-hook` skill covers the setup). After step 1 that
is the only setup needed, so every new cloud session can run the tests and examples
straight away. Cost: about a minute of install per new session.

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
  [hpp-core unbounded planning loops](../bugs/hpp-core-unbounded-planning-loops.md)) still
  needs an hpp-core change; it will reach PyPI users through a new wheel like any other fix.
