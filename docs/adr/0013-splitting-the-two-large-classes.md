# 0013. Splitting `grasp_sequence.py` and `pyhpp.py`; exceptions and logging

- Status: Accepted
- Date: 2026-11-11
- Issues/PRs: #234 (this), #233 (the profile that found a doubled optimizer pass in
  `pyhpp.py`)

## Context

Two modules hold the core of planning, each as one class:

| Module | Lines | Class | Methods | Attributes set on `self` |
|---|---|---|---|---|
| `tasks/grasp_sequence.py` | 4990 | `GraspSequencePlanner` | 40 | 27 |
| `backends/pyhpp.py` | 2933 | `PyHPPBackend` | 88 | 40 |

- Four methods are 250 to 650 lines: `_plan_phase_edges` (652),
  `find_feasible_phase_target` (361, with four nested functions),
  `_build_phase_graph_and_constraints` (265), `plan_sequence` (253).
- `PyHPPBackend` mixes six subjects: model loading, graph and constraints, path storage
  and files (about 470 lines), time parameterization, the transition planner (about 700),
  viewer playback.
- The tests reach into both: about 140 uses of a private member (`_plan_phase_edges` 22
  times, `_compute_or_plan_path` 8, `_stored_paths` 6). 55 files import the first module
  and 26 the second.
- Both are in mypy's list of unchecked modules (#228).
- `src` has 198 `except Exception` (29 with a reason in a `noqa`), 116
  `raise RuntimeError` and 177 `print(`. A catch-all has hidden a real fault at least
  once: `_paths_exist_unbudgeted` turns any exception into "no path".
- Every change to these two files needs the batch gate (V3, about 90 minutes), so the
  number of slices matters as much as their size.

## Decision

1. **Names and import paths stay.** Each module becomes a package
   (`tasks/grasp_sequence/`, `backends/pyhpp/`) whose `__init__` exports what the module
   exports today. `GraspSequencePlanner` and `PyHPPBackend` remain the classes callers
   build.
2. **Parts with little shared state become collaborators**: plain classes or functions
   that take what they need, held by the main class, which keeps one-line methods that
   delegate. In order:
   - `pyhpp`: path store and files (`save_path` … `_validate_graph_metadata`), reusing
     `planning/path_io.py`; time parameterization (`configure_time_parameterization` …
     `_apply_stp`); playback (`play_path_vector` … `play_and_record_path_vector`); the
     transition planner last (`configure_transition_planner` … `_plan_raw_path`).
   - `grasp_sequence`: persistence and replay (`_auto_save_phase_paths`,
     `load_saved_paths`, `_dump_phase_checkpoint`, `replay_sequence`,
     `get_phase_summary`); release planning (`_plan_release_*`,
     `_setup_release_phase_graph`); the feasibility search
     (`find_feasible_phase_target`); the phase loop last.
3. **Long methods are cut in place before they move.** `_plan_phase_edges` and
   `find_feasible_phase_target` are first split into methods of the same class, with no
   file move in that slice, as was done for `plan_transition_edge`.
4. **No mixins.** They would shorten the files and leave 27 and 40 shared attributes
   reachable from everywhere.
5. **One subject per slice, each with the same proof**: no behaviour change, tests for
   the moved part written before the move, the container suite, V2, and V3 against the
   current baseline. A test that patches a private method moves to the collaborator's
   public method in the same slice. The module leaves mypy's unchecked list when its last
   slice lands.
6. **Exceptions.**
   - A catch-all is allowed in three places: around a binding call whose signature
     differs between HPP builds (narrowed to `AttributeError`/`TypeError` where that is
     what is probed); at a boundary that re-raises a `long_tamp` error with `from exc`;
     and around diagnostics or cleanup that must not fail the caller. Each logs the
     exception and has a `noqa: BLE001 - reason`; none is a bare `pass`.
   - A search that treats failure as "no" (`_paths_exist`, target generation) catches
     planning errors only.
   - `long_tamp.errors` gets `PlanningError(RuntimeError)` with `ProjectionError`,
     `PathPlanningError` and `GraphError`. New code raises these; an existing
     `raise RuntimeError` is converted when its slice is touched. Callers that catch
     `RuntimeError` keep working.
   - `BLE001` is switched on in ruff for a package when its last slice lands.
7. **Logging.** Library code logs; `print` stays in the CLI and interactive modules
   (`interactive_grasp_sequence_builder`, the scripts). Ruff's `T20` is switched on with
   those files exempt. A message a user must see is a `warning`; per-edge detail is
   `debug`.

## Consequences

- About ten slices, so about ten batch gates. Collaborator slices are small and can run
  one a night; the phase loop is the risky one and goes last, when the rest is out of
  its way.
- Until the last slice, each main class is partly delegating and partly not.
- Tests stop depending on private names slice by slice; coverage is raised on a part
  before it moves, not after.
- `RuntimeError` and the new errors coexist for a while. Nothing that catches
  `RuntimeError` today breaks, and nothing can rely on the new types until a package is
  done.
- Not decided here: a second backend. The `BackendBase` protocol is unchanged, and the
  collaborators are HPP-specific.
