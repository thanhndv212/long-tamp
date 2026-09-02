# Console/file log verbosity split

**Components**: `long_tamp.logging.setup.configure_logging`,
`long_tamp.tasks.base.ManipulationTask`, plus each task script's own CLI
**Status**: Implemented 2026-08-14.

---

## Summary

The console and the per-run log file now have **independent levels**. The console can be
quieted to `INFO` (or higher) for a long run while the log file keeps full `DEBUG` detail
for postmortem debugging. Previously a single `level` drove both, so quieting the console
also threw away the detail you needed after the fact.

---

## `configure_logging()`

```python
configure_logging(
    level=logging.INFO,      # default for both handlers
    log_dir=None,
    run_id=None,
    console=True,
    console_level=None,      # NEW — defaults to `level`
    file_level=None,         # NEW — defaults to logging.DEBUG
)
```

- `console_level` defaults to `level`, so **omitting both new arguments reproduces the
  previous console behavior exactly**.
- `file_level` defaults to `logging.DEBUG` regardless of `level`: the log file exists for
  postmortem debugging, so it stays fully detailed even when the console is quiet.
- The logger's own level is `min(console_level, file_level)` when a `log_dir` is set (the
  file handler would otherwise never see records the logger filtered out first), and
  `console_level` when there is no file.
- The existing idempotency guard (skip if handlers are already attached) is unchanged.

## `ManipulationTask(log_level=...)`

`ManipulationTask.__init__` gained `log_level: str = "INFO"` (`"DEBUG"` / `"INFO"` /
`"WARNING"` / `"ERROR"`), resolved via `getattr(logging, log_level.upper(), logging.INFO)`
and passed through as `console_level`. The log file under
`/tmp/long_tamp/<task_slug>_<YYYYMMDD_HHMMSS>/` always captures full `DEBUG`.

## `--log-level` flag

A task script exposes it on the CLI (see `script/templates/` for the pattern):

```
--log-level {DEBUG,INFO,WARNING,ERROR}   Console log verbosity (default: INFO).
```

`ScrewdrivingSequenceTask.__init__` forwards it to `ManipulationTask`.

## `INFO` → `DEBUG` demotions

Per-phase setup chatter dominated the console on long runs — every phase rebuilds a graph,
so these fire hundreds of times per run. Demoted to `DEBUG` (still in the log file):

| File | Messages |
| --- | --- |
| `planning/graph.py` | factory setup (grippers, objects, environment contacts, rules, filter), `generate()`, `initialize()`, node/edge counts, global-constraint addition, `build_phase_graph` entry, non-phase object locking |
| `planning/constraints.py` | per-joint "✓ Locked joint" lines |
| `backends/pyhpp.py` | time-parameterization success lines (TOPPRA, trapezoidal, STP, generic) |

Warnings and failures stay at `WARNING` throughout — nothing that reports a problem was
demoted.

Two related tightenings in `planning/constraints.py` and `planning/graph.py`:

- After the per-joint loop, a single summary line reports the count and the joint list, so
  `DEBUG` still gives one readable line per phase instead of only N scattered ones.
- The global-constraints log only interpolates `constraint_names` when they are actually
  strings; the PyHPP path passes constraint *objects*, whose `repr` made the line useless.

## Retry-loop log throttling

`run_block_nonstop`'s resume loop was the least-throttled retry path in
`screwdriving_sequence.py` — it logged every single attempt, and it ran to attempt
#1878 for RS5's `CON0` and #1927 for RS6's WB grasp before the FG-grasp split. It now uses
the same `should_log = attempt == 1 or attempt % 25 == 0` throttle as
`move_arm_to_target_nonstop`, and passes `verbose=should_log` down into
`resume_sequence()`.

## Tests

`tests/test_logging_setup.py` (pure Python, no HPP backend needed) covers:

- default `level` applying to both handlers (backward compatibility);
- `console_level` overriding the console without touching the file level;
- a quiet console still writing full detail to the file;
- an explicit `file_level` being respected.

An autouse fixture saves/clears/restores the `long_tamp` logger's handlers around
each test — `configure_logging()` is idempotent, so without it whichever test ran first
would pin the handler configuration for every test after it in the same process.
