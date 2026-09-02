# Feature notes

Per-feature notes for changes that need more context than a commit message carries: what
failure motivated them, what was measured, and what the deliberate limitations are.
Companion to `docs/bugs/` (upstream HPP defect reports), `docs/plans/` (refactor plans), and
`docs/usage/` (step-by-step how-to-use guides).

## 2026-08-13/14 — long-horizon multi-phase sequencing fixes

| Doc | What it covers |
| --- | --- |
| [phase-target-lookahead.md](phase-target-lookahead.md) | `GraspSequencePlanner.find_feasible_phase_target()` and the `phase_q_hints` warm-start chain — commit a randomized grasp target only after verifying the *next* phase stays reachable from it. Fixes a real case of a phase failing 2300+ consecutive target draws (and another failing 878/878) because an earlier phase's random commitment made it unreachable. |
| [transit-edge-robustness.md](transit-edge-robustness.md) | Two arm-transit fixes: a deterministic dodge for `SplineGradientBased_bezier3`'s "more than 2 IPs" throw, and the missing `set_phase_indices()` sync that made transits fail their initial-configuration check on every retry. |
| [logging-verbosity.md](logging-verbosity.md) | Independent console/file log levels (`console_level` / `file_level`, `ManipulationTask(log_level=...)`, `--log-level`), per-phase `INFO`→`DEBUG` demotions, and retry-loop log throttling. |

### Tests added alongside

| File | Needs HPP? |
| --- | --- |
| `tests/test_grasp_sequence_phase_q_hints.py` | Import-only (fakes for the solver) |
| `tests/test_grasp_state_copy.py` | Import-only (pure-Python tracker) |
| `tests/test_logging_setup.py` | No |

Everything that imports `long_tamp` needs `pyhpp`, so run the suite inside the
`hpp-arm64` container.
