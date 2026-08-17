# Feature notes

Per-feature notes for changes that need more context than a commit message carries: what
failure motivated them, what was measured, and what the deliberate limitations are.
Companion to `docs/bugs/` (upstream HPP defect reports), `docs/plans/` (refactor plans), and
`docs/usage/` (step-by-step how-to-use guides).

## 2026-08-13/14 — SpaceLab screwdriving sequence

| Doc | What it covers |
| --- | --- |
| [phase-target-lookahead.md](phase-target-lookahead.md) | `GraspSequencePlanner.find_feasible_phase_target()` and the `phase_q_hints` warm-start chain — commit a randomized grasp target only after verifying the *next* phase stays reachable from it. Fixes RS6's `CON0` failing 2300+ consecutive target draws and RS5's failing 878/878. |
| [screwdriving-sequence-assembly-order.md](screwdriving-sequence-assembly-order.md) | `SCREW_PLAN_RS`, a hand-written per-turn table of (part, holes) — RS6/RS5 drive their own `CON0`+`CON1`, RS2/RS3/RS4 their own `CON2`+`CON3`, RS1 nothing — and the A-block split into `A0` (FG grasp) + `A-REST` (WB + screws) so the lookahead probes from the part's real in-place pose. |
| [transit-edge-robustness.md](transit-edge-robustness.md) | Two arm-transit fixes: a deterministic dodge for `SplineGradientBased_bezier3`'s "more than 2 IPs" throw, and the missing `set_phase_indices()` sync that made transits fail their initial-configuration check on every retry. |
| [logging-verbosity.md](logging-verbosity.md) | Independent console/file log levels (`console_level` / `file_level`, `ManipulationTask(log_level=...)`, `--log-level`), per-phase `INFO`→`DEBUG` demotions, and retry-loop log throttling. |

### Tests added alongside

| File | Needs HPP? |
| --- | --- |
| `tests/test_lookahead_phase_target.py` (+ `tests/fixtures/rs6_phase01_wb_grasp_checkpoint.json`) | Yes — real solver against real SRDF constraints; seconds, not milliseconds |
| `tests/test_grasp_sequence_phase_q_hints.py` | Import-only (fakes for the solver) |
| `tests/test_grasp_state_copy.py` | Import-only (pure-Python tracker) |
| `tests/test_logging_setup.py` | No |

Everything that imports `agimus_spacelab` needs `pyhpp`, so run the suite inside the
`hpp-arm64` container.
