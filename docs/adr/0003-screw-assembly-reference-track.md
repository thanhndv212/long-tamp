# 0003. Screw assembly is the reference mission for validation

- Status: Accepted
- Date: 2026-09-27

## Context

Architecture work (ADR-0001, ADR-0002) touches planning, execution and recovery at
once. Without one fixed, realistic mission to measure against, regressions show up late
and "it works" is argued rather than measured. `script/screw_assembly/` is long-horizon
(19 blocks, 31 grasp/release phases at 4 parts), multi-arm, built from generic
primitives, and already reliable: 10/10 missions over 10 seeds, 0/140 blocks
replanned, every failure recovered.

## Decision

- Every roadmap milestone is delivered and proven on screw assembly first; other
  examples (TWIN) are secondary checks.
- Validation is tiered (V0–V4, see `docs/development/validation.md`). Behavioural
  changes pass a **batch gate**: 10 seeds × 4 parts, all missions complete, replanning
  trigger rate < 2 %, recovery rate > 95 %, median time ≤ 1.25× the baseline, checked
  by `summarize.py --gate`.
- Results are committed as files under `script/screw_assembly/results/`, and baselines
  are per environment (the canonical one is the PyPI-wheel environment).

## Consequences

- A full batch takes hours, so V3 is required only for planner-behaviour changes,
  milestone closure and releases; CI runs a one-part smoke mission nightly.
- The example's code becomes load-bearing: changes to it follow the same review bar as
  library code.
- Features the example doesn't exercise need either a scenario added to it or a second
  reference mission; the example stays generic (no proprietary content).
