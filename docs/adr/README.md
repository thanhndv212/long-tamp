# Architecture decision records

Short records of design decisions that shape `long_tamp`: the context, the decision,
and its consequences. They explain *why* the code is the way it is, so a later change
can revisit a decision knowingly instead of by accident.

Write an ADR before (or with the first PR of) any change that adds a public interface,
a module boundary, a dependency, or changes the plan IR schema.

- File: `NNNN-short-title.md`, numbered in order, never renumbered.
- Status: `Proposed` → `Accepted` → possibly `Superseded by NNNN` or `Deprecated`.
  Don't rewrite an accepted ADR's decision; write a new one that supersedes it.
- Keep it to a page. Link issues, PRs and docs instead of repeating them.

## Template

```markdown
# NNNN. Title

- Status: Proposed | Accepted | Superseded by NNNN | Deprecated
- Date: YYYY-MM-DD
- Issues/PRs: #…

## Context
What problem, what forces, what we know (with evidence).

## Decision
What we will do.

## Consequences
What becomes easier, harder, or required. What we give up.
```

## Index

| ADR | Title | Status |
|---|---|---|
| [0001](0001-planner-refiner-executor.md) | Split task planner, refiner and executor | Accepted |
| [0002](0002-effects-as-runtime-guards.md) | Capability effects are checked at plan time and at run time | Accepted |
| [0003](0003-screw-assembly-reference-track.md) | Screw assembly is the reference mission for validation | Accepted |
| [0004](0004-execution-contract.md) | The execution contract: polled backends, heartbeats, duration-scaled deadlines | Accepted |
| [0005](0005-partial-order-plans.md) | Partial-order plans and concurrent arms | Accepted |
