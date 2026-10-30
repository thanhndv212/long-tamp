# 0011. Mission domains: one protocol for any robot and task

- Status: Accepted
- Date: 2026-10-17
- Issues/PRs: #131 (this), #132 (generic runner), #133 (chat/UI), #134 (screw port), #159
  (RoboCerebra)

## Context

The mission loop is planner → refiner → executor, with repair and a supervisor around it
(ADR-0001, -0006, -0010). It exists once, in `script/screw_assembly/task_screw_assembly.py`
(about 2000 lines). The library parts it uses are generic: `to_pddl`, `skeleton_document`,
`plan_execute_repair`, `supervise`, `GraspSequenceRefiner` and the execution backends. The
glue between them is not. `run_mission`, `run_with_repair`, `run_supervised`, `run_chat` and
`plan_from_goal` call `screw_domain` functions by name: `descriptors(n_parts)`,
`pddl_problem`, `planned_document`, `block_for`, `repair_policy`, `goal_vocabulary`. They
also have screw facts written into them: the home moves, the screw skill on edge `_12`, and
the `part_done` guard.

So a new robot or task has to copy the script. Two new ones are planned: the RoboCerebra
goal translator (#159, a single arm moving objects between regions) and the hardware
showcase (M7–M10). The loop can only move into the library (#132) once its boundary with
the domain is written down.

## Decision

1. **`long_tamp.mission.MissionDomain`**: a `typing.Protocol` (runtime-checkable), in the
   same style as `ExecutionBackend`, `TaskPlanner` and `Refiner`. It is everything the loop
   asks a domain:
   - **symbolic**: `descriptors()`, `guards()`, `objects()`, `regions()`,
     `static_facts()`, `static_preconditions()`, `recorded_predicates`, `default_goal()`,
     `pddl_problem(state, blocked, goal)`, `goal_vocabulary(state)`, `goal_notes`, and
     `plan_document()` (a hand-written plan, or None);
   - **skeleton → plan**: `expand_step(index, cap, params, held)` (steps the domain inserts,
     such as a move home after a pickup), `planned_document(steps, state)`, and
     `step_for(cap, params)`, which returns a `RefinementStep` (a grasp sequence) or a
     `Move` (a plain motion);
   - **repair**: `repair_policy(failure)` and `retry(failure)`;
   - **geometry and execution**: `scenarios()`, `setup(seed, log_dir) -> MissionScene`,
     `plan_move(scene, move, q)`, `execution_payload(phase, edge, path)` (the path, or a
     `SkillCommand` around it), and `make_backend(name, scene, **options)`.

   Everything is a method of the domain object, not a module-level function with
   `n_parts`. Parameters such as the part count and the clamp/seat pairs belong to the
   instance.
2. **`BaseMissionDomain`** provides the generic answers:
   - the PDDL export through `to_pddl`, the vocabulary through `Vocabulary.from_domain`, and
     the planned document through `skeleton_document`, which tracks what each gripper holds;
   - guards are the capabilities that change nothing (no effects, no `writes`);
   - the repair policy blocks the failed binding without `block`, and `retry` is
     `retry_missed_grasp`;
   - the backends `mock` and `none`.

   A domain writes only `descriptors`, `step_for`, `setup` and `plan_move`, plus whatever it
   does differently. `check_domain(domain)` checks the parts that need no geometry: guards,
   static preconditions, recorded predicates, the default goal, and the hand-written plan.
3. **Shared object-level goals** (`long_tamp.mission.goals`):
   - `grasped(gripper, object)` and `placed_at(object, region)`, so a goal written outside
     a domain can name objects and regions instead of handles. A benchmark's `pick` becomes
     `grasped`, and `in`/`on` becomes `placed_at`;
   - `ObjectLevelState` derives `grasped` from the tracker's `holds(gripper, handle)`;
   - `check_object_goal` rejects unknown objects and regions.

   Domains are not required to use these predicates. The screw assembly keeps `holds` and
   `screwed`.
4. **The screw assembly is the reference implementation**:
   - `script/screw_assembly/screw_mission.py` defines `ScrewAssemblyDomain(n_parts, clamps)`
     and delegates to `screw_domain` and `task_screw_assembly`. Tests check that it gives
     the same answers as the functions it wraps, and mypy checks it against the protocol;
   - `task_screw_assembly.py` is unchanged. It moves onto the generic runner in #134.

## Consequences

- #132 can move the loop into `long_tamp.mission.runner` against a written interface. A
  second domain needs no copy of the script.
- The screw assembly behaves exactly as before. Validation is at level V1 (CI): parity tests,
  a toy pick/place domain planned to a `placed_at` goal, and a mypy check of both
  implementations.
- **The protocol is provisional** until a second real domain implements it (#159). Some
  pieces are still screw-shaped:
  - `held` is gripper → handle;
  - `Move.spec` is opaque;
  - the runner's recovery rules (planning from the observed start, missed-grasp recovery)
    are not part of the protocol yet. #132 may turn them into hooks.

  Expect changes then; this ADR is superseded rather than edited.
- Amended by #133 (additive, no change to existing members): `scene_viewer(scene, port)`
  returns the domain's live 3D scene, or None, for the generic mission UI
  (`python -m long_tamp.mission ui`). The scene's `extras["refiner"]` and `extras["facts"]`
  are the runner's default refiner and the recorded facts true at the start.
- Amended by #186 (no change to the protocol): the second real domain is
  `RoboCerebraDomain` (`long_tamp.eval.robocerebra_mission`), one RoboCerebra case run as
  one mission in the benchmark's env. It needed no new member. What it settled:
  - `held` stays as it is. A domain without a grasp tracker overrides `world_state` and
    reports object-level `grasped` from what the env observes (finger contacts, progress);
  - facts that never change at run time (the case's goal chain, `next`, `rest`) go in
    `static_preconditions`. The run-time preconditions name only what `world_state`
    reports, since the session checks those against the world;
  - `Move.spec` stays opaque. It carries the capability and its binding, and
    `plan_move` refines it (here through the row C policy). The path it returns is an env
    controller with no duration, so the executor bounds it by heartbeats (ADR-0004);
  - `retry(failure)` means "replan from the observed world". The domain says no only
    when replanning can't help (the budget is spent, a goal is unsupported).

  The protocol is no longer provisional for single-arm pick/place. The hardware showcase
  (M7–M10) may still change it.
- The protocol has many members (about 25). That is the price of making the boundary
  explicit. Most of them come from `BaseMissionDomain`.
