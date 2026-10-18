# 0012. The generic mission runner

- Status: Accepted
- Date: 2026-10-18
- Issues/PRs: #132 (this), #131 / ADR-0011 (the domain interface), #133 (chat/UI), #134
  (screw port)

## Context

ADR-0011 wrote down the boundary between the mission loop and a domain. The loop itself
is still in `script/screw_assembly/task_screw_assembly.py`: `run_mission`,
`run_with_repair`, `run_supervised`, `plan_from_goal` and `understand_instruction`, with
`screw_domain` functions and screw facts called by name. A second domain (#159, the
hardware showcase) needs the loop as a library, parameterised by a domain, a refiner and
an execution backend.

Three of the script's rules were not in the protocol, as ADR-0011 noted: which world
sources a domain reads (`holds` from the grasp tracker, recorded facts), the
configuration to replan from after a drift or a missed grasp, and how to open a hand the
backend can't open. The execution summary also has screw-only keys (`screws_driven`,
`max_screw_lateral_error`).

## Decision

1. **`long_tamp.mission.MissionRunner(domain, scene, recorded, refiner)`** is the loop.
   The code is the script's, with each `screw_domain` call replaced by the domain method
   ADR-0011 named:
   - `run_mission(q_start, document, backend, ...)`: one TaskPlan, steps refined (a
     `RefinementStep` by the refiner, a `Move` by `domain.plan_move`) and executed through
     `PlanExecutor`, with plan-ahead, concurrency, the drift check, injected failures and
     `control`. The result dict is the script's.
   - `run_with_repair` (`plan_execute_repair`) and `run_supervised` (`supervise`), from a
     goal or `domain.default_goal()`;
   - `plan_from_goal` (`domain.pddl_problem` → `skeleton_document`) and
     `understand_instruction` (the LLM goal translator on `domain.goal_vocabulary`).

   The refiner defaults to a `GraspSequenceRefiner` on the scene's planner; tests pass a
   fake one and run without HPP.
2. **Four new hooks in `MissionDomain`**, with defaults in `BaseMissionDomain`:
   - `world_state(scene, recorded)`: the world the guards and the planner read. The
     default reads `holds` from the grasp tracker and `grasped` from `ObjectLevelState`,
     each only if a descriptor uses it, plus the recorded facts.
   - `observed_start(scene, planned, observed)`: the configuration to replan from after a
     drift or a missed grasp. Default: the observed one. The screw domain keeps a hand
     that carries something, and what it carries, as planned (#128).
   - `opened_config(scene, q, grippers)`: `q` with those hands open, for a missed grasp on
     a backend without `open_gripper`. Default: `q`. The screw domain resets the Robotiq
     joints to the start.
   - `execution_metrics(executions)`: the domain's keys in the execution summary. The
     screw domain gives `screws_driven` and `max_screw_lateral_error`.
3. **The run folder is unchanged**: `mission.json`, `checkpoint.json` and the per-phase
   dumps from `MissionCheckpoint`, and `trajectory.json` with `dt` and `segments`. The
   trajectory header is now `domain.scene_info()`, so the screw run also writes
   `"id": "screw-assembly"` next to `parts`; `replay.py` reads only `dt` and `segments`.
4. **The script runs on the runner** (#134). `task_screw_assembly.py` keeps its CLI and a
   `run_mission(task, planner, n_parts, **mission)` wrapper for the chat tools; the rest of
   its loop is deleted. The runner's state, `MissionRun`, is public: `refine(run, index,
   step)` and `session(run, document)` take one, for a scenario's setup steps and the BT
   host.

## Consequences

- A second domain gets the whole loop (repair, supervisor, drift, missed-grasp retry) by
  implementing ADR-0011's methods. The tests run the toy pick/place domain and the screw
  assembly on the runner with a fake planner and the `MockBackend`.
- The loop exists once. Until #134 it existed twice; the screw tests that checked the
  runner against the script now run the script on the runner.
- The runner still reads `scene.planner.grasp_tracker` and
  `reset_grasp_tracker_to_call_start`, so a refiner other than the grasp-sequence one
  must keep a tracker of that shape. That and `held` (gripper → handle) are what remain
  screw-shaped; #159 will tell.
