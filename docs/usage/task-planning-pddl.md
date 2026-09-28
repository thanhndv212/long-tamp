# Automatic task planning (PDDL)

The capabilities a mission is built from already declare their preconditions and effects.
`long_tamp.tasks.task_planning.pddl` exports them, with a world state and a goal, as PDDL,
so a classical planner can find the sequence of capability calls (the plan *skeleton*)
instead of writing it by hand. Design: [ADR-0001](../adr/0001-planner-refiner-executor.md).

```python
from long_tamp.tasks.task_planning.pddl import from_pddl_plan, to_pddl

export = to_pddl(
    descriptors,                        # {id: CapabilityDescriptor} or a list
    init=["holds(ur10_right/gripper, driver/h_grip)", *static_facts],
    goal=["screwed(part1, part1/h_hole1)", "not holds(ur10_left/gripper, _)"],
    static_preconditions={"grasp": ("can_grasp(?gripper, ?handle)",)},
)
export.domain, export.problem           # PDDL text
steps = from_pddl_plan(export, planner_actions)
# [("grasp", {"gripper": "ur10_left/gripper", "handle": "part1/h_grasp"}), ...]
```

## Translation

| long_tamp | PDDL |
|---|---|
| a capability with effects | an action (capabilities without effects, such as guard conditions, are not) |
| `holds(?g, ?h)` / `not holds(?g, ?h)` | `(holds ?g ?h)` / `(not (holds ?g ?h))` |
| precondition `not holds(?g, _)` | `(not (exists (?w0) (holds ?g ?w0)))` |
| effect `not holds(?g, _)` | `(forall (?w0) (not (holds ?g ?w0)))` |
| a parameter used in no literal (e.g. a block label) | not an action parameter; filled in when the skeleton becomes a TaskPlan |
| `ur10_left/gripper` | `ur10_left__gripper` (`export.names` maps every PDDL name back) |

**Bound the parameters.** Every action parameter ranges over every object, and a
parameter that appears only in effects isn't constrained at all. `static_preconditions`
adds, per capability, preconditions over static facts you put in `init` (which gripper can
grasp which handle, which holes belong to which part). They exist only in the export; the
capabilities' run-time preconditions are unchanged.

## Planners

All planners implement `TaskPlanner.solve(export) -> skeleton`; `default_planner()` picks
one.

| Planner | Runs | Use it |
|---|---|---|
| `FastDownwardPlanner` | Fast Downward on the exported PDDL as is | **the default**: fast (milliseconds here), reads quantifiers and negative/conditional features natively |
| `UnifiedPlanningPlanner("fast-downward")` | Unified Planning, then its Fast Downward engine | when you want Unified Planning's problem API or its other engines |
| `UnifiedPlanningPlanner("pyperplan")` | Unified Planning compiles the problem down to STRIPS, pyperplan solves it | last resort where no Fast Downward binary exists: pure Python, but slow, and the compilation can blow up on quantified problems (it warns when chosen) |

Getting Fast Downward: `pip install long-tamp[planning]` brings its binary through
`up-fast-downward` on Linux x86-64 and macOS; elsewhere build it from source (below).
Other planners (e.g. PDDLStream, roadmap M6) plug in behind the same interface.

## Planning it through Unified Planning

With the `planning` extra:

```python
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner

problem = PDDLReader().parse_problem("domain.pddl", "problem.pddl")
with OneshotPlanner(name="fast-downward") as planner:
    plan = planner.solve(problem).plan
actions = [(a.action.name, *map(str, a.actual_parameters)) for a in plan.actions]
steps = from_pddl_plan(export, actions)
```

## Screw assembly

`script/screw_assembly/screw_domain.py` exports its mission: `pddl_problem(n_parts,
state)` with `STATIC_PRECONDITIONS`, `static_facts(n)` and `mission_goal(n)`. Fast Downward
plans 1, 2 and 4 parts, and from partly done states (e.g. part 1 finished, driver in
hand) plans only the remaining work (`tests/test_task_planning_pddl.py`). Home moves have
no effects, so they aren't actions: a skeleton is the grasp / clamp and screw / release /
rack sequence.

## Replanning around failures

When a step can't be refined, its facts say why (see [the refiner](refiner.md)).
`long_tamp.tasks.task_planning.repair.plan_execute_repair(plan, execute, policy)` turns
them into *blocked bindings* and plans again from the current world state:

```python
from long_tamp.tasks.task_planning.repair import plan_execute_repair

def plan(blocked):          # blocked: [(capability, {parameter: value}), ...]
    export = to_pddl(descriptors, world_state(), goal, blocked=blocked)
    return skeleton_document(planner.solve(export), "Mission", expand)

def execute(document):      # None on success, else the failure
    ...                     # {"step", "capability", "parameters", "facts"}

outcome = plan_execute_repair(plan, execute, policy, max_rounds=3)
```

- `to_pddl(..., blocked=[("clamp_and_screw", {"clamp": "fixtures/clamp1", "seat":
  "part1/h_seat"})])` adds `(not (blocked_clamp_and_screw__clamp__seat ?clamp ?seat))`
  to the action and the blocked values to the problem, so a planner can't choose that
  pair again, whatever the other parameters.
- The policy maps a failure to bindings to block (the default blocks the failed step's
  own binding). The loop stops on success, after `max_rounds` plans, when the policy has
  nothing new to block, or when no plan avoids the blocks (`NoPlanFound`).
- Each round plans from the world state the previous one left, so completed steps are
  not redone.

Screw assembly: `task_screw_assembly.py --planner up --replan 3` runs this loop with
`screw_domain.repair_policy` (a clamp that can't reach a part's seat is blocked for that
part; anything else blocks the failed step), and `--inject-failure
clamp_and_screw:clamp=fixtures/clamp1` makes that step fail once, as if unreachable, to
test it.

## Running Fast Downward directly

`FastDownwardPlanner` runs [Fast Downward](https://github.com/aibasel/downward) on the
exported PDDL as it is: Fast Downward reads quantifiers, negative preconditions and
conditional effects, so nothing is compiled away (planning the 4-part screw mission takes
milliseconds). It finds `fast-downward.py` from its `executable` argument, then
`LONG_TAMP_FAST_DOWNWARD`, then `PATH`, then the copy bundled with `up-fast-downward`.
Where no wheel exists (Linux aarch64), build it from source:

```bash
git clone --depth 1 https://github.com/aibasel/downward.git && cd downward
./build.py -j2 release
export LONG_TAMP_FAST_DOWNWARD=$PWD/fast-downward.py
```

`default_planner()` returns a `FastDownwardPlanner` when an executable is found, else a
`UnifiedPlanningPlanner`; the screw assembly's `--planner up` uses it.

## From a goal to a TaskPlan

`long_tamp.tasks.task_planning.skeleton` closes the loop:

```python
from long_tamp.tasks.task_planning import TaskPlan
from long_tamp.tasks.task_planning.skeleton import UnifiedPlanningPlanner, skeleton_document

steps = UnifiedPlanningPlanner().solve(export)     # [(capability, parameters), ...]
document = skeleton_document(steps, mission_id="Demo", expand=expand)
plan = TaskPlan.from_dict(document, registry)       # the same validation as a hand-written plan
```

- `UnifiedPlanningPlanner(engine="auto")` uses Fast Downward if its package is installed,
  else pyperplan. When the engine can't read the problem's features (pyperplan: plain
  STRIPS), Unified Planning compiles them away first (quantifiers, disjunctions,
  conditional effects, negative conditions) and the plan is mapped back.
  `up-fast-downward` ships wheels for Linux x86-64 and macOS only; elsewhere (e.g. Linux
  aarch64) the `planning` extra installs pyperplan alone.
- `skeleton_document` makes one transaction per step. `expand(index, capability,
  parameters)` fills in implementation parameters and labels, and may insert steps the
  planner doesn't see (effect-less moves). `NoPlanFound` is raised when the goal can't be
  reached.

The screw assembly plans its own mission this way: `python3 task_screw_assembly.py
--planner up` plans the goal from the current world state (`screw_domain.expand_step`
adds the labels and the home moves), and every step runs the block `block_for(capability,
parameters)` rebuilds from it, so a plan the hand-written one never contained (another
part order, another clamp) runs the same way.
