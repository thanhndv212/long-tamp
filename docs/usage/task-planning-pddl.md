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

## Planning it

With the `planning` extra (`pip install long-tamp[planning]`: Unified Planning and its
Fast Downward engine):

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
rack sequence. Building a TaskPlan from it and planning through Unified Planning in code
is [#14](https://github.com/thanhndv212/long-tamp/issues/14).
