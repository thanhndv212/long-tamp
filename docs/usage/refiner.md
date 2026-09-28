# The refiner

A task planner decides *what* to do: a plan skeleton of grasp sequences. The refiner
decides *how*: it binds each step to grasps, configurations and paths on HPP's
constraint graph, recovering from failures on the way. Design:
[ADR-0001](../adr/0001-planner-refiner-executor.md).

```python
from long_tamp.tasks.refiner import GraspSequenceRefiner, Lookahead, RefinementStep

refiner = GraspSequenceRefiner(planner, q_scene_init=q_init, max_replans=10)
step = RefinementStep(
    label="part1 A",
    sequence=(
        ("fixtures/clamp1", "part1/h_seat"),
        ("driver/tip", "part1/h_hole1"),
        ("driver/tip", None),            # release
        ("driver/tip", "part1/h_hole2"),
        ("driver/tip", None),
    ),
    frozen={0: ["ur10_right"], 1: ["ur10_left"], 2: ["ur10_left"],
            3: ["ur10_left"], 4: ["ur10_left"]},
    # Pick the clamp target (phase 0) so that hole 1 (phase 1) and
    # hole 2 (phase 3) stay reachable from it.
    lookahead=Lookahead(pair=(0, 1), also=(3,), verify_paths=True),
)
r = refiner.refine(step, q)
if r.success:
    q = r.final_config        # r.phases: the planned phases and their paths
else:
    print(r.message, r.facts)
```

`GraspSequenceRefiner` runs `run_block_with_recovery()`, which escalates from redrawing a
phase's targets, to resuming from the last completed phase, to replanning the whole step
from its entry configuration (see `long_tamp.tasks.block_recovery`). Extra keyword
arguments (`resume_limit`, `unreachable_resumes`, `unfreeze_after`, ...) are passed to it.

## Failure facts

A failed refinement returns ground atoms in the TaskPlan predicate language, so a task
planner can plan around the failure instead of retrying it:

| Fact | Meaning |
|---|---|
| `refinement_failed(<step>)` | always present on failure |
| `unreachable(<gripper>, <handle>)` | the phase can't be reached from the step's earlier commitments (solver-only failures, no collisions) |
| `phase_failed(<gripper>, <handle>)` | the phase kept failing up to the resume limit |
| `lookahead_failed(<gripper>, <handle>)` | the lookahead's hinted target for this phase was redrawn |

A release phase reports `none` as its handle.

The interface (`Refiner`, `RefinementStep`, `Refinement`) is kept small and will be
frozen only once a second implementation exists (ADR-0001).
