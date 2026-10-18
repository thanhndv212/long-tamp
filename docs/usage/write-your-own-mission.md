# Write your own mission

A *mission domain* is what `long_tamp` needs to know about a robot and a task to run
missions on it: the operator types an instruction, a task planner turns it into a plan,
the runner refines each step into motions and executes them, and a failed step is
repaired by replanning. The loop (the runner, the chat, the mission UI) is the
library's; the domain says what the actions are and what they mean geometrically.

This page builds one from the **template domain**,
[`long_tamp.mission.template`](https://github.com/thanhndv212/long-tamp/blob/main/src/long_tamp/mission/template.py):
one arm moves a cube and a ball between a table, a tray and a bin. It needs no HPP (its
scene is kinematic), so it runs anywhere `long_tamp` is installed, and its tests run in
CI. The screw assembly
([`script/screw_assembly/screw_mission.py`](https://github.com/thanhndv212/long-tamp/blob/main/script/screw_assembly/screw_mission.py))
is the same interface on a real HPP scene.

## Run it

The task planner comes with the `planning` extra:

```bash
pip install "long-tamp[planning]"
python -m long_tamp.mission ui --domain long_tamp.mission.template:domain
```

Open the printed address: the chat and the live plan monitor (the template has no 3D
scene). Type "put the ball in the bin", then "run it". The model needs a gateway, see
[AI models](ai-models.md); without one, `serve --no-web` runs the same mission with the
chat on the terminal.

`--domain` takes `MODULE:FACTORY` or `FILE.py:FACTORY`: any function that returns a
domain. `long_tamp.mission.template:repair_demo` is the template with the tray out of
reach. Ask it to "take the cube off the table": a plan that places it in the tray
fails, the repair loop blocks that binding, and the replan uses the bin.

## The two halves of a domain

A domain subclasses `BaseMissionDomain`, which has a default for every method of the
`MissionDomain` protocol
([ADR-0011](../adr/0011-mission-domain.md)). You override what your task
needs, in two halves.

### Symbolic: what the planner and the chat see

**Capabilities**: the actions, with typed parameters, preconditions and effects over
predicates. The template uses the shared object-level predicates `grasped(gripper,
object)` and `placed_at(object, region)`, so its goals read like any other
pick-and-place domain's:

```python
def descriptors(self):
    pick = CapabilityDescriptor(
        "pick", "1.0", {"gripper": str, "object": str, "region": str},
        preconditions=("not grasped(?gripper, _)", "placed_at(?object, ?region)"),
        effects=("grasped(?gripper, ?object)", "not placed_at(?object, ?region)"),
        writes=("grasp_state",), restartable=True,
    )
    place = ...  # grasped -> not grasped, placed_at
    return {d.capability_id: d for d in (pick, place)}
```

Then the vocabulary the planner grounds over:

| Method | Template |
| --- | --- |
| `objects()`, `regions()` | `cube`, `ball`; `table`, `tray`, `bin` |
| `static_facts()` | `gripper(arm/gripper)`, `region(...)`: facts that never change |
| `static_preconditions()` | which parameters bind to which static facts (`place` needs a `region(?region)`) |
| `default_goal()` | both objects in the tray, nothing held |
| `recorded_predicates` | `{"placed_at"}`: facts the runner records from executed effects (`grasped` comes from the grasp tracker) |
| `intro`, `goal_notes` | one or two sentences each, given to the chat model |

That is enough to plan: `MissionRunner.plan_from_goal(goal=[...])` returns a TaskPlan
document, and the chat's `set_goal` / `plan` tools work.

### Geometric: what the steps are

**`step_for(capability, parameters)`** turns a planned step into a `RefinementStep`: a
label and a grasp sequence, `(gripper, handle)` to grasp, `(gripper, None)` to release.
The template's pick grasps `cube/handle`; its place releases:

```python
def step_for(self, capability, parameters):
    obj, region = parameters["object"], parameters["region"]
    if capability == "pick":
        return RefinementStep(f"pick {obj} {region}", ((GRIPPER, f"{obj}/handle"),))
    return RefinementStep(f"place {obj} {region}", ((GRIPPER, None),))
```

**`setup()`** builds the scene and returns a `MissionScene(task, planner, q_init,
extras)`. The runner reads `planner.grasp_tracker.current_grasps` and calls
`planner.reset_grasp_tracker_to_call_start()` after a failed refinement. Two extras
matter:

- `extras["refiner"]`: the refiner, anything with `refine(step, q_init) ->
  Refinement`. With HPP that is the `Refiner` over a `GraspSequencePlanner`
  ([Refiner](refiner.md)); the template's `KinematicScene` moves the tool point in a
  straight line.
- `extras["facts"]`: the recorded facts at the start (`placed_at(cube, table)`, ...).

A `Refinement` carries the phases to execute (each a dict of `gripper`, `handle`,
`paths` and `edges`; a path has `length()` and `eval(t) -> (q, ok)`) or, when it fails,
`facts` such as `cannot_reach(arm/gripper, tray)`. The default `repair_policy` blocks the
failed step's binding, and `run_with_repair` replans around it; override
`repair_policy` to block more (the screw assembly, told a clamp can't reach a
part's seat, blocks that clamp-seat pair for every step).

## Execute

`make_backend(name, scene, **options)` returns the execution backend. The default knows
`mock` (a `MockBackend` that plays the paths back) and `none`; return your simulator's
or your robot's backend for other names (see [Execution backends](execution.md) and
[Run on your robot](run-on-your-robot.md)). `--backend NAME` and `--backend-option
K=V` on the launcher reach it.

Optional hooks:

| Hook | For |
| --- | --- |
| `scene_viewer(scene, port)` | a live 3D scene in the mission UI (the screw assembly's Viser viewer) |
| `plan_move`, `scenarios` | robot moves outside the plan (e.g. going home), named scenarios |
| `world_state`, `observed_start` | where the world state and the start configuration come from (perception, the robot) |
| `execution_payload`, `execution_metrics` | what each executed path sends to the backend, and the metrics per run |

## From the template to your robot

1. Copy `template.py` into your project and rename the domain.
2. Write your capabilities and their vocabulary; check the plans with
   `MissionRunner.plan_from_goal` before any geometry.
3. Replace `KinematicScene` with your HPP scene: a task built from a YAML config
   ([`script/templates/`](https://github.com/thanhndv212/long-tamp/tree/main/script/templates)),
   its `GraspSequencePlanner`, and a `Refiner` over it, returned from `setup()`.
4. Return a backend from `make_backend` for your simulator or robot.
5. Run it: `python -m long_tamp.mission ui --domain my_mission.py:domain`.

[`tests/test_mission_template.py`](https://github.com/thanhndv212/long-tamp/blob/main/tests/test_mission_template.py)
is the template's loop as tests: the default goal planned and run on the mock backend,
a failed place repaired, and a chat mission through the launcher. Copy them with the
domain.
