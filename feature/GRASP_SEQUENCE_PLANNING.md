# Multi-Grasp Sequential Planning Implementation

## Overview

This implementation adds **incremental graph building** and **smart edge selection** for multi-grasp sequential tasks in HPP manipulation planning. It addresses the combinatorial explosion of constraint graphs when multiple grasps must be achieved in sequence.

## Problem Solved

**Before**: Creating a constraint graph with N grasps requires O(N!) states/edges, making planning infeasible for >3-4 grasps.

**After**: Phase-based regeneration creates O(N) graphs with O(grippers) states each, enabling planning for many sequential grasps.

## Components

### 1. GraspStateTracker (`planning/grasp_state.py`)
Tracks current grasp state and computes HPP edge names following factory conventions:
- **Grasp edge**: `"{gripper} > {handle} | {abbrev_state}"`
- **Release edge**: `"{gripper} < {handle} | {abbrev_state}"`
- **Loop edge**: `"Loop | {abbrev_state}"`

**Key methods**:
- `get_grasp_edge(gripper, handle)` - Compute grasp edge name
- `update_grasp(gripper, handle)` - Update state after successful plan
- `get_current_state_name()` - Human-readable state

### 2. PhaseGraphBuilder (`planning/graph.py`)
Added `build_phase_graph()` method to GraphBuilder:
- Uses `factory.setPossibleGrasps()` to restrict graph to:
  - Already-held grasps (must remain valid)
  - Single next grasp to achieve
- Dramatically reduces graph size from O(grippers! × handles!) to O(grippers)

**Signature**:
```python
def build_phase_graph(
    config: BaseTaskConfig,
    held_grasps: Dict[str, str],      # {gripper: handle} currently held
    next_grasp: Tuple[str, str],      # (gripper, handle) to grasp next
    ...
) -> ConstraintGraph
```

### 3. GraspSequencePlanner (`tasks/grasp_sequence.py`)
Orchestrates multi-phase planning:

**For each grasp in sequence**:
1. Build minimal phase graph via `build_phase_graph()`
2. Compute edge name via `GraspStateTracker.get_grasp_edge()`
3. Generate waypoint via `ConfigGenerator.generate_via_edge()`
4. Plan transition via `planner.plan_transition_edge()`
5. Update state tracker and move to next phase

**Key methods**:
- `plan_sequence(grasp_sequence, q_init)` - Plan full sequence
- `replay_sequence()` - Replay all phase paths
- `get_phase_summary()` - Print human-readable summary

### 4. UI Integration (`script/spacelab/task_display_states.py`)

**CLI**:
```bash
# Plan sequence from command line
python task_display_states.py --grasp-sequence "gripper1:handle1,gripper2:handle2"
```

**Interactive mode**:
```bash
# Use arrow-key menu to select grasps
python task_display_states.py -i
# Then select: "Plan grasp sequence (incremental)"
```

## Usage Examples

### CLI Example
```bash
cd hpp/src/agimus_spacelab/script/spacelab

# Plan sequence: grasp handle1, then handle2
python task_display_states.py --backend pyhpp \
    --grasp-sequence "spacelab/g_ur10_tool:frame_gripper/h_FG_tool,spacelab/g_vispa_left:panel/h_panel"
```

### Interactive Example
```bash
# Start interactive mode
python task_display_states.py -i

# In menu:
# 1. Select pairs to include in graph (optional)
# 2. Choose "Plan grasp sequence (incremental)"
# 3. Select grasps in order via arrow keys
# 4. Watch incremental planning progress
# 5. Optionally replay sequence
```

### Programmatic Example
```python
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner

# Initialize planner
seq_planner = GraspSequencePlanner(
    graph_builder=task.graph_builder,
    config_gen=task.config_gen,
    planner=task.planner,
    task_config=task.task_config,
    backend="pyhpp",
)

# Plan sequence
result = seq_planner.plan_sequence(
    grasp_sequence=[
        ("spacelab/g_ur10_tool", "frame_gripper/h_FG_tool"),
        ("spacelab/g_vispa_left", "panel/h_panel"),
        ("spacelab/g_vispa_right", "bracket/h_bracket"),
    ],
    q_init=q_init,
    verbose=True,
)

if result["success"]:
    print(seq_planner.get_phase_summary())
    seq_planner.replay_sequence()
```

## Architecture Benefits

### Scalability
- **Traditional approach**: O(grippers! × handles!) states
  - 3 grasps: ~10-20 states
  - 5 grasps: 100s of states
  - 7+ grasps: **infeasible**

- **Phase-based approach**: O(num_phases × grippers) states
  - Each phase: ~3-5 states
  - Scales linearly with sequence length
  - 10+ grasps: **feasible**

### Edge Selection
- **Automatic edge naming** from current/desired grasp state
- **No manual edge specification** needed
- **Factory naming convention** handled transparently

### Incremental Planning
- **Graph regeneration** per phase keeps planning fast
- **TransitionPlanner** provides edge-scoped collision checking
- **Waypoint generation** via edges ensures consistency

## Technical Details

### Edge Naming Convention
States are abbreviated as gripper-handle index pairs:
- Free state: `"f"`
- One grasp: `"0-1"` (gripper[0] holds handle[1])
- Two grasps: `"0-1:2-3"` (gripper[0]→handle[1], gripper[2]→handle[3])

Edges encode transition and source state:
- Grasp: `"gripper > handle | f"` (from free state)
- Release: `"gripper < handle | 0-1"` (to state after release)
- Loop: `"Loop | 0-1:2-3"` (move within same grasp state)

### Graph Regeneration Strategy
For sequence `[(g1,h1), (g2,h2), (g3,h3)]`:

**Phase 1**: Build graph with `{g1: [h1]}`
- States: free, g1-grasps-h1
- Plan edge: `"g1 > h1 | f"`

**Phase 2**: Build graph with `{g1: [h1], g2: [h2]}`
- States: g1-grasps-h1, g1-grasps-h1:g2-grasps-h2
- Plan edge: `"g2 > h2 | 0-1"`

**Phase 3**: Build graph with `{g1: [h1], g2: [h2], g3: [h3]}`
- Plan edge: `"g3 > h3 | 0-1:2-3"`

Each phase graph is independent and minimal.

## Future Enhancements

1. **Release transitions**: Add `plan_release_sequence()` for grasp/release patterns
2. **Loop transitions**: Support repositioning within same grasp state
3. **Graph caching**: Cache phase graphs keyed by `frozenset(held_grasps)`
4. **Parallel planning**: Plan multiple phases in parallel when independent
5. **Cost optimization**: Choose grasp order to minimize path length
6. **Collision validation**: Pre-validate sequence feasibility before planning

## Files Modified/Created

**New files**:
- `hpp/src/agimus_spacelab/src/agimus_spacelab/planning/grasp_state.py`
- `hpp/src/agimus_spacelab/src/agimus_spacelab/tasks/grasp_sequence.py`

**Modified files**:
- `hpp/src/agimus_spacelab/src/agimus_spacelab/planning/graph.py` (added `build_phase_graph`)
- `hpp/src/agimus_spacelab/script/spacelab/task_display_states.py` (added CLI + interactive)

## Testing

Recommended test cases:
1. **Single grasp**: Verify phase graph equals traditional graph
2. **Sequential grasps**: 2-3 grasps with no overlap
3. **Maintained grasps**: Hold h1, grasp h2, hold both
4. **Error handling**: Invalid gripper/handle names, collision failures
5. **Replay**: Verify all phase paths play correctly

## Documentation

See inline docstrings in each module for detailed API documentation.
