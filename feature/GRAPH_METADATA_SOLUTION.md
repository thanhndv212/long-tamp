# Graph Metadata Solution for Path Loading

## Problem
Paths planned with constraint graph edges cannot be loaded in a new session without recreating the exact same graph structure. The previous solution (Solution 1) required manual discipline to ensure the graph was set up identically.

## Solution 2: Store Graph Metadata with Paths

This solution automatically stores graph structure metadata when saving paths and provides validation when loading.

### What's Stored

When you save a path using `save_path_as_waypoints()`, the JSON file now includes:

```json
{
  "format_version": "1.0",
  "path_length": 12.34,
  "num_samples": 100,
  "waypoints": [[...], [...], ...],
  "edge_name": "edge_12",
  "graph_metadata": {
    "format_version": "1.0",
    "robot_name": "spacelab",
    "states": ["free", "pregrasp_tool", "grasp_tool", ...],
    "edges": [
      {"name": "edge_01", "id": 1},
      {"name": "edge_12", "id": 2},
      ...
    ],
    "objects": ["screw_driver", "frame_gripper", ...]
  }
}
```

### How to Use

#### Option 1: Manual Graph Setup (Recommended)

```python
# Planning session
graph = setup_constraint_graph(backend, robot, objects)
path, _ = backend.plan_transition_edge(...)
backend.save_path_as_waypoints(path, "my_phase.json")

# Loading session (new process)
graph = setup_constraint_graph(backend, robot, objects)  # Same setup!
path_idx = backend.load_path_from_waypoints("my_phase.json")
backend.play_path(path_idx)
```

#### Option 2: With Automatic Validation

```python
# Loading session with validation
graph = setup_constraint_graph(backend, robot, objects)

# This will validate that your graph matches the saved metadata
path_idx = backend.load_path_from_waypoints(
    "my_phase.json",
    auto_setup_graph=True  # Enables validation
)
```

### What Gets Validated

When `auto_setup_graph=True`:

1. **Graph Existence**: Checks that a constraint graph is initialized
2. **Robot Name**: Warns if robot names don't match
3. **State Count**: Errors if number of states doesn't match
4. **Edge Count**: Warns if number of edges doesn't match (less critical)

### Error Messages

**No graph set up:**
```
RuntimeError: Constraint graph not initialized. The saved path requires 
a graph with 15 states. Set up the graph first using the same 
robot/objects/structure.
```

**Wrong graph structure:**
```
RuntimeError: Graph structure mismatch: Saved path has 15 states, 
but current graph has 10 states. Make sure you're using the same 
constraint graph structure.
```

**No metadata in file:**
```
RuntimeError: Cannot auto-setup graph: No graph metadata in my_phase.json. 
This file was saved without graph metadata. You must manually set up 
the constraint graph before loading.
```

### API Reference

#### `extract_graph_metadata() -> Dict[str, Any]`

Extracts current graph structure for serialization.

**Returns:**
- `robot_name`: Name of the robot
- `states`: List of state names in the graph
- `edges`: List of edge dictionaries with name and ID
- `objects`: List of object names

#### `save_path_as_waypoints(path_vector, filename, num_samples=100, edge_name=None)`

Saves path with automatic metadata extraction.

**Parameters:**
- `path_vector`: PathVector CORBA object
- `filename`: Output JSON file path
- `num_samples`: Number of waypoints to sample (default: 100)
- `edge_name`: Optional edge name for context

**Behavior:**
- Automatically extracts and includes graph metadata
- Warns if metadata extraction fails (but still saves waypoints)

#### `load_path_from_waypoints(filename, add_to_problem=True, auto_setup_graph=False) -> int`

Loads path with optional validation.

**Parameters:**
- `filename`: Input JSON file path
- `add_to_problem`: If True, adds path to ProblemSolver (default: True)
- `auto_setup_graph`: If True, validates graph metadata (default: False)

**Returns:**
- Path index in ProblemSolver

### Integration with Task Scripts

The `task_display_states.py` script automatically:

1. **Detects file format** (JSON vs native .path)
2. **Creates full graph** when loading JSON files
3. **Provides helpful messages** about graph requirements

```python
# In interactive mode
if use_json:
    print("\nJSON format requires constraint graph...")
    if not ensure_task_ready(task, cfg, freeze_joint_substrings, full_graph=True):
        print("Failed to create constraint graph.")
        return
```

### Benefits

1. **Self-Documenting**: Each path file contains its graph requirements
2. **Early Detection**: Validation catches graph mismatches before planning fails
3. **Helpful Errors**: Clear messages guide users to fix graph setup
4. **Optional**: Can still load without validation (backward compatible)
5. **Portable**: Files can be shared with graph structure documented

### Limitations

- Metadata extraction depends on graph implementation details
- Doesn't capture constraint details (only structure)
- Full graph reconstruction would require more metadata
- Validation is structural, not semantic (doesn't check constraint content)

### Future Enhancements

For even more robustness, could add:
- Constraint descriptions (grasp handles, placement constraints)
- Robot/object URDF checksums for exact matching
- Graph serialization format for complete reconstruction
- Automatic graph generation from metadata
