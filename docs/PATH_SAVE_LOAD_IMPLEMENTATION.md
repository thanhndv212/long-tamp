# Path Save/Load Implementation

## Overview

This document describes the implementation of automatic path saving and cross-session path loading for the grasp sequence planner.

## Features

### 1. Automatic Path Saving

When creating a `GraspSequencePlanner`, you can specify an `auto_save_dir` parameter:

```python
planner = GraspSequencePlanner(
    robot_name="robot",
    object_name="object",
    auto_save_dir="/tmp/grasp_sequence_paths"
)
```

After each successful phase, paths are automatically saved in two formats:
- **Portable JSON format** (`.json`): Works across sessions, graph-independent
- **Native binary format** (`.path`): Session-dependent, may fail for graph-constrained paths

File naming convention: `phase_NN_edge_MM_edgename.{json,path}`

### 2. Portable JSON Waypoint Format

The JSON format solves the cross-session loading problem by storing sampled configurations instead of graph-constrained path objects.

**Format structure:**
```json
{
  "format_version": "1.0",
  "path_length": 12.34,
  "num_samples": 100,
  "waypoints": [
    [q1_0, q1_1, ..., q1_n],
    [q2_0, q2_1, ..., q2_n],
    ...
  ]
}
```

**Advantages:**
- ✅ Works across different sessions
- ✅ No constraint graph dependency
- ✅ Human-readable format
- ✅ Can be loaded without recreating the planning environment

**Trade-offs:**
- Path is reconstructed by interpolating between waypoints
- May not exactly match original path (but should be very close with 100 samples)
- Requires collision-checking during reconstruction

### 3. Native Binary Format

The native `.path` format uses HPP's boost::serialization:

**Advantages:**
- ✅ Exact path preservation
- ✅ Includes all path metadata
- ✅ Faster to save/load

**Limitations:**
- ❌ Cannot serialize paths with time parameterization
- ❌ Cannot load graph-constrained paths without the same constraint graph
- ❌ Session-dependent

## API Reference

### Backend Methods

#### `save_path_as_waypoints(path_vector, filename, num_samples=100)`

Save a path as JSON waypoints (portable format).

**Parameters:**
- `path_vector`: PathVector CORBA object
- `filename`: Output file path (`.json` extension added automatically)
- `num_samples`: Number of waypoints to sample (default: 100)

**Usage:**
```python
backend.save_path_as_waypoints(path, "/tmp/my_path.json", num_samples=100)
```

#### `load_path_from_waypoints(filename, add_to_problem=True)`

Load a path from JSON waypoints file.

**Parameters:**
- `filename`: Input JSON file path
- `add_to_problem`: If True, add to ProblemSolver and return index

**Returns:**
- Path index in ProblemSolver (if `add_to_problem=True`)
- PathVector object (if `add_to_problem=False`)

**Usage:**
```python
# Load and add to problem solver
path_idx = backend.load_path_from_waypoints("/tmp/my_path.json")

# Or just get the PathVector
path = backend.load_path_from_waypoints("/tmp/my_path.json", add_to_problem=False)
```

### GraspSequencePlanner Methods

#### Constructor Parameter: `auto_save_dir`

```python
planner = GraspSequencePlanner(
    robot_name="robot",
    object_name="object",
    auto_save_dir="/tmp/grasp_sequence_paths"  # Enable auto-save
)
```

When `auto_save_dir` is set, paths are automatically saved after each successful phase.

## Interactive UI Support

The `task_display_states.py` script provides interactive loading:

1. Select "Load & replay saved paths (from files)" from the main menu
2. Choose the directory containing saved paths
3. Select format to load:
   - JSON waypoints (portable, always works)
   - Native .path files (may require graph setup)
4. Replay loaded paths

**CLI arguments:**
```bash
python task_display_states.py \
    --auto-save-dir /tmp/grasp_sequence_paths \
    --load-paths /tmp/grasp_sequence_paths
```

## Technical Details

### Why Two Formats?

The dual-format approach provides both reliability and portability:

1. **JSON format** is the primary format for cross-session use
   - Always saved (if backend supports waypoints)
   - Guaranteed to work in new sessions
   - Reconstructs path by interpolation

2. **Native format** is a best-effort optimization
   - Saved when possible (may fail for graph paths)
   - Preserves exact path when it works
   - Useful for within-session replay

### Error Handling

**Time Parameterization Error:**
- Fixed by storing geometric paths (before time param) separately
- Auto-save uses `phase_geometric_paths` when available

**Graph Deserialization Error:**
- Native format fails when loading graph-constrained paths in new sessions
- JSON format bypasses this by not storing graph references
- UI prompts user to use JSON format when graph error occurs

### Path Reconstruction from Waypoints

The `load_path_from_waypoints()` method:
1. Reads waypoints from JSON
2. Creates path segments between consecutive waypoints using `directPath()`
3. Concatenates segments into a PathVector
4. Adds to ProblemSolver

This ensures the reconstructed path is collision-free (assuming `directPath` validates).

## Usage Examples

### Example 1: Planning with Auto-Save

```python
from agimus_spacelab.tasks.grasp_sequence import GraspSequencePlanner

# Create planner with auto-save
planner = GraspSequencePlanner(
    robot_name="panda",
    object_name="part",
    auto_save_dir="/tmp/my_grasp_paths"
)

# Plan sequence - paths automatically saved after each phase
result = planner.plan_sequence(
    q_init=q_init,
    obj_target_pose=target_pose,
    verbose=True
)

# Paths are saved as:
# - phase_01_edge_01_panda__part_01_0.json (portable)
# - phase_01_edge_01_panda__part_01_0.path (native, if possible)
# - phase_01_edge_02_panda__part_12.json
# - phase_01_edge_02_panda__part_12.path
# ... etc
```

### Example 2: Loading Paths in a New Session

```python
# In a NEW Python session (different process/restart)
from agimus_spacelab.backends.corba import CorbaBackend

backend = CorbaBackend()
# ... minimal robot setup ...

# Load portable JSON paths (no graph needed!)
import glob
json_files = sorted(glob.glob("/tmp/my_grasp_paths/phase_*.json"))

for filepath in json_files:
    idx = backend.load_path_from_waypoints(filepath)
    print(f"Loaded {filepath} -> path index {idx}")
    
    # Play the path
    backend.play_path(idx)
```

### Example 3: Converting Native to JSON

```python
# Convert existing .path file to portable JSON
backend = CorbaBackend()
# ... setup robot and graph ...

# Load native path (requires graph)
path_idx = backend.load_path("/tmp/my_path.path")

# Get the path object
path = backend.ps.client.basic.problem.getPath(path_idx)

# Save as JSON waypoints
backend.save_path_as_waypoints(path, "/tmp/my_path.json")

# Now you can load it in any session without graph!
```

## Troubleshooting

### "Cannot deserialize edges" error

This means the native `.path` file contains constraint graph references.

**Solutions:**
1. Use JSON waypoint files instead (portable format)
2. Recreate the constraint graph before loading
3. Load paths within the same session where they were created

### "Failed to create segment" warnings

When loading from waypoints, some segments may fail to interpolate.

**Possible causes:**
- Waypoints are too far apart
- Collision between waypoints
- Configuration violates joint limits

**Solutions:**
- Increase `num_samples` when saving (e.g., 200 instead of 100)
- Check robot configuration bounds
- Verify environment is set up correctly

### JSON file is too large

Each waypoint stores a full configuration vector, so files can be large.

**Solutions:**
- Reduce `num_samples` (trade-off: less accurate reconstruction)
- Use native `.path` format for within-session storage
- Compress files (gzip works well with JSON)

## Implementation Files

- `backends/corba.py`: Path save/load methods
- `backends/base.py`: Abstract interface
- `tasks/grasp_sequence.py`: Auto-save functionality
- `script/spacelab/task_display_states.py`: Interactive UI

## Future Enhancements

Potential improvements:
1. Compressed JSON format (gzip)
2. Adaptive waypoint sampling (more samples in complex regions)
3. Path validation after reconstruction
4. Metadata storage (timestamps, robot name, success rate)
5. Batch conversion tool (`.path` → `.json`)
