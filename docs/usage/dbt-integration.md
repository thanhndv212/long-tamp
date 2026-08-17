# Using `agimus_spacelab` with the Dynamic Behavior Tree (DBT)

How the `agimus_spacelab` HPP planning library is wired into the ROS 2 workspace
(`ros2_ws_agimusxads`) and driven by the DBT executive to run full assembly missions. For the
plain-Python library API, see [`standalone-usage.md`](standalone-usage.md) — everything here
sits on top of it.

## 1. The big picture

`agimus_spacelab` (this repo) has **no ROS dependency**. `ros2_ws_agimusxads` is a separate
workspace that imports it via `PYTHONPATH` (never vendored) and wraps it as ROS 2
services/actions. The DBT executive talks to that ROS wrapper only — it never touches HPP or
Python directly. Everything below the planner node is a process boundary crossed by ROS 2:

```
spacelab_bt_ros  (C++ DBT executive)
   DBTxHPPInterface  ──── ROS 2 services/actions ────►  agimus_spacelab_ros
                                                          SpacelabPlannerNode
                                                              │  in-process Python call
                                                              ▼
                                                          agimus_spacelab  (this repo)
                                                              │  PyHPP (in-process) | CORBA (hppcorbaserver :13331)
                                                              ▼
                                                          spacelab_mock_hardware (Gazebo + ros2_control)
```

Full details: `ros2_ws_agimusxads/docs/ARCHITECTURE.md`, `ros2_ws_agimusxads/AGENTS.md`.

## 2. Package map

| Package | Type | Role |
|---|---|---|
| `agimus_spacelab_interfaces` | `ament_cmake` | 4 srvs + 2 actions — the only cross-process contract |
| `agimus_spacelab_ros` | `ament_python` | `SpacelabPlannerNode`, attach service, rqt GUI — the ROS wrapper around this library |
| `spacelab_bt_ros` | `ament_cmake` | C++ DBT executive: vendored BT, proprietary `libDBT.so`, mission XML + callbacks |
| `spacelab_mock_hardware` | `ament_cmake` | URDFs, Gazebo `GraspSystem` plugin, launch files |

**Build order** (interfaces must build first — the srv/action code generation feeds both other
packages):
```bash
cd ~/devel/ros2_ws_agimusxads
colcon build --symlink-install --packages-select agimus_spacelab_interfaces
colcon build --symlink-install --packages-select agimus_spacelab_ros
colcon build --symlink-install   # everything else
```
Never use `--symlink-install` for `agimus_spacelab_ros` **outside** Docker — the generated
`lib/<pkg>/<exe>` wrapper script is required for `ros2 run` to work.

## 3. The ROS interfaces (the whole contract)

| Interface | Fields |
|---|---|
| `srv/PlanGrasp` | Req: `gripper`, `handle`, `locked_arms[]`, `use_current_state`. Resp: `success`, `message`, `trajectories[]` (`JointTrajectory` per robot), `robot_namespaces[]`. |
| `srv/PlanSequence` | Req: `grippers[]`, `handles[]`, `frozen_arms_mode` (`auto`\|`manual`\|`none`), `per_phase_locked_arms[]`/`_sizes[]`, `timeout_per_phase`, `held_grasps_grippers[]`/`held_grasps_handles[]`. Resp: `success`, `message`, `trajectories[]`, `robot_namespaces[]`, `phase_start_indices[]`, `phases_completed`. |
| `action/PlanSequence` | Goal/Result mirror `PlanSequence.srv`. Feedback: `current_phase`, `total_phases`, `status_message`, `phase_progress_percent`. |
| `action/ExecuteGraspSequence` | Goal: same planning fields + `plan_only`, plus `precomputed_trajectories`/`_robot_namespaces`/`_phase_start_indices` to **skip re-planning** and just execute a plan you already have. Feedback: same 4 fields as above. |
| `srv/AttachObject` / `DetachObject` | Req: `gripper_name`, `object_name`. Resp: `success`, `message`. |

`held_grasps_grippers/handles` is how a caller tells the planner what's already grasped before
planning the next phase — the DBT interface seeds this from its own grasp-tracking state on
every `planSequence()` call, since each ROS call is otherwise stateless from the caller's side.

## 4. `SpacelabPlannerNode` (`agimus_spacelab_ros`)

Node name `spacelab_planner_node`. Advertises:

**Services**: `plan_grasp`, `plan_sequence`, `resume_sequence` (retries from the last failed
edge of the previous in-memory sequence — see §7), `non_stop_plan_sequence` (auto-resume loop
until success or `stop_planning`), `stop_planning`, `resume_planning`, `shutdown`.

**Actions**: `plan_sequence_action` (the one the DBT actually uses for planning; rejects a new
goal while a plan is already in flight — see the busy-guard note in §6), `execute_grasp_sequence`.

**HPP invocation chain**: lazy `_ensure_task_ready()` loads a `TaskConfigurations.<X>` config →
constructs `PlannerTask` (a `ManipulationTask` subclass) with `backend=self._backend` →
`task.setup(skip_graph=True)` (phase graphs are built per-phase, never a global graph) →
`PlanningEngine.run_plan_sequence()` constructs a `GraspSequencePlanner` and calls
`plan_sequence(grasp_sequence=list(zip(grippers, handles)), q_init=..., frozen_arms_mode=...,
timeout_per_edge=...)` — this is exactly the standalone API from
[`standalone-usage.md` §6](standalone-usage.md#6-multi-phase-grasp-sequences-graspsequenceplanner).
Result `Path` objects are converted to `JointTrajectory` via
`trajectory_utils.hpp_path_to_joint_trajectories()`.

**Backend selection** is a single ROS parameter, `backend:=pyhpp|corba`, read once at startup
and passed straight through to `ManipulationTask`. With `pyhpp` HPP runs **inside the planner
node's own process**; with `corba` an external `hppcorbaserver` must already be running
(launch files start it with an extra startup delay).

**`q_init` assembly**: joint positions from `/joint_states` + object poses from TF, unless
`use_current_state=False`, in which case HPP's own default `q_init` is used (no live robot
state needed — this is what the minimal smoke-test script does, §9).

## 5. `DBTxHPPInterface` (`spacelab_bt_ros`)

A singleton owning node `dbt_hpp_interface_node`, spun on its own `MultiThreadedExecutor`
thread. It is a pure **ROS client** — no HPP/Python linkage. Its member functions are
registered as DBT Subtask/Usefulness/Precondition callbacks (`DBT_OK=0`, `DBT_FAIL=-1`,
`DBT_IDLE=-99`):

- `planSequence()` → sends a goal to `~/plan_sequence_action`, seeded with
  `held_grasps_grippers/handles` from an internal `HeldGraspsTracker`; polls the action future
  every 200 ms and only times out on **silence** (no feedback for `inactivity_timeout_`,
  default 30 s) — not on a flat wall-clock budget, so long RRT searches don't trigger DBT
  retry storms.
- `executeSequence()` → checks joint-state drift since the plan was made (`DriftDetector`); if
  drift exceeds 0.005 rad, **re-plans automatically** before executing; then sends
  `precomputed_trajectories` from the last plan to `~/execute_grasp_sequence` so the server
  skips re-planning.
- `attachObjectViaService()` / `detachObjectViaService()` → call `~/attach_object` /
  `~/detach_object`, falling back to a direct Gazebo Bool-topic publish if the service is
  unreachable.
- Vision/PLC/cartesian-servo helpers (`checkPoseClearance`, `executeVisualServoing`, …) are
  Gazebo stubs (`[GAZEBO STUB]`, return true) — HPP's constraint graph already handles
  precision approach in simulation, so these only matter on real hardware.

## 6. Mission definition (Petri-net XML + C++ callbacks)

Missions are `<Mission>` XML files (`spacelab_bt_ros/trees/`) with `<Action>` nodes carrying
`<Usefulness>`, optional `<Precondition>` (with a `<Action id="Mitigate...">` fallback), and
either a `<Subtask>` leaf (maps 1:1 to a registered C++ function) or a nested `<Taskset>`.

- **`trees/hpp_grasp_mission.xml`** (`hpp_mission_node`) — the minimal example: A1
  `planSequence` → A2 `executeSequence`.
- **`trees/spacelab_assembly_mission.xml`** (`spacelab_assembly_mission_node`) — the real
  mission: **A1** equip tool (catch from dispenser) → **A2** move+mate at the assembly target
  (`planTrajFromCurrentPosToTargetPosDuringTargetPosApproachPhase`, sets `assembly_done=true`
  on success) → **A3** release tool to storage (only active in 2-robot mode). Links:
  A1→{A2,A3}, A3→A1 (re-equip loop). Blackboard `<Variables>` set the HPP vocabulary directly:
  `targetRobot="spacelab/g_ur10_tool"`, `targetTool="frame_gripper"`,
  `targetPose="RS1/h_RS1_FG"`, `dispenserHandle="frame_gripper/h_FG_tool"`.

C++ leaf callbacks (`src/spacelab_assembly_mission.cpp`) funnel through two dedup helpers —
`planPhase(gripper, handle, flag, label)` (calls `iface().planTrajPhase(...)`) and
`execPhase(flag, label)` (calls `iface().executeCurrentPlan()`) — and read blackboard values via
`bbGripper()`, `bbAssemblyGripper()`, `bbAssemblyHandle()`, etc. A module-level `PhaseState gps`
struct of booleans (`a1_plan_done`, `a2_exec_done`, `assembly_done`, …) gates the
Usefulness/Precondition functions that drive the Petri-net links.

**Runtime params**: `config/spacelab_assembly_mission.yaml` under
`/spacelab/spacelab_assembly_mission_params` — `gripper_0`/`handle_0` (grasp pair),
`assembly_gripper`/`assembly_handle` (mating pair), `frozen_arms_mode`, `timeout_per_phase`,
`inactivity_timeout`, plus `gripper_attach_params` (`"gripper:object:attach_topic:release_topic:re_anchor"`
list, one entry per grasp relationship — same format the attach service and Gazebo bridge use).

## 7. Attach / detach (three layers, one source of truth)

1. **`GraspSystem`** — Gazebo plugin (`spacelab_mock_hardware`) managing `DetachableJoint`s,
   toggled by Bool topics.
2. **`GazeboAttachmentBridge`** (C++, `spacelab_bt_ros`) and
3. **`attach_service_node`** (Python, `agimus_spacelab_ros`) —

both read the *same* `gripper_attach_params` parameter and publish the same Bool sequence, so
GUI / planner / DBT never race on service names. `attach_service_node` is auto-started by every
full-stack launch file specifically to keep this centralized.

## 8. Running it

Watch for `[spacelab_planner_node] SpacelabPlannerNode ready` before sending any command — it's
the readiness signal every launch file waits on.

**Full DBT assembly mission** (Gazebo + planner + attach service + DBT mission, launched in
dependency order with startup delays):
```bash
ros2 launch spacelab_bt_ros spacelab_full_assembly.launch.py backend:=pyhpp
```
Order: Gazebo scene → (CORBA server, only if `backend:=corba`) → planner node (5 s pyhpp / 20 s
corba delay, waits for Gazebo + `/joint_states`) → `attach_service_node` (+2 s) → DBT mission
node (+5 s, polls `waitForPlannerReady()`).

**Planner + GUI only, no DBT** (manual/interactive use):
```bash
ros2 launch agimus_spacelab_ros spacelab_ur10_gz_full_test.launch.py backend:=pyhpp
```

**Mock hardware only** (RViz, no HPP/DBT — URDF/controllers/MoveIt smoke test):
```bash
ros2 launch spacelab_mock_hardware slrobot_system.launch.py moveit_flag:=false
```

**Manual single-grasp plan** (no DBT, exercises the planner directly):
```bash
ros2 service call /spacelab_planner_node/plan_grasp agimus_spacelab_interfaces/srv/PlanGrasp \
  '{gripper: "spacelab/g_ur10_tool", handle: "frame_gripper/h_FG_tool", locked_arms: ["auto"], use_current_state: false}'
```

**Integration tests** (exercise the exact call sequence a DBT leaf makes):
```bash
ros2 run spacelab_bt_ros test_dbt_hpp_interface \
  --ros-args -p gripper:=spacelab/g_ur10_tool -p handle:=frame_gripper/h_FG_tool
ros2 run spacelab_bt_ros test_spacelab_assembly_mission \
  --ros-args -p gripper:=spacelab/g_ur10_tool -p handle:=frame_gripper/h_FG_tool
```
`test_dbt_hpp_interface` runs three groups: no-deps unit checks, TF/Gazebo-dependent checks,
then (needs the planner running) `waitForPlannerReady()` → `planSequence()` →
`planAndExecuteGrasp()` → `planAndExecuteRelease()` — the literal sequence a DBT leaf performs.

**End-to-end smoke test** (plain ROS client, no DBT):
```bash
python3 src/agimus_spacelab_ros/scripts/test_ur10_gz.py
```
Calls `PlanGrasp` with `use_current_state=False` (HPP's built-in default `q_init`, no live
Gazebo state needed), sends the resulting `/ur10` trajectory to
`/ur10/jointTraj_controller/follow_joint_trajectory`, then publishes directly to
`/frame_gripper/attach` to simulate the grasp (bypassing `attach_service_node`) — good minimal
reference for "plan one grasp and execute it" without the DBT layer.

## 9. Which HPP-library features are exposed through ROS/DBT — and which aren't

This matters because several `agimus_spacelab` features described in
[`standalone-usage.md`](standalone-usage.md) are **not** reachable from ROS at all:

| Feature | Exposed via ROS? |
|---|---|
| Multi-phase sequence planning | Yes — `plan_sequence` / `plan_sequence_action` |
| In-memory resume (`resume_sequence()`) | Yes — `~/resume_sequence` and `~/non_stop_plan_sequence` services, operating on the planner node's last in-memory `GraspSequencePlanner` (process-lifetime only, not persisted) |
| Phase graph / `set_phase_indices()` invariant | No — purely internal to the library; the ROS layer only ever calls `plan_sequence()`, which enforces it internally |
| Phase-target lookahead (`phase_q_hints`) | No — internal to `GraspSequencePlanner`; not a service field |
| Screwdriving sequence pattern | No — only exists as the standalone script `script/spacelab/test_screwdriving_sequence.py`. The assembly mission YAML has latent 2-robot-mode fields for a screw-driver tool hand-off, but they're empty/unused by default — not currently exercised end-to-end via DBT |
| `PathRecorder` / manifest capture / replay | No — driven purely by the `AGIMUS_CHECKPOINT_DIR` env var and direct Python API calls; zero references anywhere under `ros2_ws_agimusxads/src` |
| `RunLogger` structured JSONL | No — internal to library calls made by the planner node's own process; not surfaced to ROS callers |

If a DBT mission needs one of the "No" rows, that's new work on `agimus_spacelab_ros` — extend
`PlanningEngine`/the interfaces, don't expect it to already be there.

## 10. Known quirks

**Jazzy JTC heap corruption.** `sort_to_local_joint_order` (upstream `joint_trajectory_controller`)
uses a `static std::vector<double>` shared across all JTC instances on ROS 2 Jazzy, which
corrupts state under concurrent multi-robot goal dispatch. Workaround, already implemented in
`agimus_spacelab_ros/agimus_spacelab_ros/gui/_sequence_controller.py::_send_direct_phase()`:
send **one goal per robot per phase**, and skip any robot whose trajectory is stationary
(`is_stationary_trajectory()` — a frozen/locked arm, constant position). The same
"skip stationary trajectory" logic lives in `UnifiedExecutionManager`, shared by both the
GUI-driven and DBT/action-driven execution paths — so this protects DBT missions too, not just
the GUI.

**Busy-guard semantics.** `_plan_goal_cb()` rejects a new `plan_sequence_action` goal while a
plan is already in flight (`_planning_busy`). DBT must interpret a rejected goal as
`DBT_IDLE` (retry next tick), not `DBT_FAIL` — this is already how `DBTxHPPInterface` behaves;
don't "fix" a rejected goal by treating it as a hard failure in new mission code.

**Attach service is deliberately a separate node.** Don't embed attach/detach logic directly
in the planner or DBT node — it exists standalone specifically to avoid the two colliding on
Gazebo topic names when both want to trigger the same attach.

**FastDDS SHM leftovers**: `rm -f /dev/shm/fastrtps_*` if you see `open_port_internal` errors
after a crashed run.

## 11. Robot configuration vector (70 DOF)

Referenced by any code reading/writing a raw `q` vector across the ROS boundary:
```
q[0:6]   ur10 joints          q[35:42] RS4 free-flyer
q[6:8]   vispa2 joints        q[42:49] RS5 free-flyer
q[8:14]  vispa joints         q[49:56] RS6 free-flyer
q[14:21] RS1 free-flyer       q[56:63] frame_gripper
q[21:28] RS2 free-flyer       q[63:70] screw_driver
q[28:35] RS3 free-flyer
```
