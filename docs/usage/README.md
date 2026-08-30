# Usage guides

Step-by-step docs for using `agimus_spacelab`, in the two ways it's actually used in this
workspace. Companion to [`../features/`](../features/) (why specific mechanisms exist) and
[`../bugs/`](../bugs/) (upstream HPP defects worked around here).

| Doc | For |
| --- | --- |
| [standalone-usage.md](standalone-usage.md) | Using `agimus_spacelab` as a plain Python library: writing a task, multi-phase grasp sequences, resume/replay/checkpoints, backends, example scripts. No ROS required. |
| [dbt-integration.md](dbt-integration.md) | Running assembly missions through the ROS 2 / Dynamic Behavior Tree stack (`ros2_ws_agimusxads`): services/actions, the DBT executive, launch files, and which library features are (and aren't) reachable from ROS. |
| [behaviortree-integration.md](behaviortree-integration.md) | Running missions through the ROS-free BehaviorTree.CPP host: the versioned task-plan IR, capability registry, deterministic IR-to-BT compiler, the CPython bridge, checkpointing/path capture, and how it compares to the DBT integration. |

Start with `standalone-usage.md` even if your end goal is the DBT or the BehaviorTree host —
both are thin executives around exactly that API.

For *why* the framework is built this way — architecture decisions, measured before/after
numbers, project timeline, bugs-found appendix — see [`../report/`](../report/) instead. It's
a separate, point-in-time report, not a living reference like the docs above.
