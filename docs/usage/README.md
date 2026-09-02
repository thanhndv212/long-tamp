# Usage guides

Step-by-step docs for using `long_tamp`, in the two ways it's actually used in this
workspace. Companion to [`../features/`](../features/) (why specific mechanisms exist) and
[`../bugs/`](../bugs/) (upstream HPP defects worked around here).

| Doc | For |
| --- | --- |
| [standalone-usage.md](standalone-usage.md) | Using `long_tamp` as a plain Python library: writing a task, multi-phase grasp sequences, resume/replay/checkpoints, backends, example scripts. No ROS required. |
| [behaviortree-integration.md](behaviortree-integration.md) | Running missions through the ROS-free BehaviorTree.CPP host: the versioned task-plan IR, capability registry, deterministic IR-to-BT compiler, the CPython bridge, and checkpointing/path capture. |

Start with `standalone-usage.md` even if your end goal is the BehaviorTree host — it's a
thin executive around exactly that API. A ROS 2 / Dynamic Behavior Tree integration
previously existed here (see [`../legacy/usage/dbt-integration.md`](../legacy/usage/dbt-integration.md))
but is tied to a proprietary mission executive and isn't part of this release.

For *why* the framework is built this way — architecture decisions, measured before/after
numbers, project timeline, bugs-found appendix — see [`../report/`](../report/) instead. It's
a separate, point-in-time report, not a living reference like the docs above.
