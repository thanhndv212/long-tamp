# Development report

A single point-in-time engineering report on `agimus_spacelab`: why it's built the way it is,
not just how to use it. Companion to [`../usage/`](../usage/) (the living how-to reference —
trust that over this doc wherever the two disagree), [`../features/`](../features/) (why
specific mechanisms exist), [`../bugs/`](../bugs/) (upstream HPP defects worked around here),
and [`../plans/`](../plans/) (refactor history).

| Doc | For |
| --- | --- |
| [development-report.md](development-report.md) | Architecture decisions vs. bare HPP, measured before/after numbers, a project timeline, a step-by-step mission-building walkthrough, and a bugs-found appendix. Renders natively on GitLab/GitHub (tables, Mermaid diagrams, collapsible bug entries). |
| [development-report.html](development-report.html) | Same content, standalone styled HTML — open directly in a browser (needs network access once, to load the Mermaid diagram renderer from a CDN). |
| [behaviortree-screwdriving-report.md](behaviortree-screwdriving-report.md) | Point-in-time status of the ROS-free BehaviorTree.CPP task-planning implementation: architecture, verification results, and the real-mission gap. See [`../usage/behaviortree-integration.md`](../usage/behaviortree-integration.md) for the living how-to instead. |

Not updated as the API changes — see [`../usage/standalone-usage.md`](../usage/standalone-usage.md)
for that.
