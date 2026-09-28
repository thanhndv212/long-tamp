"""A three-step mission that can SIGKILL itself mid-step (test_kill_resume.py).

Usage: kill_resume_mission.py <run dir> [<item to die on>]

Steps place a, b, c in order; each records ``placed(<item>)`` in
``<run dir>/facts.json`` when it completes. The world state is those recorded
facts only, so a restarted run knows what earlier runs achieved and nothing
else. Events go to ``<run dir>/events.jsonl``. With an item given, placing it
kills the process with SIGKILL -- no cleanup, no atexit, like a crash or an
OOM kill.
"""

import os
import signal
import sys
from pathlib import Path

from long_tamp.tasks.task_planning import (
    CapabilityDescriptor,
    CapabilityRegistry,
    RecordedFacts,
    TaskPlan,
    TaskPlanningSession,
)
from long_tamp.tasks.task_planning.events import JsonlEventWriter
from long_tamp.tasks.task_planning.runner import run_plan

ITEMS = ("a", "b", "c")


def main() -> int:
    run_dir = Path(sys.argv[1])
    die_on = sys.argv[2] if len(sys.argv) > 2 else None

    def place(parameters):
        if parameters["item"] == die_on:
            os.kill(os.getpid(), signal.SIGKILL)
        return {}

    registry = CapabilityRegistry()
    registry.register(
        CapabilityDescriptor(
            "place",
            "1.0",
            {"item": str},
            effects=("placed(?item)",),
            restartable=True,
        ),
        place,
    )
    document = {
        "schema_version": "1.0",
        "mission_id": "kill-resume",
        "scene": {"id": "fake"},
        "provenance": {"kind": "human", "generator": "test"},
        "root": {
            "type": "sequence",
            "id": "mission",
            "children": [
                {
                    "type": "transaction",
                    "id": f"place-{item}",
                    "restart_state": ["q_current"],
                    "children": [
                        {
                            "type": "operation",
                            "id": f"place-{item}.execute",
                            "capability": "place",
                            "parameters": {"item": item},
                        }
                    ],
                }
                for item in ITEMS
            ],
        },
    }
    recorded = RecordedFacts(run_dir / "facts.json", predicates={"placed"})
    plan = TaskPlan.from_dict(document, registry)
    session = TaskPlanningSession(plan, registry, world_state=recorded, recorded=recorded)
    with JsonlEventWriter(run_dir / "events.jsonl") as events:
        run = run_plan(session, on_event=events)
    return 0 if run.success else 1


if __name__ == "__main__":
    sys.exit(main())
