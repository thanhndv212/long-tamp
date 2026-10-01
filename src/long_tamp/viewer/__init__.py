"""Web mission viewer over the event stream (#24).

A replay is one self-contained HTML file built from a run's ``events.jsonl``
(``write_replay``); a live view is served next to a running mission
(``ViewerServer``). Both show the plan tree with each node's state (attempt
k/N, skipped because its effect held, drift, pauses), a timeline to scrub
through, the event list and per-node details; a live view can also embed the
3D scene and pause/resume/stop the mission. ``ViewerConfig`` and
``window.LongTamp`` customize it; see ``docs/usage/viewer.md``.

Command line::

    python -m long_tamp.viewer replay <run folder | events.jsonl> [-o page.html]
    python -m long_tamp.viewer serve <run folder | events.jsonl> [--port 8090]
"""

from .config import PANELS, ViewerConfig
from .page import find_events, load_run, render_html, write_replay
from .server import EventTail, ViewerServer

__all__ = [
    "PANELS",
    "EventTail",
    "ViewerConfig",
    "ViewerServer",
    "find_events",
    "load_run",
    "render_html",
    "write_replay",
]
