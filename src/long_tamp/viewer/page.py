"""Build the viewer page: one self-contained HTML file (no network needed)."""

from __future__ import annotations

import json
import re
from importlib import resources
from pathlib import Path
from typing import Any

from long_tamp.tasks.task_planning.events import plan_event, read_events

from .config import ViewerConfig


def _static(name: str) -> str:
    return (
        resources.files("long_tamp.viewer")
        .joinpath("static", name)
        .read_text(encoding="utf-8")
    )


def _script_json(value: Any) -> str:
    """JSON safe to inline in a <script> element."""
    return (
        json.dumps(value)
        .replace("<", "\\u003c")
        .replace(">", "\\u003e")
        .replace("&", "\\u0026")
    )


def find_events(run: str | Path) -> Path:
    """``run``'s event stream: the file itself, or ``<run>/events.jsonl``."""
    run = Path(run)
    path = run / "events.jsonl" if run.is_dir() else run
    if not path.exists():
        raise FileNotFoundError(f"no event stream at {path}")
    return path


def load_run(
    run: str | Path, plan: str | Path | dict[str, Any] | None = None
) -> list[dict[str, Any]]:
    """The events of a run (a run folder or a JSONL file).

    ``plan`` (a TaskPlan IR document, or a JSON file of one) is put first as
    a ``plan`` event, for streams recorded without one.
    """
    events = read_events(find_events(run))
    if plan is not None:
        if not isinstance(plan, dict):
            plan = json.loads(Path(plan).read_text(encoding="utf-8"))
        first = plan_event(plan, "given to the viewer")
        if events:
            first["t"] = events[0]["t"]
        events.insert(0, first)
    return events


def render_html(
    events: list[dict[str, Any]] | None = None,
    config: ViewerConfig | None = None,
    *,
    live: bool = False,
    chat: bool = False,
    control: bool = False,
) -> str:
    """The viewer page.

    A replay embeds ``events``; a ``live`` page fetches them from the server
    that serves it (``ViewerServer``), and shows the chat panel if ``chat``
    and pause/resume/stop buttons if ``control``.
    """
    config = config or ViewerConfig()
    css = _static("viewer.css") + "".join(
        "\n" + Path(p).read_text(encoding="utf-8") for p in config.extra_css
    )
    extra_js = "".join(
        f"\n// {Path(p).name}\n" + Path(p).read_text(encoding="utf-8")
        for p in config.extra_js
    )
    boot = {
        "config": config.page_settings(),
        "events": [] if live else list(events or []),
        "live": live,
        "chat": chat,
        "control": control,
    }
    parts = {
        "TITLE": _escape(config.title),
        "CSS": css.replace("</", "<\\/"),
        "BOOT": _script_json(boot),
        "VIEWER_JS": _static("viewer.js").replace("</", "<\\/"),
        "EXTRA_JS": extra_js.replace("</", "<\\/"),
    }
    # One pass, so a placeholder inside inserted text stays as it is.
    return re.sub(
        r"/\*([A-Z_]+)\*/",
        lambda m: parts.get(m.group(1), m.group(0)),
        _static("viewer.html"),
    )


def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def write_replay(
    run: str | Path,
    out: str | Path | None = None,
    config: ViewerConfig | None = None,
    plan: str | Path | dict[str, Any] | None = None,
) -> Path:
    """Write ``run``'s replay page (default: ``<run folder>/viewer.html``)."""
    events = load_run(run, plan)
    if out is None:
        out = find_events(run).parent / "viewer.html"
    out = Path(out)
    out.write_text(render_html(events, config), encoding="utf-8")
    return out
