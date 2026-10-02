"""What the mission viewer shows and how it looks (``ViewerConfig``)."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

#: The built-in panels, in their default order.
PANELS = ("summary", "timeline", "monitor", "plan", "details", "events", "chat", "scene")


@dataclass
class ViewerConfig:
    """Settings of a mission viewer page.

    Everything here is plain data, so a config can live in a JSON file
    (``ViewerConfig.load``, ``python -m long_tamp.viewer --config``).

    - ``panels``: the panels to show, in order: built-in ones (``PANELS``;
      ``scene`` shows only with a ``scene_url``, ``chat`` only on a live
      server with a ``ChatBridge``) and the ids of panels an
      ``extra_js`` file registers. Panels registered but not listed come
      last.
    - ``hidden_roles``: event roles left out of the event list by default
      (the viewer has a toggle per role); the plan tree still uses them.
    - ``status_colors`` / ``theme``: CSS colors by status, and CSS variables
      (``--bg``, ``--fg``, ``--panel``, ``--muted``, ``--accent``, ``--border``)
      overriding the light and dark defaults.
    - ``metric_labels``: display names (and units) of event metrics, e.g.
      ``{"seconds": "planning (s)"}``; unlisted metrics show by key.
    - ``scene_url``: a page to embed as the 3D scene, e.g. the Viser server
      the mission plays on (``http://localhost:8081``).
    - ``extra_css`` / ``extra_js``: files inlined after the built-in ones;
      the JS can register panels and hooks through ``window.LongTamp``
      (``docs/usage/viewer.md``).
    - ``poll_ms``: how often a live page asks for new events.
    - ``layout``: ``"screen"`` (default) fits everything on one screen,
      panels scrolling inside; ``"page"`` stacks them in a scrolling page.
      Below 1000 px wide, ``screen`` stacks them too.
    - ``screen``: where each panel goes on one screen, by area (``top``,
      ``left``, ``center``, ``right``); an inner list is a group of tabs.
      Default: ``{"top": ["summary"], "left": [["monitor", "plan"], ["details", "events"]],
      "center": ["scene", "timeline"], "right": ["chat"]}``. Panels not placed
      join the left tabs; without a scene, the tabs take the centre.
    """

    title: str = "long-tamp mission"
    panels: list[str] = field(default_factory=lambda: list(PANELS))
    hidden_roles: list[str] = field(default_factory=lambda: ["ready", "precondition"])
    status_colors: dict[str, str] = field(default_factory=dict)
    theme: dict[str, str] = field(default_factory=dict)
    metric_labels: dict[str, str] = field(default_factory=dict)
    scene_url: str | None = None
    extra_css: list[str] = field(default_factory=list)
    extra_js: list[str] = field(default_factory=list)
    poll_ms: int = 500
    layout: str = "screen"
    screen: dict[str, list[Any]] | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ViewerConfig:
        if data.get("layout", "screen") not in ("screen", "page"):
            raise ValueError('layout must be "screen" or "page"')
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f"unknown viewer settings {unknown}")
        return cls(**data)

    @classmethod
    def load(cls, path: str | Path) -> ViewerConfig:
        """A config from a JSON file; relative ``extra_*`` paths are
        resolved against the file's folder."""
        path = Path(path)
        config = cls.from_dict(json.loads(path.read_text(encoding="utf-8")))
        config.extra_css = [str(path.parent / p) for p in config.extra_css]
        config.extra_js = [str(path.parent / p) for p in config.extra_js]
        return config

    def page_settings(self) -> dict[str, Any]:
        """What the page's script sees as ``LongTamp.config``."""
        data = asdict(self)
        del data["extra_css"], data["extra_js"]
        return data
