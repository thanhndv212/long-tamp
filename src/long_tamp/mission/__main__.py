"""``python -m long_tamp.mission``: the mission UI for any domain (``ui``),
or one mission of it (``serve``); see :mod:`long_tamp.mission.ui`."""

from __future__ import annotations

import sys

from .ui import launch, serve

USAGE = """\
usage: python -m long_tamp.mission {ui,serve} --domain MODULE:FACTORY [options]

  ui     the mission UI: the 3D scene, the operator chat and the live plan
         monitor in one page; a fresh mission on the page's Reset
  serve  one mission (what ui runs); chat on stdin and, unless --no-web, the page

'python -m long_tamp.mission ui --help' lists the options."""


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(USAGE)
        return 0 if argv else 2
    command, rest = argv[0], argv[1:]
    if command == "ui":
        return launch(rest)
    if command == "serve":
        return serve(rest)
    print(f"unknown command {command!r}\n\n{USAGE}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
