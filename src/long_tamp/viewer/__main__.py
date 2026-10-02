"""``python -m long_tamp.viewer``: replay a recorded run, or follow a live one."""

from __future__ import annotations

import argparse
import time
import webbrowser

from .config import ViewerConfig
from .page import find_events, write_replay
from .server import ViewerServer


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m long_tamp.viewer", description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)
    replay = sub.add_parser("replay", help="write a self-contained replay page")
    serve = sub.add_parser("serve", help="follow a run's events live")
    for p in (replay, serve):
        p.add_argument("run", help="a run folder (with events.jsonl) or a JSONL file")
        p.add_argument("--config", help="a ViewerConfig JSON file")
        p.add_argument("--title", help="the page title")
        p.add_argument("--scene-url", help="a page to embed as the scene (Viser)")
        p.add_argument("--open", action="store_true", help="open it in a browser")
    replay.add_argument("-o", "--out", help="the page (default: <run>/viewer.html)")
    replay.add_argument(
        "--plan", help="a TaskPlan JSON, for streams without a plan event"
    )
    serve.add_argument("--port", type=int, default=8090)
    serve.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args(argv)

    config = ViewerConfig.load(args.config) if args.config else ViewerConfig()
    if args.title:
        config.title = args.title
    if args.scene_url:
        config.scene_url = args.scene_url

    if args.command == "replay":
        out = write_replay(args.run, args.out, config, plan=args.plan)
        print(f"replay: {out}")
        if args.open:
            webbrowser.open(out.resolve().as_uri())
        return 0

    server = ViewerServer(find_events(args.run), config, args.port, args.host)
    url = server.start()
    print(f"viewer: {url}  (Ctrl-C to stop)")
    if args.open:
        webbrowser.open(url)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
