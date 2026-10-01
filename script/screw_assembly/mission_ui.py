#!/usr/bin/env python3
"""Start the screw assembly's mission UI (#104): one browser page with the 3D
scene, the operator chat and the live plan monitor.

    python mission_ui.py                    # MuJoCo, chat with the default model
    python mission_ui.py --backend playback --seed 4 --port 8090

Type an instruction in the chat ("assemble parts 1 and 2, part 2 first"); the
model sets the goal and the task planner builds the plan, shown as a card
with a Start mission button. Start runs it: the plan tree and the timeline
follow it live, the scene plays the motion, and pause/resume/stop act at
step boundaries. Ctrl-C here ends everything.

The model and its endpoint come from the AI env file
(``$LONG_TAMP_AI_ENV``, else ``~/.config/long-tamp/ai.env``) and
``$LONG_TAMP_GOAL_MODEL``; ``--model`` overrides it. Arguments after ``--``
go to task_screw_assembly.py as they are.
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

HERE = Path(__file__).resolve().parent


def in_container() -> bool:
    return Path("/.dockerenv").exists() or Path("/run/.containerenv").exists()


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = []
    if "--" in argv:
        extra = argv[argv.index("--") + 1 :]
        argv = argv[: argv.index("--")]
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8090, help="the page (default 8090)")
    ap.add_argument(
        "--scene-port", type=int, default=8081, help="the Viser scene (default 8081)"
    )
    ap.add_argument(
        "--backend",
        default="mujoco",
        choices=("mujoco", "playback", "mock", "none"),
        help="what executes the motion (default: mujoco)",
    )
    ap.add_argument(
        "--watchdog",
        default="300,900",
        help="SOFT,HARD seconds a step may plan before the watchdog acts (default "
        "300,900; 'off' turns it off)",
    )
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--model", help="API:MODEL (default: $LONG_TAMP_GOAL_MODEL)")
    ap.add_argument("--run-dir", type=Path, help="default: runs/ui_<time>/")
    ap.add_argument(
        "--no-open",
        action="store_true",
        help="don't open a browser (always off in a container)",
    )
    args = ap.parse_args(argv)

    run_dir = args.run_dir or HERE / "runs" / time.strftime("ui_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        str(HERE / "task_screw_assembly.py"),
        "--chat",
        "--seed", str(args.seed),
        "--backend", args.backend,
        "--web-port", str(args.port),
        "--viewer-port", str(args.scene_port),
        "--run-dir", str(run_dir),
    ]  # fmt: skip
    if args.model:
        command += ["--goal-model", args.model]
    if args.watchdog != "off":
        command += ["--watchdog", args.watchdog]
    command += extra
    log_path = run_dir / "ui.out"
    print(f"mission UI: starting (log: {log_path})", flush=True)
    with open(log_path, "w") as log:
        # No stdin: the chat is the page's. Output goes to the log; the lines
        # an operator needs are echoed here.
        child = subprocess.Popen(
            command,
            cwd=HERE,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        try:
            for line in child.stdout:
                log.write(line)
                log.flush()
                if line.startswith("web viewer: "):
                    url = line.split(": ", 1)[1].strip()
                    print(f"\n  Open the mission UI: {url}\n", flush=True)
                    if not (args.no_open or in_container()):
                        webbrowser.open(url)
                elif (
                    line.startswith(("chat:", "Traceback", "ERROR")) or "Error" in line
                ):
                    print("  " + line.rstrip(), flush=True)
        except KeyboardInterrupt:
            print("\nmission UI: stopping", flush=True)
            child.send_signal(signal.SIGINT)
            try:
                child.wait(10)
            except subprocess.TimeoutExpired:
                child.kill()
        code = child.wait()
    print(f"mission UI: ended ({code}); run folder: {run_dir}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
