"""The mission UI for any domain (#104, #133): one browser page with the 3D
scene, the operator chat and the live plan monitor.

    python -m long_tamp.mission ui --domain my_pkg.my_domain:make_domain
    python -m long_tamp.mission ui --domain path/to/my_domain.py:make_domain

``--domain`` names a factory (a function or class, called with no
arguments) that returns a :class:`~long_tamp.mission.MissionDomain`, as
``module:attribute`` or ``file.py:attribute`` (the file's folder goes on
``sys.path``, so it may import its neighbours). Two commands:

- ``ui``: the launcher. It runs ``serve`` in a child process, echoes the
  page's address, opens a browser (not in a container), and starts a fresh
  mission when the page's Reset button asks for one (exit code
  :data:`RESTART`). Ctrl-C ends everything.
- ``serve``: one mission. ``domain.setup`` builds the scene, a
  :class:`~long_tamp.mission.MissionRunner` runs it, and the chat
  (:func:`~long_tamp.mission.chat.run_chat`) takes the operator's
  instructions from stdin and from the page. The run folder is the
  runner's (``mission.json``, ``checkpoint.json``, ``events.jsonl``,
  ``facts.json``, ``trajectory.json``).

The model and its endpoint come from the AI env file (``--ai-env``, else
``$LONG_TAMP_AI_ENV``, else ``~/.config/long-tamp/ai.env``) and
``$LONG_TAMP_GOAL_MODEL``; ``--model`` overrides it.
"""

from __future__ import annotations

import argparse
import atexit
import importlib
import importlib.util
import json
import signal
import subprocess
import sys
import time
import webbrowser
from pathlib import Path
from typing import Any

#: ``serve``'s exit code for "start a fresh mission" (the page's Reset, #114).
RESTART = 75


def load_domain(spec: str) -> Any:
    """The domain ``spec`` names: ``module:factory`` or ``file.py:factory``
    (see the module docstring)."""
    target, _, attribute = spec.rpartition(":")
    if not target or not attribute:
        raise ValueError(
            f"--domain {spec!r}: expected MODULE:FACTORY or FILE.py:FACTORY"
        )
    if target.endswith(".py"):
        path = Path(target).resolve()
        if not path.is_file():
            raise ValueError(f"--domain {spec!r}: no file {path}")
        sys.path.insert(0, str(path.parent))
        module_spec = importlib.util.spec_from_file_location(path.stem, path)
        if module_spec is None or module_spec.loader is None:
            raise ValueError(f"--domain {spec!r}: can't import {path}")
        module = importlib.util.module_from_spec(module_spec)
        sys.modules[path.stem] = module
        module_spec.loader.exec_module(module)
    else:
        module = importlib.import_module(target)
    factory: Any = module
    for name in attribute.split("."):
        factory = getattr(factory, name)
    return factory()


def in_container() -> bool:
    return Path("/.dockerenv").exists() or Path("/run/.containerenv").exists()


def parse_watchdog(text: str) -> tuple[float, float]:
    """``SOFT[,HARD]`` seconds -> (soft, hard); hard defaults to 3 x soft."""
    parts = [float(v) for v in text.split(",")]
    soft = parts[0]
    hard = parts[1] if len(parts) > 1 else 3 * soft
    if not 0 < soft <= hard:
        raise ValueError(f"--watchdog {text!r}: need 0 < SOFT <= HARD")
    return soft, hard


def parse_option(text: str) -> tuple[str, Any]:
    """``KEY=VALUE`` -> (key, value), the value as JSON if it parses
    (``speed=1.0``, ``grasp="contact"`` or ``grasp=contact``)."""
    key, sep, value = text.partition("=")
    if not sep or not key:
        raise ValueError(f"--backend-option {text!r}: expected KEY=VALUE")
    try:
        return key, json.loads(value)
    except json.JSONDecodeError:
        return key, value


def _common(ap: argparse.ArgumentParser) -> None:
    ap.add_argument(
        "--domain",
        required=True,
        help="the domain factory, MODULE:FACTORY or FILE.py:FACTORY",
    )
    ap.add_argument("--port", type=int, default=8090, help="the page (default 8090)")
    ap.add_argument(
        "--scene-port",
        type=int,
        default=8081,
        help="the domain's 3D scene, if it has one (default 8081)",
    )
    ap.add_argument(
        "--backend",
        default="mock",
        help="the domain's backend that executes the motion (default: mock)",
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


def serve(argv: list[str] | None = None) -> int:
    """One mission of the UI (see the module docstring); returns the exit
    code (:data:`RESTART` when the operator pressed Reset)."""
    ap = argparse.ArgumentParser(
        prog="python -m long_tamp.mission serve",
        description="one mission of the mission UI",
    )
    _common(ap)
    ap.add_argument(
        "--backend-option",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="an option for the domain's make_backend (repeatable)",
    )
    ap.add_argument("--no-scene", action="store_true", help="no 3D scene")
    ap.add_argument("--no-web", action="store_true", help="chat at the terminal only")
    ap.add_argument("--max-replans", type=int, default=10)
    ap.add_argument("--ai-env", help="the AI env file (endpoints, keys)")
    ap.add_argument("--title", help="the page title (default: the domain's name)")
    args = ap.parse_args(argv)

    from long_tamp.ai import configure, make_client
    from long_tamp.execution import ExecutionControl
    from long_tamp.tasks.mission_checkpoint import MissionCheckpoint
    from long_tamp.tasks.task_planning import RecordedFacts
    from long_tamp.tasks.task_planning.events import JsonlEventWriter, make_event

    from .chat import MissionChat, run_chat
    from .runner import MissionRunner, write_trajectory

    domain = load_domain(args.domain)
    options = dict(parse_option(o) for o in args.backend_option)
    run_dir = (
        args.run_dir or Path("runs") / time.strftime("ui_%Y%m%d_%H%M%S")
    ).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    scene = domain.setup(seed=args.seed, log_dir=str(run_dir))
    print(
        f"mission: {domain.name}, seed {args.seed}; run folder: {run_dir}", flush=True
    )

    checkpoint = MissionCheckpoint(
        run_dir, meta={"domain": args.domain, "seed": args.seed}
    )
    recorded = RecordedFacts(
        run_dir / "facts.json", predicates=domain.recorded_predicates
    )
    if scene.extras.get("facts"):  # the scene's starting recorded facts
        from long_tamp.tasks.task_planning.predicates import Literal

        recorded.apply([Literal.parse(f) for f in scene.extras["facts"]])
    runner = MissionRunner(domain, scene, recorded, max_replans=args.max_replans)
    events = JsonlEventWriter(run_dir / "events.jsonl")
    atexit.register(events.close)

    viewer = None
    if not args.no_scene:
        viewer = domain.scene_viewer(scene, args.scene_port)
        if viewer is not None:
            atexit.register(viewer.close)
    control, web = ExecutionControl(), None
    if not args.no_web:
        from long_tamp.viewer import ViewerConfig, ViewerServer

        web = ViewerServer(
            run_dir / "events.jsonl",
            ViewerConfig(
                title=args.title or f"{domain.name}, seed {args.seed}",
                scene_url=(
                    None if viewer is None else f"http://localhost:{args.scene_port}"
                ),
            ),
            port=args.port,
            control=control,
            separate_process=True,  # planning holds the GIL for seconds
        )
        print(f"web viewer: {web.start()}", flush=True)
        atexit.register(web.close)

    backend = domain.make_backend(
        args.backend, scene, live_viewer=viewer, run_dir=run_dir, **options
    )
    configure(args.ai_env)  # endpoints, keys; localhost in a container

    def on_call(record):
        events(
            make_event(
                "ai",
                "model",
                record.role,
                "FAILURE" if record.error else "SUCCESS",
                message=record.error or "",
                metrics=record.metrics(),
            )
        )

    client = make_client(args.model, on_call=on_call)
    on_event: Any = events
    watchdog = None
    if args.watchdog != "off":
        from long_tamp.execution.watchdog import StepWatchdog, model_decider

        soft, hard = parse_watchdog(args.watchdog)
        watchdog = StepWatchdog(
            control,
            soft,
            hard,
            decide=model_decider(client),
            sink=events,
            on_decision=lambda watch, decision: print(
                f"watchdog: {watch.label}: {decision.action} ({decision.by}): "
                f"{decision.reason}",
                flush=True,
            ),
        ).start()
        atexit.register(watchdog.close)

        def watched(event, _write=events, _watchdog=watchdog):
            _write(event)
            _watchdog.observe(event)

        on_event = watched

    trajectory: list[dict[str, Any]] = []
    mission = dict(
        backend=backend,
        trajectory=trajectory,
        checkpoint=checkpoint,
        live_viewer=viewer,
        on_event=on_event,
        control=control,
    )
    try:
        work = MissionChat(runner, mission, client=client)
        result = run_chat(work, client, events, web, watchdog)
    finally:
        if hasattr(backend, "close"):
            backend.close()  # a simulation process: stop and reap it
    checkpoint.finish(result["success"], result["seconds"])
    write_trajectory(run_dir / "trajectory.json", trajectory, domain.scene_info())
    print(
        f"mission: ended ({'success' if result['success'] else 'failure'}); "
        f"run folder: {run_dir}",
        flush=True,
    )
    if viewer is not None:
        if not result.get("reset"):
            try:
                viewer.finish()
            except Exception:  # noqa: BLE001 - the mission's result stands
                pass
        viewer.close()
    if result.get("reset"):
        return RESTART
    return 0 if result["success"] else 1


def launch(argv: list[str] | None = None, runs: Path = Path("runs")) -> int:
    """The launcher (``ui``, see the module docstring). Arguments after
    ``--`` go to ``serve`` as they are; without ``--run-dir``, each mission's
    run folder is ``runs/ui_<time>``."""
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = []
    if "--" in argv:
        extra = argv[argv.index("--") + 1 :]
        argv = argv[: argv.index("--")]
    ap = argparse.ArgumentParser(
        prog="python -m long_tamp.mission ui",
        description="the mission UI: the 3D scene, the operator chat and the live "
        "plan monitor in one page",
    )
    _common(ap)
    ap.add_argument(
        "--no-open",
        action="store_true",
        help="don't open a browser (always off in a container)",
    )
    args = ap.parse_args(argv)

    opened = False
    run_dir = args.run_dir
    while True:
        run_dir = run_dir or Path(runs) / time.strftime("ui_%Y%m%d_%H%M%S")
        code, opened = _run(args, extra, run_dir.resolve(), opened)
        if code != RESTART:
            return code
        # Reset from the page: a fresh mission on the same ports; the page
        # reloads itself when the new one answers.
        print("mission UI: reset, starting a fresh mission", flush=True)
        run_dir = None
        time.sleep(2.0)  # let the old servers release their ports


def serve_command(
    args: argparse.Namespace, extra: list[str], run_dir: Path
) -> list[str]:
    """The ``serve`` command line for one mission of the launcher."""
    command = [
        sys.executable, "-m", "long_tamp.mission", "serve",
        "--domain", args.domain,
        "--seed", str(args.seed),
        "--backend", args.backend,
        "--port", str(args.port),
        "--scene-port", str(args.scene_port),
        "--watchdog", args.watchdog,
        "--run-dir", str(run_dir),
    ]  # fmt: skip
    if args.model:
        command += ["--model", args.model]
    return command + extra


def _run(args, extra, run_dir: Path, opened: bool) -> tuple[int, bool]:
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "ui.out"
    print(f"mission UI: starting (log: {log_path})", flush=True)
    with open(log_path, "w") as log:
        # No stdin: the chat is the page's. Output goes to the log; the lines
        # an operator needs are echoed here.
        child = subprocess.Popen(
            serve_command(args, extra, run_dir),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert child.stdout is not None
        try:
            for line in child.stdout:
                log.write(line)
                log.flush()
                if line.startswith("web viewer: "):
                    url = line.split(": ", 1)[1].strip()
                    print(f"\n  Open the mission UI: {url}\n", flush=True)
                    if not (opened or args.no_open or in_container()):
                        webbrowser.open(url)
                    opened = True
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
            child.wait()
            print(f"mission UI: ended; run folder: {run_dir}", flush=True)
            return 130, opened
        code = child.wait()
    if code != RESTART:
        print(f"mission UI: ended ({code}); run folder: {run_dir}", flush=True)
    return code, opened


__all__ = [
    "RESTART",
    "launch",
    "load_domain",
    "parse_option",
    "parse_watchdog",
    "serve",
    "serve_command",
]
