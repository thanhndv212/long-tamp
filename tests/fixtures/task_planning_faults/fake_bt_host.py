#!/usr/bin/env python3
"""Fake ``agimus_taskplan_bt`` executable used by supervisor regression tests.

Behavior is selected entirely via the ``FAKE_BT_MODE`` environment variable
so the real BT host binary and PyHPP are never involved. Ignores its CLI
arguments (``--factory``/``--options``), matching only what
``run_taskplan_bt_supervised.py`` actually depends on: an exit code (or a
hang, to exercise the timeout/kill path).

Modes:
  exit0                - immediate clean exit, code 0.
  exit1                - immediate deterministic-failure exit, code 1.
  exit2                - immediate non-retryable-contract exit, code 2.
  crash                - self-delivers SIGSEGV (negative returncode).
  hang_respects_sigterm    - sleeps; exits promptly on SIGTERM (default
                              handler), exercising the graceful-kill path.
  hang_ignores_sigterm     - sleeps and ignores SIGTERM, forcing the
                              supervisor's SIGKILL fallback after its grace
                              period.
  fail_then_succeed    - crashes (code 139) on the first invocation (using
                          FAKE_BT_MARKER to remember state across restarts)
                          then exits 0 on the next, exercising the
                          retry-then-succeed path.
"""

from __future__ import annotations

import os
import signal
import sys
import time


def main() -> int:
    mode = os.environ.get("FAKE_BT_MODE", "exit0")
    # Record our own pid so the test can assert we are truly gone after a
    # supervisor-issued kill (no zombie/leak left behind).
    pid_file = os.environ.get("FAKE_BT_PID_FILE")
    if pid_file:
        with open(pid_file, "w", encoding="utf-8") as stream:
            stream.write(str(os.getpid()))

    if mode == "exit0":
        return 0
    if mode == "exit1":
        return 1
    if mode == "exit2":
        return 2
    if mode == "crash":
        os.kill(os.getpid(), signal.SIGSEGV)
        return 70  # unreachable; self-signal terminates the process first
    if mode == "hang_respects_sigterm":
        time.sleep(60)
        return 0
    if mode == "hang_ignores_sigterm":
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        time.sleep(60)
        return 0
    if mode == "fail_then_succeed":
        marker = os.environ["FAKE_BT_MARKER"]
        if os.path.exists(marker):
            return 0
        with open(marker, "w", encoding="utf-8"):
            pass
        return 139  # simulate a non-1/non-2/non-0 native-crash-like exit
    raise SystemExit(f"unknown FAKE_BT_MODE: {mode}")


if __name__ == "__main__":
    sys.exit(main())
