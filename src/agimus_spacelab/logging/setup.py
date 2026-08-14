"""
Python ``logging`` module configuration for agimus_spacelab.

Provides :func:`configure_logging` to set up file + console handlers, and
:func:`get_logger` to retrieve sub-module loggers under the
``agimus_spacelab`` namespace.

These are completely optional.  The :class:`~agimus_spacelab.logging.RunLogger`
structured logger operates independently and does not require this module.
"""

import logging
import os
import sys
from typing import Optional

_LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
_LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def configure_logging(
    level: int = logging.INFO,
    log_dir: Optional[str] = None,
    run_id: Optional[str] = None,
    console: bool = True,
    console_level: Optional[int] = None,
    file_level: Optional[int] = None,
) -> logging.Logger:
    """Configure the ``agimus_spacelab`` root logger.

    Sets up a ``StreamHandler`` (console) and, optionally, a ``FileHandler``
    writing to ``<log_dir>/<run_id>.log``.

    Calling this function multiple times is safe — duplicate handlers are
    skipped.

    Args:
        level: Default log level, used for both handlers unless overridden
            below (default: ``logging.INFO``).
        log_dir: If given, attach a ``FileHandler`` writing to
            ``<log_dir>/<run_id or "run">.log``.
        run_id: Run ID included in the log file name.
        console: Whether to attach a ``StreamHandler`` for stdout output
            (default: ``True``).
        console_level: Level for the console handler. Defaults to ``level``.
        file_level: Level for the file handler. Defaults to
            ``logging.DEBUG`` -- the log file is meant for postmortem
            debugging, so it stays fully detailed even when the console is
            quieted down via ``console_level``/``level``.

    Returns:
        The configured ``logging.Logger`` instance for ``"agimus_spacelab"``.
    """
    console_level = level if console_level is None else console_level
    file_level = logging.DEBUG if file_level is None else file_level

    logger = logging.getLogger("agimus_spacelab")
    logger.setLevel(min(console_level, file_level) if log_dir else console_level)

    # Skip if handlers already attached (idempotent)
    if logger.handlers:
        return logger

    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATE_FORMAT)

    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(console_level)
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    if log_dir is not None:
        os.makedirs(log_dir, exist_ok=True)
        fname = f"{run_id or 'run'}.log"
        fh = logging.FileHandler(os.path.join(log_dir, fname), encoding="utf-8")
        fh.setLevel(file_level)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    return logger


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """Return a logger under the ``agimus_spacelab`` namespace.

    Args:
        name: Sub-module name (e.g. ``"tasks.grasp_sequence"``).  Pass
            ``None`` to get the root package logger.

    Returns:
        A ``logging.Logger`` instance.

    Example::

        log = get_logger("planning.graph")
        log.debug("built %d states", n_states)
    """
    if name:
        return logging.getLogger(f"agimus_spacelab.{name}")
    return logging.getLogger("agimus_spacelab")
