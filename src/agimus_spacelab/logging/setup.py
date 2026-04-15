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
) -> logging.Logger:
    """Configure the ``agimus_spacelab`` root logger.

    Sets up a ``StreamHandler`` (console) and, optionally, a ``FileHandler``
    writing to ``<log_dir>/<run_id>.log``.

    Calling this function multiple times is safe — duplicate handlers are
    skipped.

    Args:
        level: Log level for the root ``agimus_spacelab`` logger
            (default: ``logging.INFO``).
        log_dir: If given, attach a ``FileHandler`` writing to
            ``<log_dir>/<run_id or "run">.log``.
        run_id: Run ID included in the log file name.
        console: Whether to attach a ``StreamHandler`` for stdout output
            (default: ``True``).

    Returns:
        The configured ``logging.Logger`` instance for ``"agimus_spacelab"``.
    """
    logger = logging.getLogger("agimus_spacelab")
    logger.setLevel(level)

    # Skip if handlers already attached (idempotent)
    if logger.handlers:
        return logger

    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATE_FORMAT)

    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(level)
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    if log_dir is not None:
        os.makedirs(log_dir, exist_ok=True)
        fname = f"{run_id or 'run'}.log"
        fh = logging.FileHandler(
            os.path.join(log_dir, fname), encoding="utf-8"
        )
        fh.setLevel(level)
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
