"""
Unit tests for ``agimus_spacelab.logging.setup.configure_logging``.

These tests require no HPP backend — they only exercise the pure-Python
logging infrastructure. Each test uses a fresh logger name (via
``logging.getLogger("agimus_spacelab").manager.loggerDict`` cleanup) so
``configure_logging``'s idempotent "skip if handlers already attached"
behavior from an earlier test doesn't leak into the next one.
"""

import logging
import os
import tempfile

import pytest

from agimus_spacelab.logging import configure_logging


@pytest.fixture(autouse=True)
def _reset_agimus_spacelab_logger():
    """configure_logging() is idempotent (skips re-attaching handlers), so
    without this, whichever test runs first would pin the handler config
    for every test after it in the same process."""
    logger = logging.getLogger("agimus_spacelab")
    saved_handlers = list(logger.handlers)
    saved_level = logger.level
    logger.handlers = []
    yield
    for h in logger.handlers:
        h.close()
    logger.handlers = saved_handlers
    logger.setLevel(saved_level)


class TestConfigureLoggingConsoleFileSplit:
    def test_default_level_applies_to_both_handlers(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = configure_logging(level=logging.INFO, log_dir=tmp, run_id="r")
            console_handlers = [
                h for h in logger.handlers if isinstance(h, logging.StreamHandler)
                and not isinstance(h, logging.FileHandler)
            ]
            file_handlers = [
                h for h in logger.handlers if isinstance(h, logging.FileHandler)
            ]
            assert len(console_handlers) == 1
            assert len(file_handlers) == 1
            # Backward-compat: omitting console_level reproduces today's
            # console behavior -- console still defaults to `level`.
            assert console_handlers[0].level == logging.INFO
            # The file handler is deliberately NOT backward-compatible here
            # -- it now always defaults to DEBUG regardless of `level`, so
            # postmortem detail is never lost just because the console
            # level wasn't explicitly raised.
            assert file_handlers[0].level == logging.DEBUG

    def test_console_level_overrides_default_without_touching_file_level(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = configure_logging(
                log_dir=tmp, run_id="r", console_level=logging.WARNING,
            )
            console_handler = next(
                h for h in logger.handlers if isinstance(h, logging.StreamHandler)
                and not isinstance(h, logging.FileHandler)
            )
            file_handler = next(
                h for h in logger.handlers if isinstance(h, logging.FileHandler)
            )
            assert console_handler.level == logging.WARNING
            # file_level defaults to DEBUG regardless of console_level, so
            # postmortem debugging never loses detail just because the
            # console was quieted down.
            assert file_handler.level == logging.DEBUG

    def test_quiet_console_still_writes_full_detail_to_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = configure_logging(
                log_dir=tmp, run_id="r", console_level=logging.WARNING,
            )
            probe = logging.getLogger("agimus_spacelab.some.module")
            probe.info("this should reach the file but not raise console level")

            log_path = os.path.join(tmp, "r.log")
            for h in logger.handlers:
                h.flush()
            with open(log_path) as f:
                contents = f.read()
            assert "this should reach the file but not raise console level" in contents

    def test_explicit_file_level_is_respected(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = configure_logging(
                log_dir=tmp, run_id="r", file_level=logging.ERROR,
            )
            file_handler = next(
                h for h in logger.handlers if isinstance(h, logging.FileHandler)
            )
            assert file_handler.level == logging.ERROR
