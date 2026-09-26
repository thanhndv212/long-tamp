"""Shared test setup.

Some tests import ``pyhpp`` directly. With the PyPI HPP wheels that only
works once long_tamp has loaded the HPP shared libraries (the wheels'
extension modules have no RPATH), so do it before any test is collected.
"""

from long_tamp.backends._hpp_libs import ensure_hpp_libraries

ensure_hpp_libraries()
