"""
Backend implementations for agimus_spacelab manipulation planning.

This module provides unified access to backend implementations:
- PyHPP: Uses hpp-python for direct Python bindings

Usage:
    from agimus_spacelab.backends import PyHPPBackend
    from agimus_spacelab.backends import get_available_backends, BackendBase
"""

from .base import BackendBase, ConstraintResult

HAS_PYHPP = False

try:
    from .pyhpp import HAS_PYHPP, PyHPPBackend
except ImportError:
    pass


def get_available_backends():
    """Get list of available backend names.

    Returns:
        List of available backend names ('pyhpp',)
    """
    backends = []
    if HAS_PYHPP:
        backends.append("pyhpp")
    return backends


def get_backend(name: str = "auto"):
    """Get a backend by name.

    Args:
        name: Backend name ('pyhpp' or 'auto')
              'auto' will return the first available backend

    Returns:
        Backend class

    Raises:
        ImportError: If requested backend is not available
    """
    if name in ("auto", "pyhpp"):
        if not HAS_PYHPP:
            raise ImportError(
                "PyHPP backend not available. Please install hpp-python."
            )
        return PyHPPBackend
    else:
        raise ValueError(f"Unknown backend: {name}")


__all__ = [
    # Availability flags
    "HAS_PYHPP",
    # Base class
    "BackendBase",
    "ConstraintResult",
    # Backend implementations
    "PyHPPBackend",
    # Utility functions
    "get_available_backends",
    "get_backend",
]
