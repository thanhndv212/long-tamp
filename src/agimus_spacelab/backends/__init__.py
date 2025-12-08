"""
Backend implementations for agimus_spacelab manipulation planning.

This module provides unified access to different backend implementations:
- CORBA: Uses hpp-manipulation-corba for communication with HPP
- PyHPP: Uses hpp-python for direct Python bindings

Usage:
    from agimus_spacelab.backends import CorbaBackend, PyHPPBackend
    from agimus_spacelab.backends import get_available_backends, BackendBase
"""

from .base import BackendBase, ConstraintResult

# Import backend implementations
HAS_CORBA = False
HAS_PYHPP = False

try:
    from .corba import CorbaBackend, HAS_CORBA
except ImportError:
    pass

try:
    from .pyhpp import PyHPPBackend, HAS_PYHPP
except ImportError:
    pass


def get_available_backends():
    """Get list of available backend names.
    
    Returns:
        List of available backend names ('corba', 'pyhpp')
    """
    backends = []
    if HAS_CORBA:
        backends.append("corba")
    if HAS_PYHPP:
        backends.append("pyhpp")
    return backends


def get_backend(name: str = "auto"):
    """Get a backend by name.
    
    Args:
        name: Backend name ('corba', 'pyhpp', or 'auto')
              'auto' will return the first available backend
              
    Returns:
        Backend class
        
    Raises:
        ImportError: If requested backend is not available
    """
    if name == "auto":
        if HAS_CORBA:
            return CorbaBackend
        elif HAS_PYHPP:
            return PyHPPBackend
        else:
            raise ImportError(
                "No backends available. Install hpp-manipulation-corba "
                "or hpp-python."
            )
    elif name == "corba":
        if not HAS_CORBA:
            raise ImportError(
                "CORBA backend not available. "
                "Please install hpp-manipulation-corba."
            )
        return CorbaBackend
    elif name == "pyhpp":
        if not HAS_PYHPP:
            raise ImportError(
                "PyHPP backend not available. "
                "Please install hpp-python."
            )
        return PyHPPBackend
    else:
        raise ValueError(f"Unknown backend: {name}")


__all__ = [
    # Base class
    "BackendBase",
    "ConstraintResult",
    # Backend implementations
    "CorbaBackend",
    "PyHPPBackend",
    # Availability flags
    "HAS_CORBA",
    "HAS_PYHPP",
    # Utility functions
    "get_available_backends",
    "get_backend",
]
