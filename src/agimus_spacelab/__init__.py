"""
Agimus Spacelab - Manipulation Planning Framework

A generalized manipulation planning framework supporting both CORBA and PyHPP backends.
"""

__version__ = "0.1.0"
__author__ = "Thanh Nguyen"
__email__ = "dvtnguyen@laas.fr"
__license__ = "LGPL-3.0"

# Import version info
from .version import __version__  # noqa: F811

# Check backend availability
_HAS_CORBA = False
_HAS_PYHPP = False

try:
    import hpp.corbaserver  # noqa: F401
    _HAS_CORBA = True
except ImportError:
    pass

try:
    import pyhpp  # noqa: F401
    _HAS_PYHPP = True
except ImportError:
    pass


def get_available_backends():
    """
    Get list of available backends.
    
    Returns:
        list: List of available backend names ('corba', 'pyhpp')
    """
    backends = []
    if _HAS_CORBA:
        backends.append("corba")
    if _HAS_PYHPP:
        backends.append("pyhpp")
    return backends


# Import unified API after defining helper functions
from .planner import ManipulationPlanner  # noqa: F401, E402


__all__ = [
    "__version__",
    "ManipulationPlanner",
    "get_available_backends",
    "check_backend",
]
