"""
CORBA backend implementation for agimus_spacelab.
"""

from .corba_manipulation import (
    CorbaManipulationPlanner,
    HAS_CORBA,
)

__all__ = [
    "CorbaManipulationPlanner",
    "HAS_CORBA",
]
