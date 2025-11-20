"""
CORBA backend implementation for agimus_spacelab.
"""

from .corba_manipulation import (
    CorbaManipulationPlanner,
    SpacelabRobot,
    HAS_CORBA,
)

__all__ = [
    "CorbaManipulationPlanner",
    "SpacelabRobot",
    "HAS_CORBA",
]
