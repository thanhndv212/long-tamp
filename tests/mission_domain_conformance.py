"""Static conformance to ``MissionDomain``, checked by mypy (#131).

Not a test module: ``test_mission_domain.py`` runs mypy on this file, which
fails if a reference implementation stops matching the protocol.
"""

from __future__ import annotations

from screw_mission import ScrewAssemblyDomain
from test_mission_domain import PickPlaceDomain

from long_tamp.mission import MissionDomain

screw: MissionDomain = ScrewAssemblyDomain(2)
toy: MissionDomain = PickPlaceDomain()
