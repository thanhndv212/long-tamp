"""The template mission domain (#135): two objects, one arm, three regions.

The smallest complete :class:`~long_tamp.mission.MissionDomain`, to copy
for a new robot or task (the tutorial, ``docs/usage/write-your-own-mission.md``,
walks through it). It runs the whole mission loop (instruction, plan,
execute, repair) with no HPP: its scene is kinematic. The arm's tool point
moves in straight lines between the objects and the regions, and a carried
object moves with it. Swap :class:`KinematicScene` for an HPP scene (see the
screw assembly, ``script/screw_assembly/screw_mission.py``) and the mock
backend for a simulator or a robot; the domain's symbolic half stays.

    python -m long_tamp.mission ui --domain long_tamp.mission.template:domain
    python -m long_tamp.mission ui --domain long_tamp.mission.template:repair_demo

``repair_demo`` can't reach the tray: a place there fails, and the repair
loop blocks that binding and replans (a goal the tray alone satisfies then
has no plan; "take the cube off the table" ends in the bin).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from long_tamp.tasks.refiner import Refinement, RefinementStep
from long_tamp.tasks.task_planning.capabilities import CapabilityDescriptor

from .domain import BaseMissionDomain, MissionScene, MissionStep
from .goals import grasped, placed_at

#: The arm's gripper (``arm/gripper``, as an HPP gripper would be named).
GRIPPER = "arm/gripper"

#: Where each region is, (x, y, z) in metres.
REGIONS: dict[str, tuple[float, float, float]] = {
    "table": (0.40, 0.00, 0.00),
    "tray": (0.30, 0.30, 0.05),
    "bin": (0.30, -0.30, 0.10),
}

#: The objects, in their start region.
OBJECTS: dict[str, str] = {"cube": "table", "ball": "table"}

#: The arm's tool point at the start, (x, y, z).
HOME = (0.0, 0.0, 0.40)


def handle_of(obj: str) -> str:
    """The handle the gripper grasps ``obj`` by (``object/handle``)."""
    return f"{obj}/handle"


# -- the kinematic scene -------------------------------------------------------


@dataclass
class GraspTracker:
    """What each gripper holds (gripper -> handle, None if empty): what the
    runner reads from a planner (``planner.grasp_tracker``)."""

    current_grasps: dict[str, str | None] = field(default_factory=dict)

    def update_grasp(self, gripper: str, handle: str | None) -> None:
        self.current_grasps[gripper] = handle


class KinematicPlanner:
    """The scene's "planner": only the grasp tracker, and the grasps at the
    start of the step being refined (to undo a failed one)."""

    def __init__(self) -> None:
        self.grasp_tracker = GraspTracker({GRIPPER: None})
        self._call_start: dict[str, str | None] = {}

    def reset_grasp_tracker_to_call_start(self) -> None:
        self.grasp_tracker.current_grasps = dict(self._call_start)


class StraightPath:
    """From ``a`` to ``b`` at ``speed`` m/s of the tool point; what the
    runner needs of a path (``length``, ``eval``)."""

    def __init__(self, a: Sequence[float], b: Sequence[float], speed: float = 0.25):
        self.a, self.b = list(a), list(b)
        distance = sum((y - x) ** 2 for x, y in zip(self.a[:3], self.b[:3])) ** 0.5
        self._length = max(distance / speed, 0.1)

    def length(self) -> float:
        return self._length

    def eval(self, t: float) -> tuple[list[float], bool]:
        s = min(max(t / self._length, 0.0), 1.0)
        return [x + (y - x) * s for x, y in zip(self.a, self.b)], True


class KinematicScene:
    """The configuration and the refiner of the template.

    A configuration is the tool point (x, y, z) followed by each object's
    position (x, y, z), in ``objects`` order. Refining a step moves the tool
    point to the object (pick) or to a free slot of the region (place), the
    carried object along with it. A place in an ``unreachable`` region
    fails with ``cannot_reach(gripper, region)``.
    """

    def __init__(
        self,
        objects: Mapping[str, str],
        regions: Mapping[str, tuple[float, float, float]],
        unreachable: Iterable[str] = (),
    ) -> None:
        self.objects = list(objects)
        self.regions = dict(regions)
        self.unreachable = set(unreachable)
        self.planner = KinematicPlanner()
        start = [list(HOME)]
        for i, obj in enumerate(self.objects):
            start.append(self.slot(objects[obj], i))
        self.q_init = [v for point in start for v in point]

    def slot(self, region: str, i: int) -> list[float]:
        """The ``i``-th object's place in ``region`` (side by side)."""
        x, y, z = self.regions[region]
        return [x, y + 0.08 * (i - (len(self.objects) - 1) / 2), z]

    def position(self, q: Sequence[float], obj: str) -> list[float]:
        i = 3 * (1 + self.objects.index(obj))
        return list(q[i : i + 3])

    def refine(self, step: RefinementStep, q_init: Sequence[float]) -> Refinement:
        """The :class:`~long_tamp.tasks.refiner.Refiner` protocol."""
        grasps = self.planner.grasp_tracker.current_grasps
        self.planner._call_start = dict(grasps)
        q, phases = list(q_init), []
        obj: str
        for gripper, handle in step.sequence:
            if handle is not None:  # pick: reach the object, close
                obj = handle.split("/")[0]
                target = self.position(q, obj)
            else:  # place: carry what is held to the step's region
                held = grasps.get(gripper)
                if held is None:
                    return self._failed(step, q_init, f"{gripper} holds nothing")
                obj = held.split("/")[0]
                region = step.label.rsplit(" ", 1)[-1]
                if region in self.unreachable:
                    return self._failed(
                        step,
                        q_init,
                        f"{region} is out of reach",
                        [f"cannot_reach({gripper}, {region})"],
                    )
                target = self.slot(region, self.objects.index(obj))
            end = list(q)
            end[0:3] = target
            if handle is None:
                i = 3 * (1 + self.objects.index(obj))
                end[i : i + 3] = target
            phases.append(
                {
                    "gripper": gripper,
                    "handle": handle,
                    "paths": [StraightPath(q, end)],
                    "edges": ["grasp_12" if handle else "release_21"],
                }
            )
            grasps[gripper] = handle
            q = end
        return Refinement(True, step.label, q, phases=phases, message="ok")

    def _failed(
        self,
        step: RefinementStep,
        q_init: Sequence[float],
        message: str,
        facts: Sequence[str] = (),
    ) -> Refinement:
        self.planner.reset_grasp_tracker_to_call_start()
        return Refinement(
            False, step.label, list(q_init), message=message, facts=list(facts)
        )


# -- the domain ----------------------------------------------------------------


class TemplateDomain(BaseMissionDomain):
    """Two objects moved between regions by one arm.

    The symbolic half is what a task planner and the chat see: the
    capabilities ``pick`` and ``place`` over the shared object-level
    predicates ``grasped(gripper, object)`` and ``placed_at(object,
    region)``. The geometric half is ``setup`` (the scene) and
    ``step_for`` (what each capability is as a grasp sequence).
    """

    name = "template"
    recorded_predicates = frozenset({"placed_at"})
    goal_notes = (
        "placed_at(object, region) holds once a place has run; grasped(gripper, "
        "object) while the gripper holds it."
    )
    intro = (
        "One arm (gripper arm/gripper) moves a cube and a ball between three "
        "regions: the table (where both start), a tray and a bin."
    )

    def __init__(
        self,
        objects: Mapping[str, str] | None = None,
        regions: Mapping[str, tuple[float, float, float]] | None = None,
        unreachable: Iterable[str] = (),
    ) -> None:
        self._objects = dict(OBJECTS if objects is None else objects)
        self._regions = dict(REGIONS if regions is None else regions)
        self.unreachable = tuple(unreachable)

    # -- symbolic --------------------------------------------------------------

    def descriptors(self) -> dict[str, CapabilityDescriptor]:
        pick = CapabilityDescriptor(
            "pick",
            "1.0",
            {"gripper": str, "object": str, "region": str},
            preconditions=("not grasped(?gripper, _)", "placed_at(?object, ?region)"),
            effects=("grasped(?gripper, ?object)", "not placed_at(?object, ?region)"),
            writes=("grasp_state",),
            restartable=True,
        )
        place = CapabilityDescriptor(
            "place",
            "1.0",
            {"gripper": str, "object": str, "region": str},
            preconditions=("grasped(?gripper, ?object)",),
            effects=("not grasped(?gripper, ?object)", "placed_at(?object, ?region)"),
            writes=("grasp_state",),
            restartable=True,
        )
        return {d.capability_id: d for d in (pick, place)}

    def objects(self) -> tuple[str, ...]:
        return tuple(self._objects)

    def regions(self) -> tuple[str, ...]:
        return tuple(self._regions)

    def static_facts(self) -> tuple[str, ...]:
        return (f"gripper({GRIPPER})", *(f"region({r})" for r in self._regions))

    def static_preconditions(self) -> dict[str, tuple[str, ...]]:
        # What the planner may bind the parameters to (PDDL export only).
        return {
            "pick": ("gripper(?gripper)",),
            "place": ("gripper(?gripper)", "region(?region)"),
        }

    def default_goal(self) -> list[str]:
        return [
            *(placed_at(obj, "tray") for obj in self._objects),
            *(grasped(obj, negate=True) for obj in self._objects),
        ]

    def scene_info(self) -> dict[str, Any]:
        return {"id": self.name, "objects": list(self._objects)}

    # -- skeleton -> geometry --------------------------------------------------

    def step_for(self, capability: str, parameters: Mapping[str, Any]) -> MissionStep:
        """A pick grasps the object's handle; a place releases it. The
        region is the label's last word, which the scene reads."""
        obj, region = parameters["object"], parameters["region"]
        gripper = parameters.get("gripper", GRIPPER)
        if capability == "pick":
            return RefinementStep(f"pick {obj} {region}", ((gripper, handle_of(obj)),))
        return RefinementStep(f"place {obj} {region}", ((gripper, None),))

    def setup(self, seed: int | None = None, log_dir: str = "auto") -> MissionScene:
        scene = KinematicScene(self._objects, self._regions, self.unreachable)
        return MissionScene(
            task=None,
            planner=scene.planner,
            q_init=scene.q_init,
            extras={
                "refiner": scene,
                "facts": [placed_at(o, r) for o, r in self._objects.items()],
                "kinematics": scene,
            },
        )


def domain() -> TemplateDomain:
    """``--domain long_tamp.mission.template:domain``."""
    return TemplateDomain()


def repair_demo() -> TemplateDomain:
    """The template with the tray out of reach: plans that place there fail
    and are repaired (``--domain long_tamp.mission.template:repair_demo``)."""
    return TemplateDomain(unreachable=("tray",))


__all__ = [
    "GRIPPER",
    "KinematicScene",
    "TemplateDomain",
    "domain",
    "repair_demo",
]
