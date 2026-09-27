"""Grasp planner: where a parallel-jaw gripper can hold an object, and how far it closes.

The motion planner (HPP, through ``long_tamp.planning``) answers "how does the
arm get the gripper frame onto the handle frame". Two questions come before
and after that, and they are this module's job:

1. **Which handle?** :meth:`GraspPlanner.plan` samples antipodal grasps on an
   object's collision primitives, rejects the ones the fingers or palm would
   collide on or that don't fit the stroke, scores the rest, and returns
   ranked :class:`GraspCandidate` poses -- each writable as an SRDF
   ``<handle>`` (:meth:`GraspCandidate.srdf_handle`) the motion planner can
   use unchanged.
2. **How far do the fingers close?** :meth:`GraspPlanner.evaluate` takes any
   grasp pose -- a planned candidate or an existing hand-written handle --
   and closes the fingers on the object: pad rays along the closing axis
   find the first contact. Because the TCP is held rigidly at the handle,
   both fingers close symmetrically and stop when the first one touches, so
   the closure is set by the *wider* side, and an off-centre grasp leaves
   one finger short of the surface (reported as ``offset``). The result
   carries finger joint values ready for the viewer or a gripper controller.

It is independent of HPP and of the constraint graph: a behaviour tree or
any orchestrator can call it as its own step (see
``long_tamp.tasks.task_planning.grasp_capability``), before motion planning
to pick handles and after it to command the gripper.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

import numpy as np

from .geometry import (
    Box,
    Cylinder,
    Primitive,
    Sphere,
    bounding_box,
    box_overlaps,
    centroid,
    line_intervals_batch,
    matrix_to_quat_xyzw,
    pose,
    pose_to_xyzquat,
)
from .gripper import ParallelGripperModel


@dataclass
class GraspEvaluation:
    """What happens when the gripper closes at one grasp pose.

    Attributes:
        feasible: Every check passed (see ``reasons`` otherwise).
        contact_width: Pad gap when the first finger touches (m).
        width: Commanded gap: ``contact_width`` minus the squeeze, clamped
            to the stroke.
        q: Driving joint value for ``width``.
        offset: Object's centre along the closing axis relative to the TCP
            (m). Non-zero means one pad touches before the other.
        contact_ratio: Fraction of pad rays that hit the object on the
            *contacting* side(s) -- how much of the pad area bears.
        collisions: Names of ``"<gripper box>:<primitive>"`` overlaps.
        reasons: Why the grasp is infeasible (empty when feasible).
    """

    feasible: bool
    contact_width: float = float("nan")
    width: float = float("nan")
    q: float = float("nan")
    offset: float = 0.0
    contact_ratio: float = 0.0
    collisions: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "feasible": self.feasible,
            "contact_width": _r(self.contact_width),
            "width": _r(self.width),
            "q": _r(self.q),
            "offset": _r(self.offset),
            "contact_ratio": round(self.contact_ratio, 3),
            "collisions": list(self.collisions),
            "reasons": list(self.reasons),
        }


@dataclass
class GraspCandidate:
    """A grasp pose on an object, in the object's root-link frame.

    ``pose`` is the canonical grasp frame (+X approach, +Y closing, origin at
    the TCP); :attr:`handle_pose` is the same grasp expressed as the frame
    the model's HPP ``<gripper>`` frame must coincide with.
    """

    pose: np.ndarray
    evaluation: GraspEvaluation
    score: float = 0.0
    source: str = ""
    handle_pose: np.ndarray = field(default_factory=lambda: np.eye(4))

    @property
    def approach(self) -> np.ndarray:
        return self.pose[:3, 0].copy()

    @property
    def closing(self) -> np.ndarray:
        return self.pose[:3, 1].copy()

    def srdf_handle(
        self,
        name: str,
        link: str = "base_link",
        clearance: float = 0.05,
        comment: str | None = None,
    ) -> str:
        """This grasp as an SRDF ``<handle>`` (approach along the frame's +X
        in the canonical frame, mapped through the model's frame rotation)."""
        xyz = " ".join(f"{v:.6g}" for v in self.handle_pose[:3, 3])
        q = matrix_to_quat_xyzw(self.handle_pose[:3, :3])
        xyzw = " ".join(f"{v:.7g}" for v in q)
        approach = self.handle_pose[:3, :3].T @ self.pose[:3, 0]
        ad = " ".join(f"{_clean(v):g}" for v in approach)
        note = comment or (
            f"planned grasp ({self.source}): width {self.evaluation.width * 1000:.1f} mm, "
            f"score {self.score:.3f}"
        )
        return (
            f"  <!-- {note} -->\n"
            f'  <handle name="{name}" clearance="{clearance}" approaching_direction="{ad}">\n'
            f'    <position xyz="{xyz}" xyzw="{xyzw}"/>\n'
            f'    <link name="{link}"/>\n'
            f"  </handle>\n"
        )

    def to_dict(self) -> dict:
        return {
            "source": self.source,
            "score": round(self.score, 4),
            "handle_xyzquat": [_r(v) for v in pose_to_xyzquat(self.handle_pose)],
            "approach": [_r(v) for v in self.approach],
            "closing": [_r(v) for v in self.closing],
            **self.evaluation.to_dict(),
        }


@dataclass
class GraspPlannerParams:
    """Sampling and scoring knobs for :class:`GraspPlanner`.

    Attributes:
        samples_along: Grasp positions sampled along each face's free axis.
        depths: Pad insertion depths, as fractions of the object's extent
            along the approach axis (0 = pads just past the entry face,
            0.5 = pads at the middle).
        cylinder_angles: Approach directions sampled around a cylinder/sphere.
        include_flipped: Also emit each grasp turned 180 deg about its
            approach axis (same fingers, mirrored wrist -- matters for IK).
        width_margin: Keep this much stroke spare on each side of the width
            (grasps closer to fully open than this are rejected).
        rays: Pad rays per axis (``rays x rays`` grid) when closing.
        max_offset: Largest tolerated off-centre (m); beyond it one finger
            ends up this far from the surface when the other touches.
        contact_tolerance: A pad ray within this distance of the contact
            surface counts as bearing (pad compliance; lets curved surfaces
            make contact over an area instead of a line or a point).
        min_contact_ratio: Least pad area that must bear on the object.
        approach_clearance: Distance swept by the open hand before the
            grasp; collisions along it reject the grasp.
        collision_margin: Clearance kept between the gripper boxes and the
            ``obstacles`` (not the grasped object, which the pads touch).
        preferred_approach: Object-frame direction the approach should
            align with (e.g. ``(0, 0, -1)`` for top-down), or ``None``.
        w_center, w_contact, w_margin, w_approach: Score weights: closeness
            to the object's centroid, pad contact, spare stroke, approach
            alignment.
    """

    samples_along: int = 5
    depths: tuple[float, ...] = (0.0, 0.25, 0.5)
    cylinder_angles: int = 8
    include_flipped: bool = True
    width_margin: float = 0.002
    rays: int = 5
    max_offset: float = 0.004
    contact_tolerance: float = 0.002
    min_contact_ratio: float = 0.1
    approach_clearance: float = 0.05
    collision_margin: float = 0.0
    preferred_approach: tuple[float, float, float] | None = None
    w_center: float = 1.0
    w_contact: float = 1.0
    w_margin: float = 0.5
    w_approach: float = 1.0


class GraspPlanner:
    """Plans and evaluates parallel-jaw grasps for one gripper model."""

    def __init__(
        self, gripper: ParallelGripperModel, params: GraspPlannerParams | None = None
    ):
        self.gripper = gripper
        self.params = params or GraspPlannerParams()

    # -- evaluation ---------------------------------------------------------

    def evaluate(
        self,
        grasp_pose: np.ndarray,
        primitives: Sequence[Primitive],
        obstacles: Sequence[Primitive] = (),
        squeeze: float | None = None,
        full: bool = True,
    ) -> GraspEvaluation:
        """Close the gripper at ``grasp_pose`` (canonical frame, object frame).

        ``obstacles`` are other bodies (a table, a fixture) in the object
        frame: the hand must not hit them, but the fingers don't close on
        them. With ``full=False`` the collision checks are skipped once the
        grasp has already failed (faster when sampling).
        """
        g, p = self.gripper, self.params
        squeeze = g.default_squeeze if squeeze is None else squeeze
        R, t = grasp_pose[:3, :3], grasp_pose[:3, 3]
        ax, ay, az = R[:, 0], R[:, 1], R[:, 2]
        open_half = g.max_width / 2
        reasons: list[str] = []

        # The pads slide forward along the approach while closing (13 mm over
        # the 2F-85's stroke), so cast once at the open depth, then again at
        # the depth for the width found.
        n = max(2, p.rays)
        zs = np.linspace(-g.pad_width / 2, g.pad_width / 2, n)
        pad_x = g.pad_x_at(g.q_open)
        for _ in range(2):
            xs = np.linspace(pad_x - g.pad_length / 2, pad_x + g.pad_length / 2, n)
            X, Z = np.meshgrid(xs, zs, indexing="ij")
            origins = t + np.outer(X.ravel(), ax) + np.outer(Z.ravel(), az)
            T0, T1 = line_intervals_batch(primitives, origins, ay)
            # Only what lies between the open fingers can be closed on.
            inside = (T1 > -open_half) & (T0 < open_half)
            T0, T1 = np.where(inside, T0, np.nan), np.where(inside, T1, np.nan)
            blocked = bool(np.any(inside & ((T0 <= -open_half) | (T1 >= open_half))))
            hit = np.any(inside, axis=0)
            hi_hits = np.nanmax(T1[:, hit], axis=0) if hit.any() else np.empty(0)
            lo_hits = np.nanmin(T0[:, hit], axis=0) if hit.any() else np.empty(0)
            if not len(hi_hits):
                break
            pad_x = g.pad_x_for_width(2 * max(hi_hits.max(), -lo_hits.min()))
        if not len(hi_hits):
            return GraspEvaluation(False, reasons=["fingers close on nothing"])
        if blocked:
            reasons.append("object wider than the open gripper")
        y_hi, y_lo = float(hi_hits.max()), float(lo_hits.min())
        half = max(y_hi, -y_lo)
        contact_width = 2 * half
        offset = (y_hi + y_lo) / 2
        tol = p.contact_tolerance
        side_hi = int(np.sum(hi_hits >= half - tol))
        side_lo = int(np.sum(-lo_hits >= half - tol))
        contact_ratio = max(side_hi, side_lo) / float(n * n)
        if abs(offset) > p.max_offset:
            reasons.append(f"off-centre by {offset * 1000:.1f} mm")
        if contact_ratio < p.min_contact_ratio:
            reasons.append(f"pad contact {contact_ratio:.0%}")
        if contact_width > g.max_width - 2 * p.width_margin:
            reasons.append(f"width {contact_width * 1000:.1f} mm exceeds the stroke")
        width = float(np.clip(contact_width - squeeze, g.min_width, g.max_width))
        q = g.q_for_width(width)
        result = GraspEvaluation(
            feasible=False,
            contact_width=contact_width,
            width=width,
            q=q,
            offset=offset,
            contact_ratio=contact_ratio,
            reasons=reasons,
        )
        if reasons and not full:
            return result

        # Collisions: the open hand sweeping in, then the closed hand. The
        # closed fingers' inner faces sit at the contact, so shrink them by a
        # hair (they touch, they don't overlap).
        collisions: list[str] = []
        checks = [
            (g.max_width, p.approach_clearance, 0.0, "open"),
            (contact_width, 0.0, 1e-4, "closed"),
        ]
        for w, sweep, inset, tag in checks:
            for name, center, hext in g.collision_boxes(w, sweep=sweep):
                c = center.copy()
                h = hext.copy()
                if inset and name.startswith("finger"):
                    sign = 1.0 if "right" in name else -1.0
                    c[1] += sign * inset / 2
                    h[1] = max(h[1] - inset / 2, 1e-6)
                box_pose = pose(R, t + R @ c)
                for body_set, kind, margin in (
                    (primitives, "object", 0.0),
                    (obstacles, "obstacle", p.collision_margin),
                ):
                    for prim in body_set:
                        if box_overlaps(prim, box_pose, h, margin=margin):
                            label = f"{tag} {name}:{prim.name or kind}"
                            if label not in collisions:
                                collisions.append(label)
        if collisions:
            reasons.append("collision: " + ", ".join(collisions[:4]))
        result.collisions = collisions
        result.feasible = not reasons
        return result

    def evaluate_handle(
        self,
        handle_pose: np.ndarray,
        primitives: Sequence[Primitive],
        obstacles: Sequence[Primitive] = (),
        squeeze: float | None = None,
    ) -> GraspEvaluation:
        """Evaluate an existing SRDF handle (the pose the ``<gripper>`` frame
        lands on, object frame): converts it to the canonical frame first."""
        return self.evaluate(
            self.canonical_from_handle(handle_pose), primitives, obstacles, squeeze
        )

    def canonical_from_handle(self, handle_pose: np.ndarray) -> np.ndarray:
        """Gripper-frame pose -> canonical grasp pose (same TCP)."""
        out = handle_pose.copy()
        out[:3, :3] = handle_pose[:3, :3] @ self.gripper.frame_rotation.T
        return out

    def handle_from_canonical(self, grasp_pose: np.ndarray) -> np.ndarray:
        out = grasp_pose.copy()
        out[:3, :3] = grasp_pose[:3, :3] @ self.gripper.frame_rotation
        return out

    # -- planning -----------------------------------------------------------

    def plan(
        self,
        primitives: Sequence[Primitive],
        obstacles: Sequence[Primitive] = (),
        max_candidates: int | None = 20,
        feasible_only: bool = True,
    ) -> list[GraspCandidate]:
        """Sample, evaluate and rank grasps on an object's primitives.

        Returns the best ``max_candidates`` (all if ``None``), highest score
        first. With ``feasible_only=False`` rejected samples are kept at the
        end with their ``reasons`` -- useful to see why an object has none.
        """
        p = self.params
        lo, hi = bounding_box(primitives)
        size = float(np.linalg.norm(hi - lo)) or 1.0
        com = centroid(primitives)
        pref = None
        if p.preferred_approach is not None:
            pref = np.asarray(p.preferred_approach, dtype=float)
            pref /= np.linalg.norm(pref)
        out: list[GraspCandidate] = []
        seen: set[tuple] = set()
        for grasp_pose, source in self._samples(primitives):
            key = tuple(np.round(grasp_pose[:3, :].ravel(), 4))
            if key in seen:
                continue
            seen.add(key)
            ev = self.evaluate(
                grasp_pose, primitives, obstacles, full=not feasible_only
            )
            if feasible_only and not ev.feasible:
                continue
            score = self._score(grasp_pose, ev, com, size, pref)
            out.append(
                GraspCandidate(
                    pose=grasp_pose,
                    evaluation=ev,
                    score=score,
                    source=source,
                    handle_pose=self.handle_from_canonical(grasp_pose),
                )
            )
        out.sort(key=lambda c: (c.evaluation.feasible, c.score), reverse=True)
        return out if max_candidates is None else out[:max_candidates]

    def _score(self, grasp_pose, ev: GraspEvaluation, com, size, pref) -> float:
        g, p = self.gripper, self.params
        if not ev.feasible:
            return 0.0
        center = 1.0 - min(
            1.0, float(np.linalg.norm(grasp_pose[:3, 3] - com)) / (size / 2)
        )
        spare = (g.max_width - ev.contact_width) / max(g.max_width - g.min_width, 1e-9)
        margin = min(1.0, spare / 0.25)  # full marks from a quarter stroke spare
        align = 0.0 if pref is None else max(0.0, float(grasp_pose[:3, 0] @ pref))
        return (
            p.w_center * center
            + p.w_contact * ev.contact_ratio
            + p.w_margin * margin
            + p.w_approach * align
        )

    # -- sampling -----------------------------------------------------------

    def _frame(self, approach, closing, pad_center, width) -> np.ndarray:
        """Canonical grasp pose whose pads, closed to ``width``, centre on
        ``pad_center``."""
        a = np.asarray(approach, dtype=float)
        a /= np.linalg.norm(a)
        c = np.asarray(closing, dtype=float)
        c = c - (c @ a) * a
        c /= np.linalg.norm(c)
        R = np.column_stack([a, c, np.cross(a, c)])
        pad_x = self.gripper.pad_x_for_width(width)
        return pose(R, np.asarray(pad_center) - pad_x * a)

    def _emit(self, approach, closing, pad_center, width, source):
        yield self._frame(approach, closing, pad_center, width), source
        if self.params.include_flipped:
            flipped = -np.asarray(closing)
            yield self._frame(approach, flipped, pad_center, width), source + " flipped"

    def _samples(
        self, primitives: Sequence[Primitive]
    ) -> Iterable[tuple[np.ndarray, str]]:
        for prim in primitives:
            if isinstance(prim, Box):
                yield from self._box_samples(prim)
            elif isinstance(prim, Cylinder):
                yield from self._cylinder_samples(prim)
            elif isinstance(prim, Sphere):
                yield from self._sphere_samples(prim)

    def _along(self, half: float, reserve: float) -> np.ndarray:
        span = max(half - reserve, 0.0)
        n = self.params.samples_along if span > 1e-4 else 1
        return np.linspace(-span, span, n) if n > 1 else np.zeros(1)

    def _depths(self, half: float) -> list[float]:
        """Pad-centre coordinates along the approach axis, entry face at -half."""
        L = self.gripper.pad_length
        # Pads just past the entry face at f=0, centred on the shape at f=0.5.
        out = {
            round(-half + min(L / 2, half) + f * max(2 * half - L, 0.0), 6)
            for f in self.params.depths
        }
        return sorted(out)

    def _box_samples(self, box: Box):
        g = self.gripper
        R, t = box.pose[:3, :3], box.pose[:3, 3]
        h = box.half_extents()
        name = box.name or "box"
        for c in range(3):
            if 2 * h[c] > g.max_width - 2 * self.params.width_margin:
                continue
            for a in range(3):
                if a == c:
                    continue
                b = 3 - a - c
                for sign in (1.0, -1.0):
                    approach = sign * R[:, a]
                    for s in self._along(h[b], g.pad_width / 2):
                        for d in self._depths(h[a]):
                            local = np.zeros(3)
                            local[b] = s
                            # d runs along the approach; entry face at d = -h[a].
                            local[a] = sign * d
                            center = t + R @ local
                            yield from self._emit(
                                approach,
                                R[:, c],
                                center,
                                2 * h[c],
                                f"{name}: close {'xyz'[c]}, approach {'+' if sign > 0 else '-'}{'xyz'[a]}",
                            )

    def _cylinder_samples(self, cyl: Cylinder):
        g, p = self.gripper, self.params
        R, t = cyl.pose[:3, :3], cyl.pose[:3, 3]
        axis = R[:, 2]
        name = cyl.name or "cylinder"
        angles = np.linspace(0, 2 * math.pi, p.cylinder_angles, endpoint=False)
        fits_diameter = 2 * cyl.radius <= g.max_width - 2 * p.width_margin
        fits_length = cyl.length <= g.max_width - 2 * p.width_margin
        for th in angles:
            radial = R @ np.array([math.cos(th), math.sin(th), 0.0])
            tangent = np.cross(axis, radial)
            if fits_diameter:
                # Side grasp: approach radially inward, squeeze the diameter.
                for s in self._along(cyl.length / 2, g.pad_width / 2):
                    for d in self._depths(cyl.radius):
                        # d runs along the approach (-radial).
                        center = t + s * axis - d * radial
                        yield from self._emit(
                            -radial, tangent, center, 2 * cyl.radius, f"{name}: side"
                        )
                # End grasp: approach along the axis, squeeze the diameter.
                for sign in (1.0, -1.0):
                    for d in self._depths(cyl.length / 2):
                        center = t + sign * d * axis
                        yield from self._emit(
                            sign * axis, radial, center, 2 * cyl.radius, f"{name}: end"
                        )
            if fits_length:
                # Edge grasp (a disc): approach radially, squeeze the faces.
                for d in self._depths(cyl.radius):
                    center = t - d * radial
                    yield from self._emit(
                        -radial, axis, center, cyl.length, f"{name}: edge"
                    )

    def _sphere_samples(self, sph: Sphere):
        g, p = self.gripper, self.params
        if 2 * sph.radius > g.max_width - 2 * p.width_margin:
            return
        n = max(6, p.cylinder_angles * 2)
        golden = math.pi * (3 - math.sqrt(5))
        for i in range(n):
            z = 1 - 2 * (i + 0.5) / n
            r = math.sqrt(1 - z * z)
            approach = np.array([r * math.cos(golden * i), r * math.sin(golden * i), z])
            helper = (
                np.array([0.0, 0.0, 1.0]) if abs(z) < 0.9 else np.array([1.0, 0.0, 0.0])
            )
            closing = np.cross(approach, helper)
            yield from self._emit(
                approach,
                closing,
                sph.center,
                2 * sph.radius,
                f"{sph.name or 'sphere'}: through centre",
            )


def _r(v: float) -> float | None:
    return (
        None
        if v is None or (isinstance(v, float) and math.isnan(v))
        else round(float(v), 5)
    )


def _clean(v: float) -> float:
    v = round(float(v), 6)
    return 0.0 if v == 0 else v
