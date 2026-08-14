#!/usr/bin/env python3
"""
Reading back what :mod:`agimus_spacelab.planning.path_recorder` captured.

A captured run is a manifest plus one JSON per segment, each holding a
sampled configuration array.  Nothing here touches a constraint graph:
that is the whole point of sampling at capture time, and it is what makes
playback a pure visualization problem rather than a graph-reconstruction
one.

Two things live here:

* :func:`load_manifest` / :class:`Segment` — the reader, which resolves
  each record to its waypoints on demand rather than pulling a whole run
  into memory at once.
* :func:`validate` — an independent continuity check.  Independent
  matters: the recorder wrote ``seam_error`` from the *path objects* it
  was handed, so re-deriving the seams from the *files on disk* is the
  only thing that proves the artifact itself is playable.  It also
  catches what the recorder could not see — a segment file that never
  landed, a configuration size that changed mid-run, and jumps *inside* a
  segment, which no seam check looks at.
"""

from __future__ import annotations

import json
import os
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from typing import Any

from agimus_spacelab.logging import get_logger
from agimus_spacelab.planning.path_recorder import MANIFEST_NAME

logger = get_logger("planning.path_replay")

__all__ = [
    "Manifest",
    "Segment",
    "ValidationReport",
    "load_manifest",
    "validate",
]


@dataclass
class Segment:
    """One recorded path: its manifest record plus lazy access to the file."""

    record: dict[str, Any]
    directory: str

    @property
    def index(self) -> int:
        return int(self.record["index"])

    @property
    def kind(self) -> str:
        return str(self.record.get("kind", "unknown"))

    @property
    def label(self) -> str:
        """Human-readable one-liner for logs and progress output."""
        edge = self.record.get("edge_name") or "?"
        step = self.record.get("step_label") or "?"
        return f"[{self.index:03d}] {self.kind:<7} {step} · {edge}"

    @property
    def waypoint_path(self) -> str:
        return os.path.join(self.directory, self.record["waypoint_file"])

    def waypoints(self) -> list[list[float]]:
        """Read this segment's configuration array from disk.

        Deliberately not cached: a full run is thousands of
        configurations, and playback only ever needs one segment at a
        time.
        """
        with open(self.waypoint_path) as f:
            return json.load(f)["waypoints"]


@dataclass
class Manifest:
    directory: str
    data: dict[str, Any]
    segments: list[Segment] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.segments)

    def __iter__(self) -> Iterator[Segment]:
        return iter(self.segments)

    @property
    def seam_tolerance(self) -> float:
        return float(self.data.get("seam_tolerance", 1e-3))

    @property
    def quaternion_starts(self) -> list[int]:
        """Config indices where a unit quaternion begins, per the capture.

        Recorded in the manifest header precisely so continuity can be
        checked without loading the scene.  Empty for a manifest written
        before that field existed, or by a caller that never supplied a
        layout -- in which case a sign flip will read as a gap, which is
        the honest answer rather than a guessed one.
        """
        return [int(i) for i in self.data.get("quaternion_starts", ())]

    def by_step(self, step_index: int) -> list[Segment]:
        return [s for s in self.segments if s.record.get("step_index") == step_index]

    def by_kind(self, kind: str) -> list[Segment]:
        return [s for s in self.segments if s.kind == kind]

    def matching(self, needle: str) -> list[Segment]:
        """Segments whose step label or edge name contains ``needle``."""
        needle = needle.lower()
        return [
            s
            for s in self.segments
            if needle in str(s.record.get("step_label", "")).lower()
            or needle in str(s.record.get("edge_name", "")).lower()
        ]


def load_manifest(directory: str) -> Manifest:
    """Read a capture directory's manifest.

    Args:
        directory: The capture directory, or the manifest file itself.
    """
    if os.path.isdir(directory):
        path = os.path.join(directory, MANIFEST_NAME)
    else:
        path, directory = directory, os.path.dirname(directory)
    with open(path) as f:
        data = json.load(f)
    segments = [
        Segment(record=r, directory=directory) for r in data.get("segments", [])
    ]
    return Manifest(directory=directory, data=data, segments=segments)


@dataclass
class ValidationReport:
    num_segments: int = 0
    missing_files: list[str] = field(default_factory=list)
    seam_errors: list[tuple[int, float]] = field(default_factory=list)
    endpoint_errors: list[tuple[int, float]] = field(default_factory=list)
    size_changes: list[tuple[int, int, int]] = field(default_factory=list)
    empty_segments: list[int] = field(default_factory=list)
    coarse_segments: set[int] = field(default_factory=set)
    worst_seam: float = 0.0
    worst_step: float = 0.0
    worst_step_at: int | None = None
    total_waypoints: int = 0

    @property
    def ok(self) -> bool:
        return not (
            self.missing_files
            or self.seam_errors
            or self.endpoint_errors
            or self.size_changes
            or self.empty_segments
        )

    def summary(self) -> str:
        lines = [
            f"{self.num_segments} segment(s), {self.total_waypoints} configurations",
            f"worst seam        : {self.worst_seam:.3e}",
            f"worst step within : {self.worst_step:.3e}"
            + (
                f" (segment {self.worst_step_at})"
                if self.worst_step_at is not None
                else ""
            ),
        ]
        if self.missing_files:
            lines.append(f"MISSING FILES     : {len(self.missing_files)}")
        if self.empty_segments:
            lines.append(f"EMPTY SEGMENTS    : {self.empty_segments}")
        if self.size_changes:
            lines.append(f"CONFIG SIZE CHANGES: {self.size_changes}")
        if self.endpoint_errors:
            lines.append(
                f"FILE/MANIFEST MISMATCHES: {len(self.endpoint_errors)} "
                f"(worst {max(e for _, e in self.endpoint_errors):.3e})"
            )
        if self.seam_errors:
            lines.append(
                f"SEAM VIOLATIONS   : {len(self.seam_errors)} "
                + ", ".join(f"#{i}={e:.3e}" for i, e in self.seam_errors[:5])
            )
        if self.coarse_segments:
            # Not a failure: sampling resolution is a playback-quality
            # question, not a continuity one, so it never sets ok=False.
            lines.append(
                f"coarsely sampled  : {len(self.coarse_segments)} segment(s) "
                f"exceed the requested max step"
            )
        lines.append("VERDICT: " + ("continuous" if self.ok else "NOT continuous"))
        return "\n".join(lines)


def _canonical_delta(
    a: Sequence[float], b: Sequence[float], quaternion_starts: Sequence[int]
) -> float:
    """Max |a - b|, treating a quaternion and its negation as equal.

    Same double-cover reasoning as the recorder's seam check: HPP flips
    freely between ``q`` and ``-q``, which is the same orientation.
    """
    if len(a) != len(b):
        return float("inf")
    b = list(b)
    for start in quaternion_starts:
        stop = start + 4
        if (
            stop <= len(b)
            and sum(x * y for x, y in zip(a[start:stop], b[start:stop])) < 0
        ):
            b[start:stop] = [-v for v in b[start:stop]]
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)


def validate(
    manifest: Manifest,
    *,
    tolerance: float | None = None,
    quaternion_starts: Sequence[int] | None = None,
    max_step: float | None = None,
) -> ValidationReport:
    """Re-derive continuity from the segment files themselves.

    Args:
        manifest: A loaded manifest.
        tolerance: Seam tolerance; defaults to the manifest's own.
        quaternion_starts: Config indices where a unit quaternion begins,
            so a double-cover sign flip is not read as a discontinuity.
            Defaults to the manifest's own layout; pass an explicit empty
            sequence to check without one.
        max_step: If given, also flag any single sampling step inside a
            segment larger than this — a coarse ``dt`` shows up here, not
            at the seams.

    Returns:
        A :class:`ValidationReport`; ``report.ok`` is the verdict.
    """
    tol = manifest.seam_tolerance if tolerance is None else tolerance
    quats = (
        manifest.quaternion_starts if quaternion_starts is None else quaternion_starts
    )
    report = ValidationReport(num_segments=len(manifest))
    q_prev: list[float] | None = None
    size: int | None = None

    for seg in manifest:
        if not os.path.exists(seg.waypoint_path):
            report.missing_files.append(seg.record["waypoint_file"])
            continue
        waypoints = seg.waypoints()
        if not waypoints:
            report.empty_segments.append(seg.index)
            continue
        report.total_waypoints += len(waypoints)

        if size is None:
            size = len(waypoints[0])
        elif len(waypoints[0]) != size:
            report.size_changes.append((seg.index, size, len(waypoints[0])))

        # The file must agree with the record that indexes it -- otherwise
        # a replayer trusting the manifest plays something else.
        for stored, key in ((waypoints[0], "q_start"), (waypoints[-1], "q_end")):
            claimed = seg.record.get(key)
            if claimed is not None:
                err = _canonical_delta(claimed, stored, quats)
                if err > tol:
                    report.endpoint_errors.append((seg.index, err))

        if q_prev is not None:
            err = _canonical_delta(q_prev, waypoints[0], quats)
            report.worst_seam = max(report.worst_seam, err)
            if err > tol:
                report.seam_errors.append((seg.index, err))

        # Worst sampling step inside the segment. Always measured -- it is
        # the only number that says whether dt is fine enough for the
        # motion, and no seam check can see it.
        for a, b in zip(waypoints, waypoints[1:]):
            step = _canonical_delta(a, b, quats)
            if step > report.worst_step:
                report.worst_step = step
                report.worst_step_at = seg.index
            if max_step is not None and step > max_step:
                report.coarse_segments.add(seg.index)

        q_prev = waypoints[-1]

    return report
