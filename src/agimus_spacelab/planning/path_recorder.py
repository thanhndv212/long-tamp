#!/usr/bin/env python3
"""
Incremental capture of planned paths for later replay.

A long sequence run (see ``script/spacelab/screwdriving_sequence.py``)
produces a valid multi-hour assembly and then throws the motion away: the
grasp blocks' paths live in ``GraspSequencePlanner.phase_results``, which
``plan_sequence()`` resets on entry, and the joint-space arm moves between
blocks never enter it at all.  Nothing survives the process.

``PathRecorder`` is the outer layer that keeps them.  It is the single
writer for a run: every producer hands it a path, it samples the path to a
configuration array, writes one self-contained JSON per segment, and
appends a record to an ordered manifest that is rewritten atomically after
every segment.  A run killed at any point leaves a manifest describing
exactly the segments already on disk.

Two properties matter for replay and are why this is not just
``GraspSequencePlanner.auto_save_dir``:

* **Ordering is global.**  ``auto_save_dir`` names files by the phase index
  within one ``plan_sequence()`` call, and a block-decomposed run makes one
  call per block — so every block writes ``phase_01_edge_01_*`` and each
  overwrites the last.  Segments here are numbered by a run-global counter
  handed out in production order, and carry the step they belong to.

* **Sampled configurations, not graph-bound paths.**  A serialized HPP path
  references its constraint-graph edges and cannot be loaded against a
  different graph — and every phase here builds its own throwaway graph.
  Sampling at capture time makes playback pure visualization with no graph
  dependency.  The native ``.path`` file is still written when the backend
  and path type allow it (time-parameterized paths cannot be serialized),
  so the rebuild-the-graph replay route stays open; it is best-effort and
  never required.

What gets recorded is *the trajectory the robot travelled*, not the
trajectory anyone would have designed.  A phase that fails mid-way is
resumed from where the failure left the robot, so the edges its failed
attempt did plan are load-bearing and are kept.  A whole *block* that is
replanned after a broken lookahead hint restarts from the block's entry
configuration instead, so that motion never happened and the caller
:meth:`~PathRecorder.rollback` s it.  The distinction is the difference
between a manifest that can be executed and one that teleports.

Seam validation (``seam_tolerance``) is done at write time: each segment's
start configuration is compared against the previous segment's end.  A
discontinuity is recorded in the manifest and logged, but does not raise —
a run that has already spent hours planning should not die at a write, and
the manifest is the honest record either way.  Pass
``strict_seams=True`` to make it fatal instead.  It earns its keep: the
resumed-phase gap above was found by this check on a live run, not by
inspection.

Usage::

    recorder = PathRecorder("/tmp/run/paths", planner=planner)
    recorder.begin_step(0, "Bootstrap: UR10 grasps FG")
    recorder.record_path(path, kind="grasp", edge_name="ur10 > FG | f_01")
    ...
    recorder.close()
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence
from datetime import datetime, timezone
from typing import Any

from agimus_spacelab.logging import get_logger

logger = get_logger("planning.path_recorder")

__all__ = ["PathRecorder", "SeamError"]

MANIFEST_NAME = "manifest.json"
MANIFEST_FORMAT_VERSION = "1.0"


class SeamError(RuntimeError):
    """Raised (only when ``strict_seams``) when segments do not join up."""


def _as_floats(q: Any) -> list[float]:
    return [float(x) for x in q]


def _safe_name(text: str) -> str:
    """Filesystem-safe token; edge names carry ``/``, ``|`` and spaces."""
    out = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in text)
    return out.strip("_")[:80] or "unnamed"


class PathRecorder:
    """Single writer for a run's planned paths plus its ordered manifest.

    Args:
        output_dir: Directory for the manifest and per-segment files.
            Created if missing.
        planner: Optional backend planner.  Only used, opportunistically,
            for the native ``.path`` sidecar via ``save_path_vector``.
        dt: Sampling step in path-parameter units.  The number of samples
            for a path of length ``L`` is ``ceil(L / dt) + 1``, clamped to
            ``[min_samples, max_samples]``.
        seam_tolerance: Max allowed infinity-norm between a segment's start
            configuration and the previous segment's end.
        strict_seams: Raise :class:`SeamError` on a seam violation instead
            of recording and warning.
        quaternion_starts: Config indices at which a unit quaternion
            begins (freeflyer/spherical joints).  Seam comparison treats
            ``q`` and ``-q`` as equal at these blocks -- see
            :meth:`_check_seam`.
        save_native: Also attempt the backend's native path serialization.
        run_logger: Optional structured RunLogger; a ``segment_saved``
            event is emitted per segment when given.
    """

    def __init__(
        self,
        output_dir: str,
        planner: Any = None,
        *,
        dt: float = 0.05,
        min_samples: int = 2,
        max_samples: int = 2000,
        seam_tolerance: float = 1e-3,
        strict_seams: bool = False,
        quaternion_starts: Sequence[int] | None = None,
        save_native: bool = True,
        run_logger: Any = None,
    ) -> None:
        self.output_dir = output_dir
        self.planner = planner
        self.dt = float(dt)
        self.min_samples = int(min_samples)
        self.max_samples = int(max_samples)
        self.seam_tolerance = float(seam_tolerance)
        self.quaternion_starts = list(quaternion_starts or ())
        self.strict_seams = bool(strict_seams)
        self.save_native = bool(save_native)
        self.run_logger = run_logger

        self.segments: list[dict[str, Any]] = []
        self.seam_violations = 0
        self._step_idx: int | None = None
        self._step_label: str | None = None
        self._q_last: list[float] | None = None

        os.makedirs(self.output_dir, exist_ok=True)
        # Only stamp an empty manifest on a fresh directory: a resumed run
        # constructs the recorder before calling resume(), and truncating
        # here would destroy the very manifest resume() is about to adopt.
        if not os.path.exists(self.manifest_path):
            self._write_manifest()

    # -- Manifest -------------------------------------------------------

    @property
    def manifest_path(self) -> str:
        return os.path.join(self.output_dir, MANIFEST_NAME)

    def _manifest(self) -> dict[str, Any]:
        return {
            "format_version": MANIFEST_FORMAT_VERSION,
            "created": datetime.now(timezone.utc).isoformat(),
            "sampling": {
                "dt": self.dt,
                "min_samples": self.min_samples,
                "max_samples": self.max_samples,
            },
            "seam_tolerance": self.seam_tolerance,
            # Carried in the header so a replayer can check continuity
            # correctly without loading the scene: without it, every
            # quaternion sign flip reads as a discontinuity.
            "quaternion_starts": list(self.quaternion_starts),
            "seam_violations": self.seam_violations,
            "num_segments": len(self.segments),
            "segments": self.segments,
        }

    def _write_manifest(self) -> None:
        """Rewrite the manifest atomically.

        Write-then-rename, for the same reason ``_save_step_checkpoint``
        does it in the run script: the supervisor bounds wall time with
        SIGTERM, and an in-place truncating write that is interrupted
        leaves a 0-byte manifest with no way back.  ``os.replace`` is
        atomic on POSIX, so the previous manifest is never observed
        half-written.
        """
        tmp = self.manifest_path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(self._manifest(), f, indent=1)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.manifest_path)
        except Exception as e:  # pragma: no cover - disk-level failure
            logger.warning("Manifest write failed: %s", e)

    # -- Step framing ---------------------------------------------------

    def begin_step(self, step_idx: int, label: str) -> None:
        """Tag subsequent segments with the run step that produces them."""
        self._step_idx = step_idx
        self._step_label = label

    def resume(self, last_completed_step: int) -> int:
        """Adopt an existing manifest and continue after a resumed step.

        A resumed run re-enters an output directory that already holds the
        earlier attempt's segments.  Without this, numbering restarts at
        zero and the new run overwrites motion it never replanned.

        Segments belonging to steps after ``last_completed_step`` are
        dropped: the run is about to replan exactly those, and their old
        files are orphans that the new segments will overwrite in place.
        The kept tail's end configuration becomes the seam reference, so
        the first newly recorded segment is checked against where the
        checkpoint actually left the robot.

        Returns:
            Number of segments carried over (0 if there is no manifest).
        """
        try:
            with open(self.manifest_path) as f:
                previous = json.load(f).get("segments", [])
        except FileNotFoundError:
            return 0
        except Exception as e:
            logger.warning("Existing manifest unreadable (%s); starting fresh", e)
            return 0

        kept = [
            s
            for s in previous
            if s.get("step_index") is not None
            and s["step_index"] <= last_completed_step
        ]
        for i, seg in enumerate(kept):
            seg["index"] = i
        self.segments = kept
        self.seam_violations = sum(
            1 for s in kept if (s.get("seam_error") or 0.0) > self.seam_tolerance
        )
        self._q_last = kept[-1]["q_end"] if kept else None
        self._write_manifest()
        logger.info(
            "Resumed path capture: %d segment(s) kept, %d dropped",
            len(kept),
            len(previous) - len(kept),
        )
        return len(kept)

    def mark(self) -> int:
        """Current segment count, for :meth:`rollback`."""
        return len(self.segments)

    def rollback(self, mark: int) -> int:
        """Drop every segment recorded since ``mark``.

        For motion that turned out not to have happened.  A block whose
        lookahead hint chain breaks is replanned from the block's entry
        configuration -- ``q_current`` is untouched -- so the abandoned
        attempt's paths lead nowhere and would read as a jump back to the
        start.  A failed *phase* attempt is discarded the same way, since
        resume_sequence restarts it from where the call began; the caller
        decides, because only it knows what the retry will start from.

        Segment files are left on disk; the next segments reuse the same
        indices and overwrite them, exactly as :meth:`resume` does.

        Returns:
            Number of segments dropped.
        """
        if mark >= len(self.segments):
            return 0
        dropped = len(self.segments) - mark
        self.segments = self.segments[:mark]
        self.seam_violations = sum(
            1
            for s in self.segments
            if (s.get("seam_error") or 0.0) > self.seam_tolerance
        )
        self._q_last = self.segments[-1]["q_end"] if self.segments else None
        self._write_manifest()
        logger.info("Rolled back %d segment(s) from an abandoned attempt", dropped)
        return dropped

    # -- Recording ------------------------------------------------------

    def record_path(
        self,
        path: Any,
        *,
        kind: str,
        edge_name: str | None = None,
        geometric_path: Any = None,
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        """Sample, write and index one path.

        Args:
            path: The planned path, or the integer id of one in the
                backend's store (what ``plan_transition_edge`` returns).
                Needs ``length()`` and an evaluator (see :meth:`_evaluator`).
            kind: Segment kind — ``"grasp"``, ``"release"`` or ``"transit"``.
            edge_name: Constraint-graph edge the path was planned on.
            geometric_path: Optional non-time-parameterized twin.  Preferred
                for the native sidecar, which cannot serialize a
                time-parameterized path.
            extra: Extra manifest fields (phase index, gripper, handle...).

        Returns:
            The manifest record, or ``None`` if the path could not be
            sampled (recording is best-effort and never fails a run).
        """
        if path is None:
            return None

        index = len(self.segments)
        # Keep the backend's stored-path id before resolving it: it is what
        # lets a same-session consumer fetch the real path object back --
        # see PyHPPBackend.concatenate_paths -- and the manifest's ordering
        # is the verified one, unlike the backend's raw store, which also
        # holds attempts that were rolled back.
        path_id = path if isinstance(path, int) and not isinstance(path, bool) else None
        path = self._resolve(path)
        if path is None:
            logger.warning(
                "Segment %d (%s): path id could not be resolved", index, kind
            )
            return None
        try:
            waypoints, length = self._sample(path)
        except Exception as e:
            logger.warning("Segment %d (%s): sampling failed: %s", index, kind, e)
            return None
        if not waypoints:
            logger.warning("Segment %d (%s): path sampled to nothing", index, kind)
            return None

        q_start = self._endpoint(path, ("initial", "getInitialConfig"), waypoints[0])
        q_end = self._endpoint(path, ("end", "getEndConfig"), waypoints[-1])

        base = f"seg_{index:04d}_{kind}"
        if edge_name:
            base = f"{base}_{_safe_name(edge_name)}"

        record: dict[str, Any] = {
            "index": index,
            "step_index": self._step_idx,
            "step_label": self._step_label,
            "kind": kind,
            "edge_name": edge_name,
            "waypoint_file": base + ".json",
            "path_file": None,
            "path_id": path_id,
            "length": length,
            "num_waypoints": len(waypoints),
            "q_start": q_start,
            "q_end": q_end,
            "seam_error": None,
        }
        if extra:
            record.update(extra)

        record["seam_error"], flips = self._check_seam(index, kind, q_start)
        if flips:
            # Physically nothing moved, but a replayer interpolating across
            # the seam needs to know the sign changed.
            record["quaternion_flips"] = flips

        self._write_waypoints(base + ".json", record, waypoints)
        record["path_file"] = self._write_native(
            base, self._resolve(geometric_path) or path
        )

        self.segments.append(record)
        self._q_last = q_end
        self._write_manifest()

        if self.run_logger is not None:
            try:
                self.run_logger.log(
                    "segment_saved",
                    index=index,
                    kind=kind,
                    edge_name=edge_name,
                    step_label=self._step_label,
                    num_waypoints=len(waypoints),
                    seam_error=record["seam_error"],
                )
            except Exception:
                pass

        return record

    def record_phase_results(
        self, phase_results: list[dict[str, Any]], *, block_label: str | None = None
    ) -> list[dict[str, Any]]:
        """Record any not-yet-recorded path in ``phase_results``, in order.

        Call this after **every** ``plan_sequence`` / ``resume_sequence``
        call, successful or not -- not once when the block settles.

        **Incomplete phases count.**  When a phase fails mid-way,
        ``resume_sequence`` restarts it from where the failure left the
        robot, not from the block's entry configuration, so the edges the
        failed attempt did plan are the ones that got the robot there.
        Dropping them leaves a manifest that teleports.  Measured on RS2's
        FG grasp: ``_01`` planned home -> pregrasp-A, ``_12`` hit a
        collision, and the resume replanned ``_01`` from pregrasp-A to
        pregrasp-B.  Keeping only the surviving pair put a 5.08 rad jump
        in the manifest, because the leg to pregrasp-A was the discarded
        one.

        And it has to happen per call, not at the end:
        ``resume_sequence`` *deletes* incomplete phases from
        ``phase_results`` before replanning, so by the time a block
        settles those paths are unreachable.

        Already-recorded edges are tracked on the phase dict itself, which
        survives exactly as long as the entry does -- a phase dropped by
        the resume filter takes its marker with it, and its paths were
        recorded on an earlier call.

        Abandoned *block* attempts are the caller's problem, not this
        method's: a lookahead replan restarts from the block's entry
        configuration, so that motion is not load-bearing and the caller
        should :meth:`rollback` it.
        """
        written = []
        for phase in phase_results or []:
            if phase.get("skipped"):
                continue
            edges = list(phase.get("edges") or [])
            paths = list(phase.get("paths") or [])
            # A release phase filters None paths out of "paths" but keeps
            # both names in "edges" (_build_release_phase_info), so the two
            # lists can differ in length; index defensively.
            kind = "release" if phase.get("handle") is None else "grasp"
            done = phase.setdefault("_recorded_edge_indices", set())
            for edge_idx, path in enumerate(paths):
                if edge_idx in done:
                    continue
                done.add(edge_idx)
                rec = self.record_path(
                    path,
                    kind=kind,
                    edge_name=edges[edge_idx] if edge_idx < len(edges) else None,
                    extra={
                        "block_label": block_label,
                        "phase": phase.get("phase"),
                        "phase_complete": bool(phase.get("complete", False)),
                        "edge_index": edge_idx,
                        "gripper": phase.get("gripper"),
                        "handle": phase.get("handle"),
                        "released": phase.get("released"),
                        "state_after": phase.get("state_after"),
                    },
                )
                if rec is not None:
                    written.append(rec)
        return written

    def close(self) -> dict[str, Any]:
        """Flush the manifest and return a small summary."""
        self._write_manifest()
        return {
            "manifest": self.manifest_path,
            "num_segments": len(self.segments),
            "seam_violations": self.seam_violations,
        }

    # -- Internals ------------------------------------------------------

    def _resolve(self, path: Any) -> Any:
        """Turn a stored-path id into the path object it refers to.

        ``plan_transition_edge(..., store=True)`` -- the default, and what
        every caller here uses -- returns the *index* of the path in the
        backend's store, not the path.  So ``phase_results["paths"]`` is a
        list of ints, and everything downstream that wants the geometry has
        to go back through ``planner.get_path(index)``.  (The second return
        value, the geometric twin, is a real object; both shapes arrive
        here, hence the check rather than an unconditional lookup.)
        """
        if not isinstance(path, int) or isinstance(path, bool):
            return path
        getter = getattr(self.planner, "get_path", None)
        if not callable(getter):
            return None
        try:
            return getter(path)
        except Exception as e:
            logger.warning("Could not resolve stored path %d: %s", path, e)
            return None

    @staticmethod
    def _evaluator(path: Any) -> Any:
        """Pick the path's configuration-at-parameter accessor.

        The two backends disagree, and so does this codebase's own code.
        pyhpp's ``PathVector`` (``pyhpp.core.path.Vector``) exposes
        ``eval(t)`` and ``__call__(t)`` but no ``call`` -- which is why
        ``PyHPPBackend.save_path_as_waypoints``, written against
        ``call(t)``, never actually sampled anything on this backend.
        ``call`` stays last in the chain for CORBA-shaped paths.
        """
        for name in ("eval", "call"):
            fn = getattr(path, name, None)
            if callable(fn):
                return fn
        if callable(path):
            return path
        raise TypeError(f"{type(path).__name__} cannot be evaluated at a parameter")

    def _sample(self, path: Any) -> tuple[list[list[float]], float]:
        """Sample ``path`` at ``dt`` in path-parameter units.

        The evaluator returns ``(q, success)`` in pyhpp; a bare ``q`` is
        accepted too so the recorder is not tied to one backend.
        """
        length = float(path.length())
        if not math.isfinite(length) or length < 0:
            raise ValueError(f"path length is {length}")
        n = math.ceil(length / self.dt) + 1 if self.dt > 0 else self.min_samples
        n = max(self.min_samples, min(self.max_samples, n))
        evaluate = self._evaluator(path)

        waypoints = []
        for i in range(n):
            t = (i / (n - 1)) * length if n > 1 else 0.0
            result = evaluate(t)
            if isinstance(result, tuple):
                q, ok = result[0], result[1]
                if not ok:
                    continue
            else:
                q = result
            waypoints.append(_as_floats(q))
        return waypoints, length

    @staticmethod
    def _endpoint(
        path: Any, getters: tuple[str, ...], fallback: list[float]
    ) -> list[float]:
        """First working endpoint accessor wins.

        pyhpp names them ``initial()``/``end()``; the CORBA-shaped paths
        this codebase's older call sites guard for use
        ``getInitialConfig()``/``getEndConfig()``. Falling back to the
        first/last sample keeps a path with neither usable.
        """
        for getter in getters:
            fn = getattr(path, getter, None)
            if callable(fn):
                try:
                    return _as_floats(fn())
                except Exception:
                    continue
        return list(fallback)

    def _canonicalize(
        self, reference: list[float], q: list[float]
    ) -> tuple[list[float], list[int]]:
        """Put ``q``'s quaternions in the same hemisphere as ``reference``.

        A unit quaternion double-covers rotation: ``q`` and ``-q`` are the
        same orientation.  HPP's solvers flip between them freely, so a
        component-wise comparison reports a gap of up to 2.0 across a seam
        where nothing physically moved -- measured on the first captured
        run, where the RS1 FG grasp's ``_01``/``_12`` boundary showed max
        |dq| = 1.366 from four exactly-negated components with dot = -1.0
        and identical translation.  Left uncorrected, that noise fires on
        most seams and the check stops meaning anything.

        Only the comparison is canonicalized; the stored waypoints keep
        whatever sign the planner produced.
        """
        if not self.quaternion_starts:
            return q, []
        out = list(q)
        flipped = []
        for start in self.quaternion_starts:
            stop = start + 4
            if stop > len(out) or stop > len(reference):
                continue
            if sum(r * v for r, v in zip(reference[start:stop], out[start:stop])) < 0.0:
                out[start:stop] = [-v for v in out[start:stop]]
                flipped.append(start)
        return out, flipped

    def _check_seam(
        self, index: int, kind: str, q_start: list[float]
    ) -> tuple[float | None, list[int]]:
        """Compare this segment's start against the previous segment's end.

        Catches a discontinuity while the run is still going rather than at
        replay — which matters because release phases can fall back to the
        direct edge instead of the ``_21``/``_10`` waypoint pair, producing
        a different endpoint shape.

        Returns ``(error, flipped_quaternion_starts)``.
        """
        if self._q_last is None:
            return None, []
        if len(self._q_last) != len(q_start):
            return self._report_seam(index, kind, float("inf")), []
        q_start, flipped = self._canonicalize(self._q_last, q_start)
        err = max((abs(a - b) for a, b in zip(self._q_last, q_start)), default=0.0)
        return self._report_seam(index, kind, err), flipped

    def _report_seam(self, index: int, kind: str, err: float) -> float:
        if err > self.seam_tolerance:
            self.seam_violations += 1
            msg = (
                f"Segment {index} ({kind}) does not join the previous segment: "
                f"max |dq| = {err:.3e} > {self.seam_tolerance:.3e}"
            )
            if self.strict_seams:
                raise SeamError(msg)
            logger.warning("%s", msg)
        return err

    def _write_waypoints(
        self, filename: str, record: dict[str, Any], waypoints: list[list[float]]
    ) -> None:
        """Write the self-contained segment file.

        Deliberately duplicates the manifest's descriptive fields: a
        segment file must be playable on its own, without the manifest.
        """
        payload = {
            "format_version": MANIFEST_FORMAT_VERSION,
            "index": record["index"],
            "kind": record["kind"],
            "edge_name": record["edge_name"],
            "step_index": record["step_index"],
            "step_label": record["step_label"],
            "length": record["length"],
            "waypoints": waypoints,
        }
        with open(os.path.join(self.output_dir, filename), "w") as f:
            json.dump(payload, f)

    def _write_native(self, base: str, path: Any) -> str | None:
        """Best-effort native sidecar; ``None`` when unsupported.

        Time-parameterized paths have no serializer, so this legitimately
        fails for most segments.  It is logged at debug level only —
        the sampled JSON is the replay format, this is just the door left
        open for the rebuild-the-graph route.
        """
        if not self.save_native or self.planner is None:
            return None
        saver = getattr(self.planner, "save_path_vector", None)
        if not callable(saver):
            return None
        filename = base + ".path"
        try:
            saver(path, os.path.join(self.output_dir, filename))
            return filename
        except Exception as e:
            logger.debug("Native path save skipped for %s: %s", base, e)
            return None
