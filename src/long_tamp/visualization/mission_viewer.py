"""Common native HPP playback lifecycle for interactive examples."""

import sys
import time


class MissionViewer:
    """Use the backend's native HPP playback, as in SpaceLab's full mission.

    Planned paths never move the fingers: a grasp is a rigid TCP constraint
    and the finger joints stay frozen open. Pass ``closures`` (a
    :class:`long_tamp.grasping.FingerClosureTable`) and the viewer plays the
    grasp planner's gripper commands too: the fingers close on the handle at
    the end of each grasp phase, stay closed while the object is carried and
    open at the start of its release. Without it, paths play natively.
    """

    closures = None
    fps = 30.0

    def __init__(self, task, port, initial, camera=None, closures=None, fps=30.0):
        from pyhpp_viser import Viewer

        self.backend = task.planner
        self.path_ids = []
        self.closures = closures
        self.fps = fps
        self._rank = task.robot.rankInConfiguration if closures is not None else {}
        # gripper -> finger values it holds now, and per played path the
        # overlay during it plus the gripper actions before/after it.
        self._fingers = {}
        self._segments = []
        self.backend.viewer = Viewer(self.backend.device, self.backend.problem)
        self.backend.viewer.start(host="0.0.0.0", port=port, open=True)

        @self.backend.viewer.viewer.on_client_connect
        def aim(client):
            if camera is not None:
                client.camera.position, client.camera.look_at = camera
            client.camera.up_direction = (0.0, 0.0, 1.0)

        self.backend.visualize(initial)
        print(f"Viser: http://localhost:{port}", flush=True)

    # -- gripper actions ----------------------------------------------------

    def _actions(self, phase):
        """(open_before, close_after) gripper actions around one phase."""
        gripper, handle = phase.get("gripper"), phase.get("handle")
        if self.closures is None or not self.closures.has(gripper):
            return None, None
        if handle is not None:
            return None, (gripper, self.closures.closed_values(gripper, handle))
        # handle None: a release, unless the gripper still holds (a loop
        # move carrying its object).
        still_holds = f"{gripper} grasps " in (phase.get("state_after") or "")
        if gripper in self._fingers and not still_holds:
            return (gripper, self.closures.open_values(gripper)), None
        return None, None

    def _overlay(self, q, fingers):
        import numpy as np

        q = np.array(q, dtype=float, copy=True)
        for values in fingers.values():
            for joint, value in values.items():
                q[self._rank[joint]] = value
        return q

    def _animate(self, q, fingers, gripper, target, steps=12):
        start = fingers.get(gripper) or self.closures.open_values(gripper)
        for i in range(1, steps + 1):
            s = i / steps
            fingers[gripper] = {
                j: start[j] + (v - start[j]) * s for j, v in target.items()
            }
            self.backend.viewer(self._overlay(q, fingers))
            time.sleep(0.02)

    @staticmethod
    def _t0(path):
        if not hasattr(path, "timeRange"):
            return 0.0
        tr = path.timeRange()
        return tr.first if hasattr(tr, "first") else tr[0]

    def _play_sampled(self, path, fingers):
        length = path.length()
        t0 = self._t0(path)
        n = max(2, int(length * self.fps) + 1)
        q = None
        for i in range(n):
            qi, ok = path.eval(t0 + length * i / (n - 1))
            if ok:
                q = qi
                self.backend.viewer(self._overlay(q, fingers))
                time.sleep(length / (n - 1) if n > 1 else 0.0)
        return q

    def _play_segment(self, segment, fingers):
        pid, open_before, close_after = segment
        path = self.backend.get_path(pid)
        if open_before is not None:
            q0, _ = path.eval(self._t0(path))
            self._animate(q0, fingers, *open_before)
            fingers.pop(open_before[0], None)
        q = self._play_sampled(path, fingers)
        if close_after is not None and q is not None:
            self._animate(q, fingers, *close_after)

    # -- lifecycle ----------------------------------------------------------

    def completed(self, phases):
        for phase in phases:
            if not phase.get("complete", True) or phase.get("skipped"):
                continue
            open_before, close_after = self._actions(phase)
            paths = [p for p in phase.get("paths", []) if p is not None]
            for k, path in enumerate(paths):
                pid = path if isinstance(path, int) else self.backend.store_path(path)
                self.path_ids.append(pid)
                if self.closures is None:
                    self.backend.play_path(pid)
                    continue
                segment = (
                    pid,
                    open_before if k == 0 else None,
                    close_after if k == len(paths) - 1 else None,
                )
                self._segments.append(segment)
                self._play_segment(segment, self._fingers)

    def _play_all(self, full):
        if self.closures is None:
            self.backend.play_path(full)
            return
        fingers = {}
        for segment in self._segments:
            self._play_segment(segment, fingers)

    def _play_one(self, index):
        if self.closures is None:
            self.backend.play_path(self.path_ids[index])
            return
        # Fingers as they were when this path started.
        fingers = {}
        for segment in self._segments[:index]:
            _, open_before, close_after = segment
            if open_before is not None:
                fingers.pop(open_before[0], None)
            if close_after is not None:
                fingers[close_after[0]] = dict(close_after[1])
        self._play_segment(self._segments[index], fingers)

    def finish(self):
        if not self.path_ids:
            return
        full = None
        if self.closures is None:
            full = self.backend.concatenate_paths(self.path_ids)
        self._play_all(full)
        if not sys.stdin.isatty():
            return
        while True:
            choice = input(
                "Enter to replay full path, path number, or q to close: "
            ).strip()
            if choice.lower() == "q":
                return
            if not choice:
                self._play_all(full)
            elif choice.isdigit() and 1 <= int(choice) <= len(self.path_ids):
                self._play_one(int(choice) - 1)
            else:
                print(f"Choose a path from 1 to {len(self.path_ids)}, Enter, or q.")

    def close(self):
        if self.backend.viewer is not None:
            self.backend.viewer.viewer.stop()
            self.backend.viewer = None
