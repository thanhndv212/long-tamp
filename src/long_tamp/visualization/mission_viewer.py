"""Common native HPP playback lifecycle for interactive examples."""
import sys

class MissionViewer:
    """Use the backend's native HPP playback, as in SpaceLab's full mission."""

    def __init__(self, task, port, initial, camera=None):
        from pyhpp_viser import Viewer

        self.backend = task.planner
        self.path_ids = []
        self.backend.viewer = Viewer(self.backend.device, self.backend.problem)
        self.backend.viewer.start(host="0.0.0.0", port=port, open=True)

        @self.backend.viewer.viewer.on_client_connect
        def aim(client):
            if camera is not None:
                client.camera.position, client.camera.look_at = camera
            client.camera.up_direction = (0.0, 0.0, 1.0)

        self.backend.visualize(initial)
        print(f"Viser: http://localhost:{port}", flush=True)

    def completed(self, phases):
        for phase in phases:
            if not phase.get("complete", True) or phase.get("skipped"):
                continue
            for path in phase.get("paths", []):
                if path is None:
                    continue
                pid = path if isinstance(path, int) else self.backend.store_path(path)
                self.path_ids.append(pid)
                self.backend.play_path(pid)

    def finish(self):
        if not self.path_ids:
            return
        full = self.backend.concatenate_paths(self.path_ids)
        self.backend.play_path(full)
        if not sys.stdin.isatty():
            return
        while True:
            choice = input("Enter to replay full path, path number, or q to close: ").strip()
            if choice.lower() == "q":
                return
            if not choice:
                self.backend.play_path(full)
            elif choice.isdigit() and 1 <= int(choice) <= len(self.path_ids):
                self.backend.play_path(self.path_ids[int(choice) - 1])
            else:
                print(f"Choose a path from 1 to {len(self.path_ids)}, Enter, or q.")

    def close(self):
        if self.backend.viewer is not None:
            self.backend.viewer.viewer.stop()
            self.backend.viewer = None


