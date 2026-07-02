#!/usr/bin/env python3
"""
Video recording utilities for path playback.

Provides functionality to record videos during path playback using either:

* **gepetto-viewer-corba** — ``startCapture``/``stopCapture`` (async frame dump)
* **viser** — ``captureImage`` (synchronous, frame-by-frame via :meth:`record_path`)
"""

import glob
import os
import subprocess
from datetime import datetime
from typing import Optional

import numpy as np


def _is_viser_viewer(viewer) -> bool:
    """Return True if *viewer* is a pyhpp_viser Viewer instance."""
    try:
        from pyhpp_viser import Viewer as _ViserViewer

        return isinstance(viewer, _ViserViewer)
    except ImportError:
        return False


class VideoRecorder:
    """
    Video recorder for path playback.

    Supports two capture modes:

    * **gepetto** — uses ``viewer.client.gui.startCapture`` / ``stopCapture``
      (async frame dump).  Use :meth:`start_recording` / :meth:`stop_recording`.
    * **viser** — uses ``viewer.captureImage`` (synchronous, PIL-based).
      Use :meth:`record_path` which handles the full capture loop in one call.
    """

    def __init__(
        self,
        viewer,
        output_dir: str = "/home/dvtnguyen/devel/demos",
        framerate: int = 25,
        frame_extension: str = "png",
        video_extension: str = "mp4",
        auto_cleanup: bool = True,
    ):
        """
        Initialize video recorder.

        Args:
            viewer: Gepetto or viser viewer instance.
            output_dir: Directory for video output (default: /home/dvtnguyen/devel/demos)
            framerate: Video framerate in fps (default: 25)
            frame_extension: Frame format - 'png' or 'jpeg' (default: 'png')
            video_extension: Video format - 'mp4', 'avi', etc. (default: 'mp4')
            auto_cleanup: Auto-delete frames after encoding (default: True)
        """
        self.viewer = viewer
        self.output_dir = output_dir
        self.framerate = framerate
        self.frame_extension = frame_extension
        self.video_extension = video_extension
        self.auto_cleanup = auto_cleanup

        self._recording = False
        self._frame_prefix = None
        self._video_file = None

    @property
    def _is_viser(self) -> bool:
        """True when the wrapped viewer is a pyhpp_viser Viewer."""
        return _is_viser_viewer(self.viewer)

    def start_recording(
        self, video_name: Optional[str] = None, path_id: Optional[int] = None
    ) -> str:
        """
        Start video recording by initiating frame capture (gepetto only).

        For viser viewers, use :meth:`record_path` instead.

        Args:
            video_name: Custom name for the output video (without extension).
                       If None, a name will be auto-generated with timestamp.
            path_id: Optional path ID for default naming

        Returns:
            The full path to the output video file
        """
        if self._is_viser:
            raise TypeError(
                "start_recording() is not supported for viser viewers. "
                "Use VideoRecorder.record_path(path, ...) instead."
            )
        if self._recording:
            raise RuntimeError(
                "Recording already in progress. Call stop_recording() first."
            )

        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate video name if not provided
        if video_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if path_id is not None:
                base_name = f"path_{path_id}_{timestamp}"
            else:
                base_name = f"recording_{timestamp}"
        else:
            base_name = video_name

        # Setup frame capture prefix (without extension)
        self._frame_prefix = os.path.join(self.output_dir, f"{base_name}_frame")

        # Video output file
        self._video_file = os.path.join(
            self.output_dir, f"{base_name}.{self.video_extension}"
        )

        # Start capture using gepetto-viewer-corba API
        print(f"[VideoRecorder] Starting recording: {self._video_file}")
        self.viewer.client.gui.startCapture(
            self.viewer.windowId, self._frame_prefix, self.frame_extension
        )

        self._recording = True
        return self._video_file

    def stop_recording(self) -> str:
        """
        Stop video recording and encode frames to video.

        Returns:
            The path to the generated video file
        """
        if not self._recording:
            raise RuntimeError("No recording in progress.")

        # Stop frame capture
        self.viewer.client.gui.stopCapture(self.viewer.windowId)
        print("[VideoRecorder] Stopped frame capture")

        self._recording = False

        # Encode video using ffmpeg
        self._encode_video()

        return self._video_file

    def record_path(
        self,
        path,
        speed: float = 1.0,
        video_name: Optional[str] = None,
        path_id: Optional[int] = None,
        width: int = 800,
        height: int = 600,
    ) -> str:
        """
        Record a path by sampling frames with ``viewer.captureImage`` (viser only).

        Iterates over the path time-domain, calls ``viewer.display(q)`` then
        ``viewer.captureImage(width, height)`` at each frame, saves frames to
        disk, and encodes to video with ffmpeg.

        Args:
            path: HPP path object supporting ``.length()`` and ``.eval(t)``.
            speed: Playback speed multiplier (default: 1.0).
            video_name: Output video file base-name (no extension).
                        Auto-generated with timestamp if *None*.
            path_id: Optional path index used in the auto-generated name.
            width: Capture width in pixels (default: 800).
            height: Capture height in pixels (default: 600).

        Returns:
            Absolute path to the encoded video file.

        Raises:
            TypeError: If the viewer is not a viser viewer. Use
                :meth:`start_recording`/:meth:`stop_recording` for gepetto.
        """
        if not self._is_viser:
            raise TypeError(
                "record_path() requires a viser viewer. "
                "Use start_recording()/stop_recording() for gepetto."
            )

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if video_name is None:
            base_name = (
                f"path_{path_id}_{timestamp}"
                if path_id is not None
                else f"recording_{timestamp}"
            )
        else:
            base_name = video_name
        self._frame_prefix = os.path.join(self.output_dir, f"{base_name}_frame")
        self._video_file = os.path.join(
            self.output_dir, f"{base_name}.{self.video_extension}"
        )

        total_time = path.length()
        n_frames = max(2, int(total_time / speed * self.framerate))
        print(
            f"[VideoRecorder] Capturing {n_frames} viser frames (path length {total_time:.3f} s)..."
        )

        for i, t in enumerate(np.linspace(0.0, total_time, n_frames)):
            result = path.eval(t)
            # result may be (q, valid) or just q depending on HPP version
            if (
                isinstance(result, (list, tuple))
                and len(result) == 2
                and isinstance(result[1], bool)
            ):
                q, valid = result
            else:
                q = result
                valid = True
            if valid:
                self.viewer.display(np.array(q, dtype=float))
            img = self.viewer.captureImage(width, height)
            frame_path = f"{self._frame_prefix}_{i}.{self.frame_extension}"
            img.save(frame_path)

        print(f"[VideoRecorder] Encoding video: {self._video_file}")
        self._encode_video()
        return self._video_file

    def _encode_video(self):
        """Encode captured frames into a video file using ffmpeg."""
        # Find all captured frames
        # Note: gepetto-viewer generates frames without zero-padding (0, 1, 2, ... not 000000, 000001)
        frame_pattern = f"{self._frame_prefix}_%d.{self.frame_extension}"

        # Check if ffmpeg is available
        try:
            subprocess.run(
                ["ffmpeg", "-version"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True,
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            print(
                "[VideoRecorder] Warning: ffmpeg not found. Frames saved but video not encoded."
            )
            print(f"[VideoRecorder] Frame pattern: {frame_pattern}")
            print("[VideoRecorder] You can manually encode with:")
            print(
                f"  ffmpeg -r {self.framerate} -i {frame_pattern} -c:v libx264 -pix_fmt yuv420p {self._video_file}"
            )
            return

        # Encode with ffmpeg
        cmd = [
            "ffmpeg",
            "-y",  # Overwrite output file
            "-r",
            str(self.framerate),  # Input framerate
            "-i",
            frame_pattern,  # Input pattern
            "-c:v",
            "libx264",  # Video codec
            "-pix_fmt",
            "yuv420p",  # Pixel format for compatibility
            "-preset",
            "medium",  # Encoding speed/quality tradeoff
            self._video_file,
        ]

        print("[VideoRecorder] Encoding video with ffmpeg...")
        try:
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            print(f"[VideoRecorder] Video saved: {self._video_file}")

            # Auto cleanup frames if enabled
            if self.auto_cleanup:
                self._cleanup_frames()
        except subprocess.CalledProcessError as e:
            print(f"[VideoRecorder] Error encoding video: {e.stderr}")
            print(
                f"[VideoRecorder] Frames preserved at: {self._frame_prefix}_*.{self.frame_extension}"
            )

    def _cleanup_frames(self):
        """Delete intermediate frame files after successful video encoding."""
        frame_files = glob.glob(f"{self._frame_prefix}_*.{self.frame_extension}")

        if frame_files:
            print(f"[VideoRecorder] Cleaning up {len(frame_files)} frame files...")
            for frame_file in frame_files:
                try:
                    os.remove(frame_file)
                except OSError as e:
                    print(
                        f"[VideoRecorder] Warning: Could not remove {frame_file}: {e}"
                    )

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording


def record_path_playback(
    viewer,
    path_player_or_path,
    path_id: int,
    video_name: Optional[str] = None,
    output_dir: str = "/home/dvtnguyen/devel/demos",
    framerate: int = 25,
    dt: float = 0.01,
    speed: float = 1.0,
    width: int = 800,
    height: int = 600,
) -> str:
    """
    Convenience function to record a path playback.

    Dispatches automatically based on the viewer type:

    * **gepetto**: uses ``startCapture``/``stopCapture``.  Pass a
      ``PathPlayer`` instance as *path_player_or_path*.
    * **viser**: uses ``captureImage`` frame-by-frame.  Pass an HPP path
      object (supporting ``.length()`` / ``.eval(t)``) as *path_player_or_path*.

    Args:
        viewer: Gepetto or viser viewer instance.
        path_player_or_path: For gepetto — a ``PathPlayer``; for viser — an HPP path object.
        path_id: Path identifier (used for naming only when viewer is viser).
        video_name: Custom name for the output video (without extension).
        output_dir: Directory for video output.
        framerate: Video framerate in fps.
        dt: Time step for gepetto path sampling.
        speed: Playback speed multiplier.
        width: Frame width in pixels (viser only, default: 800).
        height: Frame height in pixels (viser only, default: 600).

    Returns:
        The path to the generated video file.
    """
    recorder = VideoRecorder(viewer, output_dir=output_dir, framerate=framerate)

    if _is_viser_viewer(viewer):
        return recorder.record_path(
            path=path_player_or_path,
            speed=speed,
            video_name=video_name,
            path_id=path_id,
            width=width,
            height=height,
        )
    else:
        # Gepetto path: path_player_or_path is a PathPlayer
        path_player = path_player_or_path
        path_player.setDt(dt)
        path_player.setSpeed(speed)
        video_file = recorder.start_recording(video_name=video_name, path_id=path_id)
        try:
            path_player(path_id)
        finally:
            recorder.stop_recording()
        return video_file
