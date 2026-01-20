#!/usr/bin/env python3
"""
Video recording utilities for path playback.

Provides functionality to record videos during path playback using
gepetto-viewer-corba's capture capabilities and ffmpeg encoding.
"""

import glob
import os
import subprocess
import time
from datetime import datetime
from typing import Optional


class VideoRecorder:
    """
    Video recorder for gepetto-viewer path playback.
    
    Uses gepetto-viewer-corba's startCapture/stopCapture methods to record
    frames during path playback, then encodes them to video using ffmpeg.
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
            viewer: Gepetto viewer instance (from ViewerFactory)
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

    def start_recording(self, video_name: Optional[str] = None, path_id: Optional[int] = None) -> str:
        """
        Start video recording by initiating frame capture.
        
        Args:
            video_name: Custom name for the output video (without extension).
                       If None, a name will be auto-generated with timestamp.
            path_id: Optional path ID for default naming
            
        Returns:
            The full path to the output video file
        """
        if self._recording:
            raise RuntimeError("Recording already in progress. Call stop_recording() first.")
        
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
        self._video_file = os.path.join(self.output_dir, f"{base_name}.{self.video_extension}")
        
        # Start capture using gepetto-viewer-corba API
        print(f"[VideoRecorder] Starting recording: {self._video_file}")
        self.viewer.client.gui.startCapture(
            self.viewer.windowId,
            self._frame_prefix,
            self.frame_extension
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
        print(f"[VideoRecorder] Stopped frame capture")
        
        self._recording = False
        
        # Encode video using ffmpeg
        self._encode_video()
        
        return self._video_file

    def _encode_video(self):
        """Encode captured frames into a video file using ffmpeg."""
        # Find all captured frames
        frame_pattern = f"{self._frame_prefix}_%06d.{self.frame_extension}"
        
        # Check if ffmpeg is available
        try:
            subprocess.run(
                ["ffmpeg", "-version"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("[VideoRecorder] Warning: ffmpeg not found. Frames saved but video not encoded.")
            print(f"[VideoRecorder] Frame pattern: {frame_pattern}")
            print(f"[VideoRecorder] You can manually encode with:")
            print(f"  ffmpeg -r {self.framerate} -i {frame_pattern} -c:v libx264 -pix_fmt yuv420p {self._video_file}")
            return
        
        # Encode with ffmpeg
        cmd = [
            "ffmpeg", "-y",  # Overwrite output file
            "-r", str(self.framerate),  # Input framerate
            "-i", frame_pattern,  # Input pattern
            "-c:v", "libx264",  # Video codec
            "-pix_fmt", "yuv420p",  # Pixel format for compatibility
            "-preset", "medium",  # Encoding speed/quality tradeoff
            self._video_file
        ]
        
        print(f"[VideoRecorder] Encoding video with ffmpeg...")
        try:
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            print(f"[VideoRecorder] Video saved: {self._video_file}")
            
            # Auto cleanup frames if enabled
            if self.auto_cleanup:
                self._cleanup_frames()
        except subprocess.CalledProcessError as e:
            print(f"[VideoRecorder] Error encoding video: {e.stderr}")
            print(f"[VideoRecorder] Frames preserved at: {self._frame_prefix}_*.{self.frame_extension}")

    def _cleanup_frames(self):
        """Delete intermediate frame files after successful video encoding."""
        frame_files = glob.glob(f"{self._frame_prefix}_*.{self.frame_extension}")
        
        if frame_files:
            print(f"[VideoRecorder] Cleaning up {len(frame_files)} frame files...")
            for frame_file in frame_files:
                try:
                    os.remove(frame_file)
                except OSError as e:
                    print(f"[VideoRecorder] Warning: Could not remove {frame_file}: {e}")

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording


def record_path_playback(
    viewer,
    path_player,
    path_id: int,
    video_name: Optional[str] = None,
    output_dir: str = "/home/dvtnguyen/devel/demos",
    framerate: int = 25,
    dt: float = 0.01,
    speed: float = 1.0,
) -> str:
    """
    Convenience function to record a path playback.
    
    Args:
        viewer: Gepetto viewer instance
        path_player: PathPlayer instance
        path_id: Path identifier to play
        video_name: Custom name for the output video (without extension)
        output_dir: Directory for video output
        framerate: Video framerate in fps
        dt: Time step for path sampling
        speed: Playback speed multiplier
        
    Returns:
        The path to the generated video file
    """
    # Create recorder
    recorder = VideoRecorder(
        viewer,
        output_dir=output_dir,
        framerate=framerate
    )
    
    # Configure path player
    path_player.setDt(dt)
    path_player.setSpeed(speed)
    
    # Start recording
    video_file = recorder.start_recording(video_name=video_name, path_id=path_id)
    
    try:
        # Play the path
        path_player(path_id)
    finally:
        # Always stop recording
        recorder.stop_recording()
    
    return video_file
