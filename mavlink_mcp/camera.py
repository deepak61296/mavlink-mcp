"""Camera frame sources for the vision tool.

A frame source returns encoded image bytes (JPEG) or None. The main one, StreamCamera, reads a
video stream with OpenCV and snapshots the latest frame:

  - Gazebo: the GstCameraPlugin streams H.264 over UDP (default 127.0.0.1:5600). Read it with a
    GStreamer pipeline. Enable the stream first, e.g.:
      gz topic -t <...>/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
  - Real drone / IP camera: pass an rtsp:// (or http://) URL behind the same callable.

See https://ardupilot.org/dev/docs/sitl-with-gazebo.html for the streaming setup.
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Callable, Optional


class FrameHub:
    """Single reader of a frame source, sharing the latest JPEG with many consumers.

    A cv2.VideoCapture is not safe to read from several threads, so ONE grabber thread reads the
    source and everyone else (web streams, the agent's capture_camera) reads latest() instead.
    """

    def __init__(self, frame_source: Callable[[], Optional[bytes]], fps: float = 15.0):
        self._src = frame_source
        self._latest: Optional[bytes] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._period = 1.0 / max(1.0, fps)
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "FrameHub":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="frame-hub", daemon=True)
            self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._src()
            except Exception:
                frame = None
            if frame:
                with self._lock:
                    self._latest = frame
            time.sleep(self._period)

    def latest(self) -> Optional[bytes]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)          # let the grabber finish a read before closing
            self._thread = None
        close = getattr(self._src, "close", None)  # release the capture cleanly (no GStreamer spew)
        if callable(close):
            try:
                close()
            except Exception:
                pass


def gazebo_gimbal_pitch(rad: float, topic: str = "/gimbal/cmd_pitch") -> None:
    """Point the Gazebo gimbal (sim only). Straight down = +1.57. Best-effort."""
    import subprocess
    try:
        subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {rad}"],
                       env={"HOME": os.environ.get("HOME", "/root"), "PATH": "/usr/local/bin:/usr/bin:/bin"},
                       timeout=6, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


_GIMBAL_STATE_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/joint_state"


def gazebo_mount_pitch_deg(topic: str = _GIMBAL_STATE_TOPIC) -> Optional[float]:
    """The sim camera's ACTUAL pitch, read from the gz pitch joint (deg, -90 = straight down).

    The FC reports its mount TARGET immediately, but the gz joint slews slowly -- geo-tagging
    through a half-slewed camera puts finds >10 m off. Needs the JointStatePublisher that
    scripts/patch_sim_camera.sh adds; returns None if unavailable.
    """
    import re
    import subprocess
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            env={"HOME": os.environ.get("HOME", "/root"), "PATH": "/usr/local/bin:/usr/bin:/bin"},
            timeout=3, capture_output=True, text=True).stdout
        m = re.search(r'name: "pitch_joint".*?position: (-?[\d.eE+-]+)', out, re.S)
        if m:
            return -math.degrees(float(m.group(1)))
    except Exception:
        pass
    return None


def make_mount_pitch(backend, camera: Optional[str]):
    """Reader for the camera's actual pitch: the gz joint in sim, the FC's report on hardware."""
    is_sim = bool(camera and camera.startswith("gazebo"))

    def read() -> Optional[float]:
        if is_sim:
            p = gazebo_mount_pitch_deg()
            if p is not None:
                return p
        return backend.mount_pitch_deg()
    return read


def make_gimbal_aim(backend, camera: Optional[str]):
    """One 'aim the camera' callable that works in both worlds.

    In Gazebo it drives the rendered camera via the gz topic (MAVLink doesn't move the sim
    camera); on real hardware backend.point_gimbal() commands the mount over MAVLink. It does both
    when in sim so the same code path works everywhere. pitch_deg: -90 = straight down, 0 = forward.
    """
    from .interfaces import CommandResult
    is_sim = bool(camera and camera.startswith("gazebo"))

    def aim(pitch_deg: float = -90.0) -> CommandResult:
        if is_sim:
            gazebo_gimbal_pitch(-math.radians(pitch_deg))   # gz: down = +1.57 rad
            backend.point_gimbal(pitch_deg)                 # harmless; no MAVLink mount in sim
            return CommandResult.success(f"camera pitch {pitch_deg:g} (sim)")
        return backend.point_gimbal(pitch_deg)              # real hardware: MAVLink mount
    return aim


def file_frame_source(path: str) -> Callable[[], Optional[bytes]]:
    """Read a fixed image file each call (handy for piping in saved/real snapshots)."""
    def src() -> Optional[bytes]:
        try:
            with open(path, "rb") as fh:
                return fh.read()
        except OSError:
            return None
    return src


def _gst_udp_h264(port: int) -> str:
    return (f"udpsrc port={port} caps=application/x-rtp,media=video,encoding-name=H264,"
            "clock-rate=90000 ! rtph264depay ! avdec_h264 ! videoconvert ! "
            "appsink drop=true max-buffers=1 sync=false")


class StreamCamera:
    """Snapshot the latest frame from a video stream (RTSP/UDP/GStreamer pipeline) as JPEG.

    Opens lazily on first use and retries if the stream isn't up yet, so constructing it never
    blocks or raises (e.g. before the sim is streaming) -- a failed open just yields no frame.
    """

    def __init__(self, source: str, gstreamer: Optional[bool] = None, warmup_s: float = 2.0):
        import cv2
        self._cv2 = cv2
        self._source = source
        self._use_gst = gstreamer if gstreamer is not None else (" ! " in source)
        self._cap = None
        self.warmup_s = warmup_s

    def _ensure_open(self) -> bool:
        if self._cap is not None:
            return True
        cv2 = self._cv2
        cap = cv2.VideoCapture(self._source, cv2.CAP_GSTREAMER if self._use_gst else cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap.release()
            return False
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # not all backends support it
            pass
        self._cap = cap
        return True

    def __call__(self) -> Optional[bytes]:
        if not self._ensure_open():
            return None
        deadline = time.time() + self.warmup_s
        frame = None
        while time.time() < deadline:
            ok, f = self._cap.read()
            if ok and f is not None:
                frame = f
                break
            time.sleep(0.05)
        if frame is None:
            return None
        ok, buf = self._cv2.imencode(".jpg", frame)
        return buf.tobytes() if ok else None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


def make_frame_source(spec: Optional[str]) -> Optional[Callable[[], Optional[bytes]]]:
    """Build a frame source from a spec string.

    'file:<path>'        a static/snapshot image file
    'gazebo[:<port>]'    Gazebo GstCameraPlugin UDP H.264 stream (default port 5600)
    'rtsp://...' / 'udp://...' / 'http://...'   any stream OpenCV can open (e.g. a real drone)
    """
    if not spec:
        return None
    if spec.startswith("file:"):
        return file_frame_source(spec[len("file:"):])
    if spec == "gazebo" or spec.startswith("gazebo:"):
        port = int(spec.split(":", 1)[1]) if ":" in spec else 5600
        return StreamCamera(_gst_udp_h264(port), gstreamer=True)
    return StreamCamera(spec)
