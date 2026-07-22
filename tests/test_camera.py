"""Camera source selection, and the OpenCV build mismatch that used to fail silently.

`pip install "mavlink-mcp[camera]"` gets the opencv-python wheel, which is built with
GStreamer off and therefore cannot read the Gazebo stream at all. That used to show up as a
camera that simply never produced a frame.
"""
from __future__ import annotations

import pytest

from mavlink_mcp import camera as cam
from mavlink_mcp.backends.fake import FakeBackend
from mavlink_mcp.server import Settings, VehicleSession


def test_file_source_needs_no_opencv(tmp_path):
    """A file: source is just bytes off disk, so it must not depend on the OpenCV build."""
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"\xff\xd8body")
    source = cam.make_frame_source(f"file:{image}")
    assert source() == b"\xff\xd8body"


def test_no_camera_configured_is_not_a_problem():
    assert cam.make_frame_source(None) is None
    assert cam.make_frame_source("") is None


def test_gazebo_without_gstreamer_refuses_loudly(monkeypatch):
    monkeypatch.setattr(cam, "gstreamer_missing", lambda: "this OpenCV has no GStreamer")
    with pytest.raises(RuntimeError) as excinfo:
        cam.make_frame_source("gazebo")
    assert "GStreamer" in str(excinfo.value)


def test_gazebo_with_gstreamer_builds_a_stream(monkeypatch):
    monkeypatch.setattr(cam, "gstreamer_missing", lambda: None)
    monkeypatch.setattr(cam, "StreamCamera", lambda spec, **kw: ("stream", spec))
    kind, pipeline = cam.make_frame_source("gazebo:5601")
    assert kind == "stream"
    assert "udpsrc port=5601" in pipeline


def test_server_survives_an_unusable_camera(monkeypatch):
    """The flight tools are still worth having; dying at startup tells an MCP client nothing."""
    monkeypatch.setattr(cam, "gstreamer_missing", lambda: "this OpenCV has no GStreamer")
    session = VehicleSession(Settings(backend="fake", camera="gazebo"), FakeBackend())
    assert session.frames is None
    assert "GStreamer" in (session.camera_problem or "")


def test_capture_camera_explains_the_missing_build(monkeypatch):
    import asyncio

    from mavlink_mcp.server import build_server

    monkeypatch.setattr(cam, "gstreamer_missing", lambda: "this OpenCV has no GStreamer")
    mcp = build_server(Settings(backend="fake", camera="gazebo"))
    out = str(asyncio.run(mcp.call_tool("capture_camera", {})))
    assert "camera unavailable" in out and "GStreamer" in out


def test_gstreamer_probe_reads_the_real_build():
    """Whatever this machine has, the probe must answer with a string or None, never raise."""
    problem = cam.gstreamer_missing()
    assert problem is None or isinstance(problem, str)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
