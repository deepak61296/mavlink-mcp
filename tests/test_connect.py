"""Connect-path failures: bounded, and reported in words rather than raised.

The first thing a new user hits is an MCP client starting this server before SITL is up.
That path used to surface a bare ConnectionRefusedError, so the message written for it was
never actually reachable.
"""
from __future__ import annotations

import asyncio
import socket
import time

import pytest

from mavlink_mcp.backends.ardupilot import MavlinkBackend
from mavlink_mcp.config import Settings
from mavlink_mcp.server import build_server, guarded


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_refused_connection_returns_a_result_not_an_exception():
    b = MavlinkBackend()
    res = b.connect(f"tcp:127.0.0.1:{_free_port()}", timeout_s=5)
    assert not res.ok
    assert "refused" in res.message


def test_refused_connection_is_prompt():
    """Six retries at ~1s each used to make even the refused case needlessly slow."""
    b = MavlinkBackend()
    t0 = time.monotonic()
    b.connect(f"tcp:127.0.0.1:{_free_port()}", timeout_s=20)
    assert time.monotonic() - t0 < 5.0


def test_unreachable_host_respects_connect_timeout():
    """A peer that drops SYNs must not hold the server for the kernel's ~130s retry.

    192.0.2.0/24 is TEST-NET-1 (RFC 5737): reserved, routed nowhere, so the SYN is
    swallowed rather than refused - exactly the case that used to wedge the server.
    """
    b = MavlinkBackend()
    t0 = time.monotonic()
    res = b.connect("tcp:192.0.2.1:5760", timeout_s=3)
    elapsed = time.monotonic() - t0
    assert not res.ok
    assert elapsed < 15.0, f"took {elapsed:.0f}s; connect_timeout_s was ignored"


def test_connect_does_not_leak_the_socket_default_timeout():
    before = socket.getdefaulttimeout()
    MavlinkBackend().connect(f"tcp:127.0.0.1:{_free_port()}", timeout_s=2)
    assert socket.getdefaulttimeout() == before


def test_server_reports_a_missing_vehicle_in_words():
    mcp = build_server(Settings(conn=f"tcp:127.0.0.1:{_free_port()}", connect_timeout_s=3))
    out = asyncio.run(mcp.call_tool("get_status", {}))
    text = str(out)
    assert "cannot reach a vehicle" in text
    assert "Errno" not in text and "Traceback" not in text


def test_guard_turns_an_exception_into_a_message():
    @guarded
    async def boom() -> str:
        raise ConnectionRefusedError(111, "Connection refused")

    assert "error:" in asyncio.run(boom())


def test_guard_names_an_exception_that_carries_no_message():
    """FastMCP rendered these as 'Error executing tool rtl:' with nothing after the colon."""
    @guarded
    async def silent() -> str:
        raise RuntimeError()

    assert asyncio.run(silent()) == "error: RuntimeError"


def test_guard_passes_sync_tools_through():
    @guarded
    def fine() -> str:
        return "ok"

    assert fine() == "ok"


def test_camera_tool_still_works_through_the_guard():
    """capture_camera is the one sync tool; an async-only guard silently broke it."""
    mcp = build_server(Settings(backend="fake"))
    out = str(asyncio.run(mcp.call_tool("capture_camera", {})))
    assert "no camera configured" in out
    assert "await" not in out


def test_link_timeout_is_configurable_from_the_config_file(tmp_path):
    cfg = tmp_path / "d.toml"
    cfg.write_text("[connection]\nlink_timeout_s = 12.5\n")

    class Args:
        config = str(cfg)
        conn = backend = camera = None
        enable_actuation = allow_real_vehicle = allow_unsafe_params = False

    from mavlink_mcp.config import load_settings
    assert load_settings(Args()).link_timeout_s == 12.5


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
