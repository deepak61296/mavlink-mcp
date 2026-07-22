"""Fixtures for the SITL suite: a real server, a real vehicle, the real MCP protocol.

Opt-in (`-m sitl`); they need ArduPilot SITL on MAVLINK_MCP_TEST_CONN. Everything goes over
stdio JSON-RPC rather than into the package, so these exercise what an MCP client really
gets, schemas included.

There is exactly ONE server fixture on purpose. ArduPilot's SITL serves a single MAVLink
client on its TCP port: a second connection is accepted at the socket level and then never
sees a heartbeat, so two servers against one SITL silently produce a dead second client.
"""
from __future__ import annotations

import asyncio
import os
import shutil
import socket
import sys

import pytest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

CONN = os.environ.get("MAVLINK_MCP_TEST_CONN", "tcp:127.0.0.1:5760")
ENV = {"PATH": "/usr/bin:/bin:/usr/local/bin", "HOME": os.environ.get("HOME", "/tmp")}


def _reachable(conn: str, timeout: float = 3.0) -> bool:
    try:
        _, host, port = conn.split(":")
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except (ValueError, OSError):
        return False


@pytest.fixture(scope="session")
def server_bin() -> str:
    """Absolute path to the entry point: the subprocess env is scrubbed, so PATH won't do."""
    found = shutil.which("mavlink-mcp") or os.path.join(
        os.path.dirname(sys.executable), "mavlink-mcp")
    if not os.path.exists(found):
        pytest.skip("mavlink-mcp is not installed (pip install -e .)")
    return found


@pytest.fixture(scope="session")
def sitl_conn() -> str:
    if not _reachable(CONN):
        pytest.skip(f"no SITL at {CONN}; start ArduCopter SITL or set MAVLINK_MCP_TEST_CONN")
    return CONN


class Client:
    """Synchronous wrapper so tests read as flight steps rather than async plumbing."""

    def __init__(self, session: ClientSession, loop: asyncio.AbstractEventLoop):
        self._s, self._loop = session, loop

    def call(self, tool: str, **args) -> str:
        res = self.raw(tool, **args)
        return res.content[0].text if res.content else ""

    def raw(self, tool: str, **args):
        return self._loop.run_until_complete(self._s.call_tool(tool, args))

    def schema(self, tool: str) -> dict:
        return {t.name: t.inputSchema
                for t in self._loop.run_until_complete(self._s.list_tools()).tools}[tool]

    def tool_names(self) -> set:
        return {t.name for t in self._loop.run_until_complete(self._s.list_tools()).tools}

    def resource(self, uri: str) -> str:
        return self._loop.run_until_complete(self._s.read_resource(uri)).contents[0].text

    def probe_during(self, long_tool: str, long_args: dict,
                     probe_tool: str, probe_args: dict, after_s: float = 3.0):
        """Start a blocking flight command, then call another tool while it is still running.

        Uses one event loop rather than a helper thread: the loop is not thread-safe, and
        driving it from two threads is its own bug rather than a test of the server.
        Returns (probe_text, long_text).
        """
        async def run():
            running = asyncio.ensure_future(self._s.call_tool(long_tool, long_args))
            await asyncio.sleep(after_s)
            probe = await self._s.call_tool(probe_tool, probe_args)
            return probe, await running

        probe, long = self._loop.run_until_complete(run())
        text = lambda r: r.content[0].text if r.content else ""   # noqa: E731
        return text(probe), text(long)

    def sleep(self, seconds: float) -> None:
        self._loop.run_until_complete(asyncio.sleep(seconds))

    def wait_disarmed(self, timeout_s: int = 240) -> str:
        for _ in range(timeout_s // 2):
            out = self.call("get_status")
            if "armed=False" in out:
                return out
            self.sleep(2)
        return self.call("get_status")

    def until_armable(self, timeout_s: int = 120) -> bool:
        for _ in range(timeout_s // 2):
            if "ready" in self.call("check_armable"):
                return True
            self.sleep(2)
        return False


@pytest.fixture(scope="session")
def drone(sitl_conn, server_bin):
    """The one server for the whole session, actuation enabled. Lands on the way out."""
    loop = asyncio.new_event_loop()
    ctx = stdio_client(StdioServerParameters(
        command=server_bin, args=["--conn", sitl_conn, "--enable-actuation"], env=ENV))
    read, write = loop.run_until_complete(ctx.__aenter__())
    sess_ctx = ClientSession(read, write)
    sess = loop.run_until_complete(sess_ctx.__aenter__())
    loop.run_until_complete(sess.initialize())
    client = Client(sess, loop)

    yield client

    try:
        if "armed=True" in client.call("get_status"):
            client.call("rtl")
    except Exception:
        pass
    for c in (sess_ctx, ctx):
        try:
            loop.run_until_complete(c.__aexit__(None, None, None))
        except Exception:
            pass
    loop.close()


@pytest.fixture
def grounded(drone):
    """Guarantee the vehicle starts a test on the ground, whatever the last one left."""
    if "armed=True" in drone.call("get_status"):
        drone.call("rtl")
        drone.wait_disarmed()
    return drone
