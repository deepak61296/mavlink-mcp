"""Drive a full mission through the real MCP stdio protocol (client -> server -> SITL).

This is the exact path an MCP client (Claude Code, Codex, ...) takes: it launches
`mavlink-mcp` as a subprocess, speaks MCP over stdio, and calls the flight tools in order.
Run with SITL already listening on tcp:127.0.0.1:5760.
"""
from __future__ import annotations

import asyncio
import os
import shutil
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


def _text(result) -> str:
    parts = []
    for block in result.content:
        parts.append(getattr(block, "text", str(block)))
    return "\n".join(parts)


async def main() -> int:
    server_cmd = shutil.which("mavlink-mcp") or os.path.join(
        os.path.dirname(sys.executable), "mavlink-mcp")
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    params = StdioServerParameters(
        command=server_cmd,
        args=["--enable-actuation", "--conn", "tcp:127.0.0.1:5760"],
        env=env,
    )

    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            print("tools exposed:", ", ".join(sorted(t.name for t in tools.tools)))

            async def call(name: str, **kw) -> str:
                out = _text(await session.call_tool(name, kw))
                print(f"\n>>> {name}({', '.join(f'{k}={v}' for k, v in kw.items())})")
                print(out)
                return out

            print("\n=== status before flight ===")
            print(_text(await session.call_tool("get_status", {})))

            # wait for the vehicle to become armable (EKF/GPS settle on fresh boot)
            print("\n=== waiting for armable ===")
            for _ in range(45):
                s = _text(await session.call_tool("check_armable", {}))
                if s.strip() == "ready to arm":
                    print("armable:", s)
                    break
                await asyncio.sleep(1)
            else:
                print("never became armable:", s)
                return 1

            # ---- the mission ----
            await call("takeoff", altitude_m=20)
            await call("move", direction="left", distance_m=70)
            await call("move", direction="right", distance_m=10)
            await call("rtl")

            print("\n=== status after mission ===")
            print(_text(await session.call_tool("get_status", {})))
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
