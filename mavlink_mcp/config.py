"""Settings resolution: defaults < config file < environment < CLI flags.

The config file is optional TOML, given with --config or MAVLINK_MCP_CONFIG. It exists so
an MCP client entry can stay as short as {"command": "mavlink-mcp", "args": ["--config",
"drone.toml"]} while the connection, camera and safety limits live in one reviewable file.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field, fields
from typing import Optional

try:
    import tomllib
except ImportError:  # Python 3.10
    import tomli as tomllib

from .safety import SafetyLimits

_ENV_PREFIX = "MAVLINK_MCP_"
_BACKENDS = ("auto", "ardupilot", "fake")
_SECTIONS = ("connection", "backend", "safety", "camera")
# [safety] keys that are switches on Settings, not numeric fields of SafetyLimits
_SAFETY_FLAGS = ("enable_actuation", "allow_real_vehicle", "allow_unsafe_params")


@dataclass
class Settings:
    conn: str = "tcp:127.0.0.1:5760"
    backend: str = "auto"               # auto | ardupilot | fake (px4 via MAVSDK planned)
    enable_actuation: bool = False
    allow_real_vehicle: bool = False
    allow_unsafe_params: bool = False   # let set_param disable fences/failsafes
    camera: Optional[str] = None        # gazebo[:port] | rtsp://... | udp://... | file:<path>
    connect_timeout_s: float = 25.0
    link_timeout_s: float = 5.0         # silence after which the link counts as lost
    limits: SafetyLimits = field(default_factory=SafetyLimits)


def _read_config(path: str) -> dict:
    try:
        with open(path, "rb") as f:
            cfg = tomllib.load(f)
    except FileNotFoundError:
        raise SystemExit(f"config file not found: {path}")
    except tomllib.TOMLDecodeError as exc:
        raise SystemExit(f"config file {path}: {exc}")
    for section in cfg:
        if section not in _SECTIONS:
            print(f"config: ignoring unknown section [{section}]", file=sys.stderr)
    return cfg


def _limits_from(safety_cfg: dict) -> SafetyLimits:
    limits = SafetyLimits()
    known = {f.name for f in fields(SafetyLimits)}
    for key, value in safety_cfg.items():
        if key in _SAFETY_FLAGS:
            continue
        if key in known:
            setattr(limits, key, float(value))
        else:
            print(f"config: ignoring unknown safety key '{key}'", file=sys.stderr)
    return limits


def load_settings(args) -> Settings:
    """Resolve Settings from an argparse namespace (unset options must be None/False)."""
    cfg: dict = {}
    path = getattr(args, "config", None) or os.environ.get(_ENV_PREFIX + "CONFIG")
    if path:
        cfg = _read_config(path)

    def section(name: str) -> dict:
        value = cfg.get(name, {})
        return value if isinstance(value, dict) else {}

    conn_cfg, backend_cfg = section("connection"), section("backend")
    safety_cfg, camera_cfg = section("safety"), section("camera")

    def pick(cli, env_key, file_value, default):
        if cli is not None:
            return cli
        env = os.environ.get(_ENV_PREFIX + env_key)
        if env is not None:
            return env
        if file_value is not None:
            return file_value
        return default

    backend = pick(args.backend, "BACKEND", backend_cfg.get("type"), "auto")
    if backend not in _BACKENDS:
        raise SystemExit(f"config: backend must be one of {'|'.join(_BACKENDS)}, got '{backend}'")

    return Settings(
        conn=pick(args.conn, "CONN", conn_cfg.get("uri"), "tcp:127.0.0.1:5760"),
        backend=backend,
        enable_actuation=bool(args.enable_actuation or safety_cfg.get("enable_actuation", False)),
        allow_real_vehicle=bool(args.allow_real_vehicle or safety_cfg.get("allow_real_vehicle", False)),
        allow_unsafe_params=bool(getattr(args, "allow_unsafe_params", False)
                                 or safety_cfg.get("allow_unsafe_params", False)),
        camera=pick(args.camera, "CAMERA", camera_cfg.get("source"), None),
        connect_timeout_s=float(conn_cfg.get("timeout_s", 25.0)),
        link_timeout_s=float(conn_cfg.get("link_timeout_s", 5.0)),
        limits=_limits_from(safety_cfg),
    )
