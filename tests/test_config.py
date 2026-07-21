"""Settings resolution: defaults < config file < env < CLI."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from mavlink_mcp.config import Settings, load_settings


def _args(**kw) -> SimpleNamespace:
    base = dict(config=None, conn=None, backend=None, camera=None,
                enable_actuation=False, allow_real_vehicle=False)
    base.update(kw)
    return SimpleNamespace(**base)


def test_defaults():
    s = load_settings(_args())
    assert s.conn == "tcp:127.0.0.1:5760"
    assert s.backend == "auto"
    assert not s.enable_actuation
    assert s.limits.max_takeoff_alt_m == 120.0


def test_config_file_overrides_defaults(tmp_path):
    cfg = tmp_path / "drone.toml"
    cfg.write_text(
        "[connection]\nuri = 'udp:127.0.0.1:14550'\ntimeout_s = 10\n"
        "[backend]\ntype = 'fake'\n"
        "[safety]\nenable_actuation = true\nmax_takeoff_alt_m = 30\n"
        "[camera]\nsource = 'gazebo'\n")
    s = load_settings(_args(config=str(cfg)))
    assert s.conn == "udp:127.0.0.1:14550"
    assert s.connect_timeout_s == 10.0
    assert s.backend == "fake"
    assert s.enable_actuation
    assert s.camera == "gazebo"
    assert s.limits.max_takeoff_alt_m == 30.0
    assert s.limits.max_move_m == 500.0   # untouched keys keep their defaults


def test_cli_beats_config_file(tmp_path):
    cfg = tmp_path / "drone.toml"
    cfg.write_text("[connection]\nuri = 'udp:127.0.0.1:14550'\n[backend]\ntype = 'fake'\n")
    s = load_settings(_args(config=str(cfg), conn="tcp:127.0.0.1:5762", backend="ardupilot"))
    assert s.conn == "tcp:127.0.0.1:5762"
    assert s.backend == "ardupilot"


def test_env_beats_config_file(tmp_path, monkeypatch):
    cfg = tmp_path / "drone.toml"
    cfg.write_text("[connection]\nuri = 'udp:127.0.0.1:14550'\n")
    monkeypatch.setenv("MAVLINK_MCP_CONN", "tcp:127.0.0.1:5763")
    s = load_settings(_args(config=str(cfg)))
    assert s.conn == "tcp:127.0.0.1:5763"


def test_bad_backend_in_config_rejected(tmp_path):
    cfg = tmp_path / "drone.toml"
    cfg.write_text("[backend]\ntype = 'px5'\n")
    with pytest.raises(SystemExit):
        load_settings(_args(config=str(cfg)))


def test_missing_config_file_rejected():
    with pytest.raises(SystemExit):
        load_settings(_args(config="/nonexistent/drone.toml"))


def test_settings_default_limits_are_independent():
    a, b = Settings(), Settings()
    a.limits.max_takeoff_alt_m = 5.0
    assert b.limits.max_takeoff_alt_m == 120.0
