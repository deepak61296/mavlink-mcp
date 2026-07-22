"""Link-loss handling: staleness detection and the guards that depend on it.

These cover the failure that used to be silent - the reader thread dying and the server
serving its last snapshot forever - without needing a vehicle.
"""
from __future__ import annotations

import threading
import time
import types

import pytest

from mavlink_mcp.backends.ardupilot import LINK_TIMEOUT_S, MavlinkBackend
from mavlink_mcp.flight import format_telemetry
from mavlink_mcp.interfaces import Telemetry


def _backend(age_s: float, timeout_s: float = LINK_TIMEOUT_S) -> MavlinkBackend:
    """A backend that believes it is connected and last heard from the vehicle age_s ago."""
    b = MavlinkBackend(link_timeout_s=timeout_s)
    b._connected.set()
    b._tel.connected = True
    b._tel.last_update_s = time.time() - age_s
    return b


def test_fresh_link_has_no_error():
    b = _backend(age_s=0.2)
    b._check_link_stale()
    assert b.link_error() is None
    assert b.get_telemetry().connected


def test_silent_link_is_reported_down():
    b = _backend(age_s=LINK_TIMEOUT_S + 2)
    b._check_link_stale()
    err = b.link_error()
    assert err is not None and "link down" in err
    assert not b.get_telemetry().connected


def test_link_recovers_when_traffic_returns():
    b = _backend(age_s=LINK_TIMEOUT_S + 2)
    b._check_link_stale()
    assert b.link_error() is not None
    b._tel.last_update_s = time.time()          # a message arrives
    b._check_link_stale()
    assert b.link_error() is None               # recovery is implicit, like MAVProxy


def test_arming_status_refuses_on_a_dead_link():
    """The regression that mattered: check_armable said 'ready to arm' for a dead vehicle."""
    b = _backend(age_s=LINK_TIMEOUT_S + 2)
    b._tel.ekf_ok = True
    b._tel.fix_type = 3
    b._home_lat = 51.0
    b._check_link_stale()
    res = b.arming_status()
    assert not res.ok
    assert "link down" in res.message


def test_commands_refuse_on_a_dead_link():
    b = _backend(age_s=LINK_TIMEOUT_S + 2)
    b._check_link_stale()
    for res in (b.set_mode("GUIDED"), b.enable(True), b.emergency_stop()):
        assert not res.ok
        assert "link down" in res.message


def test_stale_telemetry_is_never_printed_as_current():
    t = Telemetry(connected=False, armed=True, mode="GUIDED", alt_rel_m=40.0,
                  last_update_s=time.time() - 30)
    out = format_telemetry(t)
    assert "LINK DOWN" in out
    assert "40" not in out          # the altitude it last saw must not read as current
    assert "armed=True" not in out


def test_never_connected_reads_differently_from_lost():
    assert "NOT CONNECTED" in format_telemetry(Telemetry(connected=False))


def test_reader_marks_link_down_when_every_read_raises(monkeypatch):
    """Drive the real reader loop, not _check_link_stale() directly.

    A link that dies by raising on every read is the common case (pymavlink reconnects from
    inside recv and throws when the peer refuses), and it is the one an early version of this
    fix missed by skipping the staleness check on the exception path.
    """
    b = MavlinkBackend(link_timeout_s=0.3)
    monkeypatch.setattr(b, "_wait_first_heartbeat",
                        lambda: types.SimpleNamespace(autopilot=3, type=2))
    monkeypatch.setattr(b, "_maybe_heartbeat", lambda: None)

    class DeadConn:
        def recv_match(self, **kw):
            raise OSError("link gone")

    b._conn = DeadConn()
    b._tel.connected = True
    b._tel.last_update_s = time.time()
    reader = threading.Thread(target=b._run, daemon=True)
    reader.start()
    try:
        deadline = time.time() + 5
        while time.time() < deadline and b.link_error() is None:
            time.sleep(0.05)
        assert b.link_error() is not None, "reader never flagged a link that raises on every read"
        assert reader.is_alive(), "reader thread died instead of riding out the failure"
    finally:
        b._stop.set()
        reader.join(timeout=3)


def test_link_timeout_is_configurable():
    b = _backend(age_s=2.0, timeout_s=1.0)
    b._check_link_stale()
    assert b.link_error() is not None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
