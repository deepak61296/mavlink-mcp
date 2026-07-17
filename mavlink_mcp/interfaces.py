"""Backend contract: the single seam between the agent core and the vehicle.

The agent core only ever talks to a RobotBackend, so the LLM loop, safety gate and tool
registry stay independent of MAVLink details. Vehicle actions (takeoff, goto, ...) are NOT
methods here; they flow through execute_primitive() and are advertised by capabilities(),
which keeps this interface to ~8 methods.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from typing import Any, Optional


class RiskTier(str, Enum):
    """How dangerous an action is. Drives the safety confirm gate in the core."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class Telemetry:
    """Neutral snapshot of robot state. Each backend maps its native fields into this.

    Distances in metres, angles in degrees, speeds in metres/second.
    """
    connected: bool = False
    armed: bool = False
    mode: str = ""
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_rel_m: Optional[float] = None      # height above home/takeoff point
    alt_msl_m: Optional[float] = None
    heading_deg: Optional[float] = None
    groundspeed_ms: Optional[float] = None
    climb_ms: Optional[float] = None
    battery_voltage_v: Optional[float] = None
    battery_remaining_pct: Optional[float] = None
    satellites: Optional[int] = None
    fix_type: Optional[int] = None         # 0=no fix, 3=3D fix
    ekf_ok: bool = False
    last_update_s: float = 0.0

    def copy(self) -> "Telemetry":
        return replace(self)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CommandResult:
    """Uniform result for every backend call. Backends throw nothing across the seam."""
    ok: bool
    message: str = ""
    detail: dict = field(default_factory=dict)

    @classmethod
    def success(cls, message: str = "", **detail: Any) -> "CommandResult":
        return cls(True, message, detail)

    @classmethod
    def failure(cls, message: str = "", **detail: Any) -> "CommandResult":
        return cls(False, message, detail)


@dataclass
class Primitive:
    """A robot-specific action request, dispatched through execute_primitive()."""
    name: str
    params: dict = field(default_factory=dict)


@dataclass
class PrimitiveSpec:
    """Describes one primitive so the core can build the LLM tool catalog + risk table."""
    name: str
    description: str
    params_schema: dict                    # JSON schema for the LLM tool parameters
    risk: RiskTier = RiskTier.MEDIUM


@dataclass
class Capability:
    """What a backend can do. The core reads this at connect time, never hardcodes it."""
    modes: list[str] = field(default_factory=list)
    primitives: list[PrimitiveSpec] = field(default_factory=list)


class RobotBackend(ABC):
    """The minimal contract the vehicle backend implements (~8 methods)."""

    @abstractmethod
    def connect(self, uri: str, timeout_s: float = 30.0) -> CommandResult:
        """Open the link. For MAVLink this also waits for a heartbeat so the target locks."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the link and stop any background threads."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        ...

    @abstractmethod
    def get_telemetry(self) -> Telemetry:
        """Return the latest neutral snapshot. Cheap, non-blocking, safe to call often."""

    @abstractmethod
    def set_mode(self, mode: str) -> CommandResult:
        """Switch operating mode by platform name; backend resolves and confirms it."""

    @abstractmethod
    def enable(self, on: bool) -> CommandResult:
        """Generalised arm/disarm: enable or disable actuation."""

    @abstractmethod
    def execute_primitive(self, primitive: Primitive) -> CommandResult:
        """Dispatch a robot-specific action. Returns once the command is accepted, NOT once
        it physically completes -- completion is observed by polling get_telemetry()."""

    @abstractmethod
    def emergency_stop(self) -> CommandResult:
        """Best-effort safe stop. Maps to RTL/LAND for a drone, brake/cutoff for a rover."""

    @abstractmethod
    def capabilities(self) -> Capability:
        """List supported modes and primitives with their param schema and risk tier."""

    def arming_status(self) -> CommandResult:
        """Whether the robot is ready to be enabled/armed, with a human reason if not.

        Default assumes ready; backends with prearm checks (e.g. ArduPilot needs a settled
        position estimate) override this so arm/takeoff can wait and report the real blocker.
        """
        return CommandResult.success("ready")

    def point_gimbal(self, pitch_deg: float, yaw_deg: float = 0.0) -> CommandResult:
        """Point the camera gimbal (degrees; pitch -90 = straight down, 0 = forward).

        Default is a no-op for vehicles without a gimbal; backends with a mount (e.g. ArduPilot
        over MAVLink) override this so precision landing and perception can look down.
        """
        return CommandResult.success("no gimbal")

    def mount_pitch_deg(self) -> Optional[float]:
        """The gimbal's ACTUAL pitch as reported by the vehicle, or None if unknown.

        Commands are not state: a mount slews slowly, so geolocating what the camera sees
        must use the reported angle, not the last commanded one.
        """
        return None

    def fence_ceiling_m(self) -> Optional[float]:
        """The highest safe target altitude under the vehicle's altitude fence, or None.

        Commanding a takeoff above the fence makes the FC refuse or breach-RTL, so
        altitude clamps should honour this over any static limit.
        """
        return None
