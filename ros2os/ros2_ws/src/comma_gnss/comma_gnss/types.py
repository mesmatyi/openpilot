"""Receiver-independent GNSS solution types (no ROS)."""
from dataclasses import dataclass, field

# sensor_msgs/NavSatStatus
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2

SERVICE_GPS = 1
SERVICE_GLONASS = 2
SERVICE_COMPASS = 4  # BeiDou
SERVICE_GALILEO = 8

# sensor_msgs/NavSatFix covariance types
COV_UNKNOWN = 0
COV_APPROXIMATED = 1
COV_DIAGONAL_KNOWN = 2
COV_KNOWN = 3


def diag3(a: float, b: float, c: float) -> tuple[float, ...]:
  return (a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c)


@dataclass
class GnssFix:
  latitude: float                  # deg, WGS84
  longitude: float                 # deg
  altitude: float                  # m above the WGS84 ellipsoid
  status: int = STATUS_NO_FIX
  service: int = SERVICE_GPS
  position_covariance: tuple[float, ...] = field(default_factory=lambda: (0.0,) * 9)  # ENU, m^2
  covariance_type: int = COV_UNKNOWN
  utc_ns: int | None = None        # receiver time of the solution, if the date is known
  num_sv: int = 0
  hdop: float = float("nan")


@dataclass
class GnssVelocity:
  enu: tuple[float, float, float]  # m/s east, north, up
  covariance: tuple[float, ...]    # ENU, (m/s)^2
  vertical_known: bool = True
  utc_ns: int | None = None


@dataclass
class GnssTime:
  utc_ns: int
