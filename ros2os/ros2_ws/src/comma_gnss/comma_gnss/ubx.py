"""u-blox UBX protocol: framing, NAV-PVT/MON-HW decoding and the receiver init sequence (no ROS)."""
import calendar
import struct
from collections.abc import Iterator

from comma_gnss.types import (COV_APPROXIMATED, COV_UNKNOWN, SERVICE_GLONASS, SERVICE_GPS, STATUS_FIX,
                              STATUS_GBAS_FIX, STATUS_NO_FIX, STATUS_SBAS_FIX, GnssFix, GnssTime, GnssVelocity, diag3)

SYNC = b"\xb5\x62"
CLS_NAV, CLS_ACK, CLS_CFG, CLS_MON, CLS_MGA = 0x01, 0x05, 0x06, 0x0A, 0x13
NAV_PVT, NAV_SAT = 0x07, 0x35
ACK_NAK, ACK_ACK = 0x00, 0x01
MON_HW = 0x09


def checksum(data: bytes) -> bytes:
  """8-bit Fletcher checksum over class, id, length and payload."""
  a = b = 0
  for c in data:
    a = (a + c) & 0xFF
    b = (b + a) & 0xFF
  return bytes((a, b))


def frame(cls: int, msg_id: int, payload: bytes = b"") -> bytes:
  body = struct.pack("<BBH", cls, msg_id, len(payload)) + payload
  return SYNC + body + checksum(body)


def frame_valid(msg: bytes) -> bool:
  return msg.startswith(SYNC) and len(msg) >= 8 and checksum(msg[2:-2]) == msg[-2:]


class UbxParser:
  """Incremental UBX frame extractor; NMEA or garbage between frames is skipped."""

  MAX_PAYLOAD = 8192

  def __init__(self):
    self.buf = bytearray()

  def feed(self, data: bytes) -> Iterator[tuple[int, int, bytes]]:
    self.buf += data
    while True:
      start = self.buf.find(SYNC)
      if start < 0:
        del self.buf[:-1]  # keep a possible partial sync byte
        return
      del self.buf[:start]
      if len(self.buf) < 6:
        return
      cls, msg_id, length = struct.unpack_from("<BBH", self.buf, 2)
      if length > self.MAX_PAYLOAD:
        del self.buf[:2]
        continue
      if len(self.buf) < 8 + length:
        return
      msg = bytes(self.buf[:8 + length])
      if frame_valid(msg):
        del self.buf[:8 + length]
        yield cls, msg_id, msg[6:-2]
      else:
        del self.buf[:2]


def decode_nav_pvt(p: bytes) -> tuple[GnssFix, GnssVelocity | None, GnssTime | None]:
  """UBX-NAV-PVT (92 bytes) -> fix, velocity and receiver time."""
  if len(p) < 92:
    raise ValueError(f"NAV-PVT payload too short: {len(p)}")
  (_itow, year, month, day, hour, minute, sec, valid, _tacc, nano, fix_type, flags, _flags2, num_sv,
   lon, lat, height, _hmsl, hacc, vacc, vel_n, vel_e, vel_d, _gspeed, _headmot, sacc, _headacc, pdop) = \
    struct.unpack_from("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH", p, 0)

  gnss_fix_ok = bool(flags & 0x01)
  diff_soln = bool(flags & 0x02)
  carr_soln = (flags >> 6) & 0x03

  utc_ns = None
  if (valid & 0x07) == 0x07:  # validDate, validTime, fullyResolved
    utc_ns = calendar.timegm((year, month, day, hour, minute, sec)) * 1_000_000_000 + nano

  if not gnss_fix_ok or fix_type in (0, 5):
    status = STATUS_NO_FIX
  elif carr_soln > 0:
    status = STATUS_GBAS_FIX
  elif diff_soln:
    status = STATUS_SBAS_FIX
  else:
    status = STATUS_FIX

  fix = GnssFix(latitude=lat * 1e-7, longitude=lon * 1e-7, altitude=height * 1e-3, status=status,
                service=SERVICE_GPS | SERVICE_GLONASS, utc_ns=utc_ns, num_sv=num_sv, hdop=pdop * 0.01)
  if status != STATUS_NO_FIX:
    # hAcc/vAcc are accuracy estimates without a stated confidence level; like common u-blox drivers (and
    # tools/mcap) treat them as 1-sigma and apply hAcc to both horizontal axes -> APPROXIMATED.
    h, v = hacc * 1e-3, vacc * 1e-3
    fix.position_covariance, fix.covariance_type = diag3(h * h, h * h, v * v), COV_APPROXIMATED
  else:
    fix.covariance_type = COV_UNKNOWN

  vel = None
  if status != STATUS_NO_FIX:
    s = (sacc * 1e-3) ** 2
    vel = GnssVelocity((vel_e * 1e-3, vel_n * 1e-3, -vel_d * 1e-3), diag3(s, s, s), vertical_known=True, utc_ns=utc_ns)
  return fix, vel, (GnssTime(utc_ns) if utc_ns is not None else None)


ANTENNA_STATUS = {0: "init", 1: "unknown", 2: "ok", 3: "short", 4: "open"}


def decode_mon_hw(p: bytes) -> dict:
  if len(p) < 60:
    raise ValueError("MON-HW payload too short")
  noise, agc, a_status, a_power = struct.unpack_from("<HHBB", p, 16)
  return {"noise_per_ms": noise, "agc_count": agc, "antenna": ANTENNA_STATUS.get(a_status, str(a_status)),
          "antenna_power": a_power, "jamming_indicator": p[45]}


# *** receiver configuration ***
# Port, navigation and odometer configuration from openpilot's pigeond, which has
# been used on every comma 3X: UART1 at 460800 baud, UBX only, automotive model.
PIGEOND_CONFIG = [
  b"\xb5\x62\x06\x00\x14\x00\x03\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x00\x00\x00\x00\x00\x1E\x7F",
  b"\xb5\x62\x06\x00\x14\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x19\x35",
  b"\xb5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x01\x00\x00\x00\x00\x00\xF4\x80",
  b"\xb5\x62\x06\x00\x14\x00\x04\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1D\x85",
  b"\xb5\x62\x06\x00\x00\x00\x06\x18",
  b"\xb5\x62\x06\x00\x01\x00\x01\x08\x22",
  b"\xb5\x62\x06\x00\x01\x00\x03\x0A\x24",
  "RATE",  # placeholder for UBX-CFG-RATE, generated from the requested rate
  (b"\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" +
   b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63"),
  b"\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37",
  b"\xB5\x62\x06\x39\x08\x00\xFF\xAD\x62\xAD\x1E\x63\x00\x00\x83\x0C",
  (b"\xB5\x62\x06\x23\x28\x00\x00\x00\x00\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00" +
   b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x56\x24"),
  b"\xB5\x62\x06\x24\x00\x00\x2A\x84",
  b"\xB5\x62\x06\x23\x00\x00\x29\x81",
  b"\xB5\x62\x06\x1E\x00\x00\x24\x72",
  b"\xB5\x62\x06\x39\x00\x00\x3F\xC3",
]

# $PUBX,41,1,0007,0003,460800,0*15 : switch UART1 from the 9600 baud power-on default to 460800
PUBX_BAUD_460800 = b"$PUBX,41,1,0007,0003,460800,0*15\r\n"
CFG_RST_GNSS_STOP = frame(CLS_CFG, 0x04, b"\x00\x00\x08\x00")  # controlled GNSS stop (pigeond shutdown)


def cfg_rate(rate_hz: float) -> bytes:
  meas_ms = max(25, min(10000, round(1000.0 / rate_hz)))
  return frame(CLS_CFG, 0x08, struct.pack("<HHH", meas_ms, 1, 0))


def cfg_msg(cls: int, msg_id: int, rate: int) -> bytes:
  return frame(CLS_CFG, 0x01, bytes((cls, msg_id, rate)))


def init_sequence(rate_hz: float = 10.0) -> list[bytes]:
  seq = [cfg_rate(rate_hz) if m == "RATE" else m for m in PIGEOND_CONFIG]
  seq += [
    cfg_msg(CLS_NAV, NAV_PVT, 1),                      # every navigation solution
    cfg_msg(CLS_MON, MON_HW, max(1, round(rate_hz))),  # ~1 Hz antenna / jamming status
  ]
  return seq


def mga_ini_time_utc(year: int, month: int, day: int, hour: int, minute: int, second: int, acc_s: int = 30) -> bytes:
  """UBX-MGA-INI-TIME_UTC to speed up the first fix when the system clock is known good (as pigeond does)."""
  payload = struct.pack("<BBBBHBBBBBxIHxxI", 0x10, 0x00, 0x00, 0x80, year, month, day, hour, minute, second, 0, acc_s, 0)
  return frame(CLS_MGA, 0x40, payload)


def is_ack(cls: int, msg_id: int, payload: bytes, for_msg: bytes) -> bool | None:
  """True for ACK-ACK of `for_msg`, False for ACK-NAK, None if unrelated."""
  if cls != CLS_ACK or len(payload) < 2 or (payload[0], payload[1]) != (for_msg[2], for_msg[3]):
    return None
  return msg_id == ACK_ACK
