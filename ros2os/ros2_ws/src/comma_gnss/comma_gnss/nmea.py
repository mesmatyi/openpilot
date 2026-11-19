"""NMEA 0183 decoding into GnssFix / GnssVelocity / GnssTime (no ROS).

Handles GGA (position), RMC (velocity, date/time), GSA (DOPs, constellations),
GST (pseudorange error statistics) and ZDA (date/time) from any talker
(GP, GL, GA, GB/BD, GN). A fix is emitted on every valid GGA, a velocity on every
valid RMC, mirroring the behaviour of the common nmea_navsat_driver.
"""
import calendar
import math

from comma_gnss.types import (COV_APPROXIMATED, COV_DIAGONAL_KNOWN, COV_UNKNOWN, SERVICE_COMPASS, SERVICE_GALILEO,
                              SERVICE_GLONASS, SERVICE_GPS, STATUS_FIX, STATUS_GBAS_FIX, STATUS_NO_FIX, STATUS_SBAS_FIX,
                              GnssFix, GnssTime, GnssVelocity, diag3)

KNOTS_TO_MPS = 0.514444
TALKER_SERVICE = {"GP": SERVICE_GPS, "GL": SERVICE_GLONASS, "GA": SERVICE_GALILEO, "GB": SERVICE_COMPASS,
                  "BD": SERVICE_COMPASS, "GQ": SERVICE_GPS}  # QZSS augments GPS
GSA_SYSTEM_ID = {1: SERVICE_GPS, 2: SERVICE_GLONASS, 3: SERVICE_GALILEO, 4: SERVICE_COMPASS}
# GGA fix quality -> NavSatStatus
GGA_QUALITY = {0: STATUS_NO_FIX, 1: STATUS_FIX, 2: STATUS_SBAS_FIX, 3: STATUS_FIX, 4: STATUS_GBAS_FIX,
               5: STATUS_GBAS_FIX, 6: STATUS_FIX, 7: STATUS_NO_FIX, 8: STATUS_NO_FIX}


def checksum_ok(sentence: str) -> bool:
  if not sentence.startswith(("$", "!")) or "*" not in sentence:
    return False
  body, _, cs = sentence[1:].partition("*")
  try:
    expected = int(cs[:2], 16)
  except ValueError:
    return False
  calc = 0
  for c in body:
    calc ^= ord(c)
  return calc == expected


def add_checksum(body: str) -> str:
  calc = 0
  for c in body:
    calc ^= ord(c)
  return f"${body}*{calc:02X}"


def _float(v: str) -> float | None:
  try:
    return float(v) if v else None
  except ValueError:
    return None


def _int(v: str) -> int | None:
  try:
    return int(v) if v else None
  except ValueError:
    return None


def parse_coord(value: str, hemisphere: str) -> float | None:
  """ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed degrees."""
  if not value or "." not in value:
    return None
  dot = value.index(".")
  try:
    deg = float(value[:dot - 2]) if dot > 2 else 0.0
    minutes = float(value[dot - 2:])
  except ValueError:
    return None
  out = deg + minutes / 60.0
  return -out if hemisphere in ("S", "W") else out


def parse_tod(value: str) -> float | None:
  """hhmmss(.sss) -> seconds of day."""
  if len(value) < 6:
    return None
  try:
    return int(value[0:2]) * 3600 + int(value[2:4]) * 60 + float(value[4:])
  except ValueError:
    return None


def date_to_epoch_day(day: int, month: int, year: int) -> int:
  return calendar.timegm((year, month, day, 0, 0, 0)) // 86400


class NmeaDecoder:
  def __init__(self, uere_m: float = 4.0, velocity_stdev: float = 0.2, gst_max_age_s: float = 1.5):
    self.uere_m = uere_m
    self.velocity_stdev = velocity_stdev
    self.gst_max_age_s = gst_max_age_s
    self.epoch_day: int | None = None     # days since 1970 of the last RMC/ZDA date
    self.date_tod: float | None = None    # time of day that date belongs to
    self.hdop = self.vdop = self.pdop = None
    self.gst: tuple[float, float, float, float] | None = None  # (tod, lat_std, lon_std, alt_std)
    self.service = 0
    self.geoid_missing = False

  def _utc_ns(self, tod: float | None) -> int | None:
    if tod is None or self.epoch_day is None or self.date_tod is None:
      return None
    day = self.epoch_day
    if tod < self.date_tod - 43200:  # crossed midnight since the date was received
      day += 1
    elif tod > self.date_tod + 43200:
      day -= 1
    return day * 86_400_000_000_000 + round(tod * 1e9)

  def feed(self, sentence: str) -> list:
    sentence = sentence.strip()
    if not checksum_ok(sentence):
      return []
    fields = sentence[1:sentence.index("*")].split(",")
    talker, kind = fields[0][:2], fields[0][2:]
    handler = getattr(self, f"_{kind.lower()}", None)
    if handler is None:
      return []
    try:
      return handler(talker, fields)
    except (IndexError, ValueError):
      return []

  # *** sentences ***
  def _gsa(self, talker, f):
    self.pdop, self.hdop, self.vdop = _float(f[15]), _float(f[16]), _float(f[17].split("*")[0])
    system_id = _int(f[18]) if len(f) > 18 else None
    if system_id in GSA_SYSTEM_ID:
      self.service |= GSA_SYSTEM_ID[system_id]
    elif talker in TALKER_SERVICE:
      self.service |= TALKER_SERVICE[talker]
    return []

  def _gsv(self, talker, f):
    if talker in TALKER_SERVICE and (_int(f[3]) or 0) > 0:
      self.service |= TALKER_SERVICE[talker]
    return []

  def _gst(self, talker, f):
    tod = parse_tod(f[1])
    lat_std, lon_std, alt_std = _float(f[6]), _float(f[7]), _float(f[8])
    if None not in (tod, lat_std, lon_std, alt_std):
      self.gst = (tod, lat_std, lon_std, alt_std)
    return []

  def _zda(self, talker, f):
    tod, day, month, year = parse_tod(f[1]), _int(f[2]), _int(f[3]), _int(f[4])
    if None in (tod, day, month, year):
      return []
    self.epoch_day, self.date_tod = date_to_epoch_day(day, month, year), tod
    return [GnssTime(self._utc_ns(tod))]

  def _rmc(self, talker, f):
    tod, status, date = parse_tod(f[1]), f[2], f[9]
    out = []
    if len(date) == 6 and tod is not None:
      day, month, yy = int(date[0:2]), int(date[2:4]), int(date[4:6])
      self.epoch_day, self.date_tod = date_to_epoch_day(day, month, 2000 + yy), tod
    if status != "A":
      return out
    utc = self._utc_ns(tod)
    if utc is not None:
      out.append(GnssTime(utc))
    speed = _float(f[7])
    if speed is not None:
      speed *= KNOTS_TO_MPS
      course = math.radians(_float(f[8]) or 0.0)
      var = self.velocity_stdev ** 2
      out.append(GnssVelocity((speed * math.sin(course), speed * math.cos(course), 0.0), diag3(var, var, 1e6),
                              vertical_known=False, utc_ns=utc))
    return out

  def _gga(self, talker, f):
    tod = parse_tod(f[1])
    lat, lon = parse_coord(f[2], f[3]), parse_coord(f[4], f[5])
    quality = _int(f[6]) or 0
    num_sv = _int(f[7]) or 0
    hdop = _float(f[8])
    msl, sep = _float(f[9]), _float(f[11])
    status = GGA_QUALITY.get(quality, STATUS_NO_FIX)
    if lat is None or lon is None:
      status = STATUS_NO_FIX
    altitude = float("nan")
    if msl is not None:
      if sep is None:
        self.geoid_missing = True
      altitude = msl + (sep or 0.0)

    fix = GnssFix(latitude=lat if lat is not None else float("nan"), longitude=lon if lon is not None else float("nan"),
                  altitude=altitude, status=status, service=self.service or TALKER_SERVICE.get(talker, SERVICE_GPS),
                  utc_ns=self._utc_ns(tod), num_sv=num_sv, hdop=hdop if hdop is not None else float("nan"))
    if status == STATUS_NO_FIX:
      fix.covariance_type = COV_UNKNOWN
    elif self.gst is not None and tod is not None and abs(self.gst[0] - tod) <= self.gst_max_age_s:
      _, lat_std, lon_std, alt_std = self.gst
      fix.position_covariance, fix.covariance_type = diag3(lon_std ** 2, lat_std ** 2, alt_std ** 2), COV_DIAGONAL_KNOWN
    elif hdop is not None:
      sh = hdop * self.uere_m
      sv = (self.vdop if self.vdop is not None else 2.0 * hdop) * self.uere_m
      fix.position_covariance, fix.covariance_type = diag3(sh ** 2, sh ** 2, sv ** 2), COV_APPROXIMATED
    return [fix]
