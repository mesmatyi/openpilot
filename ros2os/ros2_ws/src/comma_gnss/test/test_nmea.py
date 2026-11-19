import calendar
import math

import pytest

from comma_gnss.nmea import NmeaDecoder, add_checksum, checksum_ok, parse_coord, parse_tod
from comma_gnss.types import (COV_APPROXIMATED, COV_DIAGONAL_KNOWN, COV_UNKNOWN, SERVICE_GALILEO, SERVICE_GLONASS,
                              SERVICE_GPS, STATUS_FIX, STATUS_GBAS_FIX, STATUS_NO_FIX, GnssFix, GnssTime, GnssVelocity)


def s(body):
  return add_checksum(body)


def test_checksum():
  assert checksum_ok("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
  assert not checksum_ok("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48")
  assert not checksum_ok("GPGGA,123519*47")
  assert not checksum_ok("$GPGGA,123519")


def test_coords_and_time():
  assert parse_coord("4807.038", "N") == pytest.approx(48 + 7.038 / 60)
  assert parse_coord("01131.000", "W") == pytest.approx(-(11 + 31.0 / 60))
  assert parse_coord("", "N") is None
  assert parse_tod("123519.50") == pytest.approx(12 * 3600 + 35 * 60 + 19.5)


def test_classic_gga_example():
  d = NmeaDecoder(uere_m=4.0)
  (fix,) = d.feed("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
  assert isinstance(fix, GnssFix)
  assert fix.status == STATUS_FIX
  assert fix.latitude == pytest.approx(48.1173)
  assert fix.longitude == pytest.approx(11.516666, abs=1e-6)
  assert fix.altitude == pytest.approx(545.4 + 46.9)  # ellipsoidal height
  assert fix.num_sv == 8
  assert fix.utc_ns is None  # no date yet
  assert fix.covariance_type == COV_APPROXIMATED
  sh = 0.9 * 4.0
  assert fix.position_covariance[0] == pytest.approx(sh ** 2)
  assert fix.position_covariance[8] == pytest.approx((2 * 0.9 * 4.0) ** 2)


def test_rmc_velocity_time_and_dated_fix():
  d = NmeaDecoder(velocity_stdev=0.2)
  out = d.feed(s("GNRMC,083559.00,A,4717.11437,N,00833.91522,E,10.0,90.0,091202,,,A"))
  t = next(x for x in out if isinstance(x, GnssTime))
  v = next(x for x in out if isinstance(x, GnssVelocity))
  expected = calendar.timegm((2002, 12, 9, 8, 35, 59)) * 10**9
  assert t.utc_ns == expected
  assert v.enu[0] == pytest.approx(10.0 * 0.514444)  # course 90 deg -> due east
  assert v.enu[1] == pytest.approx(0.0, abs=1e-9)
  assert not v.vertical_known
  assert v.covariance[0] == pytest.approx(0.04)

  (fix,) = d.feed(s("GNGGA,083559.00,4717.11437,N,00833.91522,E,1,08,1.01,499.6,M,48.0,M,,"))
  assert fix.utc_ns == expected


def test_void_rmc_produces_no_velocity():
  d = NmeaDecoder()
  assert d.feed(s("GPRMC,083559.00,V,,,,,,,091202,,,N")) == []


def test_midnight_rollover():
  d = NmeaDecoder()
  d.feed(s("GPRMC,235959.00,A,4717.0,N,00833.0,E,0.0,,311224,,,A"))
  (fix,) = d.feed(s("GPGGA,000000.00,4717.0,N,00833.0,E,1,08,1.0,10.0,M,1.0,M,,"))
  assert fix.utc_ns == calendar.timegm((2025, 1, 1, 0, 0, 0)) * 10**9


def test_gst_gives_known_diagonal_covariance_in_enu_order():
  d = NmeaDecoder()
  d.feed(s("GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031"))
  (fix,) = d.feed(s("GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031"))
  assert fix.covariance_type == COV_DIAGONAL_KNOWN
  assert fix.position_covariance[0] == pytest.approx(0.020 ** 2)  # east = longitude std
  assert fix.position_covariance[4] == pytest.approx(0.023 ** 2)  # north = latitude std
  assert fix.position_covariance[8] == pytest.approx(0.031 ** 2)
  assert fix.longitude < 0


def test_stale_gst_falls_back_to_dop():
  d = NmeaDecoder()
  d.feed(s("GPGST,172800.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031"))
  d.feed(s("GNGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1,1"))
  (fix,) = d.feed(s("GPGGA,172814.0,3723.4,N,12202.2,W,1,6,1.3,18.8,M,-25.6,M,,"))
  assert fix.covariance_type == COV_APPROXIMATED
  assert fix.position_covariance[8] == pytest.approx((2.1 * 4.0) ** 2)  # VDOP from GSA


def test_rtk_and_no_fix_quality():
  d = NmeaDecoder()
  (fix,) = d.feed(s("GPGGA,120000.0,4717.0,N,00833.0,E,4,12,0.6,10.0,M,1.0,M,1.0,0001"))
  assert fix.status == STATUS_GBAS_FIX
  (fix,) = d.feed(s("GPGGA,120001.0,,,,,0,00,99.99,,,,,,"))
  assert fix.status == STATUS_NO_FIX
  assert fix.covariance_type == COV_UNKNOWN
  assert math.isnan(fix.latitude)


def test_constellations_from_gsa_and_gsv():
  d = NmeaDecoder()
  d.feed(s("GNGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2,1"))
  d.feed(s("GNGSA,A,3,65,66,,,,,,,,,,,1.5,0.9,1.2,2"))
  d.feed(s("GAGSV,1,1,02,01,40,083,46,02,17,308,41,,,,"))
  (fix,) = d.feed(s("GNGGA,120000.0,4717.0,N,00833.0,E,1,12,0.9,10.0,M,1.0,M,,"))
  assert fix.service == SERVICE_GPS | SERVICE_GLONASS | SERVICE_GALILEO


def test_missing_geoid_separation_is_flagged():
  d = NmeaDecoder()
  (fix,) = d.feed(s("GPGGA,120000.0,4717.0,N,00833.0,E,1,08,1.0,100.0,M,,M,,"))
  assert fix.altitude == pytest.approx(100.0)
  assert d.geoid_missing


def test_garbage_is_ignored():
  d = NmeaDecoder()
  assert d.feed("") == []
  assert d.feed("$GPGGA,1,2*00") == []
  assert d.feed(s("GPXYZ,1,2,3")) == []
  assert d.feed(s("GPGGA,short")) == []
