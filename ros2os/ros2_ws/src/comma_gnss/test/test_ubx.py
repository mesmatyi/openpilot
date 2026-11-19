import calendar
import struct

import pytest

from comma_gnss import ubx
from comma_gnss.types import COV_APPROXIMATED, STATUS_FIX, STATUS_GBAS_FIX, STATUS_NO_FIX


def nav_pvt(fix_type=3, flags=0x01, valid=0x07, lat=47.123456789, lon=8.5, height_mm=500_000, hacc=1500, vacc=2500,
            vel=(1000, 2000, -300), sacc=150, nano=-250_000_000):
  p = struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH", 123456, 2026, 11, 19, 12, 30, 15, valid, 50, nano, fix_type, flags,
                  0, 14, round(lon * 1e7), round(lat * 1e7), height_mm, height_mm - 48_000, hacc, vacc, *vel, 2236,
                  0, sacc, 0, 123)
  return p + bytes(92 - len(p))


def test_checksum_and_frame_roundtrip():
  msg = ubx.frame(0x06, 0x08, b"\x64\x00\x01\x00\x00\x00")
  # pigeond's literal UBX-CFG-RATE (100 ms) must be reproduced exactly
  assert msg == b"\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10"
  assert ubx.frame_valid(msg)
  assert not ubx.frame_valid(msg[:-1] + b"\x00")


def test_pigeond_frames_have_valid_checksums():
  for m in ubx.PIGEOND_CONFIG:
    if m != "RATE":
      assert ubx.frame_valid(m), m.hex()
      assert len(m) == 8 + struct.unpack_from("<H", m, 4)[0]
  assert ubx.frame_valid(ubx.CFG_RST_GNSS_STOP)
  assert ubx.CFG_RST_GNSS_STOP == b"\xB5\x62\x06\x04\x04\x00\x00\x00\x08\x00\x16\x74"  # pigeond literal


def test_init_sequence():
  seq = ubx.init_sequence(10.0)
  assert all(ubx.frame_valid(m) for m in seq)
  assert ubx.cfg_rate(10.0) in seq
  assert ubx.cfg_msg(ubx.CLS_NAV, ubx.NAV_PVT, 1) in seq
  assert ubx.cfg_msg(0x01, 0x07, 1) == b"\xB5\x62\x06\x01\x03\x00\x01\x07\x01\x13\x51"  # pigeond literal
  assert struct.unpack_from("<H", ubx.cfg_rate(5.0), 6)[0] == 200


def test_pubx_baud_sentence_checksum():
  from comma_gnss.nmea import checksum_ok
  assert checksum_ok(ubx.PUBX_BAUD_460800.decode().strip())


def test_parser_resyncs_through_garbage_and_nmea():
  msg = ubx.frame(ubx.CLS_NAV, ubx.NAV_PVT, nav_pvt())
  stream = b"$GPGGA,garbage*00\r\n\xb5" + b"\xb5\x62\x01\x07\xff\xff" + msg + msg
  parser = ubx.UbxParser()
  out = []
  for i in range(0, len(stream), 7):  # feed in small chunks
    out += list(parser.feed(stream[i:i + 7]))
  assert [(c, i) for c, i, _ in out] == [(1, 7), (1, 7)]


def test_decode_nav_pvt():
  fix, vel, t = ubx.decode_nav_pvt(nav_pvt())
  assert fix.status == STATUS_FIX
  assert fix.latitude == pytest.approx(47.1234568, abs=1e-7)
  assert fix.longitude == pytest.approx(8.5)
  assert fix.altitude == pytest.approx(500.0)
  assert fix.covariance_type == COV_APPROXIMATED
  assert fix.position_covariance[0] == pytest.approx(1.5 ** 2)
  assert fix.position_covariance[4] == pytest.approx(1.5 ** 2)
  assert fix.position_covariance[8] == pytest.approx(2.5 ** 2)
  assert fix.num_sv == 14
  assert vel.enu == pytest.approx((2.0, 1.0, 0.3))  # (velE, velN, -velD)
  assert vel.covariance[0] == pytest.approx(0.15 ** 2)
  assert t.utc_ns == calendar.timegm((2026, 11, 19, 12, 30, 15)) * 10**9 - 250_000_000


def test_decode_nav_pvt_status_variants():
  fix, vel, _ = ubx.decode_nav_pvt(nav_pvt(flags=0x00))  # gnssFixOK cleared
  assert fix.status == STATUS_NO_FIX and vel is None
  fix, _, _ = ubx.decode_nav_pvt(nav_pvt(flags=0x01 | (2 << 6)))  # carrier phase fixed
  assert fix.status == STATUS_GBAS_FIX
  fix, _, t = ubx.decode_nav_pvt(nav_pvt(valid=0x03))  # time not fully resolved
  assert t is None and fix.utc_ns is None
  with pytest.raises(ValueError):
    ubx.decode_nav_pvt(b"\x00" * 10)


def test_ack_matching():
  msg = ubx.cfg_rate(10)
  assert ubx.is_ack(ubx.CLS_ACK, ubx.ACK_ACK, bytes((0x06, 0x08)), msg) is True
  assert ubx.is_ack(ubx.CLS_ACK, ubx.ACK_NAK, bytes((0x06, 0x08)), msg) is False
  assert ubx.is_ack(ubx.CLS_ACK, ubx.ACK_ACK, bytes((0x06, 0x01)), msg) is None
  assert ubx.is_ack(ubx.CLS_NAV, 0x07, bytes((0x06, 0x08)), msg) is None


def test_mon_hw():
  p = bytearray(60)
  struct.pack_into("<HHBB", p, 16, 80, 4000, 2, 1)
  p[45] = 7
  assert ubx.decode_mon_hw(bytes(p)) == {"noise_per_ms": 80, "agc_count": 4000, "antenna": "ok", "antenna_power": 1,
                                         "jamming_indicator": 7}
