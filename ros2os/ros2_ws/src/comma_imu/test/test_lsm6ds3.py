import math
import struct

import pytest

from comma_imu import lsm6ds3
from comma_imu.lsm6ds3 import LSM6DS3, chip_to_device_flu, default_stdevs


class FakeBus:
  """Register-level LSM6DS3 simulation on one I2C address."""

  def __init__(self, who_am_i=0x6A, address=0x6A):
    self.address = address
    self.regs = dict.fromkeys(range(0x80), 0)
    self.regs[lsm6ds3.REG_WHO_AM_I] = who_am_i
    self.writes = []

  def set_vec(self, reg, raw):
    for i, v in enumerate(struct.pack("<3h", *raw)):
      self.regs[reg + i] = v

  def read_i2c_block_data(self, addr, reg, length):
    assert addr == self.address
    return [self.regs[reg + i] for i in range(length)]

  def write_byte_data(self, addr, reg, value):
    assert addr == self.address
    self.writes.append((reg, value))
    self.regs[reg] = value


def test_probe_variants():
  assert LSM6DS3(FakeBus(0x69)).probe() == "lsm6ds3"
  assert LSM6DS3(FakeBus(0x6A)).probe() == "lsm6ds3trc"
  with pytest.raises(LSM6DS3.NotFound):
    LSM6DS3(FakeBus(0x12)).probe()


def test_configure_registers():
  bus = FakeBus()
  bus.regs[lsm6ds3.REG_INT1_CTRL] = 0x80  # unrelated bit must be preserved
  imu = LSM6DS3(bus, odr_hz=208, accel_range_g=4, gyro_range_dps=1000)
  imu.configure()
  assert bus.regs[lsm6ds3.REG_CTRL3_C] == lsm6ds3.CTRL3_IF_INC | lsm6ds3.CTRL3_BDU
  assert bus.regs[lsm6ds3.REG_CTRL1_XL] == (0b0101 << 4) | (0b10 << 2)
  assert bus.regs[lsm6ds3.REG_CTRL2_G] == (0b0101 << 4) | (0b10 << 2)
  assert bus.regs[lsm6ds3.REG_DRDY_PULSE_CFG] == lsm6ds3.DRDY_PULSED
  assert bus.regs[lsm6ds3.REG_INT1_CTRL] == 0x80 | lsm6ds3.INT1_DRDY_XL | lsm6ds3.INT1_DRDY_G

  imu.configure(interrupts=False)
  assert bus.regs[lsm6ds3.REG_INT1_CTRL] == 0x80

  imu.shutdown()
  assert bus.regs[lsm6ds3.REG_CTRL1_XL] >> 4 == 0
  assert bus.regs[lsm6ds3.REG_CTRL2_G] >> 4 == 0


def test_openpilot_default_config_matches_sensord():
  # sensord: ODR 104 Hz, +-2 g, +-250 dps (8.75 mdps/LSB)
  bus = FakeBus()
  imu = LSM6DS3(bus)
  imu.configure()
  assert bus.regs[lsm6ds3.REG_CTRL1_XL] == 0b0100 << 4
  assert bus.regs[lsm6ds3.REG_CTRL2_G] == 0b0100 << 4
  assert imu.accel_scale == pytest.approx(9.81 * 2.0 / (1 << 15), rel=1e-3)
  assert imu.gyro_scale == pytest.approx((8.75 / 1000.0) * (math.pi / 180.0))


@pytest.mark.parametrize("rng,sens_mg", [(2, 0.061), (4, 0.122), (8, 0.244), (16, 0.488)])
def test_accel_scaling(rng, sens_mg):
  bus = FakeBus()
  imu = LSM6DS3(bus, accel_range_g=rng)
  bus.regs[lsm6ds3.REG_STATUS] = lsm6ds3.STATUS_XLDA
  bus.set_vec(lsm6ds3.REG_OUTX_L_XL, (1000, -1000, 0))
  s = imu.read_sample()
  assert s.gyro is None
  assert s.accel[0] == pytest.approx(1000 * sens_mg * 1e-3 * 9.80665)
  assert s.accel[1] == pytest.approx(-1000 * sens_mg * 1e-3 * 9.80665)
  assert s.accel[2] == 0.0


def test_read_sample_both_and_none():
  bus = FakeBus()
  imu = LSM6DS3(bus, gyro_range_dps=2000)
  bus.set_vec(lsm6ds3.REG_OUTX_L_G, (100, 0, -100))
  bus.set_vec(lsm6ds3.REG_OUTX_L_XL, (0, 16384, 0))
  bus.regs[lsm6ds3.REG_STATUS] = 0
  s = imu.read_sample()
  assert s.accel is None and s.gyro is None

  bus.regs[lsm6ds3.REG_STATUS] = lsm6ds3.STATUS_XLDA | lsm6ds3.STATUS_GDA
  s = imu.read_sample()
  assert s.gyro[0] == pytest.approx(math.radians(100 * 0.070))
  assert s.gyro[2] == pytest.approx(-math.radians(100 * 0.070))
  assert s.accel[1] == pytest.approx(16384 * 0.061e-3 * 9.80665)


def test_temperature_scale_per_variant():
  for who, lsb in ((0x69, 16.0), (0x6A, 256.0)):
    bus = FakeBus(who)
    imu = LSM6DS3(bus)
    imu.probe()
    for i, v in enumerate(struct.pack("<h", int(10 * lsb))):
      bus.regs[lsm6ds3.REG_OUT_TEMP_L + i] = v
    assert imu.read_temperature() == pytest.approx(35.0)


def test_axis_mapping_matches_openpilot_device_frame():
  # openpilot: sensord logs v = [y, -x, z]; locationd device FRD = [-v2, -v1, -v0]
  for chip in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.3, -2.0, 7.5)):
    x, y, z = chip
    v = (y, -x, z)
    frd = (-v[2], -v[1], -v[0])
    flu = (frd[0], -frd[1], -frd[2])
    assert chip_to_device_flu(chip) == pytest.approx(flu)


def test_axis_mapping_is_proper_rotation():
  cols = [chip_to_device_flu(e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
  m = [[cols[c][r] for c in range(3)] for r in range(3)]
  det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
  assert det == 1


def test_gravity_reads_up_when_upright():
  # Upright device in a windshield mount: openpilot logs ~+9.81 on acceleration.v[0] (= chip y),
  # so the specific force must point up (+z) in the FLU device frame.
  assert chip_to_device_flu((0.0, 9.81, 0.0)) == pytest.approx((0.0, 0.0, 9.81))


def test_rejects_unsupported_settings():
  with pytest.raises(ValueError):
    LSM6DS3(FakeBus(), odr_hz=100)
  with pytest.raises(ValueError):
    LSM6DS3(FakeBus(), accel_range_g=3)
  with pytest.raises(ValueError):
    LSM6DS3(FakeBus(), gyro_range_dps=125)


def test_default_stdevs():
  accel, gyro = default_stdevs("lsm6ds3trc", 104)
  assert accel == pytest.approx(90e-6 * 9.80665 * math.sqrt(52))
  assert gyro == pytest.approx(math.radians(4e-3) * math.sqrt(52))
