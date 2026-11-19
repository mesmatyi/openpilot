"""LSM6DS3 / LSM6DS3TR-C accelerometer + gyroscope driver core (no ROS).

Register usage follows openpilot's sensord (system/sensord/sensors/lsm6ds3_*.py)
and the ST datasheets. The bus object only needs read_i2c_block_data(),
read_byte_data() and write_byte_data(), so tests can substitute a fake.
"""
import ctypes
import math
import time
from dataclasses import dataclass

STANDARD_GRAVITY = 9.80665

# registers
REG_DRDY_PULSE_CFG = 0x0B
REG_INT1_CTRL = 0x0D
REG_WHO_AM_I = 0x0F
REG_CTRL1_XL = 0x10
REG_CTRL2_G = 0x11
REG_CTRL3_C = 0x12
REG_CTRL5_C = 0x14
REG_STATUS = 0x1E
REG_OUT_TEMP_L = 0x20
REG_OUTX_L_G = 0x22
REG_OUTX_L_XL = 0x28

# bits
CTRL3_SW_RESET = 0x01
CTRL3_IF_INC = 0x04
CTRL3_BDU = 0x40
DRDY_PULSED = 0x80
INT1_DRDY_XL = 0x01
INT1_DRDY_G = 0x02
STATUS_XLDA = 0x01
STATUS_GDA = 0x02
STATUS_TDA = 0x04

WHO_AM_I_IDS = {0x69: "lsm6ds3", 0x6A: "lsm6ds3trc"}

ODR_BITS = {12.5: 0b0001, 26: 0b0010, 52: 0b0011, 104: 0b0100, 208: 0b0101, 416: 0b0110, 833: 0b0111, 1660: 0b1000}
# full scale -> (FS bits, sensitivity)
ACCEL_FS = {2: (0b00, 0.061e-3), 4: (0b10, 0.122e-3), 8: (0b11, 0.244e-3), 16: (0b01, 0.488e-3)}  # g/LSB
GYRO_FS = {250: (0b00, 8.75e-3), 500: (0b01, 17.5e-3), 1000: (0b10, 35e-3), 2000: (0b11, 70e-3)}  # dps/LSB
TEMP_LSB_PER_DEGC = {"lsm6ds3": 16.0, "lsm6ds3trc": 256.0}

# datasheet noise densities (high-performance mode): accel ug/sqrt(Hz), gyro mdps/sqrt(Hz)
NOISE_DENSITY = {"lsm6ds3": (90.0, 7.0), "lsm6ds3trc": (90.0, 4.0)}


def chip_to_device_flu(v: tuple[float, float, float]) -> tuple[float, float, float]:
  """Rotate a vector from LSM6DS3 chip axes into the device body frame (x forward, y left, z up).

  openpilot's sensord logs [y, -x, z] of the chip axes and locationd turns that
  into its [forward, right, down] device frame with [-v2, -v1, -v0]; flipping
  right/down to left/up gives forward = -z, left = -x, up = +y. The same
  mounting is used on the comma 3X and comma four (neither codebase branches on
  device type for IMU axes).
  """
  x, y, z = v
  return (-z, -x, y)


def default_stdevs(variant: str, odr_hz: float) -> tuple[float, float]:
  """White-noise standard deviations (m/s^2, rad/s) at the ODR's bandwidth (~ODR/2)."""
  accel_nd, gyro_nd = NOISE_DENSITY.get(variant, NOISE_DENSITY["lsm6ds3"])
  bw = odr_hz / 2.0
  return accel_nd * 1e-6 * STANDARD_GRAVITY * math.sqrt(bw), math.radians(gyro_nd * 1e-3) * math.sqrt(bw)


def _s16(lsb: int, msb: int) -> int:
  return ctypes.c_int16((msb << 8) | lsb).value


@dataclass
class Sample:
  status: int
  accel: tuple[float, float, float] | None  # m/s^2, chip axes
  gyro: tuple[float, float, float] | None   # rad/s, chip axes


class LSM6DS3:
  class NotFound(Exception):
    pass

  class SelfTestFailed(Exception):
    pass

  def __init__(self, bus, address: int = 0x6A, odr_hz: float = 104, accel_range_g: int = 2, gyro_range_dps: int = 250):
    if odr_hz not in ODR_BITS:
      raise ValueError(f"unsupported ODR {odr_hz} Hz, choose from {sorted(ODR_BITS)}")
    if accel_range_g not in ACCEL_FS:
      raise ValueError(f"unsupported accel range {accel_range_g} g, choose from {sorted(ACCEL_FS)}")
    if gyro_range_dps not in GYRO_FS:
      raise ValueError(f"unsupported gyro range {gyro_range_dps} dps, choose from {sorted(GYRO_FS)}")
    self.bus = bus
    self.address = address
    self.odr_hz = odr_hz
    self.accel_range_g = accel_range_g
    self.gyro_range_dps = gyro_range_dps
    self.variant = "unknown"
    self.accel_scale = ACCEL_FS[accel_range_g][1] * STANDARD_GRAVITY
    self.gyro_scale = math.radians(GYRO_FS[gyro_range_dps][1])

  # *** register helpers ***
  def _read(self, reg: int, length: int) -> bytes:
    return bytes(self.bus.read_i2c_block_data(self.address, reg, length))

  def _read_byte(self, reg: int) -> int:
    return self._read(reg, 1)[0]

  def _write(self, reg: int, value: int) -> None:
    self.bus.write_byte_data(self.address, reg, value & 0xFF)

  # *** lifecycle ***
  def probe(self) -> str:
    who = self._read_byte(REG_WHO_AM_I)
    if who not in WHO_AM_I_IDS:
      raise self.NotFound(f"unexpected WHO_AM_I 0x{who:02X} at address 0x{self.address:02X}")
    self.variant = WHO_AM_I_IDS[who]
    return self.variant

  def reset(self, sleep=time.sleep) -> None:
    self._write(REG_CTRL3_C, CTRL3_SW_RESET)
    sleep(0.1)

  def configure(self, interrupts: bool = True) -> None:
    """Enable both sensors at the configured ODR/range with data-ready pulses on INT1."""
    odr = ODR_BITS[self.odr_hz] << 4
    self._write(REG_CTRL3_C, CTRL3_IF_INC | CTRL3_BDU)
    self._write(REG_CTRL1_XL, odr | (ACCEL_FS[self.accel_range_g][0] << 2))
    self._write(REG_CTRL2_G, odr | (GYRO_FS[self.gyro_range_dps][0] << 2))
    self._write(REG_DRDY_PULSE_CFG, DRDY_PULSED)
    int1 = self._read_byte(REG_INT1_CTRL)
    if interrupts:
      int1 |= INT1_DRDY_XL | INT1_DRDY_G
    else:
      int1 &= ~(INT1_DRDY_XL | INT1_DRDY_G)
    self._write(REG_INT1_CTRL, int1)

  def shutdown(self) -> None:
    self._write(REG_INT1_CTRL, self._read_byte(REG_INT1_CTRL) & ~(INT1_DRDY_XL | INT1_DRDY_G))
    self._write(REG_CTRL1_XL, self._read_byte(REG_CTRL1_XL) & 0x0F)  # clear ODR = power down
    self._write(REG_CTRL2_G, self._read_byte(REG_CTRL2_G) & 0x0F)

  # *** data ***
  def read_sample(self) -> Sample:
    """Read whichever of accel/gyro has new data (one 12-byte burst: gyro then accel)."""
    status = self._read_byte(REG_STATUS)
    if not status & (STATUS_XLDA | STATUS_GDA):
      return Sample(status, None, None)
    b = self._read(REG_OUTX_L_G, 12)
    gyro = tuple(_s16(b[i], b[i + 1]) * self.gyro_scale for i in (0, 2, 4)) if status & STATUS_GDA else None
    accel = tuple(_s16(b[i], b[i + 1]) * self.accel_scale for i in (6, 8, 10)) if status & STATUS_XLDA else None
    return Sample(status, accel, gyro)

  def read_temperature(self) -> float:
    b = self._read(REG_OUT_TEMP_L, 2)
    return 25.0 + _s16(b[0], b[1]) / TEMP_LSB_PER_DEGC.get(self.variant, 16.0)

  # *** self test (datasheet procedure, as in openpilot sensord) ***
  def _avg(self, reg: int, ready_bit: int, scale: float, n: int = 5, timeout: float = 1.0) -> list[float]:
    acc = [0.0, 0.0, 0.0]
    for _ in range(n):
      t0 = time.monotonic()
      while not self._read_byte(REG_STATUS) & ready_bit:
        if time.monotonic() - t0 > timeout:
          raise self.SelfTestFailed("timed out waiting for data")
      b = self._read(reg, 6)
      for j in range(3):
        acc[j] += _s16(b[2 * j], b[2 * j + 1]) * scale
    return [a / n for a in acc]

  def self_test(self, sleep=time.sleep) -> None:
    """Run accel and gyro self tests in both polarities; raises SelfTestFailed. Leaves sensors off."""
    self._write(REG_CTRL3_C, CTRL3_IF_INC | CTRL3_BDU)
    # accel: 52 Hz, +-4 g on TR-C (0.122 mg/LSB) or +-2 g (0.061 mg/LSB); limits 90..1700 mg
    if self.variant == "lsm6ds3trc":
      xl_cfg, xl_scale = (ODR_BITS[52] << 4) | (ACCEL_FS[4][0] << 2), 0.122
    else:
      xl_cfg, xl_scale = ODR_BITS[52] << 4, 0.061
    for mode in (0b01, 0b10):
      self._write(REG_CTRL1_XL, xl_cfg)
      sleep(0.1)
      off = self._avg(REG_OUTX_L_XL, STATUS_XLDA, xl_scale)
      self._write(REG_CTRL5_C, mode)
      sleep(0.1)
      on = self._avg(REG_OUTX_L_XL, STATUS_XLDA, xl_scale)
      self._write(REG_CTRL1_XL, 0)
      self._write(REG_CTRL5_C, 0)
      if not all(90.0 <= abs(a - b) <= 1700.0 for a, b in zip(on, off, strict=True)):
        raise self.SelfTestFailed(f"accelerometer self test failed (mode {mode})")
    # gyro: 208 Hz, 2000 dps (70 mdps/LSB); limits 150..700 dps
    for mode in (0b01 << 2, 0b11 << 2):
      self._write(REG_CTRL2_G, (ODR_BITS[208] << 4) | (GYRO_FS[2000][0] << 2))
      sleep(0.15)
      off = self._avg(REG_OUTX_L_G, STATUS_GDA, 70.0)
      self._write(REG_CTRL5_C, mode)
      sleep(0.05)
      on = self._avg(REG_OUTX_L_G, STATUS_GDA, 70.0)
      self._write(REG_CTRL2_G, 0)
      self._write(REG_CTRL5_C, 0)
      if not all(150000.0 <= abs(a - b) <= 700000.0 for a, b in zip(on, off, strict=True)):
        raise self.SelfTestFailed(f"gyroscope self test failed (mode {mode})")
