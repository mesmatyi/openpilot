"""Minimal SMBus access through the Linux i2c-dev ioctl interface.

Adapted from openpilot/common/i2c.py so the drivers do not need smbus2.
"""
import ctypes
import fcntl
import os

# constants from linux/i2c-dev.h and linux/i2c.h
I2C_SLAVE = 0x0703
I2C_SLAVE_FORCE = 0x0706
I2C_SMBUS = 0x0720

I2C_SMBUS_READ = 1
I2C_SMBUS_WRITE = 0
I2C_SMBUS_BYTE_DATA = 2
I2C_SMBUS_I2C_BLOCK_DATA = 8

I2C_SMBUS_BLOCK_MAX = 32


class _I2cSmbusData(ctypes.Union):
  _fields_ = [
    ("byte", ctypes.c_uint8),
    ("word", ctypes.c_uint16),
    ("block", ctypes.c_uint8 * (I2C_SMBUS_BLOCK_MAX + 2)),
  ]


class _I2cSmbusIoctlData(ctypes.Structure):
  _fields_ = [
    ("read_write", ctypes.c_uint8),
    ("command", ctypes.c_uint8),
    ("size", ctypes.c_uint32),
    ("data", ctypes.POINTER(_I2cSmbusData)),
  ]


class SMBus:
  """Byte and block register access to devices on one I2C bus."""

  def __init__(self, bus: int):
    self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
    self._addr: int | None = None

  def __enter__(self) -> "SMBus":
    return self

  def __exit__(self, *args) -> None:
    self.close()

  def close(self) -> None:
    if getattr(self, "_fd", -1) >= 0:
      os.close(self._fd)
      self._fd = -1

  def _set_address(self, addr: int, force: bool = False) -> None:
    if addr != self._addr or force:
      fcntl.ioctl(self._fd, I2C_SLAVE_FORCE if force else I2C_SLAVE, addr)
      self._addr = addr

  def _access(self, read_write: int, command: int, size: int, data: _I2cSmbusData) -> None:
    fcntl.ioctl(self._fd, I2C_SMBUS, _I2cSmbusIoctlData(read_write, command, size, ctypes.pointer(data)))

  def read_byte_data(self, addr: int, register: int) -> int:
    self._set_address(addr)
    data = _I2cSmbusData()
    self._access(I2C_SMBUS_READ, register, I2C_SMBUS_BYTE_DATA, data)
    return int(data.byte)

  def write_byte_data(self, addr: int, register: int, value: int) -> None:
    self._set_address(addr)
    data = _I2cSmbusData()
    data.byte = value & 0xFF
    self._access(I2C_SMBUS_WRITE, register, I2C_SMBUS_BYTE_DATA, data)

  def read_i2c_block_data(self, addr: int, register: int, length: int) -> list[int]:
    if not 0 <= length <= I2C_SMBUS_BLOCK_MAX:
      raise ValueError(f"length must be 0..{I2C_SMBUS_BLOCK_MAX}")
    self._set_address(addr)
    data = _I2cSmbusData()
    data.block[0] = length
    self._access(I2C_SMBUS_READ, register, I2C_SMBUS_I2C_BLOCK_DATA, data)
    n = min(int(data.block[0]) or length, length)
    return [int(b) for b in data.block[1:n + 1]]
