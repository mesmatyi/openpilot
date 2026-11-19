"""GPIO access: sysfs for outputs, the gpiochip character device for edge events."""
import ctypes
import fcntl
import os

SYSFS_GPIO = "/sys/class/gpio"


def gpio_export(pin: int, root: str = SYSFS_GPIO) -> None:
  if os.path.isdir(f"{root}/gpio{pin}"):
    return
  with open(f"{root}/export", "w") as f:
    f.write(str(pin))


def gpio_init(pin: int, output: bool, root: str = SYSFS_GPIO) -> None:
  if not os.path.isdir(f"{root}/gpio{pin}"):
    try:
      gpio_export(pin, root)
    except OSError:
      pass
  with open(f"{root}/gpio{pin}/direction", "wb") as f:
    f.write(b"out" if output else b"in")


def gpio_set(pin: int, high: bool, root: str = SYSFS_GPIO) -> None:
  with open(f"{root}/gpio{pin}/value", "wb") as f:
    f.write(b"1" if high else b"0")


def gpio_read(pin: int, root: str = SYSFS_GPIO) -> bool:
  with open(f"{root}/gpio{pin}/value", "rb") as f:
    return bool(int(f.read().strip()))


# *** gpiochip v1 ABI (linux/gpio.h), available on the 4.9 AGNOS kernel ***

GPIOHANDLE_REQUEST_INPUT = 0x1
GPIOEVENT_REQUEST_RISING_EDGE = 0x1
GPIOEVENT_REQUEST_FALLING_EDGE = 0x2
GPIOEVENT_REQUEST_BOTH_EDGES = 0x3
GPIO_GET_LINEEVENT_IOCTL = 0xC030B404  # _IOWR(0xB4, 0x04, struct gpioevent_request)


class GpioEventData(ctypes.Structure):
  _fields_ = [
    ("timestamp", ctypes.c_uint64),
    ("id", ctypes.c_uint32),
  ]


class _GpioEventRequest(ctypes.Structure):
  _fields_ = [
    ("lineoffset", ctypes.c_uint32),
    ("handleflags", ctypes.c_uint32),
    ("eventflags", ctypes.c_uint32),
    ("consumer_label", ctypes.c_char * 32),
    ("fd", ctypes.c_int),
  ]


def request_line_events(chip: int, line: int, label: str, edges: int = GPIOEVENT_REQUEST_BOTH_EDGES) -> int:
  """Request edge events on a GPIO line; returns a pollable fd yielding GpioEventData records."""
  rq = _GpioEventRequest()
  rq.lineoffset = line
  rq.handleflags = GPIOHANDLE_REQUEST_INPUT
  rq.eventflags = edges
  rq.consumer_label = label.encode()[:31]
  chip_fd = os.open(f"/dev/gpiochip{chip}", os.O_RDONLY)
  try:
    fcntl.ioctl(chip_fd, GPIO_GET_LINEEVENT_IOCTL, rq)
  finally:
    os.close(chip_fd)
  return int(rq.fd)


def read_line_events(fd: int, max_events: int = 16) -> list[GpioEventData]:
  size = ctypes.sizeof(GpioEventData)
  buf = os.read(fd, size * max_events)
  return [GpioEventData.from_buffer_copy(buf, i) for i in range(0, len(buf) - size + 1, size)]
