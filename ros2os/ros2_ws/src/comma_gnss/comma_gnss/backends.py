"""Receiver backends: power/configure the GNSS hardware and yield solutions.

Both backends expose `start()`, `read()` (returns a list of GnssFix /
GnssVelocity / GnssTime / dict status items, blocking up to ~0.1 s) and `stop()`.
"""
import datetime
import fcntl
import os
import time

from comma_gnss import ubx
from comma_gnss.nmea import NmeaDecoder

UBLOX_TTY = "/dev/ttyHS0"
QUECTEL_AT_PORT = "/dev/modem_at0"
QUECTEL_NMEA_PORT = "/dev/ttyUSB1"
MODEM_LOCK = "/dev/shm/modem.lock"  # shared with openpilot's modem manager and LPA
USE_QUECTEL_FLAG = "/persist/comma/use-quectel-gps"


def detect_backend() -> str:
  """Same rule as openpilot's process_config.ublox_available()."""
  if os.path.exists(UBLOX_TTY) and not os.path.exists(USE_QUECTEL_FLAG):
    return "ublox"
  return "quectel"


def system_time_valid(now: datetime.datetime | None = None) -> bool:
  now = now or datetime.datetime.now(datetime.UTC)
  return datetime.datetime(2025, 2, 21, tzinfo=datetime.UTC) < now < datetime.datetime(2035, 1, 1, tzinfo=datetime.UTC)


class UbloxBackend:
  """u-blox M8 on UART1 (/dev/ttyHS0), configured like openpilot's pigeond, decoded from UBX NAV-PVT."""

  def __init__(self, logger, rate_hz: float = 10.0, port: str = UBLOX_TTY, gpio=None, pins=None):
    self.log = logger
    self.rate_hz = rate_hz
    self.port = port
    self.gpio = gpio
    self.pins = pins
    self.ser = None
    self.parser = ubx.UbxParser()
    self.status: dict = {}

  def _power(self, on: bool) -> None:
    if self.gpio is None:
      return
    try:
      for pin in (self.pins.UBLOX_SAFEBOOT_N, self.pins.GNSS_PWR_EN, self.pins.UBLOX_RST_N):
        self.gpio.gpio_init(pin, True)
      self.gpio.gpio_set(self.pins.UBLOX_SAFEBOOT_N, True)
      self.gpio.gpio_set(self.pins.GNSS_PWR_EN, on)
      self.gpio.gpio_set(self.pins.UBLOX_RST_N, on)
    except OSError as e:
      self.log.warning(f"could not drive u-blox power GPIOs: {e}")

  def _send_with_ack(self, msg: bytes, timeout: float = 0.5) -> bool:
    self.ser.write(msg)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      for cls, msg_id, payload in self.parser.feed(self.ser.read(256)):
        ack = ubx.is_ack(cls, msg_id, payload, msg)
        if ack is not None:
          return ack
    raise TimeoutError

  def start(self) -> None:
    import serial  # pyserial

    self._power(False)
    time.sleep(0.1)
    self._power(True)
    time.sleep(0.5)

    self.ser = serial.Serial(self.port, 9600, timeout=0.05)
    self.ser.write(ubx.PUBX_BAUD_460800)
    time.sleep(0.1)
    self.ser.baudrate = 460800
    self.ser.reset_input_buffer()

    for attempt in range(10):
      try:
        for msg in ubx.init_sequence(self.rate_hz):
          if not self._send_with_ack(msg):
            self.log.warning(f"u-blox NAK for config message class 0x{msg[2]:02X} id 0x{msg[3]:02X}")
        break
      except TimeoutError:
        self.log.warning(f"u-blox config timed out, retrying ({attempt + 1}/10)")
    else:
      raise RuntimeError("failed to configure the u-blox receiver")

    if system_time_valid():
      t = datetime.datetime.now(datetime.UTC)
      self.ser.write(ubx.mga_ini_time_utc(t.year, t.month, t.day, t.hour, t.minute, t.second))
    self.log.info(f"u-blox configured on {self.port} at 460800 baud, {self.rate_hz} Hz")

  def read(self) -> list:
    out = []
    for cls, msg_id, payload in self.parser.feed(self.ser.read(4096)):
      if (cls, msg_id) == (ubx.CLS_NAV, ubx.NAV_PVT):
        fix, vel, t = ubx.decode_nav_pvt(payload)
        out += [x for x in (t, fix, vel) if x is not None]
      elif (cls, msg_id) == (ubx.CLS_MON, ubx.MON_HW):
        self.status = ubx.decode_mon_hw(payload)
        out.append(dict(self.status))
    return out

  def stop(self) -> None:
    if self.ser is not None:
      try:
        self.ser.write(ubx.CFG_RST_GNSS_STOP)
        self.ser.close()
      except OSError:
        pass
    self._power(False)


def quectel_at(cmd: str, port: str = QUECTEL_AT_PORT, timeout: float = 5.0) -> str:
  """Send one AT command to the modem, serialized with other users of the AT port."""
  import serial  # pyserial

  with os.fdopen(os.open(MODEM_LOCK, os.O_CREAT | os.O_RDWR, 0o666), "r+") as lock:
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
    with serial.Serial(port, 115200, timeout=timeout) as ser:
      ser.reset_input_buffer()
      ser.write(f"{cmd}\r".encode())
      lines = []
      while True:
        line = ser.readline()
        if not line:
          raise TimeoutError(f"AT command timeout: {cmd}")
        text = line.decode("utf-8", errors="replace").strip()
        if text == "OK":
          return "\n".join(lines)
        if text == "ERROR" or text.startswith("+CME ERROR"):
          raise RuntimeError(f"{cmd}: {text}")
        if text and text != cmd:
          lines.append(text)


class QuectelBackend:
  """GNSS engine of the Quectel modem: configured over AT commands, NMEA read from the usbnmea port."""

  def __init__(self, logger, at=quectel_at, nmea_port: str = QUECTEL_NMEA_PORT, gpio=None, pins=None,
               uere_m: float = 4.0, wait_timeout_s: float = 120.0):
    self.log = logger
    self.at = at
    self.nmea_port = nmea_port
    self.gpio = gpio
    self.pins = pins
    self.decoder = NmeaDecoder(uere_m=uere_m)
    self.wait_timeout_s = wait_timeout_s
    self.f = None
    self.buf = b""
    self.on_sentence = None  # optional callback for raw NMEA

  def _try(self, cmd: str) -> str | None:
    try:
      return self.at(cmd)
    except (RuntimeError, TimeoutError, OSError) as e:
      self.log.warning(f"{cmd} failed: {e}")
      return None

  def start(self) -> None:
    deadline = time.monotonic() + self.wait_timeout_s
    while True:
      try:
        if "+QGPS:" in self.at("AT+QGPS?"):
          break
      except (RuntimeError, TimeoutError, OSError):
        pass
      if time.monotonic() > deadline:
        raise RuntimeError("modem GNSS engine did not respond (is the modem up?)")
      time.sleep(0.5)

    if self.gpio is not None:
      try:
        self.gpio.gpio_init(self.pins.GNSS_PWR_EN, True)
        self.gpio.gpio_set(self.pins.GNSS_PWR_EN, True)
      except OSError as e:
        self.log.warning(f"could not enable the GNSS antenna supply: {e}")

    if "QGPS: 1" in (self._try("AT+QGPS?") or ""):
      self._try("AT+QGPSEND")
    self._try('AT+QGPSCFG="dpoenable",0')   # no duty cycling, better accuracy
    self._try('AT+QGPSCFG="autogps",0')
    self._try('AT+QGPSCFG="outport","usbnmea"')
    self._try('AT+QGPSCFG="gpsnmeatype",31')  # GGA, RMC, GSV, GSA, VTG
    if system_time_valid():
      t = datetime.datetime.now(datetime.UTC).strftime("%Y/%m/%d,%H:%M:%S")
      self._try(f'AT+QGPSXTRATIME=0,"{t}",1,1,1000')
    if self._try("AT+QGPS=1") is None:
      raise RuntimeError("AT+QGPS=1 failed")

    self.f = os.open(self.nmea_port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    self.log.info(f"Quectel GNSS started, reading NMEA from {self.nmea_port}")

  def read(self) -> list:
    import select

    out = []
    r, _, _ = select.select([self.f], [], [], 0.1)
    if not r:
      return out
    self.buf += os.read(self.f, 4096)
    *lines, self.buf = self.buf.split(b"\n")
    for raw in lines:
      line = raw.decode("ascii", errors="replace").strip()
      if not line.startswith("$"):
        continue
      if self.on_sentence is not None:
        self.on_sentence(line)
      out += self.decoder.feed(line)
    return out

  def stop(self) -> None:
    if self.f is not None:
      os.close(self.f)
      self.f = None
    self._try('AT+QGPSCFG="outport","none"')
    self._try("AT+QGPSEND")
    if self.gpio is not None:
      try:
        self.gpio.gpio_set(self.pins.GNSS_PWR_EN, False)
      except OSError:
        pass
