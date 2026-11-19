import datetime
import os

from comma_gnss import backends
from comma_gnss.nmea import add_checksum
from comma_gnss.types import GnssFix


class Log:
  def __init__(self):
    self.lines = []

  def info(self, m):
    self.lines.append(m)

  warning = error = info


def test_system_time_valid():
  assert backends.system_time_valid(datetime.datetime(2026, 11, 19, tzinfo=datetime.UTC))
  assert not backends.system_time_valid(datetime.datetime(1970, 1, 1, tzinfo=datetime.UTC))


def test_quectel_startup_sequence_and_nmea(tmp_path):
  sent = []

  def at(cmd):
    sent.append(cmd)
    if cmd == "AT+QGPS?":
      return "+QGPS: 1" if len(sent) <= 2 else "+QGPS: 0"
    return ""

  fifo = tmp_path / "nmea"
  os.mkfifo(fifo)
  b = backends.QuectelBackend(Log(), at=at, nmea_port=str(fifo))
  writer = None

  def open_writer():
    nonlocal writer
    writer = os.open(fifo, os.O_WRONLY)

  import threading
  t = threading.Thread(target=open_writer)
  t.start()
  b.start()
  t.join()
  assert sent[:3] == ["AT+QGPS?", "AT+QGPS?", "AT+QGPSEND"]
  assert 'AT+QGPSCFG="outport","usbnmea"' in sent
  assert sent[-1] == "AT+QGPS=1"

  raw = []
  b.on_sentence = raw.append
  line = add_checksum("GPGGA,120000.0,4717.0,N,00833.0,E,1,08,1.0,10.0,M,1.0,M,,")
  os.write(writer, (line + "\r\n").encode())
  out = []
  for _ in range(5):
    out += b.read()
    if out:
      break
  assert raw == [line]
  assert isinstance(out[0], GnssFix)

  os.close(writer)
  sent.clear()
  b.stop()
  assert sent == ['AT+QGPSCFG="outport","none"', "AT+QGPSEND"]
