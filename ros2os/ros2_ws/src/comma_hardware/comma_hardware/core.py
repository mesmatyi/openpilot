"""Thermal, fan, power-protection and system-statistics logic (no ROS).

Constants and control laws follow openpilot's hardwared, fan_controller and
power_monitoring so the device behaves thermally the same as under openpilot.
"""
import os
import re
import subprocess
from dataclasses import dataclass

THERMAL_OK, THERMAL_OVERHEATED, THERMAL_CRITICAL = 0, 1, 2

# (min_temp, max_temp) per band; we stay in a band while inside its bounds (hysteresis)
THERMAL_BANDS = {
  "mici": {THERMAL_OK: (None, 100.0), THERMAL_OVERHEATED: (92.0, 107.0), THERMAL_CRITICAL: (98.0, None)},
  "default": {THERMAL_OK: (None, 96.0), THERMAL_OVERHEATED: (88.0, 107.0), THERMAL_CRITICAL: (94.0, None)},
}

VBATT_PAUSE_CHARGING = 11.8      # V, low-passed car battery voltage limit
CAR_VOLTAGE_LOW_PASS_K = 0.011   # 45 s time constant at 2 Hz
MAX_TIME_OFFROAD_S = 30 * 3600
DELAY_SHUTDOWN_TIME_S = 300
VOLTAGE_SHUTDOWN_MIN_OFFROAD_TIME_S = 60
MIN_ON_TIME_S = 3600


class FirstOrderFilter:
  def __init__(self, tau_s: float, dt_s: float):
    self.k = (dt_s / tau_s) / (1.0 + dt_s / tau_s)
    self.x: float | None = None

  def update(self, value: float) -> float:
    self.x = value if self.x is None else (1.0 - self.k) * self.x + self.k * value
    return self.x


class FanController:
  """openpilot FanController: integral-only PI around 75 C (+5 C on tici/tizi) plus feed-forward 60..100 C."""

  def __init__(self, rate_hz: float, device_type: str):
    self.dt = 1.0 / rate_hz
    self.offset = 0.0 if device_type == "mici" else 5.0
    self.k_i = 4e-3
    self.i = 0.0
    self.last_active: bool | None = None

  def update(self, temp_c: float, active: bool) -> int:
    pos_limit, neg_limit = (100.0, 30.0) if active else (30.0, 0.0)
    if active != self.last_active:
      self.i = 0.0
    self.last_active = active
    if temp_c != temp_c:  # NaN
      return 100
    error = temp_c - (75.0 + self.offset)
    lo, hi = 60.0 + self.offset, 100.0 + self.offset
    ff = 0.0 if temp_c <= lo else 100.0 if temp_c >= hi else (temp_c - lo) / (hi - lo) * 100.0
    i = self.i + self.k_i * self.dt * error
    test = i + ff
    i_upper = self.i if test > pos_limit else pos_limit
    i_lower = self.i if test < neg_limit else neg_limit
    self.i = min(max(i, i_lower), i_upper)
    return int(min(max(self.i + ff, neg_limit), pos_limit))


class ThermalStatus:
  def __init__(self, device_type: str):
    self.bands = THERMAL_BANDS["mici" if device_type == "mici" else "default"]
    self.status = THERMAL_OK

  def update(self, temp_c: float) -> int:
    lo, hi = self.bands[self.status]
    if lo is not None and temp_c < lo:
      self.status = max(THERMAL_OK, self.status - 1)
    elif hi is not None and temp_c > hi:
      self.status = min(THERMAL_CRITICAL, self.status + 1)
    return self.status


class PowerProtection:
  """Shut the device down before it drains a parked car's battery (openpilot power_monitoring.should_shutdown)."""

  def __init__(self, low_voltage: float = VBATT_PAUSE_CHARGING, max_offroad_s: float = MAX_TIME_OFFROAD_S):
    self.low_voltage = low_voltage
    self.max_offroad_s = max_offroad_s
    self.car_voltage = 12.0
    self.offroad_since: float | None = None
    self.started_seen = False

  def update(self, now: float, voltage: float | None, ignition: bool) -> None:
    if voltage is not None and voltage > 0:
      self.car_voltage = voltage if self.car_voltage is None else \
        (1 - CAR_VOLTAGE_LOW_PASS_K) * self.car_voltage + CAR_VOLTAGE_LOW_PASS_K * voltage
    if ignition:
      self.started_seen = True
      self.offroad_since = None
    elif self.offroad_since is None:
      self.offroad_since = now

  def should_shutdown(self, now: float, uptime_s: float, ignition: bool, in_car: bool) -> tuple[bool, str]:
    if self.offroad_since is None or ignition or not in_car:
      return False, ""
    offroad = now - self.offroad_since
    if offroad <= DELAY_SHUTDOWN_TIME_S or not (self.started_seen or uptime_s > MIN_ON_TIME_S):
      return False, ""
    if offroad > self.max_offroad_s:
      return True, f"offroad for {offroad / 3600:.1f} h"
    if self.car_voltage < self.low_voltage and offroad > VOLTAGE_SHUTDOWN_MIN_OFFROAD_TIME_S:
      return True, f"car battery low ({self.car_voltage:.2f} V)"
    return False, ""


# *** system statistics ***

class CpuUsage:
  """Per-core utilisation from /proc/stat deltas; offline cores report 0."""

  def __init__(self, path: str = "/proc/stat"):
    self.path = path
    self.prev: dict[str, tuple[int, int]] = {}

  def update(self) -> list[float]:
    out = []
    try:
      with open(self.path) as f:
        lines = [ln.split() for ln in f if re.match(r"^cpu\d+ ", ln)]
    except OSError:
      return out
    for parts in lines:
      vals = [int(v) for v in parts[1:]]
      idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
      total = sum(vals)
      p_idle, p_total = self.prev.get(parts[0], (idle, total))
      self.prev[parts[0]] = (idle, total)
      dt = total - p_total
      out.append(100.0 * (1.0 - (idle - p_idle) / dt) if dt > 0 else 0.0)
    return out


def memory_usage_percent(path: str = "/proc/meminfo") -> float:
  info = {}
  try:
    with open(path) as f:
      for line in f:
        k, v = line.split(":", 1)
        info[k] = int(v.split()[0])
  except (OSError, ValueError):
    return float("nan")
  total, avail = info.get("MemTotal", 0), info.get("MemAvailable", 0)
  return 100.0 * (1.0 - avail / total) if total else float("nan")


def free_space_percent(path: str) -> float:
  try:
    st = os.statvfs(path)
  except OSError:
    return float("nan")
  return 100.0 * st.f_bavail / st.f_blocks if st.f_blocks else float("nan")


def gpu_usage_percent(path: str = "/sys/class/kgsl/kgsl-3d0/gpubusy") -> float:
  try:
    with open(path) as f:
      used, total = (int(x) for x in f.read().split())
    return 100.0 * used / total if total else 0.0
  except (OSError, ValueError):
    return float("nan")


@dataclass
class NetworkInfo:
  network_type: str
  ip_addresses: list[str]


def default_route_iface(path: str = "/proc/net/route") -> str | None:
  try:
    with open(path) as f:
      rows = [ln.split() for ln in f.readlines()[1:]]
  except OSError:
    return None
  routes = [(int(r[6]), r[0]) for r in rows if len(r) > 6 and r[1] == "00000000" and int(r[3], 16) & 0x1]
  return min(routes)[1] if routes else None


def iface_type(iface: str | None) -> str:
  if iface is None:
    return "none"
  if iface.startswith("wlan"):
    return "wifi"
  if iface.startswith(("eth", "en", "usb")):
    return "ethernet"
  if iface.startswith(("ppp", "rmnet", "wwan")):
    return "cellular"
  return "other"


def ip_addresses() -> list[str]:
  try:
    out = subprocess.run(["ip", "-o", "addr", "show", "scope", "global"], capture_output=True, text=True, timeout=2).stdout
  except (OSError, subprocess.TimeoutExpired):
    return []
  return [f"{p[1]}:{p[3].split('/')[0]}" for p in (ln.split() for ln in out.splitlines()) if len(p) > 3]


def network_info() -> NetworkInfo:
  return NetworkInfo(iface_type(default_route_iface()), ip_addresses())
