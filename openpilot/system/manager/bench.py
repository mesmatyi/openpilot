"""Experimental built-in-camera bench session policy. No hardware health is synthesized."""
import math
import os
import time


LEASE_TIMEOUT = 3.0
ADAPTER_TIMEOUT = 1.0
START_TIMEOUT = 15.0

BENCH_ALWAYS = frozenset({
  "manage_athenad", "logmessaged", "timed", "ui", "pandad", "hardwared", "modem",
  "deleter", "uploader", "tombstoned",
})
BENCH_ONROAD = frozenset({
  "loggerd", "encoderd", "proclogd", "journald", "camerad", "modeld", "dmonitoringmodeld",
  "sensord", "locationd", "calibrationd", "selfdrived", "dmonitoringd", "qcomgpsd", "ubloxd", "pigeond",
})
TRANSIENT_CAR_KEYS = ("CarParams", "ControlsReady", "FirmwareQueryDone", "ObdMultiplexingEnabled", "ObdMultiplexingChanged")
BENCH_KEYS = ("BenchModeRequested", "BenchMode", "BenchModeLease", "BenchAdapterHeartbeat", "BenchModeStatus")
BENCH_WARNING = (
  "Experimental Bench Camera Mode (built-in cameras, not a USB webcam).",
  "Disconnect the device completely from any car and harness before starting. Keep it disconnected throughout the session.",
  "Uses real cameras, IMU and available GPS. Existing recording and upload settings apply: sensitive video and location may be recorded and uploaded.",
  "Cannot engage or control a vehicle. Stationary synthetic vehicle inputs are not a real drive. Cloud listing and full automatic uploads are not guaranteed.",
)


def request_start(params, offroad: bool, confirmed: bool) -> bool:
  if not offroad or not confirmed or params.get_bool("BenchMode"):
    return False
  params.put_bool("BenchModeRequested", True, block=True)
  return True


def fresh(timestamp, now: float, timeout: float) -> bool:
  return timestamp is not None and math.isfinite(timestamp) and 0 <= now - timestamp < timeout


def hardware_safe(sm, now: float) -> bool:
  """Conservative raw panda check; never use an overridden ignition signal."""
  if not sm.all_checks(["pandaStates"]) or not fresh(sm.recv_time["pandaStates"], now, ADAPTER_TIMEOUT):
    return False
  states = sm["pandaStates"]
  if len(states) != 1:
    return False
  ps = states[0]
  return (str(ps.pandaType) != "unknown" and str(ps.harnessStatus) == "notConnected" and
          not ps.ignitionLine and not ps.ignitionCan and not ps.controlsAllowed and
          str(ps.safetyModel) == "noOutput" and
          all(cs.totalRxCnt == 0 for cs in (ps.canState0, ps.canState1, ps.canState2)))


def bench_ignition(params, sm, now: float) -> bool:
  return (params.get_bool("BenchMode") and params.get_bool("BenchModeRequested") and
          fresh(params.get("BenchModeLease"), now, LEASE_TIMEOUT) and
          fresh(params.get("BenchAdapterHeartbeat"), now, ADAPTER_TIMEOUT) and hardware_safe(sm, now))


def process_allowed(name: str, started: bool, params) -> bool:
  if name == "benchd":
    return params.get_bool("BenchMode") and params.get_bool("BenchModeRequested")
  return not params.get_bool("BenchMode") or name in BENCH_ALWAYS or (started and name in BENCH_ONROAD)


def clear_car_session(params) -> None:
  for key in TRANSIENT_CAR_KEYS:
    params.remove(key)
  params.put_bool("IsEngaged", False, block=True)


class BenchController:
  def __init__(self, params, supported: bool):
    self.params = params
    self.supported = supported
    self.start_time = None
    self.stopping = False
    self.was_started = False

  def stop(self, reason: str) -> None:
    self.params.put_bool("BenchModeRequested", False, block=True)
    self.params.remove("BenchModeLease")
    self.params.put("BenchModeStatus", reason, block=True)
    self.stopping = True

  def update(self, sm, adapter_failed: bool, now: float | None = None) -> bool:
    """Return True once on accepted entry, before starting any session processes."""
    now = time.monotonic() if now is None else now
    active = self.params.get_bool("BenchMode")
    requested = self.params.get_bool("BenchModeRequested")
    if not active:
      if not requested:
        return False
      if (not self.supported or any(k in os.environ for k in ("STARTED", "FAKESEND", "USE_WEBCAM", "REPLAY", "SIMULATION")) or
          not sm.all_checks(["deviceState"]) or sm["deviceState"].started or
          self.params.get_bool("IsDriverViewEnabled") or not hardware_safe(sm, now)):
        self.params.put_bool("BenchModeRequested", False, block=True)
        self.params.put("BenchModeStatus", "Refused: requires comma four, offroad, fresh no-output panda and no harness/ignition/CAN.", block=True)
        return False
      clear_car_session(self.params)
      self.params.remove("BenchAdapterHeartbeat")
      self.params.put_bool("BenchMode", True, block=True)
      self.params.put("BenchModeStatus", "Starting; normal startup and thermal checks apply.", block=True)
      self.start_time = now
      self.was_started = False
      self.stopping = False
      self.params.put("BenchModeLease", now, block=True)
      return True

    if self.stopping:
      return False
    if not requested:
      self.stop("Stopping; waiting for offroad and disconnected hardware.")
    elif not hardware_safe(sm, now) or not sm.all_checks(["deviceState"]):
      self.stop("Stopped: raw hardware connected, unsafe, or stale. Disconnect before leaving bench mode.")
    elif adapter_failed or (now - self.start_time > START_TIMEOUT and
                            not fresh(self.params.get("BenchAdapterHeartbeat"), now, ADAPTER_TIMEOUT)):
      self.stop("Stopped: bench adapter failed or stale.")
    elif self.was_started and (not sm["deviceState"].started or
                              not fresh(self.params.get("BenchAdapterHeartbeat"), now, ADAPTER_TIMEOUT)):
      self.stop("Stopped: adapter heartbeat or hardware startup/thermal permission lost.")
    else:
      self.was_started |= sm["deviceState"].started
      self.params.put("BenchModeLease", now, block=True)
      if self.was_started:
        self.params.put("BenchModeStatus", "Running with real cameras/sensors; stationary synthetic vehicle, uncalibrated.", block=True)
    return False

  def finish_stop(self, sm, now: float | None = None) -> bool:
    """Caller must first synchronously stop all session processes/publishers."""
    now = time.monotonic() if now is None else now
    if self.stopping and sm.all_checks(["deviceState"]) and not sm["deviceState"].started and hardware_safe(sm, now):
      clear_car_session(self.params)
      self.params.remove("BenchAdapterHeartbeat")
      self.params.put_bool("BenchMode", False, block=True)
      self.stopping = False
      return True
    return False
