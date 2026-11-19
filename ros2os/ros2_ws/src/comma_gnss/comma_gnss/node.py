"""ROS 2 node publishing GNSS from the comma device.

Topics
  gps/fix             sensor_msgs/NavSatFix                     frame `gps`
  gps/velocity        geometry_msgs/Vector3Stamped              ENU velocity, frame `gps_enu` (same as tools/mcap exports)
  gps/twist           geometry_msgs/TwistWithCovarianceStamped  ENU velocity with covariance, for state estimators
  gps/time_reference  sensor_msgs/TimeReference                 receiver UTC time
  gps/nmea            nmea_msgs/Sentence                        raw NMEA (Quectel backend, publish_nmea: true)
  /diagnostics        diagnostic_msgs/DiagnosticArray
"""
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference

from comma_gnss import backends
from comma_gnss.types import GnssFix, GnssTime, GnssVelocity
from comma_hal import clock, gpio
from comma_hal.pins import GPIO

STATUS_NAMES = {-1: "no fix", 0: "fix", 1: "SBAS/DGPS fix", 2: "RTK fix"}


class GnssNode(Node):
  def __init__(self):
    super().__init__("gnss")
    p = self.declare_parameter
    self.backend_name = p("backend", "auto").value  # auto | ublox | quectel
    self.rate_hz = p("rate_hz", 10.0).value          # u-blox navigation rate (Quectel is fixed at 1 Hz)
    self.frame_id = p("frame_id", "gps").value
    self.enu_frame_id = p("enu_frame_id", "gps_enu").value
    self.stamp_source = p("stamp_source", "receive").value  # receive | gnss
    self.uere_m = p("uere_m", 4.0).value
    self.publish_nmea = p("publish_nmea", False).value
    self.set_system_time = p("set_system_time", False).value

    name = backends.detect_backend() if self.backend_name == "auto" else self.backend_name
    if name == "ublox":
      self.backend = backends.UbloxBackend(self.get_logger(), rate_hz=self.rate_hz, gpio=gpio, pins=GPIO)
    elif name == "quectel":
      self.backend = backends.QuectelBackend(self.get_logger(), gpio=gpio, pins=GPIO, uere_m=self.uere_m)
    else:
      raise ValueError(f"unknown GNSS backend '{name}'")
    self.backend_name = name

    self.pub_fix = self.create_publisher(NavSatFix, "gps/fix", qos_profile_sensor_data)
    self.pub_vel = self.create_publisher(Vector3Stamped, "gps/velocity", qos_profile_sensor_data)
    self.pub_twist = self.create_publisher(TwistWithCovarianceStamped, "gps/twist", qos_profile_sensor_data)
    self.pub_time = self.create_publisher(TimeReference, "gps/time_reference", qos_profile_sensor_data)
    self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
    if self.publish_nmea and name == "quectel":
      from nmea_msgs.msg import Sentence
      self.pub_nmea = self.create_publisher(Sentence, "gps/nmea", 10)
      self.backend.on_sentence = self._publish_sentence

    self.last_fix: GnssFix | None = None
    self.last_fix_mono = 0.0
    self.extra_status: dict = {}
    self.time_set = False
    self.create_timer(1.0, self.publish_diagnostics)

    self.stop = threading.Event()
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()

  # *** helpers ***
  def _stamp_ns(self, utc_ns: int | None) -> int:
    if self.stamp_source == "gnss" and utc_ns is not None:
      return utc_ns
    return time.time_ns()

  def _header(self, msg, ts_ns: int, frame: str) -> None:
    msg.header.stamp.sec, msg.header.stamp.nanosec = clock.split_ns(ts_ns)
    msg.header.frame_id = frame

  def _publish_sentence(self, line: str) -> None:
    from nmea_msgs.msg import Sentence
    msg = Sentence()
    self._header(msg, time.time_ns(), self.frame_id)
    msg.sentence = line
    self.pub_nmea.publish(msg)

  # *** publishing ***
  def publish_fix(self, fix: GnssFix) -> None:
    msg = NavSatFix()
    self._header(msg, self._stamp_ns(fix.utc_ns), self.frame_id)
    msg.status.status = fix.status
    msg.status.service = fix.service
    msg.latitude, msg.longitude, msg.altitude = fix.latitude, fix.longitude, fix.altitude
    msg.position_covariance = [float(x) for x in fix.position_covariance]
    msg.position_covariance_type = fix.covariance_type
    self.pub_fix.publish(msg)
    self.last_fix, self.last_fix_mono = fix, time.monotonic()

  def publish_velocity(self, vel: GnssVelocity) -> None:
    ts = self._stamp_ns(vel.utc_ns)
    v = Vector3Stamped()
    self._header(v, ts, self.enu_frame_id)
    v.vector.x, v.vector.y, v.vector.z = vel.enu
    self.pub_vel.publish(v)

    t = TwistWithCovarianceStamped()
    self._header(t, ts, self.enu_frame_id)
    t.twist.twist.linear = v.vector
    cov = [0.0] * 36
    for i in range(3):
      cov[i * 6 + i] = vel.covariance[i * 3 + i]
    for i in range(3, 6):
      cov[i * 6 + i] = 1e6  # angular velocity is not measured
    t.twist.covariance = cov
    self.pub_twist.publish(t)

  def publish_time(self, t: GnssTime) -> None:
    msg = TimeReference()
    now = time.time_ns()
    self._header(msg, now, self.frame_id)
    msg.time_ref.sec, msg.time_ref.nanosec = clock.split_ns(t.utc_ns)
    msg.source = f"gnss:{self.backend_name}"
    self.pub_time.publish(msg)
    if self.set_system_time and not self.time_set:
      self.maybe_set_system_time(t.utc_ns, now)

  def maybe_set_system_time(self, utc_ns: int, now_ns: int) -> None:
    """Set the clock from GNSS when it is clearly wrong (no RTC battery / no NTP yet), like openpilot's timed."""
    if backends.system_time_valid() and abs(utc_ns - now_ns) < 10e9:
      self.time_set = True
      return
    try:
      synced = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True,
                              text=True, timeout=2).stdout.strip() == "yes"
    except (OSError, subprocess.TimeoutExpired):
      synced = False
    if synced and backends.system_time_valid():
      self.time_set = True
      return
    sec = utc_ns / 1e9
    r = subprocess.run(["sudo", "-n", "date", "-u", "-s", f"@{sec:.3f}"], capture_output=True, text=True)
    if r.returncode == 0:
      self.get_logger().warning(f"system time set from GNSS to {sec:.3f}")
      self.time_set = True
    else:
      self.get_logger().error(f"could not set system time: {r.stderr.strip()}", throttle_duration_sec=60.0)

  def publish_diagnostics(self) -> None:
    st = DiagnosticStatus(name="gnss: receiver", hardware_id=self.backend_name)
    fix = self.last_fix
    age = time.monotonic() - self.last_fix_mono if fix is not None else float("inf")
    if fix is None or age > 5.0:
      st.level, st.message = DiagnosticStatus.ERROR, "no data from receiver"
    elif fix.status < NavSatStatus.STATUS_FIX:
      st.level, st.message = DiagnosticStatus.WARN, "searching (no fix)"
    else:
      st.level, st.message = DiagnosticStatus.OK, STATUS_NAMES.get(fix.status, "fix")
    values = {"backend": self.backend_name}
    if fix is not None:
      values.update(satellites=str(fix.num_sv), hdop=f"{fix.hdop:.2f}", status=STATUS_NAMES.get(fix.status, "?"),
                    last_solution_age_s=f"{age:.1f}")
    if getattr(self.backend, "decoder", None) is not None and self.backend.decoder.geoid_missing:
      values["warning"] = "receiver reports no geoid separation; altitude is above mean sea level"
    values.update({k: str(v) for k, v in self.extra_status.items()})
    st.values = [KeyValue(key=k, value=v) for k, v in values.items()]
    msg = DiagnosticArray(status=[st])
    msg.header.stamp = self.get_clock().now().to_msg()
    self.pub_diag.publish(msg)

  # *** receive loop ***
  def run(self) -> None:
    while not self.stop.is_set():
      try:
        self.backend.start()
        while not self.stop.is_set():
          for item in self.backend.read():
            if isinstance(item, GnssFix):
              self.publish_fix(item)
            elif isinstance(item, GnssVelocity):
              self.publish_velocity(item)
            elif isinstance(item, GnssTime):
              self.publish_time(item)
            elif isinstance(item, dict):
              self.extra_status = item
      except Exception as e:  # keep retrying: the modem may reboot, the serial port may vanish
        self.get_logger().error(f"GNSS backend '{self.backend_name}' failed: {e}; restarting in 5 s")
        try:
          self.backend.stop()
        except Exception:
          pass
        self.stop.wait(5.0)

  def destroy_node(self):
    self.stop.set()
    self.thread.join(timeout=2.0)
    try:
      self.backend.stop()
    except Exception as e:
      self.get_logger().warning(f"GNSS shutdown: {e}")
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = GnssNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
