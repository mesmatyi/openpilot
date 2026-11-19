"""ROS 2 node publishing the comma device IMU.

Topics
  imu/data_raw      sensor_msgs/Imu          accel + gyro, no orientation (orientation_covariance[0] = -1)
  imu/temperature   sensor_msgs/Temperature  die temperature of the IMU
  imu/acceleration, imu/angular_velocity     (split_topics: true) same layout as tools/mcap exports

Samples are stamped with the kernel timestamp of the data-ready interrupt
(GPIO line event), converted to ROS (realtime) time.
"""
import os
import select
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature

from comma_hal import clock, gpio
from comma_hal.i2c import SMBus
from comma_hal.pins import GPIO
from comma_imu.lsm6ds3 import LSM6DS3, chip_to_device_flu, default_stdevs

SETTLE_TIME_S = 0.5  # openpilot discards the first 0.5 s after init as well


class ImuNode(Node):
  def __init__(self):
    super().__init__("imu")
    p = self.declare_parameter
    self.i2c_bus = p("i2c_bus", 1).value
    self.address = p("address", 0x6A).value
    self.odr_hz = p("odr_hz", 104).value
    self.accel_range_g = p("accel_range_g", 2).value
    self.gyro_range_dps = p("gyro_range_dps", 250).value
    self.use_interrupt = p("use_interrupt", True).value
    self.gpiochip = p("gpiochip", 0).value
    self.irq_line = p("irq_line", GPIO.LSM_INT).value
    self.frame_id = p("frame_id", "imu_link").value
    self.axes = p("axes", "device_flu").value  # "device_flu" or "sensor"
    self.temperature_rate_hz = p("temperature_rate_hz", 1.0).value
    self.split_topics = p("split_topics", False).value
    self.run_self_test = p("self_test", False).value
    accel_stdev = p("linear_acceleration_stdev", -1.0).value
    gyro_stdev = p("angular_velocity_stdev", -1.0).value

    if self.axes not in ("device_flu", "sensor"):
      raise ValueError("axes must be 'device_flu' or 'sensor'")

    self.lock = threading.Lock()
    self.bus = SMBus(self.i2c_bus)
    self.imu = LSM6DS3(self.bus, self.address, self.odr_hz, self.accel_range_g, self.gyro_range_dps)
    self.imu.reset()
    variant = self.imu.probe()
    if self.run_self_test:
      try:
        self.imu.self_test()
        self.get_logger().info("IMU self test passed")
      except LSM6DS3.SelfTestFailed as e:
        self.get_logger().error(str(e))
      self.imu.reset()
    self.imu.configure(interrupts=self.use_interrupt)

    d_accel, d_gyro = default_stdevs(variant, self.odr_hz)
    self.accel_cov = (accel_stdev if accel_stdev > 0 else d_accel) ** 2
    self.gyro_cov = (gyro_stdev if gyro_stdev > 0 else d_gyro) ** 2
    mode = "interrupt" if self.use_interrupt else "polling"
    self.get_logger().info(f"{variant} on i2c-{self.i2c_bus} @0x{self.address:02X}: {self.odr_hz} Hz, +-{self.accel_range_g} g, " +
                           f"+-{self.gyro_range_dps} dps, frame '{self.frame_id}' ({self.axes}), {mode} mode")

    self.pub_imu = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
    self.pub_temp = self.create_publisher(Temperature, "imu/temperature", qos_profile_sensor_data)
    if self.split_topics:
      self.pub_accel = self.create_publisher(Imu, "imu/acceleration", qos_profile_sensor_data)
      self.pub_gyro = self.create_publisher(Imu, "imu/angular_velocity", qos_profile_sensor_data)

    if self.temperature_rate_hz > 0:
      self.create_timer(1.0 / self.temperature_rate_hz, self.publish_temperature)

    self.start_mono = time.monotonic()
    self.event_clock: str | None = None
    self.stop = threading.Event()
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()

  # *** data path ***
  def _vec(self, v):
    return chip_to_device_flu(v) if self.axes == "device_flu" else v

  def _stamp(self, msg, ts_ns: int) -> None:
    msg.header.stamp.sec, msg.header.stamp.nanosec = clock.split_ns(ts_ns)
    msg.header.frame_id = self.frame_id

  def publish_sample(self, ts_ns: int) -> None:
    with self.lock:
      s = self.imu.read_sample()
    if s.accel is None and s.gyro is None:
      return  # the duplicate edge of a pulse, or a shared IRQ for the other sensor
    if time.monotonic() - self.start_mono < SETTLE_TIME_S:
      return

    msg = Imu()
    self._stamp(msg, ts_ns)
    msg.orientation_covariance[0] = -1.0
    if s.accel is not None:
      a = self._vec(s.accel)
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
      msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = self.accel_cov
    else:
      msg.linear_acceleration_covariance[0] = -1.0
    if s.gyro is not None:
      g = self._vec(s.gyro)
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
      msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = self.gyro_cov
    else:
      msg.angular_velocity_covariance[0] = -1.0
    self.pub_imu.publish(msg)

    if self.split_topics:
      for pub, keep_accel in ((self.pub_accel, True), (self.pub_gyro, False)):
        if (s.accel if keep_accel else s.gyro) is None:
          continue
        m = Imu()
        m.header = msg.header
        m.orientation_covariance[0] = -1.0
        if keep_accel:
          m.linear_acceleration = msg.linear_acceleration
          m.linear_acceleration_covariance = msg.linear_acceleration_covariance
          m.angular_velocity_covariance[0] = -1.0
        else:
          m.angular_velocity = msg.angular_velocity
          m.angular_velocity_covariance = msg.angular_velocity_covariance
          m.linear_acceleration_covariance[0] = -1.0
        pub.publish(m)

  def publish_temperature(self) -> None:
    try:
      with self.lock:
        t = self.imu.read_temperature()
    except OSError as e:
      self.get_logger().warning(f"temperature read failed: {e}", throttle_duration_sec=10.0)
      return
    msg = Temperature()
    self._stamp(msg, time.time_ns())
    msg.temperature = t
    msg.variance = 0.0
    self.pub_temp.publish(msg)

  def _event_time(self, ts: int) -> int:
    # The line-event clock differs between kernels; detect it once, re-detect if it stops matching.
    if self.event_clock is None or abs(clock.now_ns(self.event_clock) - ts) > 1_000_000_000:
      self.event_clock = clock.guess_clock(ts)
      self.get_logger().info(f"IMU interrupt timestamps use CLOCK_{self.event_clock.upper()}")
    return clock.to_realtime_ns(ts, self.event_clock)

  def run(self) -> None:
    if self.use_interrupt:
      try:
        self.run_interrupt()
        return
      except OSError as e:
        self.get_logger().error(f"IMU interrupt unavailable ({e}); falling back to polling")
        with self.lock:
          self.imu.configure(interrupts=False)
    self.run_polling()

  def run_interrupt(self) -> None:
    fd = gpio.request_line_events(self.gpiochip, self.irq_line, "comma_imu")
    poller = select.poll()
    poller.register(fd, select.POLLIN | select.POLLPRI)
    try:
      while not self.stop.is_set():
        if not poller.poll(100):
          self.get_logger().warning("no IMU interrupt for 100 ms", throttle_duration_sec=5.0)
          continue
        events = gpio.read_line_events(fd)
        if not events:
          continue
        # with both edges requested a single pulse can produce two events; the
        # last one belongs to the newest sample, which is the one we read
        try:
          self.publish_sample(self._event_time(events[-1].timestamp))
        except OSError as e:
          self.get_logger().error(f"IMU read failed: {e}", throttle_duration_sec=5.0)
    finally:
      os.close(fd)

  def run_polling(self) -> None:
    period = 1.0 / (2.0 * self.odr_hz)
    while not self.stop.is_set():
      try:
        self.publish_sample(time.time_ns())
      except OSError as e:
        self.get_logger().error(f"IMU read failed: {e}", throttle_duration_sec=5.0)
      time.sleep(period)

  def destroy_node(self):
    self.stop.set()
    self.thread.join(timeout=1.0)
    try:
      with self.lock:
        self.imu.shutdown()
    except OSError:
      pass
    self.bus.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = ImuNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
