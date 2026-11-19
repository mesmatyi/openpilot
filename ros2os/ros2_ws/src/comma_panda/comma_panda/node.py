"""ROS 2 node for the panda CAN interface.

Topics (published)
  can/rx          comma_msgs/CanFrameArray  every frame drained from the panda (100 Hz batches)
  panda/state     comma_msgs/PandaState     health, ignition, voltage, per-bus CAN health (2 Hz)
  /diagnostics    diagnostic_msgs/DiagnosticArray

Topics (subscribed)
  can/tx                   comma_msgs/CanFrameArray  frames to send (only leave the panda if the safety model allows)
  hardware/fan_speed_cmd   std_msgs/UInt8            fan power [%] (from comma_hardware)
  hardware/ir_power_cmd    std_msgs/UInt8            driver-camera IR LED power [%], expires after 2 s

The panda enforces its safety model in firmware. The default `noOutput` blocks
every transmission; `allOutput` lets anything onto the bus and is for bench
debugging only.
"""
import os
import queue
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import UInt8

from comma_msgs.msg import CanBusHealth, CanFrame, CanFrameArray, PandaState
from comma_hal import clock, device, gpio
from comma_hal.pins import GPIO
from comma_panda import core

FAN_CMD_TIMEOUT_S = 10.0
IR_CMD_TIMEOUT_S = 2.0
TX_QUEUE_MAX = 4096


def reset_internal_panda(recover: bool = False) -> None:
  """Pulse the STM32 reset line (openpilot HardwareComma.reset_internal_panda / recover_internal_panda)."""
  gpio.gpio_init(GPIO.STM_RST_N, True)
  gpio.gpio_init(GPIO.STM_BOOT0, True)
  gpio.gpio_set(GPIO.STM_RST_N, True)
  gpio.gpio_set(GPIO.STM_BOOT0, recover)
  time.sleep(0.01)
  gpio.gpio_set(GPIO.STM_RST_N, False)
  if recover:
    time.sleep(0.01)
    gpio.gpio_set(GPIO.STM_BOOT0, False)


def flash_if_needed(Panda, McuType, FW_PATH, serial: str, log) -> None:
  """Bring the panda firmware in line with the bundled panda library (port of openpilot's pandad.py)."""
  fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
  if not os.path.isfile(fn):
    log.warning(f"no bundled panda firmware at {fn}; not checking the firmware version")
    return
  expected = Panda.get_signature_from_firmware(fn)
  p = Panda(serial)
  try:
    internal = p.is_internal()
    if p.bootstub or p.get_signature() != expected:
      log.warning("panda firmware out of date, flashing")
      p.flash()
    if p.bootstub:
      log.warning("flashed firmware does not boot, recovering bootstub")
      if internal:
        reset_internal_panda(recover=True)
      p.recover(reset=not internal)
    if p.bootstub or p.get_signature() != expected:
      raise RuntimeError("panda firmware mismatch after flashing")
  finally:
    p.close()


class PandaNode(Node):
  def __init__(self):
    super().__init__("panda")
    p = self.declare_parameter
    self.serial = p("serial", "").value
    self.safety_mode = p("safety_mode", "noOutput").value
    self.safety_param = p("safety_param", 0).value
    self.can_speed_kbps = list(p("can_speed_kbps", [500, 500, 500]).value)
    self.can_data_speed_kbps = list(p("can_data_speed_kbps", [2000, 2000, 2000]).value)
    self.can_fd_auto = p("can_fd_auto", True).value
    self.poll_rate_hz = p("poll_rate_hz", 100.0).value
    self.state_rate_hz = p("state_rate_hz", 2.0).value
    self.frame_id = p("frame_id", "panda").value
    self.flash_firmware = p("flash_firmware", True).value
    self.reset_on_start = p("reset_on_start", True).value
    self.power_save_when_off = p("power_save_when_ignition_off", False).value
    self.control_fan = p("control_fan", True).value
    self.control_ir = p("control_ir", True).value
    core.safety_model_value(self.safety_mode)  # validate early
    self.add_on_set_parameters_callback(self.on_set_parameters)

    rx_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    self.pub_rx = self.create_publisher(CanFrameArray, "can/rx", rx_qos)
    self.pub_state = self.create_publisher(PandaState, "panda/state", qos_profile_sensor_data)
    self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
    self.create_subscription(CanFrameArray, "can/tx", self.on_tx, rx_qos)
    self.create_subscription(UInt8, "hardware/fan_speed_cmd", self.on_fan, 10)
    self.create_subscription(UInt8, "hardware/ir_power_cmd", self.on_ir, 10)

    self.tx_queue: queue.Queue = queue.Queue(maxsize=TX_QUEUE_MAX)
    self.tx_dropped = 0
    self.fan_cmd: int | None = None
    self.fan_cmd_mono = 0.0
    self.ir_cmd = 0
    self.ir_cmd_mono = 0.0
    self.pending_safety = True
    self.thermal = device.thermal_config(device.device_type())
    self.panda = None
    self.info = {"serial": "", "hw_type": "unknown", "firmware_version": ""}
    self.last_state: PandaState | None = None
    self.last_state_mono = 0.0

    self.stop = threading.Event()
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()
    self.create_timer(1.0, self.publish_diagnostics)

  # *** parameter / topic callbacks (executor thread) ***
  def on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
    for prm in params:
      if prm.name == "safety_mode":
        try:
          core.safety_model_value(prm.value)
        except ValueError as e:
          return SetParametersResult(successful=False, reason=str(e))
        if prm.value not in core.LISTEN_ONLY_MODES:
          self.get_logger().warning(f"safety mode '{prm.value}' allows the panda to transmit on the CAN bus")
        self.safety_mode = prm.value
        self.pending_safety = True
      elif prm.name == "safety_param":
        self.safety_param = prm.value
        self.pending_safety = True
    return SetParametersResult(successful=True)

  def on_tx(self, msg: CanFrameArray) -> None:
    for f in msg.frames:
      try:
        item = core.to_panda(f.bus, f.id, bytes(f.data), f.is_extended)
      except ValueError as e:
        self.get_logger().warning(f"dropping invalid TX frame: {e}", throttle_duration_sec=5.0)
        continue
      try:
        self.tx_queue.put_nowait(item)
      except queue.Full:
        self.tx_dropped += 1

  def on_fan(self, msg: UInt8) -> None:
    self.fan_cmd, self.fan_cmd_mono = min(100, msg.data), time.monotonic()

  def on_ir(self, msg: UInt8) -> None:
    self.ir_cmd, self.ir_cmd_mono = min(100, msg.data), time.monotonic()

  # *** panda thread ***
  def connect(self):
    from panda import FW_PATH, McuType, Panda

    if self.reset_on_start and device.is_comma_device():
      try:
        reset_internal_panda()
        time.sleep(1.0)
      except OSError as e:
        self.get_logger().warning(f"could not reset the internal panda: {e}")

    serials = Panda.list()
    if not serials:
      raise RuntimeError("no panda found")
    serial = self.serial or serials[0]
    if self.flash_firmware:
      flash_if_needed(Panda, McuType, FW_PATH, serial, self.get_logger())

    pnd = Panda(serial)
    for bus in range(core.NUM_BUSES):
      pnd.set_can_speed_kbps(bus, int(self.can_speed_kbps[bus]))
      pnd.set_can_data_speed_kbps(bus, int(self.can_data_speed_kbps[bus]))
      if self.can_fd_auto:
        pnd.set_canfd_auto(bus, True)
    pnd.set_power_save(False)
    self.info = {"serial": serial, "hw_type": core.hw_type_name(pnd.get_type()), "firmware_version": pnd.get_version()}
    self.get_logger().info(f"connected to panda {serial} ({self.info['hw_type']}, fw {self.info['firmware_version']})")
    self.pending_safety = True
    return pnd

  def apply_safety(self, pnd) -> None:
    pnd.set_safety_mode(core.safety_model_value(self.safety_mode), self.safety_param)
    self.pending_safety = False
    # re-applied whenever the panda reports another model (e.g. it fell back to silent after a lost heartbeat)
    self.get_logger().info(f"safety model set to {self.safety_mode} (param {self.safety_param})",
                           throttle_duration_sec=30.0)

  def drain_tx(self, pnd) -> None:
    batch = []
    while len(batch) < 512:
      try:
        batch.append(self.tx_queue.get_nowait())
      except queue.Empty:
        break
    if batch:
      pnd.can_send_many(batch, timeout=10)

  def publish_rx(self, frames: list) -> None:
    if not frames:
      return
    msg = CanFrameArray()
    msg.header.stamp.sec, msg.header.stamp.nanosec = clock.split_ns(time.time_ns())
    msg.header.frame_id = self.frame_id
    out = []
    for address, data, src in frames:
      f = core.from_panda(address, data, src)
      out.append(CanFrame(bus=f.bus, id=f.id, is_extended=f.is_extended, returned=f.returned, rejected=f.rejected,
                          data=f.data))
    msg.frames = out
    self.pub_rx.publish(msg)

  def update_peripherals(self, pnd, ignition: bool) -> None:
    now = time.monotonic()
    if self.control_fan:
      if self.fan_cmd is not None and now - self.fan_cmd_mono < FAN_CMD_TIMEOUT_S:
        fan = self.fan_cmd
      else:
        temps = [z.read() for z in self.thermal.cpu + self.thermal.gpu + [self.thermal.memory]]
        temps = [t for t in temps if t == t]
        fan = core.fallback_fan_percent(max(temps) if temps else float("nan"))
      pnd.set_fan_power(fan)
    if self.control_ir:
      ir = self.ir_cmd if now - self.ir_cmd_mono < IR_CMD_TIMEOUT_S else 0
      pnd.set_ir_power(core.ir_percent_to_panda(ir))
    if self.power_save_when_off:
      pnd.set_power_save(not ignition)

  def publish_state(self, pnd) -> bool:
    h = pnd.health()
    fields = core.health_fields(h)
    msg = PandaState(**fields, serial=self.info["serial"], hw_type=self.info["hw_type"],
                     firmware_version=self.info["firmware_version"])
    msg.header.stamp.sec, msg.header.stamp.nanosec = clock.split_ns(time.time_ns())
    msg.header.frame_id = self.frame_id
    try:
      msg.fan_speed_rpm = int(pnd.get_fan_rpm())
    except Exception:
      pass
    health = []
    for bus in range(core.NUM_BUSES):
      try:
        health.append(CanBusHealth(**core.can_health_fields(bus, pnd.can_health(bus))))
      except Exception:
        pass
    msg.can_health = health
    self.pub_state.publish(msg)
    self.last_state, self.last_state_mono = msg, time.monotonic()
    # the silent model does not ACK; like pandad, keep the bus live unless silent was explicitly requested
    if fields["safety_model"] != self.safety_mode:
      self.pending_safety = True
    return fields["ignition"]

  def run(self) -> None:
    while not self.stop.is_set():
      try:
        self.panda = pnd = self.connect()
        period = 1.0 / self.poll_rate_hz
        state_every = max(1, round(self.poll_rate_hz / self.state_rate_hz))
        heartbeat_every = max(1, round(self.poll_rate_hz / 10.0))  # 10 Hz, like pandad
        frame = 0
        next_t = time.monotonic()
        while not self.stop.is_set():
          if self.pending_safety:
            self.apply_safety(pnd)
          self.drain_tx(pnd)
          self.publish_rx(pnd.can_recv())
          if frame % heartbeat_every == 0:
            pnd.send_heartbeat(engaged=False)
          if frame % state_every == 0:
            ignition = self.publish_state(pnd)
            self.update_peripherals(pnd, ignition)
          frame += 1
          next_t += period
          delay = next_t - time.monotonic()
          if delay > 0:
            time.sleep(delay)
          else:
            next_t = time.monotonic()
      except Exception as e:
        self.get_logger().error(f"panda error: {e}; reconnecting in 1 s")
        self.close_panda()
        self.stop.wait(1.0)

  def close_panda(self) -> None:
    if self.panda is not None:
      try:
        self.panda.set_safety_mode(core.safety_model_value("noOutput"))
      except Exception:
        pass
      try:
        self.panda.close()
      except Exception:
        pass
      self.panda = None

  def publish_diagnostics(self) -> None:
    st = DiagnosticStatus(name="panda: CAN interface", hardware_id=self.info["serial"] or "panda")
    s = self.last_state
    if s is None or time.monotonic() - self.last_state_mono > 3.0:
      st.level, st.message = DiagnosticStatus.ERROR, "panda not connected"
    elif s.heartbeat_lost or s.fault_status != 0:
      st.level, st.message = DiagnosticStatus.WARN, "panda fault" if s.fault_status else "heartbeat lost"
    else:
      st.level, st.message = DiagnosticStatus.OK, f"safety {s.safety_model}, ignition {'on' if s.ignition else 'off'}"
    if s is not None:
      st.values = [KeyValue(key=k, value=str(v)) for k, v in (
        ("hw_type", s.hw_type), ("firmware", s.firmware_version), ("voltage_V", f"{s.voltage:.2f}"),
        ("harness", s.harness_status), ("controls_allowed", s.controls_allowed), ("faults", hex(s.faults)),
        ("tx_dropped_host", self.tx_dropped))]
    msg = DiagnosticArray(status=[st])
    msg.header.stamp = self.get_clock().now().to_msg()
    self.pub_diag.publish(msg)

  def destroy_node(self):
    self.stop.set()
    self.thread.join(timeout=2.0)
    self.close_panda()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = PandaNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
