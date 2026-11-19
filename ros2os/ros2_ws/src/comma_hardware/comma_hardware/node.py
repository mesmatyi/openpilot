"""ROS 2 device-health node for comma devices.

Topics (published)
  hardware/state           comma_msgs/DeviceState  temperatures, fan, power, resources, network (2 Hz)
  hardware/fan_speed_cmd   std_msgs/UInt8          fan power [%], applied by the panda node
  /diagnostics             diagnostic_msgs/DiagnosticArray

Topics (subscribed)
  panda/state              comma_msgs/PandaState   ignition, car voltage, harness status
  hardware/ir_power_cmd    std_msgs/UInt8          driver-camera IR LEDs [%]; expires after 2 s
"""
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8

from comma_msgs.msg import DeviceState, PandaState
from comma_hal import device
from comma_hardware import core

RATE_HZ = 2.0
TEMP_TAU_S = 5.0
IR_CMD_TIMEOUT_S = 2.0
HWMON = "/sys/class/hwmon/hwmon1"
BACKLIGHT = "/sys/class/backlight/panel0-backlight"
THERMAL_NAMES = {core.THERMAL_OK: "ok", core.THERMAL_OVERHEATED: "overheated", core.THERMAL_CRITICAL: "critical"}


def write_sysfs(path: str, value: str) -> bool:
  try:
    with open(path, "w") as f:
      f.write(value)
    return True
  except OSError:
    return False


class HardwareNode(Node):
  def __init__(self):
    super().__init__("hardware")
    p = self.declare_parameter
    self.fan_policy = p("fan_policy", "performance").value      # performance | openpilot
    self.power_protection = p("power_protection", True).value   # shut down before draining a parked car battery
    self.low_voltage = p("low_voltage_threshold", core.VBATT_PAUSE_CHARGING).value
    self.max_offroad_h = p("max_offroad_hours", 30.0).value
    self.display = p("display", "off").value                    # off | on | unchanged
    self.data_path = p("data_path", "/data").value
    if self.fan_policy not in ("performance", "openpilot"):
      raise ValueError("fan_policy must be 'performance' or 'openpilot'")

    self.dev_type = device.device_type()
    self.serial = device.serial()
    self.os_version = device.os_version()
    self.thermal = device.thermal_config(self.dev_type)
    self.fan = core.FanController(RATE_HZ, self.dev_type)
    self.temp_filter = core.FirstOrderFilter(TEMP_TAU_S, 1.0 / RATE_HZ)
    self.thermal_status = core.ThermalStatus(self.dev_type)
    self.protection = core.PowerProtection(self.low_voltage, self.max_offroad_h * 3600)
    self.cpu = core.CpuUsage()
    self.panda_state: PandaState | None = None
    self.ir_cmd, self.ir_cmd_mono, self.ir_applied = 0, 0.0, -1
    self.net = core.NetworkInfo("none", [])
    self.net_counter = 0
    self.shutting_down = False

    self.pub_state = self.create_publisher(DeviceState, "hardware/state", qos_profile_sensor_data)
    self.pub_fan = self.create_publisher(UInt8, "hardware/fan_speed_cmd", 10)
    self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
    self.create_subscription(PandaState, "panda/state", self.on_panda_state, qos_profile_sensor_data)
    self.create_subscription(UInt8, "hardware/ir_power_cmd", self.on_ir, 10)

    if self.display in ("on", "off"):
      write_sysfs(f"{BACKLIGHT}/bl_power", "0" if self.display == "on" else "4")
    product = device.PRODUCTS.get(self.dev_type, self.dev_type)
    self.get_logger().info(f"{product} ({self.dev_type}), serial {self.serial}, OS {self.os_version}, fan policy {self.fan_policy}")
    self.create_timer(1.0 / RATE_HZ, self.tick)

  def on_panda_state(self, msg: PandaState) -> None:
    self.panda_state = msg

  def on_ir(self, msg: UInt8) -> None:
    self.ir_cmd, self.ir_cmd_mono = min(100, msg.data), time.monotonic()

  def apply_ir(self) -> None:
    # the comma 3X drives its IR LEDs through the panda; tici/mici use the PMIC flash LED driver
    if self.dev_type in ("tizi", "pc"):
      return
    pct = self.ir_cmd if time.monotonic() - self.ir_cmd_mono < IR_CMD_TIMEOUT_S else 0
    if pct == self.ir_applied:
      return
    value = int(pct / 100 * 300)
    write_sysfs("/sys/class/leds/led:switch_2/brightness", "0\n")
    write_sysfs("/sys/class/leds/led:torch_2/brightness", f"{value}\n")
    if write_sysfs("/sys/class/leds/led:switch_2/brightness", f"{value}\n"):
      self.ir_applied = pct

  def tick(self) -> None:
    now = time.monotonic()
    t = self.thermal
    cpu = [z.read() for z in t.cpu]
    gpu = [z.read() for z in t.gpu]
    pmic = [z.read() for z in t.pmic]
    mem = t.memory.read()
    finite = lambda xs: [x for x in xs if x == x]  # noqa: E731
    sources = finite([mem, max(finite(cpu), default=float("nan")), max(finite(gpu), default=float("nan")),
                      max(finite(pmic), default=float("nan"))])
    max_temp = self.temp_filter.update(max(sources)) if sources else float("nan")
    status = self.thermal_status.update(max_temp) if max_temp == max_temp else core.THERMAL_OK

    ps = self.panda_state
    ignition = bool(ps.ignition) if ps is not None else False
    active = True if self.fan_policy == "performance" else ignition
    fan = self.fan.update(max_temp, active)
    self.pub_fan.publish(UInt8(data=fan))
    self.apply_ir()

    if self.net_counter % int(5 * RATE_HZ) == 0:  # network every 5 s
      self.net = core.network_info()
    self.net_counter += 1

    car_voltage = float(ps.voltage) if ps is not None else float("nan")
    in_car = ps is not None and ps.harness_status != PandaState.HARNESS_NOT_CONNECTED
    uptime = time.clock_gettime(time.CLOCK_BOOTTIME)
    self.protection.update(now, car_voltage if ps is not None else None, ignition)

    msg = DeviceState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.device_type, msg.serial, msg.os_version, msg.uptime_s = self.dev_type, self.serial, self.os_version, uptime
    msg.cpu_temp_c, msg.gpu_temp_c, msg.pmic_temp_c = cpu, gpu, pmic
    msg.dsp_temp_c, msg.memory_temp_c = t.dsp.read(), mem
    nan = float("nan")
    msg.intake_temp_c = t.intake.read() if t.intake else nan
    msg.exhaust_temp_c = t.exhaust.read() if t.exhaust else nan
    msg.gnss_temp_c = t.gnss.read() if t.gnss else nan
    msg.bottom_soc_temp_c = t.bottom_soc.read() if t.bottom_soc else nan
    msg.max_temp_c, msg.thermal_status, msg.fan_speed_percent_desired = max_temp, status, fan
    msg.cpu_usage_percent = self.cpu.update()
    msg.memory_usage_percent = core.memory_usage_percent()
    msg.gpu_usage_percent = core.gpu_usage_percent()
    msg.free_space_percent = core.free_space_percent(self.data_path)
    msg.power_draw_w = device.read_number(f"{HWMON}/power1_input", scale=1e6)
    msg.som_power_draw_w = (device.read_number("/sys/class/power_supply/bms/voltage_now") *
                            device.read_number("/sys/class/power_supply/bms/current_now") / 1e12)
    msg.car_voltage, msg.ignition = car_voltage, ignition
    msg.network_type, msg.ip_addresses = self.net.network_type, self.net.ip_addresses
    self.pub_state.publish(msg)
    self.publish_diagnostics(msg, in_car)

    if self.power_protection and not self.shutting_down:
      shutdown, reason = self.protection.should_shutdown(now, uptime, ignition, in_car)
      if shutdown:
        self.shutting_down = True
        self.get_logger().fatal(f"powering off to protect the car battery: {reason}")
        subprocess.Popen(["sudo", "-n", "systemctl", "poweroff"])

  def publish_diagnostics(self, s: DeviceState, in_car: bool) -> None:
    kv = lambda d: [KeyValue(key=k, value=str(v)) for k, v in d.items()]  # noqa: E731
    thermal = DiagnosticStatus(name="hardware: thermal", hardware_id=self.serial)
    thermal.level = {core.THERMAL_OK: DiagnosticStatus.OK, core.THERMAL_OVERHEATED: DiagnosticStatus.WARN,
                     core.THERMAL_CRITICAL: DiagnosticStatus.ERROR}[s.thermal_status]
    thermal.message = f"{THERMAL_NAMES[s.thermal_status]}, max {s.max_temp_c:.1f} C, fan {s.fan_speed_percent_desired} %"
    thermal.values = kv({"cpu_max_C": f"{max(s.cpu_temp_c, default=float('nan')):.1f}",
                         "gpu_max_C": f"{max(s.gpu_temp_c, default=float('nan')):.1f}",
                         "memory_C": f"{s.memory_temp_c:.1f}", "intake_C": f"{s.intake_temp_c:.1f}",
                         "exhaust_C": f"{s.exhaust_temp_c:.1f}"})

    power = DiagnosticStatus(name="hardware: power", hardware_id=self.serial, level=DiagnosticStatus.OK)
    power.message = f"{s.power_draw_w:.2f} W"
    if in_car and self.protection.car_voltage < self.low_voltage:
      power.level, power.message = DiagnosticStatus.WARN, f"car battery low ({self.protection.car_voltage:.2f} V)"
    power.values = kv({"car_voltage_V": f"{s.car_voltage:.2f}", "ignition": s.ignition, "in_car": in_car,
                       "som_power_W": f"{s.som_power_draw_w:.2f}", "power_protection": self.power_protection})

    res = DiagnosticStatus(name="hardware: resources", hardware_id=self.serial, level=DiagnosticStatus.OK)
    if s.free_space_percent == s.free_space_percent and s.free_space_percent < 10:
      res.level = DiagnosticStatus.WARN
    res.message = f"mem {s.memory_usage_percent:.0f} %, {self.data_path} free {s.free_space_percent:.0f} %"
    res.values = kv({"cpu_usage_percent": " ".join(f"{c:.0f}" for c in s.cpu_usage_percent),
                     "gpu_usage_percent": f"{s.gpu_usage_percent:.0f}", "uptime_s": f"{s.uptime_s:.0f}"})

    net = DiagnosticStatus(name="hardware: network", hardware_id=self.serial, level=DiagnosticStatus.OK)
    net.message = s.network_type
    net.values = kv({"addresses": " ".join(s.ip_addresses)})

    msg = DiagnosticArray(status=[thermal, power, res, net])
    msg.header = s.header
    self.pub_diag.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = HardwareNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
  main()
