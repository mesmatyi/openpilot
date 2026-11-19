import os

import pytest

from comma_hal.device import ThermalZone, device_type, thermal_config
from comma_hardware import core


def test_first_order_filter():
  f = core.FirstOrderFilter(5.0, 0.5)
  assert f.update(50.0) == 50.0  # initialised by the first sample
  assert 50.0 < f.update(60.0) < 60.0


def test_fan_controller_feedforward_and_limits():
  fan = core.FanController(2.0, "mici")
  assert fan.update(40.0, active=True) == 30   # onroad lower limit
  assert fan.update(40.0, active=False) == 0
  assert fan.update(80.0, active=False) <= 30  # offroad cap
  fan = core.FanController(2.0, "mici")
  # openpilot's PIDController clamps the integrator into [neg_limit, pos_limit], so onroad it starts at 30
  assert fan.update(80.0, active=True) == 80   # 30 (integrator floor) + 50 (feed-forward)
  assert fan.update(110.0, active=True) == 100
  assert fan.update(float("nan"), active=True) == 100


def test_fan_integrator_and_tizi_offset():
  fan = core.FanController(2.0, "mici")
  outs = [fan.update(85.0, True) for _ in range(200)]
  assert outs[-1] > outs[0]  # integrates above the 75 C setpoint
  tizi = core.FanController(2.0, "tizi")
  assert tizi.update(80.0, True) == 67  # 30 + ff 60..100 shifted by +5 C: (80-65)/40 -> 37.5


def test_thermal_bands_hysteresis():
  t = core.ThermalStatus("mici")
  assert t.update(95.0) == core.THERMAL_OK
  assert t.update(101.0) == core.THERMAL_OVERHEATED
  assert t.update(95.0) == core.THERMAL_OVERHEATED  # still above 92
  assert t.update(108.0) == core.THERMAL_CRITICAL
  assert t.update(97.0) == core.THERMAL_OVERHEATED
  assert t.update(90.0) == core.THERMAL_OK
  assert core.ThermalStatus("tizi").update(97.0) == core.THERMAL_OVERHEATED


def test_power_protection():
  p = core.PowerProtection()
  p.update(0.0, 12.6, ignition=True)
  assert p.should_shutdown(10.0, 5000, ignition=True, in_car=True) == (False, "")
  p.update(100.0, 12.6, ignition=False)
  assert not p.should_shutdown(100.0 + core.DELAY_SHUTDOWN_TIME_S - 1, 5000, False, True)[0]
  for i in range(2000):  # low-pass converges to the low voltage
    p.update(200.0 + i, 11.0, ignition=False)
  down, reason = p.should_shutdown(100.0 + core.DELAY_SHUTDOWN_TIME_S + 10, 5000, False, True)
  assert down and "battery" in reason
  assert not p.should_shutdown(100.0 + core.DELAY_SHUTDOWN_TIME_S + 10, 5000, False, in_car=False)[0]


def test_power_protection_max_offroad():
  p = core.PowerProtection(max_offroad_s=3600)
  p.update(0.0, 12.6, ignition=True)
  p.update(1.0, 12.6, ignition=False)
  down, reason = p.should_shutdown(3700.0, 4000, False, True)
  assert down and "offroad" in reason


def test_power_protection_needs_started_or_min_uptime():
  p = core.PowerProtection(max_offroad_s=10)
  p.update(0.0, 12.6, ignition=False)
  assert not p.should_shutdown(1000.0, 100.0, False, True)[0]   # never started, just booted
  assert p.should_shutdown(1000.0, core.MIN_ON_TIME_S + 1, False, True)[0]


def test_cpu_usage(tmp_path):
  stat = tmp_path / "stat"
  stat.write_text("cpu  1 1 1 1\ncpu0 100 0 100 800 0 0 0\ncpu1 0 0 0 1000 0 0 0\n")
  c = core.CpuUsage(str(stat))
  assert c.update() == [0.0, 0.0]
  stat.write_text("cpu  1 1 1 1\ncpu0 150 0 150 900 0 0 0\ncpu1 0 0 0 1100 0 0 0\n")
  assert c.update() == pytest.approx([50.0, 0.0])


def test_memory_and_network_helpers(tmp_path):
  mi = tmp_path / "meminfo"
  mi.write_text("MemTotal: 1000 kB\nMemFree: 100 kB\nMemAvailable: 250 kB\n")
  assert core.memory_usage_percent(str(mi)) == pytest.approx(75.0)
  route = tmp_path / "route"
  route.write_text("\n".join([
    "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask",
    "wlan0\t00000000\t0101A8C0\t0003\t0\t0\t600\t00000000",
    "ppp0\t00000000\t00000000\t0001\t0\t0\t1000\t00000000",
  ]) + "\n")
  assert core.default_route_iface(str(route)) == "wlan0"
  assert core.iface_type("wlan0") == "wifi" and core.iface_type("ppp0") == "cellular" and core.iface_type(None) == "none"
  assert 0 <= core.free_space_percent(str(tmp_path)) <= 100


def test_thermal_zones(tmp_path):
  for i, (name, temp) in enumerate((("cpu0-silver-usr", "45000"), ("intake", "30500"))):
    d = tmp_path / f"thermal_zone{i}"
    d.mkdir()
    (d / "type").write_text(name + "\n")
    (d / "temp").write_text(temp + "\n")
  (tmp_path / "cooling_device0").mkdir()
  assert ThermalZone("cpu0-silver-usr", root=str(tmp_path)).read() == 45.0
  cfg = thermal_config("mici", root=str(tmp_path))
  assert cfg.intake.read() == 30.5
  assert cfg.gpu[0].read() != cfg.gpu[0].read()  # missing zone -> NaN
  assert thermal_config("tizi", root=str(tmp_path)).intake is None


def test_device_type(tmp_path):
  m = tmp_path / "model"
  m.write_bytes(b"comma mici\x00")
  assert device_type(str(m)) == "mici"
  m.write_bytes(b"Generic PC")
  assert device_type(str(m)) == "pc"
  assert device_type(os.path.join(str(tmp_path), "missing")) == "pc"
