import pytest

from comma_panda import core


def test_from_panda_bus_offsets():
  f = core.from_panda(0x123, b"\x01\x02", 1)
  assert (f.bus, f.id, f.data, f.is_extended, f.returned, f.rejected) == (1, 0x123, b"\x01\x02", False, False, False)
  f = core.from_panda(0x18DAF110, b"\x00" * 8, 0x80 + 2)
  assert (f.bus, f.is_extended, f.returned, f.rejected) == (2, True, True, False)
  f = core.from_panda(0x200, b"", 0xC0)
  assert (f.bus, f.returned, f.rejected) == (0, False, True)


def test_to_panda_validation():
  assert core.to_panda(0, 0x7FF, b"\x00" * 8, False) == (0x7FF, b"\x00" * 8, 0)
  assert core.to_panda(2, 0x18DAF110, bytes(64), True)[0] == 0x18DAF110
  with pytest.raises(ValueError):
    core.to_panda(0, 0x800, b"", False)        # too large for 11-bit
  with pytest.raises(ValueError):
    core.to_panda(0, 0x100, b"", True)         # panda API cannot express small extended IDs
  with pytest.raises(ValueError):
    core.to_panda(0, 0x100, bytes(9), False)   # 9 is not a valid CAN FD length
  with pytest.raises(ValueError):
    core.to_panda(7, 0x100, b"", False)
  for n in core.CAN_FD_LENGTHS:
    core.to_panda(1, 0x10, bytes(n), False)


def test_safety_model_names_fallback():
  assert core.safety_model_name(19) == "noOutput"
  assert core.safety_model_name(17) == "allOutput"
  assert core.safety_model_name(0) == "silent"
  assert core.safety_model_name(999) == "999"
  assert core.safety_model_value("elm327") in (3, "elm327") or int(core.safety_model_value("elm327")) == 3
  with pytest.raises(ValueError):
    core.safety_model_value("notAModel")


def test_hw_type_and_ir_mapping():
  assert core.hw_type_name(b"\x0a") == "cuatro"
  assert core.hw_type_name(b"\x09") == "tres"
  assert core.hw_type_name(b"") == "unknown"
  assert core.ir_percent_to_panda(0) == 0
  assert core.ir_percent_to_panda(100) == 50
  assert core.ir_percent_to_panda(150) == 50
  assert core.ir_percent_to_panda(50) == 25


def test_fallback_fan_curve():
  assert core.fallback_fan_percent(40.0) == 0
  assert core.fallback_fan_percent(80.0) == 50
  assert core.fallback_fan_percent(120.0) == 100
  assert core.fallback_fan_percent(float("nan")) == 100


def test_health_fields_units_and_ignition():
  h = {"uptime": 12, "voltage": 12500, "current": 350, "ignition_line": 0, "ignition_can": 1, "safety_mode": 19,
       "car_harness_status": 1, "fan_power": 40, "sbu1_voltage_mV": 3300, "interrupt_load": 0.12}
  f = core.health_fields(h)
  assert f["voltage"] == pytest.approx(12.5)
  assert f["current"] == pytest.approx(0.35)
  assert f["ignition"] is True and f["ignition_line"] is False
  assert f["safety_model"] == "noOutput"
  assert f["harness_status"] == 1
  assert f["sbu1_voltage_mv"] == 3300
  assert core.health_fields({})["voltage"] == 0.0


def test_can_health_fields():
  f = core.can_health_fields(1, {"bus_off": 1, "last_error": 3, "can_speed": 500, "canfd_enabled": True})
  assert f["bus"] == 1 and f["bus_off"] is True
  assert f["last_error"] == "ackError"
  assert f["can_speed_kbps"] == 500 and f["canfd_enabled"] is True


def test_message_fields_match_msg_definitions():
  # every key produced must exist in the .msg definitions, otherwise the PandaState(**fields) call fails at runtime
  import os
  import re
  here = os.path.dirname(__file__)
  msg_dir = os.path.join(here, "..", "..", "comma_msgs", "msg")

  def names(fn):
    out = set()
    for line in open(os.path.join(msg_dir, fn)):
      line = line.split("#")[0].strip()
      m = re.match(r"^\S+\s+([a-z_0-9]+)$", line)
      if m:
        out.add(m.group(1))
    return out

  assert set(core.health_fields({})) <= names("PandaState.msg")
  assert set(core.can_health_fields(0, {})) <= names("CanBusHealth.msg")
  assert {"serial", "hw_type", "firmware_version", "fan_speed_rpm", "can_health", "header"} <= names("PandaState.msg")
