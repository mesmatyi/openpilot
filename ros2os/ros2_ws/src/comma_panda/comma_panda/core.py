"""panda helpers independent of ROS: CAN frame conversion, health decoding, fan fallback."""
from dataclasses import dataclass

CAN_RETURNED_BUS_OFFSET = 0x80  # frames the panda echoes back after sending them
CAN_REJECTED_BUS_OFFSET = 0xC0  # frames the safety model refused to send
MAX_STD_ID = 0x7FF
MAX_EXT_ID = 0x1FFFFFFF
CAN_FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)
NUM_BUSES = 3

HW_TYPES = {0: "unknown", 1: "whitePanda", 2: "greyPanda", 3: "blackPanda", 4: "pedal", 5: "uno", 6: "dos",
            7: "redPanda", 8: "redPandaV2", 9: "tres", 10: "cuatro"}
LEC_ERRORS = {0: "noError", 1: "stuffError", 2: "formError", 3: "ackError", 4: "bit1Error", 5: "bit0Error",
              6: "crcError", 7: "noChange"}
# opendbc CarParams.SafetyModel ids (stable, they are the panda's wire protocol); used only if opendbc is unavailable
SAFETY_MODELS = {0: "silent", 1: "hondaNidec", 2: "toyota", 3: "elm327", 4: "gm", 5: "hondaBoschGiraffe", 6: "ford",
                 7: "cadillac", 8: "hyundai", 9: "chrysler", 10: "tesla", 11: "subaru", 12: "gmPassive", 13: "mazda",
                 14: "nissan", 15: "volkswagen", 16: "toyotaIpas", 17: "allOutput", 18: "gmAscm", 19: "noOutput",
                 20: "hondaBosch", 21: "volkswagenPq", 22: "subaruPreglobal", 23: "hyundaiLegacy",
                 24: "hyundaiCommunity", 25: "volkswagenMlb", 26: "hongqi", 27: "body", 28: "hyundaiCanfd",
                 29: "volkswagenMqbEvo", 30: "chryslerCusw", 31: "psa", 32: "fcaGiorgio", 33: "rivian",
                 34: "volkswagenMeb"}
# modes that are safe to select without a car port: no TX, or diagnostics only
LISTEN_ONLY_MODES = ("silent", "noOutput")


@dataclass
class Frame:
  bus: int
  id: int
  data: bytes
  is_extended: bool = False
  returned: bool = False
  rejected: bool = False


def from_panda(address: int, data: bytes, src: int) -> Frame:
  """Convert a (address, data, bus) tuple from Panda.can_recv()."""
  returned = rejected = False
  if src >= CAN_REJECTED_BUS_OFFSET:
    rejected, src = True, src - CAN_REJECTED_BUS_OFFSET
  elif src >= CAN_RETURNED_BUS_OFFSET:
    returned, src = True, src - CAN_RETURNED_BUS_OFFSET
  # the panda wire format carries an extended flag, but the Python API infers it from the ID like can_send does
  return Frame(bus=src, id=address, data=bytes(data), is_extended=address > MAX_STD_ID, returned=returned, rejected=rejected)


def to_panda(bus: int, can_id: int, data: bytes, is_extended: bool) -> tuple[int, bytes, int]:
  """Validate a frame for Panda.can_send_many(); raises ValueError."""
  if not 0 <= bus < NUM_BUSES + 1:
    raise ValueError(f"bus {bus} out of range")
  if len(data) not in CAN_FD_LENGTHS:
    raise ValueError(f"invalid CAN/CAN FD payload length {len(data)}")
  limit = MAX_EXT_ID if is_extended else MAX_STD_ID
  if not 0 <= can_id <= limit:
    raise ValueError(f"id 0x{can_id:X} out of range for {'extended' if is_extended else 'standard'} frame")
  if is_extended and can_id <= MAX_STD_ID:
    # panda.can_send only marks IDs >= 0x800 as extended; an extended frame with a small ID cannot be expressed
    raise ValueError(f"extended frame id 0x{can_id:X} <= 0x7FF is not supported by the panda API")
  return can_id, bytes(data), bus


def safety_model_name(value) -> str:
  try:
    from opendbc.car.structs import CarParams
    enum = CarParams.SafetyModel
    for name in dir(enum):
      member = getattr(enum, name)
      if not name.startswith("_") and isinstance(member, int) and int(member) == int(value):
        return name
  except Exception:
    pass
  return SAFETY_MODELS.get(int(value), str(value))


def safety_model_value(name: str):
  """Resolve a safety model name to what Panda.set_safety_mode() expects."""
  try:
    from opendbc.car.structs import CarParams
    return getattr(CarParams.SafetyModel, name)
  except ImportError:
    for k, v in SAFETY_MODELS.items():
      if v == name:
        return k
  except AttributeError:
    pass
  raise ValueError(f"unknown safety model '{name}'")


def hw_type_name(raw) -> str:
  if isinstance(raw, (bytes, bytearray)):
    raw = raw[0] if raw else 0
  return HW_TYPES.get(int(raw or 0), f"type{raw}")


def ir_percent_to_panda(percent: int, max_panda_val: int = 50) -> int:
  """pandad maps 0..100 % onto 0..MAX_IR_PANDA_VAL (50)."""
  percent = max(0, min(100, int(percent)))
  return round(percent * max_panda_val / 100)


def fallback_fan_percent(max_temp_c: float) -> int:
  """Feed-forward fan curve used when no fan command arrives (openpilot: 0 % at 60 C -> 100 % at 100 C)."""
  if max_temp_c != max_temp_c:  # NaN: no thermal data, be safe
    return 100
  return int(max(0.0, min(100.0, (max_temp_c - 60.0) * 2.5)))


def health_fields(h: dict) -> dict:
  """Map Panda.health() keys onto comma_msgs/PandaState fields (SI units)."""
  g = h.get
  return {
    "uptime_s": float(g("uptime", 0)),
    "voltage": g("voltage", 0) / 1000.0,
    "current": g("current", 0) / 1000.0,
    "ignition_line": bool(g("ignition_line", False)),
    "ignition_can": bool(g("ignition_can", False)),
    "ignition": bool(g("ignition_line", False) or g("ignition_can", False)),
    "controls_allowed": bool(g("controls_allowed", False)),
    "safety_model": safety_model_name(g("safety_mode", 0)),
    "safety_param": int(g("safety_param", 0)),
    "alternative_experience": int(g("alternative_experience", 0)),
    "heartbeat_lost": bool(g("heartbeat_lost", False)),
    "power_save_enabled": bool(g("power_save_enabled", False)),
    "harness_status": int(g("car_harness_status", 0)),
    "fault_status": int(g("fault_status", 0)),
    "faults": int(g("faults", 0)),
    "safety_tx_blocked": int(g("safety_tx_blocked", 0)),
    "safety_rx_invalid": int(g("safety_rx_invalid", 0)),
    "tx_buffer_overflow": int(g("tx_buffer_overflow", 0)),
    "rx_buffer_overflow": int(g("rx_buffer_overflow", 0)),
    "spi_error_count": int(g("spi_error_count", 0)),
    "interrupt_load": float(g("interrupt_load", 0.0)),
    "fan_power": int(g("fan_power", 0)),
    "sbu1_voltage_mv": int(g("sbu1_voltage_mV", 0)),
    "sbu2_voltage_mv": int(g("sbu2_voltage_mV", 0)),
  }


def can_health_fields(bus: int, h: dict) -> dict:
  g = h.get
  return {
    "bus": bus,
    "bus_off": bool(g("bus_off", False)),
    "bus_off_cnt": int(g("bus_off_cnt", 0)),
    "error_warning": bool(g("error_warning", False)),
    "error_passive": bool(g("error_passive", False)),
    "last_error": LEC_ERRORS.get(int(g("last_error", 0)), str(g("last_error"))),
    "receive_error_cnt": int(g("receive_error_cnt", 0)),
    "transmit_error_cnt": int(g("transmit_error_cnt", 0)),
    "total_error_cnt": int(g("total_error_cnt", 0)),
    "total_tx_lost_cnt": int(g("total_tx_lost_cnt", 0)),
    "total_rx_lost_cnt": int(g("total_rx_lost_cnt", 0)),
    "total_tx_cnt": int(g("total_tx_cnt", 0)),
    "total_rx_cnt": int(g("total_rx_cnt", 0)),
    "total_fwd_cnt": int(g("total_fwd_cnt", 0)),
    "can_speed_kbps": int(g("can_speed", 0)),
    "can_data_speed_kbps": int(g("can_data_speed", 0)),
    "canfd_enabled": bool(g("canfd_enabled", False)),
    "brs_enabled": bool(g("brs_enabled", False)),
    "canfd_non_iso": bool(g("canfd_non_iso", False)),
    "irq0_call_rate": int(g("irq0_call_rate", 0)),
    "irq1_call_rate": int(g("irq1_call_rate", 0)),
    "irq2_call_rate": int(g("irq2_call_rate", 0)),
    "can_core_reset_cnt": int(g("can_core_reset_cnt", 0)),
  }
