"""Identity, thermal zones and sysfs readouts of comma devices."""
import os
from dataclasses import dataclass, field

DT_MODEL = "/sys/firmware/devicetree/base/model"
THERMAL_ROOT = "/sys/class/thermal"

# device codename -> product name
PRODUCTS = {"tici": "comma three", "tizi": "comma 3X", "mici": "comma four"}


def read_text(path: str, default: str = "") -> str:
  try:
    with open(path) as f:
      return f.read().strip().strip("\x00")
  except OSError:
    return default


def read_number(path: str, default: float = float("nan"), scale: float = 1.0) -> float:
  try:
    return float(read_text(path)) / scale
  except ValueError:
    return default


def device_type(model_path: str = DT_MODEL) -> str:
  """Codename of the device ("mici", "tizi", "tici"), or "pc" off-device."""
  model = read_text(model_path)
  if not model.startswith("comma"):
    return "pc"
  return model.split("comma ")[-1].strip()


def is_comma_device() -> bool:
  return os.path.isfile("/AGNOS") or device_type() != "pc"


def cmdline(path: str = "/proc/cmdline") -> dict[str, str]:
  return dict(kv.split("=", 1) for kv in read_text(path).split() if "=" in kv)


def serial() -> str:
  return cmdline().get("androidboot.serialno", "unknown")


def os_version() -> str:
  return read_text("/VERSION", "unknown")


@dataclass
class ThermalZone:
  """A /sys/class/thermal/thermal_zone* entry looked up by its `type` name."""
  name: str
  scale: float = 1000.0
  root: str = THERMAL_ROOT
  _path: str | None = field(default=None, repr=False)

  def _find(self) -> str | None:
    try:
      entries = sorted(os.listdir(self.root))
    except OSError:
      return None
    for entry in entries:
      if entry.startswith("thermal_zone") and read_text(os.path.join(self.root, entry, "type")) == self.name:
        return os.path.join(self.root, entry, "temp")
    return None

  def read(self) -> float:
    """Temperature in degC, NaN when the zone does not exist."""
    if self._path is None:
      self._path = self._find() or ""
    if not self._path:
      return float("nan")
    return read_number(self._path, scale=self.scale)


@dataclass
class ThermalConfig:
  cpu: list[ThermalZone]
  gpu: list[ThermalZone]
  dsp: ThermalZone
  memory: ThermalZone
  pmic: list[ThermalZone]
  intake: ThermalZone | None = None
  exhaust: ThermalZone | None = None
  gnss: ThermalZone | None = None
  bottom_soc: ThermalZone | None = None


def thermal_config(dev_type: str, root: str = THERMAL_ROOT) -> ThermalConfig:
  """Thermal zones of the SDM845 comma devices (see openpilot HardwareComma.get_thermal_config)."""
  z = lambda name: ThermalZone(name, root=root)  # noqa: E731
  extra = {}
  if dev_type == "mici":
    extra = {"intake": z("intake"), "exhaust": z("exhaust"), "gnss": z("gnss"), "bottom_soc": z("bottom_soc")}
  return ThermalConfig(
    cpu=[z(f"cpu{i}-silver-usr") for i in range(4)] + [z(f"cpu{i}-gold-usr") for i in range(4)],
    gpu=[z("gpu0-usr"), z("gpu1-usr")],
    dsp=z("compute-hvx-usr"),
    memory=z("ddr-usr"),
    pmic=[z("pm8998_tz"), z("pm8005_tz")],
    **extra,
  )
