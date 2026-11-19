#!/usr/bin/env bash
# Install the drivers' Python dependencies and build the colcon workspace into $DEST/install.
# Runs in the build stage after build_openpilot.sh.
set -euo pipefail
SRC="${1:?source dir}"
DEST="${2:?dest dir}"
PYDIR="$DEST/openpilot/python"
# shellcheck disable=SC1091
. /tmp/openpilot_python.env   # PY: the python that built openpilot (for its vendored library paths)

# ---- Python dependencies for the system python3 that ROS 2 uses ----
# (always /usr/bin/python3: AGNOS' own venv may come first in PATH and has an incompatible numpy)
mapfile -t deps < <(/usr/bin/python3 - "$SRC" <<'EOF'
import re, sys, tomllib
src = sys.argv[1]
skip = {"numpy", "opendbc", "panda", "scons", "setuptools", "cython", "pytest"}
reqs = {"libusb1": "libusb1", "spidev": "spidev", "sounddevice": "sounddevice"}  # panda over USB/SPI, microphone
for path in (f"{src}/panda/pyproject.toml", f"{src}/opendbc_repo/pyproject.toml"):
  try:
    project = tomllib.load(open(path, "rb")).get("project", {})
  except FileNotFoundError:
    continue
  for req in project.get("dependencies", []):
    name = re.split(r"[ <>=!~;\[@]", req, maxsplit=1)[0].strip().lower().replace("_", "-")
    if "@" in req or not name or name in skip:  # git references are vendored instead
      continue
    reqs[name] = req  # one requirement per package; the projects' own specifiers win
print("\n".join(reqs[k] for k in sorted(reqs)))
EOF
)
echo "installing Python dependencies into $PYDIR: ${deps[*]}"
/usr/bin/python3 -m pip install --break-system-packages --no-cache-dir --target "$PYDIR" "${deps[@]}"
# ROS 2 message modules are compiled against the system numpy; never shadow it
rm -rf "$PYDIR"/numpy "$PYDIR"/numpy-* "$PYDIR"/numpy.libs "$PYDIR"/bin
PYTHONPATH="$PYDIR" /usr/bin/python3 -c "
from panda import Panda, PandaDFU, McuType, FW_PATH
from opendbc.car.structs import CarParams
assert CarParams.SafetyModel.noOutput is not None
import usb1, spidev, serial, sounddevice
print('python dependencies ok, panda firmware dir:', FW_PATH)"

# ---- colcon workspace ----
# shellcheck disable=SC1090
set +u; . "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
libdirs="$("$PY" -c "import json11, zeromq, capnproto; print(';'.join(m.LIB_DIR for m in (json11, zeromq, capnproto)))")"
ws=/tmp/ws
rm -rf "$ws"
mkdir -p "$ws"
cp -r "$SRC/ros2os/ros2_ws/src" "$ws/src"
cd "$ws"
export PYTHONPATH="${PYTHONPATH:-}:$PYDIR"
colcon build --merge-install --install-base "$DEST/install" --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DOPENPILOT_DIR="$SRC" "-DOPENPILOT_LIB_DIRS=$libdirs"

if [[ ! -x "$DEST/install/lib/comma_camera/camera_node" ]]; then
  echo "comma_camera was built without VisionIPC (see the CMake warning above)" >&2
  exit 1
fi
if ldd "$DEST/install/lib/libcamera_component.so" | grep -q "not found"; then
  ldd "$DEST/install/lib/libcamera_component.so"
  exit 1
fi

# ---- tests ----
if [[ "${RUN_TESTS:-1}" == "1" ]]; then
  for pkg in comma_imu comma_gnss comma_panda comma_hardware comma_audio; do
    PYTHONPATH="src/comma_hal:src/$pkg:$PYTHONPATH" /usr/bin/python3 -m pytest -q "src/$pkg/test"
  done
  colcon build --packages-select comma_camera --build-base build-test --install-base install-test \
    --cmake-args -DBUILD_TESTING=ON -DOPENPILOT_DIR=""
  colcon test --packages-select comma_camera --build-base build-test --install-base install-test --event-handlers console_direct+
  colcon test-result --test-result-base build-test --verbose
fi
rm -rf "$ws"
