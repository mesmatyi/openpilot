#!/usr/bin/env bash
# Build the openpilot components the ROS 2 drivers sit on and stage them in $DEST/openpilot:
#   bin/camerad          Qualcomm Spectra ISP driver (feeds comma_camera over VisionIPC)
#   python/panda         panda Python library + the matching signed firmware (comma_panda)
#   python/opendbc       opendbc Python package (safety model enums used by the panda library)
#   modem.py             openpilot's LTE modem manager (stdlib only)
# The scons tree in $SRC is kept: comma_camera links VisionIPC from it in build_workspace.sh.
set -euo pipefail
SRC="${1:?source dir}"
DEST="${2:?dest dir}/openpilot"
cd "$SRC"

# openpilot builds with the AGNOS Python venv, which carries its native dependencies
PY=/usr/local/venv/bin/python
need="import SCons, numpy, capnproto, json11, zeromq, ncurses, zstd, acados, ffmpeg"
if ! "$PY" -c "$need" 2>/dev/null; then
  echo "AGNOS venv lacks openpilot's build dependencies, creating one from uv.lock"
  python3 -m pip install --break-system-packages -q uv
  UV_PROJECT_ENVIRONMENT=/tmp/opvenv uv sync --frozen --no-install-project
  PY=/tmp/opvenv/bin/python
  "$PY" -c "$need"
fi
echo "PY=$PY" > /tmp/openpilot_python.env
export PATH="$(dirname "$PY"):$PATH"

mkdir -p /data/scons_cache  # openpilot's scons cache location on comma hardware (build stage only)
jobs="$(nproc)"
scons -j"$jobs" --minimal openpilot/system/camerad/camerad

fw=panda/board/obj/panda_h7.bin.signed
if ! scons -j"$jobs" --minimal "$fw"; then
  echo "WARNING: could not build the panda firmware; comma_panda will use whatever firmware the panda runs" >&2
fi

mkdir -p "$DEST/bin" "$DEST/python"
install -m 0755 openpilot/system/camerad/camerad "$DEST/bin/camerad"
if ldd "$DEST/bin/camerad" | grep -q "not found"; then
  ldd "$DEST/bin/camerad"
  echo "camerad has unresolved libraries" >&2
  exit 1
fi
install -m 0755 openpilot/common/hardware/comma/modem.py "$DEST/modem.py"

# Python packages, laid out like openpilot's PYTHONPATH so FW_PATH resolves to panda/board/obj
copy_py() { tar -C "$1" --exclude='.git' --exclude='__pycache__' --exclude='*.o' --exclude='tests' -cf - "$2" | tar -C "$DEST/python" -xf -; }
copy_py "$SRC" panda
copy_py "$SRC/opendbc_repo" opendbc
ls -l "$DEST/python/panda/board/obj/" 2>/dev/null || true
rm -rf /data/scons_cache
