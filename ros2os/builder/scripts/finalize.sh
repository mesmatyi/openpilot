#!/usr/bin/env bash
# Last step of the image build: wire ros2os into the AGNOS system.
set -euo pipefail
: "${ROS_DISTRO:?}" "${ROS2OS_VERSION:?}"

# ---- environment ----
sed -i "s/@ROS_DISTRO@/$ROS_DISTRO/g" /etc/ros2os/ros2os.env
cat > /etc/ros2os/build-info <<EOF
ROS2OS_VERSION=$ROS2OS_VERSION
ROS2OS_COMMIT=${GIT_COMMIT:-unknown}
AGNOS_VERSION=${AGNOS_VERSION:-unknown}
ROS_DISTRO=$ROS_DISTRO
BUILD_DATE=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
echo "$ROS2OS_VERSION" > /etc/ros2os/VERSION
chmod 0755 /usr/local/bin/ros2os-* /etc/NetworkManager/dispatcher.d/90-ros2os

# ---- stop AGNOS from launching openpilot or its setup UI ----
# AGNOS starts /usr/comma/comma.sh, which runs /data/continue.sh (openpilot) or the installer.
# Masking it leaves /data untouched, so booting back into the AGNOS slot restores openpilot.
masked=()
for dir in /etc/systemd/system /lib/systemd/system /usr/lib/systemd/system; do
  [[ -d "$dir" ]] || continue
  while IFS= read -r unit; do
    name="$(basename "$unit")"
    [[ "$name" == ros2os* ]] && continue
    systemctl mask "$name" && masked+=("$name")
  done < <(grep -lsE '/usr/comma/comma\.sh|launch_openpilot|continue\.sh' "$dir"/*.service 2>/dev/null || true)
done
if [[ -e /lib/systemd/system/comma.service || -e /etc/systemd/system/comma.service ]]; then
  systemctl mask comma.service && masked+=(comma.service)
fi
echo "masked AGNOS units: ${masked[*]:-none found}"

# ---- services ----
systemctl enable ros2os-hwinit.service ros2os-hostname.service ros2os-camerad.service ros2os.service \
  ros2os-modem.service ros2os-deleter.timer
systemctl enable avahi-daemon.service 2>/dev/null || true
# ros2os-record.service is installed but disabled: `sudo systemctl enable --now ros2os-record`

# ---- device permissions for the comma user ----
if getent group comma >/dev/null; then perm='GROUP="comma", MODE="0660"'; else perm='MODE="0666"'; fi
cat > /etc/udev/rules.d/90-ros2os.rules <<EOF
# devices used by the ros2os drivers (they run as the comma user)
SUBSYSTEM=="i2c-dev", $perm
SUBSYSTEM=="spidev", $perm
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", $perm
SUBSYSTEM=="tty", KERNEL=="ttyHS0", $perm
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", $perm
EOF
getent group audio >/dev/null && usermod -aG audio comma 2>/dev/null || true

# ---- ssh: also accept keys from /data/ros2os/authorized_keys (written by the installer) ----
cfg=/etc/ssh/sshd_config
if [[ -f "$cfg" ]]; then
  current="$(grep -hsE '^[[:space:]]*AuthorizedKeysFile' /etc/ssh/sshd_config.d/*.conf "$cfg" | head -1 \
             | sed -E 's/^[[:space:]]*AuthorizedKeysFile[[:space:]]+//')"
  [[ -n "$current" ]] || current=".ssh/authorized_keys .ssh/authorized_keys2"
  line="AuthorizedKeysFile $current /data/ros2os/authorized_keys"
  if grep -qE '^[[:space:]]*Include[[:space:]]+/etc/ssh/sshd_config.d/' "$cfg"; then
    mkdir -p /etc/ssh/sshd_config.d
    echo "$line" > /etc/ssh/sshd_config.d/00-ros2os.conf   # first value wins in sshd
  elif grep -qE '^[[:space:]]*AuthorizedKeysFile' "$cfg"; then
    sed -i -E "s|^[[:space:]]*AuthorizedKeysFile.*|$line|" "$cfg"
  else
    echo "$line" >> "$cfg"
  fi
fi

# ---- login banner ----
cat > /etc/motd <<EOF

  ros2os $ROS2OS_VERSION  -  ROS 2 $ROS_DISTRO on comma hardware (AGNOS ${AGNOS_VERSION:-?})

  ros2os-status          services, drivers and topics
  ros2 topic list        everything the device publishes
  /data/ros2os/          params.yaml, ros2os.env, launch.py overrides (persist across updates)
  /data/ros2_ws          your colcon workspace (sourced automatically when built)
  ros2os-rollback        boot back into the other slot (stock AGNOS / openpilot)

EOF

# ---- sanity checks ----
for f in /opt/ros2os/openpilot/bin/camerad /opt/ros2os/install/lib/comma_camera/camera_node; do
  [[ -x "$f" ]] || { echo "missing $f" >&2; exit 1; }
done
if ldd /opt/ros2os/openpilot/bin/camerad /opt/ros2os/install/lib/libcamera_component.so | grep "not found"; then
  echo "unresolved shared libraries" >&2
  exit 1
fi
bash -c 'set +u; . /opt/ros2os/setup.bash && ros2 pkg list' | grep -q '^comma_bringup$'

# ---- trim ----
find /usr/share/doc -type f ! -name copyright -delete 2>/dev/null || true
rm -rf /usr/share/man/* /var/cache/apt/archives/*.deb /root/.cache /tmp/* 2>/dev/null || true
