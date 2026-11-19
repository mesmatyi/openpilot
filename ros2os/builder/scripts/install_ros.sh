#!/usr/bin/env bash
# Install ROS 2 into the AGNOS-based image.
#   install_ros.sh runtime   ROS 2 base + everything the drivers and services need on the device
#   install_ros.sh build     compilers and dev packages for the build stage only
set -euo pipefail
mode="${1:?runtime|build}"
export DEBIAN_FRONTEND=noninteractive

. /etc/os-release
case "$VERSION_CODENAME" in
  noble) native=jazzy ;;
  jammy) native=humble ;;
  *) echo "unsupported AGNOS base: Ubuntu $VERSION_CODENAME" >&2; exit 1 ;;
esac
ROS_DISTRO="${ROS_DISTRO:-$native}"
if [[ "$ROS_DISTRO" != "$native" ]]; then
  echo "ROS 2 $ROS_DISTRO has no binaries for Ubuntu $VERSION_CODENAME (use $native)" >&2
  exit 1
fi
R="ros-$ROS_DISTRO"

apt_install() { apt-get install -y --no-install-recommends "$@"; }

if [[ "$mode" == "runtime" ]]; then
  # AGNOS may ship without apt sources; make sure the Ubuntu ports archive is configured
  if ! grep -rqsE "ports\.ubuntu\.com|archive\.ubuntu\.com" /etc/apt/sources.list /etc/apt/sources.list.d/; then
    cat > /etc/apt/sources.list.d/ubuntu.sources <<EOF
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports
Suites: $VERSION_CODENAME $VERSION_CODENAME-updates $VERSION_CODENAME-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
  fi
  apt-get update
  apt_install ca-certificates curl gnupg
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update

  apt_install \
    "$R-ros-base" \
    "$R-rmw-fastrtps-cpp" "$R-rmw-cyclonedds-cpp" \
    "$R-robot-state-publisher" "$R-xacro" "$R-tf2-ros" "$R-tf2-tools" \
    "$R-foxglove-bridge" "$R-foxglove-msgs" \
    "$R-rosbag2-storage-mcap" "$R-rosbag2-compression-zstd" \
    "$R-nmea-msgs" "$R-diagnostic-msgs" "$R-sensor-msgs-py" "$R-topic-tools" \
    python3-colcon-common-extensions python3-rosdep python3-numpy python3-serial python3-yaml python3-cffi \
    libturbojpeg libportaudio2 avahi-daemon libnss-mdns
  # optional extras: newer middleware, user-requested packages
  for pkg in "$R-rmw-zenoh-cpp" ${EXTRA_APT_PACKAGES:-}; do
    apt_install "$pkg" || echo "WARNING: could not install $pkg"
  done
  rm -rf /var/lib/apt/lists/*

elif [[ "$mode" == "build" ]]; then
  apt-get update
  apt_install build-essential cmake pkg-config python3-dev python3-pip python3-pytest \
    libturbojpeg0-dev "$R-ament-cmake-gtest" git
  # openpilot builds the panda firmware with the ARM embedded toolchain (AGNOS normally has it)
  command -v arm-none-eabi-gcc >/dev/null || apt_install gcc-arm-none-eabi libnewlib-arm-none-eabi
  command -v clang >/dev/null || apt_install clang
  rm -rf /var/lib/apt/lists/*
else
  echo "unknown mode $mode" >&2
  exit 1
fi
