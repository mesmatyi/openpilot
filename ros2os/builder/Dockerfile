# syntax=docker/dockerfile:1.6
# ros2os root filesystem = the pinned AGNOS release (kernel modules, firmware, vendor
# libraries, NetworkManager, ssh, ...) + ROS 2 + openpilot's camerad + the comma ROS 2
# drivers + systemd services. Built by ros2os/build.sh with the repository root as context.
ARG BASE_IMAGE=ros2os-agnos:latest

# ---------- runtime: AGNOS + ROS 2 packages ----------
FROM ${BASE_IMAGE} AS runtime
ARG ROS_DISTRO=jazzy
ARG EXTRA_APT_PACKAGES=""
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY ros2os/builder/scripts/install_ros.sh /tmp/ros2os/install_ros.sh
RUN /tmp/ros2os/install_ros.sh runtime

# ---------- build: openpilot pieces + the colcon workspace ----------
FROM runtime AS build
ARG RUN_TESTS=1
RUN /tmp/ros2os/install_ros.sh build
COPY . /src
RUN /src/ros2os/builder/scripts/build_openpilot.sh /src /opt/ros2os
RUN /src/ros2os/builder/scripts/build_workspace.sh /src /opt/ros2os

# ---------- final image ----------
FROM runtime AS final
ARG ROS2OS_VERSION=dev
ARG GIT_COMMIT=unknown
ARG AGNOS_VERSION=unknown
COPY --from=build /opt/ros2os /opt/ros2os
COPY ros2os/builder/rootfs/ /
COPY ros2os/builder/scripts/finalize.sh /tmp/ros2os/finalize.sh
RUN ROS2OS_VERSION="${ROS2OS_VERSION}" GIT_COMMIT="${GIT_COMMIT}" AGNOS_VERSION="${AGNOS_VERSION}" \
    /tmp/ros2os/finalize.sh && rm -rf /tmp/ros2os
