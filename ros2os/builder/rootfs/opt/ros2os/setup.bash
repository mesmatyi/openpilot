# ros2os environment: ROS 2, the comma drivers and (if built) the user workspace in /data/ros2_ws.
# Sourced by login shells (/etc/profile.d/ros2os.sh) and the ros2os services.
set -a
. /etc/ros2os/ros2os.env
[ -f /data/ros2os/ros2os.env ] && . /data/ros2os/ros2os.env
set +a
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true
. "/opt/ros/$ROS_DISTRO/setup.bash"
. /opt/ros2os/install/local_setup.bash
# panda, opendbc and the drivers' Python dependencies (after ROS so the system numpy wins)
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}/opt/ros2os/openpilot/python"
if [ -f /data/ros2_ws/install/local_setup.bash ]; then
  . /data/ros2_ws/install/local_setup.bash
fi
