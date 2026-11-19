# ROS 2 environment for interactive shells
if [ -n "$BASH_VERSION" ] && [ -f /opt/ros2os/setup.bash ]; then
  . /opt/ros2os/setup.bash
  # ros2 CLI tab completion
  command -v register-python-argcomplete3 >/dev/null 2>&1 && eval "$(register-python-argcomplete3 ros2)"
  command -v register-python-argcomplete3 >/dev/null 2>&1 && eval "$(register-python-argcomplete3 colcon)"
fi
