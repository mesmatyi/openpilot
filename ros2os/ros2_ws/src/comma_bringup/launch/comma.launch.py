"""Start every comma device driver.

  ros2 launch comma_bringup comma.launch.py [camera:=false] [foxglove:=false] [prefix:=front_]

Driver nodes respawn on failure. Parameters come from config/comma.yaml,
overridden by /data/ros2os/params.yaml when that file exists.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DRIVERS = {
  # launch arg: (package, executable, node name)
  "imu": ("comma_imu", "imu_node", "imu"),
  "gnss": ("comma_gnss", "gnss_node", "gnss"),
  "panda": ("comma_panda", "panda_node", "panda"),
  "hardware": ("comma_hardware", "hardware_node", "hardware"),
  "audio": ("comma_audio", "audio_node", "audio"),
  "camera": ("comma_camera", "camera_node", "camera"),
}


def package_available(name: str) -> bool:
  try:
    get_package_share_directory(name)
    return True
  except PackageNotFoundError:
    return False


def truthy(value: str) -> bool:
  return value.lower() in ("1", "true", "yes", "on")


def setup(context):
  share = get_package_share_directory("comma_bringup")
  params = [os.path.join(share, "config", "comma.yaml")]
  user_params = LaunchConfiguration("user_params").perform(context)
  actions = []
  if user_params and os.path.isfile(user_params):
    params.append(user_params)
    actions.append(LogInfo(msg=f"using parameter overrides from {user_params}"))
  prefix = LaunchConfiguration("prefix").perform(context)
  namespace = LaunchConfiguration("namespace").perform(context)

  # frame ids follow the URDF prefix
  frame_overrides = {
    "imu": {"frame_id": f"{prefix}imu_link"},
    "gnss": {"frame_id": f"{prefix}gps", "enu_frame_id": f"{prefix}gps_enu"},
    "panda": {"frame_id": f"{prefix}panda"},
    "audio": {"frame_id": f"{prefix}microphone"},
  }

  for arg, (pkg, exe, name) in DRIVERS.items():
    if not truthy(LaunchConfiguration(arg).perform(context)):
      continue
    if not package_available(pkg):
      actions.append(LogInfo(msg=f"[comma_bringup] {pkg} is not installed, skipping {name}"))
      continue
    actions.append(Node(
      package=pkg, executable=exe, name=name, namespace=namespace, output="screen",
      parameters=params + ([frame_overrides[arg]] if prefix and arg in frame_overrides else []),
      respawn=True, respawn_delay=2.0,
    ))

  if truthy(LaunchConfiguration("description").perform(context)):
    import xacro
    urdf = os.path.join(get_package_share_directory("comma_description"), "urdf", "comma_device.urdf.xacro")
    robot_description = xacro.process_file(urdf, mappings={"prefix": prefix}).toxml()
    actions.append(Node(
      package="robot_state_publisher", executable="robot_state_publisher", namespace=namespace, output="screen",
      parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}],
    ))

  if truthy(LaunchConfiguration("foxglove").perform(context)):
    if package_available("foxglove_bridge"):
      actions.append(Node(
        package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge", output="screen",
        parameters=params, respawn=True, respawn_delay=2.0,
      ))
    else:
      actions.append(LogInfo(msg="[comma_bringup] foxglove_bridge is not installed"))
  return actions


def generate_launch_description():
  args = [DeclareLaunchArgument(name, default_value="true", description=f"start the {name} driver") for name in DRIVERS]
  args += [
    DeclareLaunchArgument("description", default_value="true", description="publish the device TF tree"),
    DeclareLaunchArgument("foxglove", default_value="true", description="start foxglove_bridge (ws://<device>:8765)"),
    DeclareLaunchArgument("prefix", default_value="", description="TF frame prefix, e.g. 'front_'"),
    DeclareLaunchArgument("namespace", default_value="", description="namespace for all driver nodes"),
    DeclareLaunchArgument("user_params", default_value="/data/ros2os/params.yaml",
                          description="optional YAML with parameter overrides"),
  ]
  return LaunchDescription(args + [OpaqueFunction(function=setup)])
