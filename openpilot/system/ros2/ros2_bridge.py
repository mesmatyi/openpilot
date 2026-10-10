import json
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


SERVICES = (
  "accelerometer",
  "cameraOdometry",
  "carControl",
  "carOutput",
  "carState",
  "controlsState",
  "deviceMotion",
  "deviceState",
  "driverAssistance",
  "driverMonitoringState",
  "driverStateV2",
  "drivingModelData",
  "extrinsicsCalibration",
  "gpsLocation",
  "gpsLocationExternal",
  "gyroscope",
  "longitudinalPlan",
  "lateralManeuverPlan",
  "managerState",
  "modelV2",
  "onroadEvents",
  "pandaStates",
  "radarState",
  "radarTracks",
  "selfdriveState",
  "temperatureSensor",
  "vehicleParameters",
)

MAX_PUBLISH_RATE = 20


def main():
  params = Params()
  try:
    import rclpy
    from rclpy.qos import QoSProfile
    from std_msgs.msg import String
  except ImportError:
    cloudlog.exception("ROS 2 telemetry enabled, but ROS 2 Python packages are unavailable")
    params.put_bool("Ros2Enabled", False, block=True)
    return

  rclpy.init()
  node = rclpy.create_node("openpilot_telemetry")
  try:
    qos = QoSProfile(depth=1)
    publishers = {
      service: node.create_publisher(String, f"/openpilot/{service}", qos)
      for service in SERVICES
    }
    sm = messaging.SubMaster(list(SERVICES))
    last_publish = dict.fromkeys(SERVICES, 0.0)
    min_publish_interval = 1 / MAX_PUBLISH_RATE

    while rclpy.ok() and params.get_bool("Ros2Enabled"):
      sm.update(100)
      now = time.monotonic()
      for service, publisher in publishers.items():
        if not sm.updated[service] or now - last_publish[service] < min_publish_interval:
          continue

        message = String()
        message.data = json.dumps({
          "logMonoTime": sm.logMonoTime[service],
          "valid": sm.valid[service],
          "data": sm[service].to_dict(),
          }, separators=(",", ":"))
        publisher.publish(message)
        last_publish[service] = now

      rclpy.spin_once(node, timeout_sec=0)
  finally:
    node.destroy_node()
    rclpy.shutdown()
