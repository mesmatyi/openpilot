# ROS 2 telemetry

Enable **ROS 2 Telemetry** in Settings > Developer to publish selected openpilot Cereal services as `std_msgs/msg/String` JSON messages. Each service is published to `/openpilot/<service>` at up to 20 Hz. The payload contains `logMonoTime`, `valid`, and the service data. Lane lines and other model outputs are part of `modelV2`; camera image frames are not published.

ROS 2 Python (`rclpy`) and `std_msgs` must be installed and available to openpilot's Python process. ROS 2 network discovery and domain selection use the standard DDS configuration, including `ROS_DOMAIN_ID`. The stream includes vehicle and location data and is transmitted to peers reachable in that ROS 2 domain; disable the setting to stop it.
