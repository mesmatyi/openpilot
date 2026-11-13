import math
from collections import Counter
from collections.abc import Collection, Iterable, Iterator
from dataclasses import dataclass

import numpy as np
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros2_dashing import sensor_msgs__msg__NavSatStatus as NavSatStatus
from rosbags.typesys.stores.ros2_jazzy import (
  builtin_interfaces__msg__Duration as Duration,
  builtin_interfaces__msg__Time as Time,
  geometry_msgs__msg__Point as Point,
  geometry_msgs__msg__Pose as Pose,
  geometry_msgs__msg__PoseStamped as PoseStamped,
  geometry_msgs__msg__Quaternion as Quaternion,
  geometry_msgs__msg__Transform as Transform,
  geometry_msgs__msg__TransformStamped as TransformStamped,
  geometry_msgs__msg__Twist as Twist,
  geometry_msgs__msg__TwistWithCovariance as TwistWithCovariance,
  geometry_msgs__msg__TwistWithCovarianceStamped as TwistWithCovarianceStamped,
  geometry_msgs__msg__Vector3 as Vector3,
  geometry_msgs__msg__Vector3Stamped as Vector3Stamped,
  nav_msgs__msg__Path as PathMessage,
  sensor_msgs__msg__CameraInfo as CameraInfo,
  sensor_msgs__msg__CompressedImage as CompressedImage,
  sensor_msgs__msg__Imu as Imu,
  sensor_msgs__msg__NavSatFix as NavSatFix,
  sensor_msgs__msg__RegionOfInterest as RegionOfInterest,
  sensor_msgs__msg__Temperature as Temperature,
  std_msgs__msg__Bool as Bool,
  std_msgs__msg__ColorRGBA as ColorRGBA,
  std_msgs__msg__Float64 as Float64,
  std_msgs__msg__Header as Header,
  tf2_msgs__msg__TFMessage as TFMessage,
  visualization_msgs__msg__Marker as Marker,
  visualization_msgs__msg__MarkerArray as MarkerArray,
  visualization_msgs__msg__MeshFile as MeshFile,
)

from openpilot.common.transformations.camera import CameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import quat_from_rot, rot_from_euler

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)
DEVICE_TO_FLU = np.diag([1.0, -1.0, -1.0])
NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

Message = (
  CompressedImage | CameraInfo | TFMessage | Marker | MarkerArray | Imu | NavSatFix | Temperature |
  Vector3Stamped | TwistWithCovarianceStamped | Float64 | Bool | PathMessage
)
TimedMessage = tuple[int, str, Message]


@dataclass(frozen=True)
class Signal:
  source: str
  datatype: str
  topics: tuple[str, ...]


SIGNALS = {
  "narrow-road-camera": Signal("narrowRoadEncodeIdx", "sensor_msgs/msg/CompressedImage", ("/camera/narrowRoad/image_raw/compressed",)),
  "wide-road-camera": Signal("wideRoadEncodeIdx", "sensor_msgs/msg/CompressedImage", ("/camera/wideRoad/image_raw/compressed",)),
  "cabin-camera": Signal("cabinEncodeIdx", "sensor_msgs/msg/CompressedImage", ("/camera/cabin/image_raw/compressed",)),
  "gps-fix": Signal("gpsLocationExternal", "sensor_msgs/msg/NavSatFix", ("/gps/fix",)),
  "gps-velocity": Signal("gpsLocationExternal", "geometry_msgs/msg/Vector3Stamped", ("/gps/velocity",)),
  "raw-acceleration": Signal("accelerometer", "sensor_msgs/msg/Imu", ("/imu/acceleration",)),
  "raw-angular-velocity": Signal("gyroscope", "sensor_msgs/msg/Imu", ("/imu/angular_velocity",)),
  "temperature": Signal("temperatureSensor", "sensor_msgs/msg/Temperature", ("/imu/temperature",)),
  "estimated-imu": Signal("deviceMotion", "sensor_msgs/msg/Imu", ("/motion/imu",)),
  "device-velocity": Signal("deviceMotion", "geometry_msgs/msg/TwistWithCovarianceStamped", ("/motion/velocity",)),
  "camera-velocity": Signal("cameraOdometry", "geometry_msgs/msg/TwistWithCovarianceStamped", ("/motion/camera_velocity",)),
  "vehicle-speed": Signal("carState", "std_msgs/msg/Float64", ("/vehicle/speed",)),
  "steering-angle": Signal("carState", "std_msgs/msg/Float64", ("/vehicle/steering_angle",)),
  "steering-torque": Signal("carState", "std_msgs/msg/Float64", ("/vehicle/steering_torque",)),
  "vehicle-yaw-rate": Signal("carState", "std_msgs/msg/Float64", ("/vehicle/yaw_rate",)),
  "autonomy-flags": Signal("selfdriveState", "std_msgs/msg/Bool", ("/autonomy/enabled", "/autonomy/active", "/autonomy/engageable")),
  "predicted-path": Signal("modelV2", "nav_msgs/msg/Path", ("/model/path",)),
  "lane-lines": Signal("modelV2", "nav_msgs/msg/Path", tuple(f"/model/lane_lines/line_{i}" for i in range(4))),
  "road-edges": Signal("modelV2", "nav_msgs/msg/Path", tuple(f"/model/road_edges/edge_{i}" for i in range(2))),
  "lead-vehicles": Signal("modelV2", "visualization_msgs/msg/MarkerArray", ("/model/leads",)),
  "target-acceleration": Signal("longitudinalPlan", "std_msgs/msg/Float64", ("/planning/target_acceleration",)),
}


def header(timestamp: int, frame: str) -> Header:
  sec, nsec = divmod(timestamp, 1_000_000_000)
  if not 0 <= sec < 2**31:
    raise ValueError(f"Timestamp is outside the ROS 2 Time range: {timestamp}")
  return Header(Time(sec, nsec), frame)


def shift_timestamps(message: Message, offset: int) -> None:
  if isinstance(message, (Float64, Bool)):
    return
  if isinstance(message, TFMessage):
    headers = [transform.header for transform in message.transforms]
  elif isinstance(message, MarkerArray):
    headers = [item for marker in message.markers for item in (marker.header, marker.texture.header)]
  elif isinstance(message, Marker):
    headers = [message.header, message.texture.header]
  else:
    headers = [message.header]
  if isinstance(message, PathMessage):
    headers.extend(pose.header for pose in message.poses)
  for message_header in headers:
    stamp = message_header.stamp
    timestamp = stamp.sec * 1_000_000_000 + stamp.nanosec + offset
    message_header.stamp = header(timestamp, message_header.frame_id).stamp


def camera_info(timestamp: int, frame: str, camera: CameraConfig) -> CameraInfo:
  return CameraInfo(
    header(timestamp, frame), camera.height, camera.width, "plumb_bob", np.zeros(5),
    camera.intrinsics.ravel(), np.eye(3).ravel(), np.column_stack((camera.intrinsics, np.zeros(3))).ravel(),
    0, 0, RegionOfInterest(0, 0, 0, 0, False),
  )


def camera_transforms(data, timestamp: int, frames: Collection[str]) -> TFMessage:
  def rotation(values, name: str) -> np.ndarray:
    angles = np.asarray(tuple(values), dtype=np.float64)
    if angles.shape != (3,) or not np.all(np.isfinite(angles)):
      raise ValueError(f"Camera calibration {name} must contain three finite angles")
    return rot_from_euler(angles)

  def transform(child: str, matrix: np.ndarray) -> TransformStamped:
    w, x, y, z = quat_from_rot(matrix)
    return TransformStamped(
      header(timestamp, "device_flu"), child,
      Transform(Vector3(0.0, 0.0, 0.0), Quaternion(float(x), float(y), float(z), float(w))),
    )

  unknown = set(frames) - {"narrow_road_optical", "wide_road_optical"}
  if unknown:
    raise ValueError(f"Unsupported calibrated camera frames: {sorted(unknown)}")
  device_from_calib = rotation(data.rpyCalib, "rpyCalib")
  transforms = [transform("model_flu", DEVICE_TO_FLU @ device_from_calib @ DEVICE_TO_FLU)]
  for frame in sorted(frames):
    camera_from_device = np.eye(3)
    if frame == "wide_road_optical":
      camera_from_device = rotation(data.wideFromDeviceEuler, "wideFromDeviceEuler")
    # TF stores parent-from-child, the inverse of the UI's optical-from-device projection.
    transforms.append(transform(frame, DEVICE_TO_FLU @ camera_from_device.T @ view_frame_from_device_frame.T))
  return TFMessage(transforms)


def marker_message(timestamp: int, frame: str, namespace: str, marker_id: int, marker_type: int) -> Marker:
  return Marker(
    header=header(timestamp, frame), ns=namespace, id=marker_id, type=marker_type, action=Marker.ADD,
    pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    scale=Vector3(0.06, 0.0, 0.0), color=ColorRGBA(0.2, 1.0, 0.2, 1.0),
    lifetime=Duration(0, 250_000_000), frame_locked=False, points=[], colors=[],
    texture_resource="", texture=CompressedImage(header(timestamp, frame), "", np.empty(0, dtype=np.uint8)),
    uv_coordinates=[], text="", mesh_resource="", mesh_file=MeshFile("", np.empty(0, dtype=np.uint8)),
    mesh_use_embedded_materials=False,
  )


def path_marker(path: PathMessage) -> Marker:
  timestamp = path.header.stamp.sec * 1_000_000_000 + path.header.stamp.nanosec
  marker = marker_message(timestamp, path.header.frame_id, "model_geometry", 0, Marker.LINE_STRIP)
  marker.points = [pose.pose.position for pose in path.poses]
  return marker


def lead_markers(data, timestamp: int, skipped: Counter[str]) -> MarkerArray:
  clear = marker_message(timestamp, "model_flu", "model_lead", 0, Marker.SPHERE)
  clear.action = Marker.DELETEALL
  result = MarkerArray([clear])
  current = [lead for lead in data.leadsV3 if lead.probTime == 0.0]
  if len(current) > 1:
    raise ValueError("Expected at most one current model lead; future hypotheses are not additional vehicles")
  if not current:
    skipped["lead-vehicles: no current lead prediction"] += 1
    return result
  lead = current[0]
  if not math.isfinite(lead.prob) or not 0 <= lead.prob <= 1:
    raise ValueError("Lead probability must be finite and between zero and one")
  if lead.prob <= 0.5:
    skipped["lead-vehicles: no confident current lead"] += 1
    return result
  if not len(lead.t) or len(lead.x) != len(lead.t) or len(lead.y) != len(lead.t) or lead.t[0] != 0.0:
    raise ValueError("Current lead position arrays must match and start at time zero")
  distance, lateral = float(lead.x[0]), float(lead.y[0])
  if not all(math.isfinite(value) for value in (distance, lateral)):
    raise ValueError("Lead position must be finite")
  if distance <= 0:
    skipped["lead-vehicles: lead is not ahead"] += 1
    return result
  if len(data.laneLines) != 4:
    raise ValueError("Lead ground indicators require four model lane lines")
  heights = []
  for line in (data.laneLines[1], data.laneLines[2]):
    xs, zs = np.asarray(line.x), np.asarray(line.z)
    if len(xs) < 2 or xs.shape != zs.shape or not np.all(np.isfinite(xs)) or not np.all(np.isfinite(zs)) or np.any(np.diff(xs) <= 0):
      raise ValueError("Lead ground indicators require finite lane geometry with increasing distances")
    if not xs[0] <= distance <= xs[-1]:
      skipped["lead-vehicles: outside model road geometry"] += 1
      return result
    heights.append(float(np.interp(distance, xs, zs)))
  # Leads have no logged z or box dimensions; show an inferred road contact point, not a vehicle box.
  ground_z = sum(heights) / len(heights)
  marker = marker_message(timestamp, "model_flu", "model_lead", 0, Marker.SPHERE)
  marker.pose.position = Point(distance, -lateral, -ground_z)
  marker.scale = Vector3(0.7, 0.7, 0.7)
  marker.color = ColorRGBA(1.0, 0.2, 0.1, 1.0)
  label = marker_message(timestamp, "model_flu", "model_lead", 1, Marker.TEXT_VIEW_FACING)
  label.pose.position = Point(distance, -lateral, -ground_z + 1.0)
  label.scale = Vector3(0.0, 0.0, 0.6)
  label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
  label.text = f"Lead {distance:.1f} m ({lead.prob:.0%})"
  result.markers.extend((marker, label))
  return result


def vector(values: Iterable[float], *, device: bool = False) -> Vector3:
  values = tuple(float(value) for value in values)
  if len(values) != 3 or not all(math.isfinite(value) for value in values):
    raise ValueError("Expected three finite vector components")
  x, y, z = values
  return Vector3(x, -y if device else y, -z if device else z)


def xyz(measurement) -> tuple[float, float, float]:
  return measurement.x, measurement.y, measurement.z


def variance(stds: Iterable[float]) -> np.ndarray:
  values = np.array(tuple(stds), dtype=np.float64)
  if not np.all(np.isfinite(values)) or np.any(values < 0):
    raise ValueError("Expected finite, nonnegative standard deviations")
  return np.diag(values**2).ravel()


def unavailable() -> np.ndarray:
  covariance = np.zeros(9)
  covariance[0] = -1.0
  return covariance


def empty_imu(timestamp: int, frame: str) -> Imu:
  return Imu(
    header(timestamp, frame), Quaternion(0.0, 0.0, 0.0, 1.0), unavailable(), Vector3(0.0, 0.0, 0.0), unavailable(), Vector3(0.0, 0.0, 0.0), unavailable()
  )


def measurement_variance(measurement) -> np.ndarray:
  return variance((measurement.xStd, measurement.yStd, measurement.zStd))


def motion_twist(
  timestamp: int, linear: Iterable[float], angular: Iterable[float], linear_stds: Iterable[float], angular_stds: Iterable[float]
) -> TwistWithCovarianceStamped:
  linear_stds, angular_stds = tuple(linear_stds), tuple(angular_stds)
  if len(linear_stds) != 3 or len(angular_stds) != 3:
    raise ValueError("Velocity uncertainty must have three linear and three angular components")
  covariance = variance((*linear_stds, *angular_stds))
  return TwistWithCovarianceStamped(
    header(timestamp, "device_flu"),
    TwistWithCovariance(Twist(vector(linear, device=True), vector(angular, device=True)), covariance),
  )


def path_message(data, timestamp: int, *, future: bool) -> PathMessage:
  if not len(data.x) or len(data.x) != len(data.y) or len(data.x) != len(data.z):
    raise ValueError("Path coordinate arrays must be nonempty and have equal lengths")
  if future and len(data.t) != len(data.x):
    raise ValueError("Predicted path timestamps must match the coordinate arrays")
  poses = []
  for i, (x, y, z) in enumerate(zip(data.x, data.y, data.z, strict=True)):
    point = vector((x, y, z), device=True)
    offset = data.t[i] if future else 0.0
    if not math.isfinite(offset) or offset < 0:
      raise ValueError("Path time offsets must be finite and nonnegative")
    pose_header = header(timestamp + round(offset * 1_000_000_000), "model_flu")
    poses.append(PoseStamped(pose_header, Pose(Point(point.x, point.y, point.z), Quaternion(0.0, 0.0, 0.0, 1.0))))
  return PathMessage(header(timestamp, "model_flu"), poses)


def signal_messages(signal: str, data, timestamp: int, skipped: Counter[str]) -> Iterator[tuple[str, Message]]:
  topics = SIGNALS[signal].topics
  if signal in ("raw-acceleration", "raw-angular-velocity", "temperature"):
    if data.timestamp <= 0:
      raise ValueError(f"{signal}: missing sensor acquisition timestamp")
    frame = f"imu_{data.source}"
    if signal == "temperature":
      if not math.isfinite(data.temperature):
        raise ValueError("Non-finite sensor temperature")
      yield topics[0], Temperature(header(data.timestamp, frame), data.temperature, 0.0)
    else:
      message = empty_imu(data.timestamp, frame)
      if signal == "raw-acceleration":
        message.linear_acceleration = vector(data.acceleration.v)
        message.linear_acceleration_covariance = np.zeros(9)
      else:
        message.angular_velocity = vector(data.gyroUncalibrated.v)
        message.angular_velocity_covariance = np.zeros(9)
      yield topics[0], message
  elif signal == "gps-fix":
    # Accuracy radii are not a full ENU covariance; report the covariance as unknown.
    yield (
      topics[0],
      NavSatFix(
        header(timestamp, "gps"),
        NavSatStatus(0 if data.hasFix else -1, 0),
        data.latitude,
        data.longitude,
        data.altitude,
        np.zeros(9),
        NavSatFix.COVARIANCE_TYPE_UNKNOWN,
      ),
    )
  elif signal == "gps-velocity":
    if not data.hasFix:
      skipped[f"{signal}: no GNSS fix"] += 1
      return
    velocity = vector(data.vNED)
    yield topics[0], Vector3Stamped(header(timestamp, "gps_enu"), Vector3(velocity.y, velocity.x, -velocity.z))
  elif signal == "estimated-imu":
    if data.timestamp <= 0:
      raise ValueError("Estimated IMU is missing its state timestamp")
    message = empty_imu(data.timestamp, "device_flu")
    if data.orientationNED.valid:
      angles = vector(xyz(data.orientationNED))
      rotation = NED_TO_ENU @ rot_from_euler((angles.x, angles.y, angles.z)) @ DEVICE_TO_FLU
      w, x, y, z = quat_from_rot(rotation)
      message.orientation = Quaternion(float(x), float(y), float(z), float(w))
      # Euler-angle standard deviations do not directly describe quaternion-error covariance.
      message.orientation_covariance = np.zeros(9)
    if data.angularVelocityDevice.valid:
      message.angular_velocity = vector(xyz(data.angularVelocityDevice), device=True)
      message.angular_velocity_covariance = measurement_variance(data.angularVelocityDevice)
    if data.accelerationDevice.valid:
      message.linear_acceleration = vector(xyz(data.accelerationDevice), device=True)
      message.linear_acceleration_covariance = measurement_variance(data.accelerationDevice)
    yield topics[0], message
  elif signal == "device-velocity":
    if not data.velocityDevice.valid or not data.angularVelocityDevice.valid:
      skipped[f"{signal}: unavailable velocity estimate"] += 1
      return
    if data.timestamp <= 0:
      raise ValueError("Device velocity is missing its state timestamp")
    linear, angular = data.velocityDevice, data.angularVelocityDevice
    yield (
      topics[0],
      motion_twist(data.timestamp, xyz(linear), xyz(angular), (linear.xStd, linear.yStd, linear.zStd), (angular.xStd, angular.yStd, angular.zStd)),
    )
  elif signal == "camera-velocity":
    if data.timestampEof <= 0:
      raise ValueError("Camera velocity is missing its capture timestamp")
    yield topics[0], motion_twist(data.timestampEof, data.trans, data.rot, data.transStd, data.rotStd)
  elif signal in ("vehicle-speed", "steering-angle", "steering-torque", "vehicle-yaw-rate", "target-acceleration"):
    fields = {
      "vehicle-speed": "vEgo",
      "steering-angle": "steeringAngleDeg",
      "steering-torque": "steeringTorque",
      "vehicle-yaw-rate": "yawRate",
      "target-acceleration": "aTarget",
    }
    value = float(getattr(data, fields[signal]))
    if signal == "steering-angle":
      value = math.radians(value)
    if not math.isfinite(value):
      raise ValueError(f"{signal}: non-finite scalar value")
    yield topics[0], Float64(value)
  elif signal == "autonomy-flags":
    for topic, name in zip(topics, ("enabled", "active", "engageable"), strict=True):
      yield topic, Bool(bool(getattr(data, name)))
  elif signal == "lead-vehicles":
    if data.timestampEof <= 0:
      raise ValueError("Lead predictions are missing their capture timestamp")
    yield topics[0], lead_markers(data, data.timestampEof, skipped)
  elif signal in ("predicted-path", "lane-lines", "road-edges"):
    if data.timestampEof <= 0:
      raise ValueError("Model geometry is missing its capture timestamp")
    paths = [data.position] if signal == "predicted-path" else list(getattr(data, "laneLines" if signal == "lane-lines" else "roadEdges"))
    if len(paths) != len(topics):
      raise ValueError(f"{signal}: expected {len(topics)} paths, found {len(paths)}")
    for topic, path in zip(topics, paths, strict=True):
      yield topic, path_message(path, data.timestampEof, future=signal == "predicted-path")
  else:
    raise ValueError(f"Unsupported signal: {signal}")


def log_messages(events: list, selected: set[str], skipped: Counter[str]) -> Iterator[TimedMessage]:
  by_source: dict[str, list[str]] = {}
  for signal in sorted(selected):
    by_source.setdefault(SIGNALS[signal].source, []).append(signal)
  for event in events:
    for signal in by_source.get(event.which(), ()):
      if not event.valid:
        skipped[f"{signal}: invalid source event"] += 1
        continue
      for topic, message in signal_messages(signal, getattr(event, event.which()), event.logMonoTime, skipped):
        yield event.logMonoTime, topic, message
