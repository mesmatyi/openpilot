import io
import math
import tempfile
import unittest
import warnings
from collections import Counter
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import av
import numpy as np
from PIL import Image
from mcap.reader import NonSeekingReader, make_reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from openpilot.cereal import log
from openpilot.common.transformations.camera import CameraConfig, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rotations_from_quats, rot_from_euler
from openpilot.tools.lib.route import RouteName, Segment
from openpilot.tools.mcap.convert import camera_configs, convert_route, main, video_messages
from openpilot.tools.mcap.ros2 import (
  CompressedImage, DEVICE_TO_FLU, Marker, SIGNALS, TYPESTORE, camera_info, camera_transforms, header, lead_markers,
  log_messages, path_marker, path_message, signal_messages,
)

ROUTE = "0123456789abcdef|2024-01-01--00-00-00"
MODULE = "openpilot.tools.mcap.convert"
UTC_TIME = 1_704_067_200_123_456_789
TIME_OFFSET = UTC_TIME - 50
CAMERA_CONFIG = DeviceCameraConfig(CameraConfig(64, 48, 80.0), CameraConfig(64, 48, 40.0), CameraConfig(64, 48, 40.0))


def event(name: str, timestamp: int = 1_000_000_000, *, valid: bool = True):
  message = log.Event.new_message(logMonoTime=timestamp, valid=valid)
  message.init(name)
  return message


def encode_event(frame: int, timestamp: int, segment: int = 0, name: str = "narrowRoadEncodeIdx"):
  message = event(name, timestamp + 100)
  index = getattr(message, name)
  index.type = "fullHEVC"
  index.segmentNum = segment
  index.segmentId = frame
  index.timestampSof = timestamp
  return message


def model_event(timestamp: int = 1_000):
  message = event("modelV2", timestamp + 200)
  model = message.modelV2
  model.timestampEof = timestamp
  model.position.x, model.position.y, model.position.z, model.position.t = [0, 10, 20, 40], [0] * 4, [0] * 4, [0, 1, 2, 3]
  for i, line in enumerate(model.init("laneLines", 4)):
    line.x, line.y, line.z = [0, 10, 20, 40], [-3.5 + i * 2] * 4, [1.5] * 4
  for i, lead in enumerate(model.init("leadsV3", 3)):
    lead.prob, lead.probTime = 0.9, i * 2
    lead.t, lead.x, lead.y = [0, 2], [10, 12], [2, 2]
  return message


def calibration_event(timestamp: int = 90):
  message = event("extrinsicsCalibration", timestamp)
  message.extrinsicsCalibration.calStatus = "calibrated"
  message.extrinsicsCalibration.rpyCalib = [0.03, -0.05, 0.08]
  message.extrinsicsCalibration.wideFromDeviceEuler = [-0.02, 0.04, -0.06]
  message.extrinsicsCalibration.height = [1.5]
  return message


class TestCameraProjection(unittest.TestCase):
  def test_tf_chain_reprojects_like_openpilot_for_both_cameras(self):
    calibration = calibration_event().extrinsicsCalibration
    result = camera_transforms(calibration, 1_000, ["narrow_road_optical", "wide_road_optical"])
    rotations = {}
    for transform in result.transforms:
      self.assertEqual(transform.header.frame_id, "device_flu")
      self.assertEqual(transform.header.stamp.nanosec, 1_000)
      self.assertEqual((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z), (0, 0, 0))
      q = transform.transform.rotation
      quaternion = np.array([q.w, q.x, q.y, q.z])
      self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0)
      rotations[transform.child_frame_id] = rotations_from_quats(quaternion)
    device_from_model = rot_from_euler(calibration.rpyCalib)
    points = np.array([[10.0, -1.0, 1.5], [30.0, 1.5, 1.4], [80.0, -3.0, 2.0]]).T
    for frame, focal in (("narrow_road_optical", 600.0), ("wide_road_optical", 300.0)):
      with self.subTest(frame=frame):
        optical_from_model_flu = rotations[frame].T @ rotations["model_flu"]
        actual = optical_from_model_flu @ DEVICE_TO_FLU @ points
        camera_from_device = rot_from_euler(calibration.wideFromDeviceEuler) if frame == "wide_road_optical" else np.eye(3)
        expected = view_frame_from_device_frame @ camera_from_device @ device_from_model @ points
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        k = CameraConfig(640, 480, focal).intrinsics
        actual_pixels, expected_pixels = k @ actual, k @ expected
        np.testing.assert_allclose(actual_pixels[:2] / actual_pixels[2], expected_pixels[:2] / expected_pixels[2], atol=1e-9)
        self.assertTrue(np.all(actual[2] > 0))

  def test_camera_info_uses_full_resolution_pinhole_intrinsics(self):
    config = CameraConfig(1344, 760, 1141.5)
    info = camera_info(123, "narrow_road_optical", config)
    self.assertEqual((info.width, info.height), config.size)
    self.assertEqual(info.header.frame_id, "narrow_road_optical")
    self.assertEqual(info.header.stamp.nanosec, 123)
    np.testing.assert_allclose(info.k.reshape(3, 3), config.intrinsics)
    np.testing.assert_allclose(info.p.reshape(3, 4), np.column_stack((config.intrinsics, np.zeros(3))))
    np.testing.assert_array_equal(info.r, np.eye(3).ravel())
    np.testing.assert_array_equal(info.d, np.zeros(5))

  def test_missing_or_nonfinite_calibration_is_not_invented(self):
    calibration = calibration_event().extrinsicsCalibration
    calibration.rpyCalib = [0, float("nan"), 0]
    with self.assertRaisesRegex(ValueError, "three finite"):
      camera_transforms(calibration, 100, ["narrow_road_optical"])
    calibration.rpyCalib = [0, 0, 0]
    calibration.wideFromDeviceEuler = []
    self.assertEqual(len(camera_transforms(calibration, 100, ["narrow_road_optical"]).transforms), 2)
    with self.assertRaisesRegex(ValueError, "wideFromDeviceEuler"):
      camera_transforms(calibration, 100, ["wide_road_optical"])

  def test_hardware_metadata_is_required(self):
    device = event("deviceState")
    device.deviceState.deviceType = "mici"
    camera = event("narrowRoadCameraState")
    camera.narrowRoadCameraState.sensor = "os04c10"
    self.assertEqual(camera_configs([device, camera], ["narrow-road-camera"])["narrow-road-camera"].size, (1344, 760))
    with self.assertRaisesRegex(ValueError, "deviceState"):
      camera_configs([camera], ["narrow-road-camera"])
    camera.valid = False
    with self.assertRaisesRegex(ValueError, "sensor"):
      camera_configs([device, camera], ["narrow-road-camera"])
    camera.valid = True
    device.deviceState.deviceType = "unknown"
    with self.assertRaisesRegex(ValueError, "Unsupported camera hardware"):
      camera_configs([device, camera], ["narrow-road-camera"])


class TestMappings(unittest.TestCase):
  def mapped(self, signal: str, message):
    result = []
    for timestamp, topic, native in log_messages([message], {signal}, Counter()):
      definition, _ = TYPESTORE.generate_msgdef(native.__msgtype__, ros_version=2)
      store = get_typestore(Stores.EMPTY)
      store.register(get_types_from_msg(definition, native.__msgtype__))
      decoded = store.deserialize_cdr(TYPESTORE.serialize_cdr(native, native.__msgtype__), native.__msgtype__)
      result.append((timestamp, topic, decoded))
    return result

  def test_ros_topic_names(self):
    for signal in SIGNALS.values():
      for topic in signal.topics:
        self.assertRegex(topic, r"^/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$")

  def test_raw_imu_axes_and_unknown_components(self):
    message = event("accelerometer")
    data = message.accelerometer
    data.timestamp = 123_456_789
    data.source = "lsm6ds3"
    data.init("acceleration").v = [9.8, 2.0, -3.0]
    _, topic, result = self.mapped("raw-acceleration", message)[0]
    self.assertEqual(topic, "/imu/acceleration")
    self.assertEqual(result.header.frame_id, "imu_lsm6ds3")
    self.assertEqual(result.header.stamp.nanosec, 123_456_789)
    np.testing.assert_allclose([result.linear_acceleration.x, result.linear_acceleration.y, result.linear_acceleration.z], [9.8, 2, -3])
    self.assertEqual(result.orientation_covariance[0], -1)
    self.assertEqual(result.angular_velocity_covariance[0], -1)
    self.assertEqual(result.linear_acceleration_covariance[0], 0)

  def test_raw_gyroscope_and_temperature(self):
    message = event("gyroscope")
    message.gyroscope.timestamp = 100
    message.gyroscope.init("gyroUncalibrated").v = [1.0, 2.0, 3.0]
    result = self.mapped("raw-angular-velocity", message)[0][2]
    self.assertEqual(result.angular_velocity.z, 3.0)
    self.assertEqual(result.linear_acceleration_covariance[0], -1)
    temperature = event("temperatureSensor")
    temperature.temperatureSensor.timestamp = 200
    temperature.temperatureSensor.temperature = 25.0
    self.assertEqual(self.mapped("temperature", temperature)[0][2].temperature, 25.0)

  def test_gps_and_ned_to_enu_velocity(self):
    message = event("gpsLocationExternal")
    data = message.gpsLocationExternal
    data.hasFix = True
    data.latitude, data.longitude, data.altitude = 47.0, 19.0, 123.0
    data.vNED = [1.0, 2.0, 3.0]
    fix = self.mapped("gps-fix", message)[0][2]
    self.assertEqual((fix.latitude, fix.longitude, fix.altitude), (47.0, 19.0, 123.0))
    self.assertEqual(fix.status.status, 0)
    self.assertEqual(fix.position_covariance_type, 0)
    velocity = self.mapped("gps-velocity", message)[0][2]
    self.assertEqual(velocity.__msgtype__, "geometry_msgs/msg/Vector3Stamped")
    self.assertEqual((velocity.vector.x, velocity.vector.y, velocity.vector.z), (2.0, 1.0, -3.0))
    data.hasFix = False
    self.assertEqual(self.mapped("gps-fix", message)[0][2].status.status, -1)
    skipped = Counter()
    self.assertEqual(list(signal_messages("gps-velocity", data, message.logMonoTime, skipped)), [])
    self.assertEqual(skipped["gps-velocity: no GNSS fix"], 1)

  def make_motion(self):
    message = event("deviceMotion")
    message.deviceMotion.timestamp = 200
    for name in ("orientationNED", "velocityDevice", "angularVelocityDevice", "accelerationDevice"):
      value = getattr(message.deviceMotion, name)
      value.valid = True
      value.xStd, value.yStd, value.zStd = 1.0, 2.0, 3.0
    message.deviceMotion.angularVelocityDevice.x = 1.0
    message.deviceMotion.angularVelocityDevice.y = 2.0
    message.deviceMotion.angularVelocityDevice.z = 3.0
    message.deviceMotion.velocityDevice.x = 4.0
    message.deviceMotion.velocityDevice.y = 5.0
    message.deviceMotion.velocityDevice.z = 6.0
    return message

  def test_estimated_orientation_and_flu_axes(self):
    message = self.make_motion()
    result = self.mapped("estimated-imu", message)[0][2]
    self.assertAlmostEqual(result.orientation.z, math.sqrt(0.5))
    self.assertAlmostEqual(result.orientation.w, math.sqrt(0.5))
    self.assertEqual((result.angular_velocity.x, result.angular_velocity.y, result.angular_velocity.z), (1.0, -2.0, -3.0))
    np.testing.assert_array_equal(np.diag(result.angular_velocity_covariance.reshape(3, 3)), [1, 4, 9])
    message.deviceMotion.orientationNED.valid = False
    self.assertEqual(self.mapped("estimated-imu", message)[0][2].orientation_covariance[0], -1)

  def test_device_velocity_covariance(self):
    message = self.make_motion()
    result = self.mapped("device-velocity", message)[0][2]
    self.assertEqual((result.twist.twist.linear.x, result.twist.twist.linear.y, result.twist.twist.linear.z), (4.0, -5.0, -6.0))
    np.testing.assert_array_equal(np.diag(result.twist.covariance.reshape(6, 6)), [1, 4, 9, 1, 4, 9])
    message.deviceMotion.velocityDevice.valid = False
    skipped = Counter()
    self.assertEqual(list(log_messages([message], {"device-velocity"}, skipped)), [])
    self.assertEqual(sum(skipped.values()), 1)

  def test_camera_velocity(self):
    message = event("cameraOdometry")
    data = message.cameraOdometry
    data.timestampEof = 100
    data.trans, data.rot = [1, 2, 3], [4, 5, 6]
    data.transStd, data.rotStd = [1, 2, 3], [4, 5, 6]
    result = self.mapped("camera-velocity", message)[0][2]
    self.assertEqual(result.twist.twist.linear.y, -2)
    self.assertEqual(result.twist.twist.angular.z, -6)
    np.testing.assert_array_equal(np.diag(result.twist.covariance.reshape(6, 6)), [1, 4, 9, 16, 25, 36])

  def test_vehicle_values_and_autonomy_flags(self):
    message = event("carState")
    message.carState.vEgo = 12.0
    message.carState.steeringAngleDeg = 90.0
    message.carState.steeringTorque = 123.0
    message.carState.yawRate = 0.5
    for signal, value in (("vehicle-speed", 12.0), ("steering-angle", math.pi / 2), ("steering-torque", 123.0), ("vehicle-yaw-rate", 0.5)):
      self.assertAlmostEqual(self.mapped(signal, message)[0][2].data, value)
    autonomy = event("selfdriveState")
    autonomy.selfdriveState.enabled = True
    self.assertEqual([result.data for _, _, result in self.mapped("autonomy-flags", autonomy)], [True, False, False])
    plan = event("longitudinalPlan")
    plan.longitudinalPlan.aTarget = -2.0
    self.assertEqual(self.mapped("target-acceleration", plan)[0][2].data, -2.0)

  def test_paths_and_future_timestamps(self):
    message = event("modelV2")
    model = message.modelV2
    model.timestampEof = 100
    model.position.x, model.position.y, model.position.z, model.position.t = [1, 2], [3, 4], [5, 6], [0, 1]
    for name, count in (("laneLines", 4), ("roadEdges", 2)):
      for line in model.init(name, count):
        line.x, line.y, line.z = [1, 2], [3, 4], [5, 6]
    path = self.mapped("predicted-path", message)[0][2]
    self.assertEqual(path.header.frame_id, "model_flu")
    self.assertEqual((path.poses[0].pose.position.x, path.poses[0].pose.position.y, path.poses[0].pose.position.z), (1, -3, -5))
    self.assertEqual(path.poses[1].header.stamp.sec, 1)
    self.assertEqual(path.poses[1].header.stamp.nanosec, 100)
    self.assertEqual(path.poses[0].pose.orientation.w, 1)
    self.assertEqual(len(self.mapped("lane-lines", message)), 4)
    self.assertEqual(len(self.mapped("road-edges", message)), 2)
    model.position.t = []
    with self.assertRaisesRegex(ValueError, "timestamps"):
      path_message(model.position, 100, future=True)

  def test_path_marker_preserves_geometry_and_capture_time(self):
    model = model_event().modelV2
    path = path_message(model.laneLines[0], model.timestampEof, future=False)
    marker = path_marker(path)
    self.assertEqual(marker.type, Marker.LINE_STRIP)
    self.assertEqual(marker.header.frame_id, "model_flu")
    self.assertEqual(marker.header.stamp.nanosec, model.timestampEof)
    self.assertEqual(
      [(point.x, point.y, point.z) for point in marker.points],
      [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in path.poses],
    )
    self.assertEqual(marker.pose.orientation.w, 1.0)

  def test_only_current_lead_is_shown_as_a_ground_indicator(self):
    result = self.mapped("lead-vehicles", model_event())[0][2]
    self.assertEqual(len(result.markers), 3)
    self.assertEqual(result.markers[0].action, Marker.DELETEALL)
    marker, label = result.markers[1:]
    self.assertEqual(marker.type, Marker.SPHERE)
    self.assertEqual(marker.header.frame_id, "model_flu")
    self.assertEqual((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z), (10, -2, -1.5))
    self.assertEqual(label.type, Marker.TEXT_VIEW_FACING)
    self.assertIn("10.0 m", label.text)
    self.assertEqual(marker.lifetime.nanosec, 250_000_000)

  def test_lead_disappearance_clears_old_markers(self):
    model = model_event().modelV2
    model.leadsV3[0].prob = 0.5
    skipped = Counter()
    result = lead_markers(model, model.timestampEof, skipped)
    self.assertEqual([marker.action for marker in result.markers], [Marker.DELETEALL])
    self.assertEqual(skipped["lead-vehicles: no confident current lead"], 1)
    model.leadsV3[0].probTime = 6.0
    result = lead_markers(model, model.timestampEof, skipped)
    self.assertEqual(len(result.markers), 1)
    self.assertEqual(skipped["lead-vehicles: no current lead prediction"], 1)

  def test_lead_geometry_is_validated_and_not_extrapolated(self):
    model = model_event().modelV2
    model.leadsV3[0].prob = float("nan")
    with self.assertRaisesRegex(ValueError, "probability"):
      lead_markers(model, 100, Counter())
    model.leadsV3[0].prob = 0.9
    model.leadsV3[0].x = [50, 60]
    skipped = Counter()
    self.assertEqual(len(lead_markers(model, 100, skipped).markers), 1)
    self.assertEqual(skipped["lead-vehicles: outside model road geometry"], 1)
    model.leadsV3[0].x = [10, 12]
    model.laneLines[1].x = [0, 10, 10, 40]
    with self.assertRaisesRegex(ValueError, "increasing distances"):
      lead_markers(model, 100, Counter())

  def test_invalid_event_is_counted_not_exported(self):
    message = event("carState", valid=False)
    skipped = Counter()
    self.assertEqual(list(log_messages([message], {"vehicle-speed"}, skipped)), [])
    self.assertEqual(skipped["vehicle-speed: invalid source event"], 1)

  def test_invalid_inputs_fail(self):
    with self.assertRaises(ValueError):
      header(-1, "test")
    with self.assertRaises(ValueError):
      header(2**31 * 1_000_000_000, "test")
    message = event("accelerometer")
    with self.assertRaisesRegex(ValueError, "acquisition timestamp"):
      self.mapped("raw-acceleration", message)
    message.accelerometer.timestamp = 100
    message.accelerometer.init("acceleration").v = [1, 2]
    with self.assertRaisesRegex(ValueError, "three finite"):
      self.mapped("raw-acceleration", message)


class TestConversion(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.root = Path(self.directory.name)
    self.output = self.root / "route.mcap"
    first, second = event("carState", 200), event("carState", 100)
    first.carState.vEgo, second.carState.vEgo = 2.0, 1.0
    self.segment = self.make_segment(0, [first, second])
    route_patch = patch(f"{MODULE}.Route")
    self.route = route_patch.start().return_value
    self.addCleanup(route_patch.stop)
    self.route.name = RouteName(ROUTE)
    self.route.segments = [self.segment]
    self.route.max_seg_number = 0

  def make_segment(self, number: int, events: list, *, with_clock: bool = True) -> Segment:
    if with_clock:
      clock = event("clocks", 50)
      clock.clocks.wallTimeNanos = UTC_TIME
      events = [clock, *events]
    path = self.root / f"rlog{number}"
    path.write_bytes(b"".join(message.to_bytes() for message in events))
    return Segment(f"{ROUTE}--{number}", str(path), None, None, None, None, None)

  def read_messages(self):
    with self.output.open("rb") as stream:
      reader = make_reader(stream)
      self.assertEqual(reader.get_header().profile, "ros2")
      summary = reader.get_summary()
      metadata = {item.name: item.metadata for item in reader.iter_metadata()}
    # Decode from the embedded definitions in a fresh store, not the writer's cached types.
    store = get_typestore(Stores.EMPTY)
    for schema in summary.schemas.values():
      self.assertEqual(schema.encoding, "ros2msg")
      store.register(get_types_from_msg(schema.data.decode(), schema.name))
    with self.output.open("rb") as stream:
      messages = [
        (channel, record, store.deserialize_cdr(record.data, schema.name))
        for schema, channel, record in NonSeekingReader(stream, validate_crcs=True).iter_messages(log_time_order=False)
      ]
    return messages, summary, metadata

  def make_video(self, frames: list[np.ndarray]) -> Path:
    video = self.root / "video.hevc"
    with av.open(str(video), mode="w", format="hevc") as output:
      stream = output.add_stream("libx265", rate=20)
      stream.width, stream.height, stream.pix_fmt = frames[0].shape[1], frames[0].shape[0], "yuv420p"
      stream.codec_context.thread_count = 1
      stream.options = {"x265-params": "log-level=error:pools=none:frame-threads=1:bframes=0"}
      for array in frames:
        for packet in stream.encode(av.VideoFrame.from_ndarray(array, format="rgb24")):
          output.mux(packet)
      for packet in stream.encode():
        output.mux(packet)
    return video

  def make_overlay_segment(self, *, with_calibration: bool = True) -> Segment:
    device = event("deviceState", 70)
    device.deviceState.deviceType = "mici"
    events = [device, model_event(1_000), model_event(1_050)]
    events[-1].modelV2.leadsV3[0].prob = 0.0
    if with_calibration:
      events.append(calibration_event())
    for camera in ("narrowRoad", "wideRoad"):
      state = event(f"{camera}CameraState", 80)
      getattr(state, state.which()).sensor = "os04c10"
      events.append(state)
      events.extend(encode_event(i, 1_000 + i * 50, name=f"{camera}EncodeIdx") for i in range(2))
    segment = self.make_segment(0, events)
    video = self.make_video([np.full((48, 64, 3), 128, dtype=np.uint8)] * 2)
    segment.camera_path = segment.ecamera_path = str(video)
    self.route.segments = [segment]
    return segment

  def test_both_camera_overlays_round_trip_with_tf_and_leads(self):
    self.make_overlay_segment()
    topics = ["narrow-road-camera", "wide-road-camera", "lane-lines", "lead-vehicles"]
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}):
      counts = convert_route(ROUTE, self.output, topics=topics, camera_overlays=True)
    messages, _, metadata = self.read_messages()
    self.assertEqual(metadata["openpilot"]["camera_overlays"], "true")
    self.assertEqual(metadata["openpilot"]["camera_calibration_policy"], "calibrated_only")
    self.assertEqual(metadata["camera_calibration_counts"], {"calibrated": "1"})
    self.assertEqual(counts["/tf"], 1)
    transform_message = next(message for channel, _, message in messages if channel.topic == "/tf")
    self.assertEqual(
      {transform.child_frame_id for transform in transform_message.transforms},
      {"model_flu", "narrow_road_optical", "wide_road_optical"},
    )
    for transform in transform_message.transforms:
      self.assertEqual(transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec, TIME_OFFSET + 90)
    for camera, config in (("narrowRoad", CAMERA_CONFIG.narrow_road), ("wideRoad", CAMERA_CONFIG.wide_road)):
      self.assertEqual(counts[f"/camera/{camera}/camera_info"], 2)
      images = [(record, message) for channel, record, message in messages if channel.topic == f"/camera/{camera}/image_raw/compressed"]
      infos = [(record, message) for channel, record, message in messages if channel.topic == f"/camera/{camera}/camera_info"]
      for (image_record, image), (info_record, info) in zip(images, infos, strict=True):
        self.assertEqual(image_record.log_time, info_record.log_time)
        self.assertEqual(info.header, image.header)
        self.assertEqual(info.header.stamp.sec * 1_000_000_000 + info.header.stamp.nanosec, info_record.log_time)
        self.assertEqual((info.width, info.height), (64, 48))
        np.testing.assert_allclose(info.k, config.intrinsics.ravel())
    for i in range(4):
      self.assertEqual(counts[f"/model/lane_lines/line_{i}/marker"], 2)
    leads = [(record, message) for channel, record, message in messages if channel.topic == "/model/leads"]
    self.assertEqual([len(message.markers) for _, message in leads], [3, 1])
    for index, (record, message) in enumerate(leads):
      self.assertEqual(record.log_time, TIME_OFFSET + 1_200 + index * 50)
      for marker in message.markers:
        self.assertEqual(marker.header.stamp.sec * 1_000_000_000 + marker.header.stamp.nanosec, TIME_OFFSET + 1_000 + index * 50)
        self.assertEqual(marker.lifetime.nanosec, 250_000_000)
    self.assertEqual(metadata["skipped_samples"]["lead-vehicles: no confident current lead"], "1")

  def test_missing_overlay_calibration_fails_atomically(self):
    self.make_overlay_segment(with_calibration=False)
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}):
      with self.assertRaisesRegex(ValueError, "/tf"):
        convert_route(ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True)
    self.assertFalse(self.output.exists())

  def test_uncalibrated_overlays_require_explicit_opt_in(self):
    self.make_overlay_segment(with_calibration=False)
    uncalibrated = calibration_event()
    uncalibrated.extrinsicsCalibration.calStatus = "uncalibrated"
    with (self.root / "rlog0").open("ab") as stream:
      stream.write(uncalibrated.to_bytes())
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}):
      with self.assertRaisesRegex(ValueError, "/tf"):
        convert_route(ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True)
      self.assertFalse(self.output.exists())
      with patch(f"{MODULE}.cloudlog.warning") as warning:
        counts = convert_route(
          ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True, allow_uncalibrated_overlays=True,
        )
    self.assertEqual(counts["/tf"], 1)
    warning.assert_called_once()
    self.assertIn("APPROXIMATE", warning.call_args.args[0])
    self.assertIn("uncalibrated", warning.call_args.args[0])
    messages, _, metadata = self.read_messages()
    self.assertEqual(metadata["openpilot"]["camera_calibration_policy"], "allow_uncalibrated")
    self.assertIn("APPROXIMATE", metadata["openpilot"]["camera_projection"])
    self.assertEqual(metadata["camera_calibration_counts"], {"uncalibrated": "1"})
    actual = next(message for channel, _, message in messages if channel.topic == "/tf")
    expected = camera_transforms(uncalibrated.extrinsicsCalibration, 90, ["narrow_road_optical"])
    for transform, reference in zip(actual.transforms, expected.transforms, strict=True):
      q, r = transform.transform.rotation, reference.transform.rotation
      np.testing.assert_allclose([q.w, q.x, q.y, q.z], [r.w, r.x, r.y, r.z], atol=1e-12)

  def test_uncalibrated_opt_in_still_excludes_invalid_and_recalibrating_updates(self):
    self.make_overlay_segment()
    uncalibrated = calibration_event(95)
    uncalibrated.extrinsicsCalibration.calStatus = "uncalibrated"
    invalid_event = calibration_event(85)
    invalid_event.valid = False
    invalid_event.extrinsicsCalibration.calStatus = "uncalibrated"
    invalid_status = calibration_event(86)
    invalid_status.extrinsicsCalibration.calStatus = "invalid"
    recalibrating = calibration_event(87)
    recalibrating.extrinsicsCalibration.calStatus = "recalibrating"
    with (self.root / "rlog0").open("ab") as stream:
      stream.write(b"".join(item.to_bytes() for item in (uncalibrated, invalid_event, invalid_status, recalibrating)))
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}), patch(f"{MODULE}.cloudlog.warning") as warning:
      counts = convert_route(
        ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True, allow_uncalibrated_overlays=True,
      )
    self.assertEqual(counts["/tf"], 2)
    self.assertEqual(warning.call_count, 2)
    _, _, metadata = self.read_messages()
    self.assertEqual(metadata["camera_calibration_counts"], {"calibrated": "1", "uncalibrated": "1"})
    self.assertEqual(metadata["skipped_samples"]["camera-overlays: unavailable calibration"], "3")

  def test_uncalibrated_opt_in_does_not_invent_missing_rotations(self):
    self.make_overlay_segment(with_calibration=False)
    uncalibrated = calibration_event()
    uncalibrated.extrinsicsCalibration.calStatus = "uncalibrated"
    uncalibrated.extrinsicsCalibration.rpyCalib = []
    with (self.root / "rlog0").open("ab") as stream:
      stream.write(uncalibrated.to_bytes())
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}):
      with self.assertRaisesRegex(ValueError, "three finite"):
        convert_route(
          ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True, allow_uncalibrated_overlays=True,
        )
    self.assertFalse(self.output.exists())

  def test_uncalibrated_opt_in_requires_camera_overlays(self):
    with self.assertRaisesRegex(ValueError, "requires --camera-overlays"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"], allow_uncalibrated_overlays=True)

  def test_overlay_resolution_mismatch_fails_atomically(self):
    self.make_overlay_segment()
    with self.assertRaisesRegex(ValueError, "does not match camera calibration"):
      convert_route(ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True)
    self.assertFalse(self.output.exists())

  def test_overlay_requires_a_road_camera(self):
    with self.assertRaisesRegex(ValueError, "selected road camera"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"], camera_overlays=True)

  def test_unavailable_calibration_is_reported(self):
    self.make_overlay_segment()
    bad = calibration_event(85)
    bad.valid = False
    uncalibrated = calibration_event(86)
    uncalibrated.extrinsicsCalibration.calStatus = "uncalibrated"
    with (self.root / "rlog0").open("ab") as stream:
      stream.write(bad.to_bytes() + uncalibrated.to_bytes())
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}), patch(f"{MODULE}.cloudlog.warning") as warning:
      counts = convert_route(ROUTE, self.output, topics=["narrow-road-camera"], camera_overlays=True)
    self.assertEqual(counts["/tf"], 1)
    self.assertEqual(warning.call_count, 1)
    _, _, metadata = self.read_messages()
    self.assertEqual(metadata["skipped_samples"]["camera-overlays: unavailable calibration"], "2")

  def test_calibration_is_not_backdated_to_initial_images(self):
    self.make_overlay_segment(with_calibration=False)
    with (self.root / "rlog0").open("ab") as stream:
      stream.write(calibration_event(1_500).to_bytes())
    with patch.dict(f"{MODULE}.DEVICE_CAMERAS", {("mici", "os04c10"): CAMERA_CONFIG}):
      convert_route(ROUTE, self.output, topics=["narrow-road-camera", "lane-lines"], camera_overlays=True)
    messages, _, _ = self.read_messages()
    tf = next((record, message) for channel, record, message in messages if channel.topic == "/tf")
    self.assertEqual(tf[0].log_time, TIME_OFFSET + 1_500)
    for transform in tf[1].transforms:
      self.assertEqual(transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec, TIME_OFFSET + 1_500)
    image_times = [record.log_time for channel, record, _ in messages if channel.topic.endswith("/compressed")]
    self.assertEqual(image_times, [TIME_OFFSET + 1_000, TIME_OFFSET + 1_050])

  def test_overlay_cli_forwards_explicit_opt_in(self):
    topics = ["narrow-road-camera", "lane-lines", "lead-vehicles"]
    for allow_uncalibrated in (False, True):
      with self.subTest(allow_uncalibrated=allow_uncalibrated):
        argv = ["convert", ROUTE, "--output", str(self.output), "--topics", *topics, "--camera-overlays"]
        if allow_uncalibrated:
          argv.append("--allow-uncalibrated-overlays")
        with patch("sys.argv", argv), patch(f"{MODULE}.convert_route", return_value=Counter()) as convert, redirect_stdout(io.StringIO()):
          main()
        convert.assert_called_once_with(
          ROUTE, self.output, topics=topics, camera_overlays=True, allow_uncalibrated_overlays=allow_uncalibrated,
        )

  def test_native_cdr_round_trip_and_filtering(self):
    counts = convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    messages, summary, metadata = self.read_messages()
    self.assertEqual(counts, {"/vehicle/speed": 2})
    self.assertEqual([record.log_time for _, record, _ in messages], [TIME_OFFSET + 100, TIME_OFFSET + 200])
    self.assertEqual([record.publish_time for _, record, _ in messages], [TIME_OFFSET + 100, TIME_OFFSET + 200])
    self.assertEqual([message.data for _, _, message in messages], [1.0, 2.0])
    self.assertEqual([record.sequence for _, record, _ in messages], [0, 1])
    self.assertEqual(summary.statistics.message_count, 2)
    self.assertEqual(metadata["openpilot"]["selected_signals"], "vehicle-speed")
    self.assertEqual(metadata["openpilot"]["time_basis"], "Unix epoch nanoseconds (UTC)")
    self.assertEqual(metadata["openpilot"]["time_offset_ns"], str(TIME_OFFSET))
    self.assertEqual(metadata["openpilot"]["time_anchor"], "clocks.wallTimeNanos")
    for channel, record, _ in messages:
      self.assertEqual(channel.message_encoding, "cdr")
      self.assertEqual(channel.metadata["offered_qos_profiles"], "[]")
      self.assertTrue(record.data.startswith(b"\x00\x01\x00\x00"))

  def test_multiple_segments_and_range(self):
    self.route.segments.append(self.make_segment(1, [event("carState", 300)]))
    self.route.max_seg_number = 1
    counts = convert_route(f"{ROUTE}/1", self.output, topics=["vehicle-speed"])
    messages, _, metadata = self.read_messages()
    self.assertEqual(counts, {"/vehicle/speed": 1})
    self.assertEqual(messages[0][1].log_time, TIME_OFFSET + 300)
    self.assertEqual(metadata["openpilot"]["segments"], "1")

  def test_full_route_with_two_selected_cameras(self):
    second = self.make_segment(1, [event("carState", 300)])
    self.route.segments.append(second)
    self.route.max_seg_number = 1
    for segment in self.route.segments:
      segment.camera_path = segment.ecamera_path = "mocked-video"
    image = io.BytesIO()
    Image.new("RGB", (64, 48), (240, 10, 20)).save(image, format="JPEG")

    def video(path, signal, segment, events):
      for frame in range(2):
        timestamp = 100 + segment * 200 + frame * 50
        yield (
          timestamp,
          SIGNALS[signal].topics[0],
          CompressedImage(header(timestamp, "test_optical"), "rgb8; jpeg compressed bgr8", np.frombuffer(image.getvalue(), dtype=np.uint8)),
        )

    with patch(f"{MODULE}.video_messages", side_effect=video):
      counts = convert_route(ROUTE, self.output, topics=["narrow-road-camera", "wide-road-camera", "vehicle-speed"])
    messages, summary, metadata = self.read_messages()
    self.assertEqual(summary.statistics.message_count, 11)
    self.assertEqual(metadata["openpilot"]["segments"], "0,1")
    for signal in ("narrow-road-camera", "wide-road-camera"):
      topic = SIGNALS[signal].topics[0]
      self.assertEqual(counts[topic], 4)
      frames = [record for channel, record, _ in messages if channel.topic == topic]
      self.assertEqual([frame.log_time for frame in frames], [TIME_OFFSET + timestamp for timestamp in (100, 150, 300, 350)])
      self.assertEqual([frame.sequence for frame in frames], [0, 1, 2, 3])

  def test_unselected_camera_is_not_read(self):
    self.segment.camera_path = "unselected-video"
    with patch(f"{MODULE}.FileReader", side_effect=AssertionError("Unselected camera was read")):
      counts = convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    self.assertEqual(counts, {"/vehicle/speed": 2})

  def test_missing_selected_source_fails_atomically(self):
    with self.assertRaisesRegex(ValueError, "produced no available"):
      convert_route(ROUTE, self.output, topics=["gps-fix"])
    self.assertFalse(self.output.exists())

  def test_bad_selection(self):
    for topics in ([], ["not-a-signal"]):
      with self.subTest(topics=topics), self.assertRaisesRegex(ValueError, "supported signal"):
        convert_route(ROUTE, self.output, topics=topics)

  def test_source_inventory_reports_validity(self):
    self.route.segments = [self.make_segment(0, [event("carState", 100), event("carState", 200, valid=False)])]
    output = io.StringIO()
    with patch("sys.argv", ["convert", ROUTE, "--list-sources"]), redirect_stdout(output):
      main()
    self.assertIn("carState: 2 total, 1 valid", output.getvalue())
    self.assertFalse(self.output.exists())

  def test_existing_output_is_untouched(self):
    self.output.write_bytes(b"existing")
    with self.assertRaises(FileExistsError):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    self.assertEqual(self.output.read_bytes(), b"existing")

  def test_missing_inputs(self):
    with self.assertRaisesRegex(ValueError, "no narrowRoad video"):
      convert_route(ROUTE, self.output, topics=["narrow-road-camera"])
    self.segment.log_path = None
    with self.assertRaisesRegex(ValueError, "no rlog"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])

  def test_missing_segment_and_invalid_selector(self):
    self.route.max_seg_number = 1
    with self.assertRaisesRegex(ValueError, "missing segments"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    with self.assertRaisesRegex(ValueError, "requires rlogs"):
      convert_route(f"{ROUTE}/0/q", self.output, topics=["vehicle-speed"])
    with self.assertRaisesRegex(ValueError, "No segments"):
      convert_route(f"{ROUTE}/0:0", self.output, topics=["vehicle-speed"])

  def test_failure_does_not_leave_partial_export(self):
    before = set(self.root.iterdir())
    with patch(f"{MODULE}.LogReader", side_effect=OSError("download failed")), self.assertRaisesRegex(OSError, "download failed"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    self.assertEqual(set(self.root.iterdir()), before)

  def test_corrupt_logs_are_not_silently_truncated(self):
    def corrupt_log(*args, **kwargs):
      warnings.warn("Corrupted events detected", RuntimeWarning, stacklevel=1)
      return []

    with patch(f"{MODULE}.LogReader", side_effect=corrupt_log), self.assertRaisesRegex(RuntimeWarning, "Corrupted events"):
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    self.assertFalse(self.output.exists())

  def test_unavailable_samples_are_reported(self):
    valid = event("carState", 200)
    invalid = event("carState", 100, valid=False)
    self.route.segments = [self.make_segment(0, [invalid, valid])]
    with patch(f"{MODULE}.cloudlog.warning") as warning:
      convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    self.assertEqual(warning.call_count, 1)
    _, _, metadata = self.read_messages()
    self.assertEqual(metadata["skipped_samples"]["vehicle-speed: invalid source event"], "1")

  def test_utc_headers_preserve_acquisition_and_future_timestamps(self):
    acceleration = event("accelerometer", 2_000_000_000)
    acceleration.accelerometer.timestamp = 1_900_000_000
    acceleration.accelerometer.init("acceleration").v = [1, 2, 3]
    gyroscope = event("gyroscope", 2_100_000_000)
    gyroscope.gyroscope.timestamp = 2_000_000_000
    gyroscope.gyroscope.init("gyroUncalibrated").v = [1, 2, 3]
    temperature = event("temperatureSensor", 2_200_000_000)
    temperature.temperatureSensor.timestamp = 2_100_000_000
    temperature.temperatureSensor.temperature = 25.0
    motion = event("deviceMotion", 2_300_000_000)
    motion.deviceMotion.timestamp = 2_200_000_000
    motion.deviceMotion.velocityDevice.valid = motion.deviceMotion.angularVelocityDevice.valid = True
    camera = event("cameraOdometry", 2_400_000_000)
    camera.cameraOdometry.timestampEof = 2_300_000_000
    camera.cameraOdometry.trans = camera.cameraOdometry.rot = [1, 2, 3]
    camera.cameraOdometry.transStd = camera.cameraOdometry.rotStd = [1, 2, 3]
    model = event("modelV2", 2_500_000_000)
    model.modelV2.timestampEof = 2_400_000_000
    model.modelV2.position.x, model.modelV2.position.y, model.modelV2.position.z, model.modelV2.position.t = [1, 2], [3, 4], [5, 6], [0, 1]
    for name, count in (("laneLines", 4), ("roadEdges", 2)):
      for line in model.modelV2.init(name, count):
        line.x, line.y, line.z = [1, 2], [3, 4], [5, 6]
    gps = event("gpsLocationExternal", 2_600_000_000)
    gps.gpsLocationExternal.hasFix = True
    gps.gpsLocationExternal.vNED = [1, 2, 3]
    self.route.segments = [self.make_segment(0, [acceleration, gyroscope, temperature, motion, camera, model, gps])]
    samples = {
      "raw-acceleration": (acceleration, 1_900_000_000),
      "raw-angular-velocity": (gyroscope, 2_000_000_000),
      "temperature": (temperature, 2_100_000_000),
      "estimated-imu": (motion, 2_200_000_000),
      "device-velocity": (motion, 2_200_000_000),
      "camera-velocity": (camera, 2_300_000_000),
      "predicted-path": (model, 2_400_000_000),
      "lane-lines": (model, 2_400_000_000),
      "road-edges": (model, 2_400_000_000),
      "gps-fix": (gps, gps.logMonoTime),
      "gps-velocity": (gps, gps.logMonoTime),
    }
    convert_route(ROUTE, self.output, topics=samples.keys())
    messages, _, _ = self.read_messages()
    by_topic = {topic: (source, timestamp) for signal, (source, timestamp) in samples.items() for topic in SIGNALS[signal].topics}
    self.assertEqual(len(messages), len(by_topic))
    for channel, record, message in messages:
      source, timestamp = by_topic[channel.topic]
      self.assertEqual(record.log_time, TIME_OFFSET + source.logMonoTime)
      self.assertEqual(record.publish_time, record.log_time)
      self.assertEqual(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec, TIME_OFFSET + timestamp)
      if message.__msgtype__ == "nav_msgs/msg/Path":
        for i, pose in enumerate(message.poses):
          future = i * 1_000_000_000 if channel.topic == "/model/path" else 0
          self.assertEqual(pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec, TIME_OFFSET + timestamp + future)

  def test_clock_anchor_takes_precedence_over_init_data(self):
    init = event("initData", 50)
    init.initData.wallTimeNanos = UTC_TIME - 1_000_000_000
    self.route.segments = [self.make_segment(0, [init, event("carState", 100)])]
    convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    messages, _, metadata = self.read_messages()
    self.assertEqual(messages[0][1].log_time, TIME_OFFSET + 100)
    self.assertEqual(metadata["openpilot"]["time_anchor"], "clocks.wallTimeNanos")

  def test_init_data_anchor_when_clocks_are_invalid(self):
    init = event("initData", 50)
    init.initData.wallTimeNanos = UTC_TIME
    clock = event("clocks", 75, valid=False)
    clock.clocks.wallTimeNanos = UTC_TIME + 1_000_000_000
    self.route.segments = [self.make_segment(0, [init, clock, event("carState", 100)], with_clock=False)]
    convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    messages, _, metadata = self.read_messages()
    self.assertEqual(messages[0][1].log_time, TIME_OFFSET + 100)
    self.assertEqual(metadata["openpilot"]["time_anchor"], "initData.wallTimeNanos")

  def test_missing_utc_anchor_fails_atomically(self):
    invalid = event("clocks", 50, valid=False)
    invalid.clocks.wallTimeNanos = UTC_TIME
    for anchors in ([], [invalid], [event("clocks", 50)], [event("initData", 50)]):
      with self.subTest(anchors=[anchor.which() for anchor in anchors]):
        self.route.segments = [self.make_segment(0, [*anchors, event("carState", 100)], with_clock=False)]
        before = set(self.root.iterdir())
        with self.assertRaisesRegex(ValueError, "UTC clock anchor"):
          convert_route(ROUTE, self.output, topics=["vehicle-speed"])
        self.assertEqual(set(self.root.iterdir()), before)

  def test_later_clock_changes_preserve_monotonic_intervals(self):
    clock = event("clocks", 250)
    clock.clocks.wallTimeNanos = UTC_TIME + 10_000_000_000
    self.route.segments.append(self.make_segment(1, [clock, event("carState", 300)], with_clock=False))
    self.route.max_seg_number = 1
    convert_route(ROUTE, self.output, topics=["vehicle-speed"])
    messages, _, _ = self.read_messages()
    self.assertEqual([record.log_time for _, record, _ in messages], [TIME_OFFSET + timestamp for timestamp in (100, 200, 300)])

  def test_real_video_jpeg_and_cdr(self):
    frames = [np.full((48, 64, 3), color, dtype=np.uint8) for color in ((240, 10, 20), (20, 220, 10))]
    video = self.make_video(frames)
    self.segment = self.make_segment(0, [encode_event(1, 150), encode_event(0, 100)])
    self.segment.camera_path = str(video)
    self.route.segments = [self.segment]
    counts = convert_route(ROUTE, self.output, topics=["narrow-road-camera"])
    messages, _, _ = self.read_messages()
    self.assertEqual(counts, {SIGNALS["narrow-road-camera"].topics[0]: 2})
    for (_, record, message), expected in zip(messages, frames, strict=True):
      self.assertEqual(message.__msgtype__, "sensor_msgs/msg/CompressedImage")
      self.assertEqual(message.format, "rgb8; jpeg compressed bgr8")
      self.assertEqual(message.header.frame_id, "narrow_road_optical")
      self.assertEqual(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec, record.log_time)
      self.assertEqual(record.publish_time, record.log_time)
      self.assertEqual(record.log_time, TIME_OFFSET + 100 + record.sequence * 50)
      with Image.open(io.BytesIO(message.data.tobytes())) as image:
        image.load()
        self.assertEqual(image.format, "JPEG")
        self.assertEqual(image.size, (64, 48))
        self.assertLess(np.abs(np.asarray(image).astype(float) - expected).mean(), 5)
    with self.assertRaisesRegex(ValueError, "more frames"):
      list(video_messages(str(video), "narrow-road-camera", 0, [encode_event(0, 100)]))
    with self.assertRaisesRegex(ValueError, "do not match"):
      list(video_messages(str(video), "narrow-road-camera", 0, [encode_event(i, 100 + i * 50) for i in range(3)]))

  def test_camera_index_validation(self):
    cases = [
      ([encode_event(0, 0)], "missing capture timestamp"),
      ([encode_event(0, 100), encode_event(0, 150)], "duplicate encode index"),
      ([encode_event(1, 100)], "noncontiguous"),
      ([encode_event(0, 150), encode_event(1, 100)], "strictly increasing"),
    ]
    for events, reason in cases:
      with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
        list(video_messages("unused", "narrow-road-camera", 0, events))


if __name__ == "__main__":
  unittest.main()
