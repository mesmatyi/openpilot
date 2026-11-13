#!/usr/bin/env python3
import argparse
import heapq
import io
import warnings
from collections import Counter
from collections.abc import Collection, Iterator
from pathlib import Path

import av
import capnp
import numpy as np
from mcap.writer import Writer
from opendbc.car.structs import car
from tqdm import tqdm

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import CameraConfig, DEVICE_CAMERAS
from openpilot.common.utils import atomic_write
from openpilot.tools.lib.filereader import FileReader
from openpilot.tools.lib.logreader import LogReader, parse_indirect
from openpilot.tools.lib.route import Route, Segment, SegmentRange
from openpilot.tools.mcap.ros2 import (
  NAV_SAT_COVARIANCE_TYPES, CompressedImage, PathMessage, SIGNALS, TYPESTORE, TimedMessage, camera_info, camera_transforms, header, log_messages,
  path_marker, shift_timestamps,
)

CAMERAS = {
  "narrow-road-camera": ("narrowRoad", "camera_path", "narrow_road_optical"),
  "wide-road-camera": ("wideRoad", "ecamera_path", "wide_road_optical"),
  "cabin-camera": ("cabin", "dcamera_path", "cabin_optical"),
}
ROAD_CAMERAS = {"narrow-road-camera": "narrow_road", "wide-road-camera": "wide_road"}
# DeviceCameraConfig attribute holding each camera's nominal intrinsics.
CAMERA_CONFIG_FIELDS = {**ROAD_CAMERAS, "cabin-camera": "cabin"}
CAMERA_DISTORTION = (
  "not modeled for any camera: plumb_bob with zero coefficients is openpilot's distortion-free pinhole approximation; " +
  "ROS and Foxglove tools will not undistort the images, although the real lenses are distorted (strongly for the fisheye wide-road " +
  "and cabin cameras)"
)
CAMERA_INTRINSICS_SOURCE = (
  "openpilot DEVICE_CAMERAS nominal pinhole selected by the logged deviceState.deviceType and <camera>CameraState.sensor; " +
  "rlogs contain no lens calibration, so these are not measured for the individual device"
)
GPS_FIX_COVARIANCE = (
  "APPROXIMATED for u-blox fixes with hasFix: diag(hAcc^2, hAcc^2, vAcc^2) m^2 east/north/up from NAV-PVT accuracy estimates " +
  "treated as 1-sigma (u-blox does not document their confidence level); UNKNOWN (zeros) without a fix or for other sources"
)


def video_messages(
  path: str, signal: str, segment_num: int, events: list, *, camera_config: CameraConfig | None = None,
) -> Iterator[TimedMessage]:
  camera, _, frame_name = CAMERAS[signal]
  timestamps: dict[int, int] = {}
  for event in events:
    if event.which() != SIGNALS[signal].source:
      continue
    index = getattr(event, event.which())
    if index.segmentNum != segment_num:
      continue
    if index.type != "fullHEVC":
      raise ValueError(f"{camera} segment {segment_num}: expected fullHEVC encode indexes")
    if index.segmentId in timestamps:
      raise ValueError(f"{camera} segment {segment_num}: duplicate encode index {index.segmentId}")
    if index.timestampSof <= 0:
      raise ValueError(f"{camera} segment {segment_num}: missing capture timestamp for frame {index.segmentId}")
    timestamps[index.segmentId] = index.timestampSof
  if not timestamps or timestamps.keys() != set(range(len(timestamps))):
    raise ValueError(f"{camera} segment {segment_num}: missing or noncontiguous encode indexes")
  if any(timestamps[i] <= timestamps[i - 1] for i in range(1, len(timestamps))):
    raise ValueError(f"{camera} segment {segment_num}: capture timestamps are not strictly increasing")

  with FileReader(path) as source:
    data = source.read()
  frame_count = 0
  with av.open(io.BytesIO(data), format="hevc") as container:
    container.streams.video[0].codec_context.thread_count = 2
    for frame_count, frame in enumerate(container.decode(video=0), start=1):
      frame_id = frame_count - 1
      if frame_id not in timestamps:
        raise ValueError(f"{camera} segment {segment_num}: video has more frames than encode indexes")
      output = io.BytesIO()
      frame.to_image().save(output, format="JPEG", quality=90, subsampling=2)
      timestamp = timestamps[frame_id]
      if camera_config is not None:
        if (frame.width, frame.height) != camera_config.size:
          raise ValueError(f"{camera}: decoded size {(frame.width, frame.height)} does not match camera calibration {camera_config.size}")
        yield timestamp, f"/camera/{camera}/camera_info", camera_info(timestamp, frame_name, camera_config)
      yield (
        timestamp,
        SIGNALS[signal].topics[0],
        CompressedImage(
          header(timestamp, frame_name),
          "rgb8; jpeg compressed bgr8",
          np.frombuffer(output.getvalue(), dtype=np.uint8),
        ),
      )
  if frame_count != len(timestamps):
    raise ValueError(f"{camera} segment {segment_num}: {frame_count} decoded frames do not match {len(timestamps)} encode indexes")


def camera_hardware(events: list, cameras: Collection[str]) -> dict[str, tuple[str, str]]:
  device_types = {str(event.deviceState.deviceType) for event in events if event.which() == "deviceState" and event.valid}
  if len(device_types) != 1:
    raise ValueError("Camera calibration requires one logged, valid deviceState.deviceType per segment")
  device_type = next(iter(device_types))
  hardware = {}
  for signal in cameras:
    source = f"{CAMERAS[signal][0]}CameraState"
    sensors = {str(getattr(event, source).sensor) for event in events if event.which() == source and event.valid}
    if len(sensors) != 1:
      raise ValueError(f"Camera calibration requires one logged, valid {source}.sensor per segment")
    hardware[signal] = (device_type, next(iter(sensors)))
  return hardware


def camera_config(signal: str, hardware: tuple[str, str]) -> CameraConfig:
  if hardware not in DEVICE_CAMERAS:
    raise ValueError(f"Unsupported camera hardware for calibration: {hardware}")
  config = getattr(DEVICE_CAMERAS[hardware], CAMERA_CONFIG_FIELDS[signal])
  if config.width <= 0 or config.height <= 0 or config.focal_length <= 0:
    raise ValueError(f"Unsupported camera hardware for calibration: {hardware} has no {CAMERAS[signal][0]} camera")
  return config


def camera_configs(events: list, cameras: Collection[str]) -> dict[str, CameraConfig]:
  return {signal: camera_config(signal, hardware) for signal, hardware in camera_hardware(events, cameras).items()}


def intrinsics_description(hardware: tuple[str, str], config: CameraConfig) -> str:
  k = config.intrinsics
  description = f"{hardware[0]}/{hardware[1]} {config.width}x{config.height} fx=fy={k[0, 0]:g} cx={k[0, 2]:g} cy={k[1, 2]:g}"
  if "unknown" in hardware or hardware[0] == "pc":
    description += " (hardware partly assumed by an openpilot DEVICE_CAMERAS alias)"
  return description


def calibration_basis(events: list) -> str:
  # When CarParams.notCar is set (opendbc's flag for non-car platforms, also logged by some bench setups), calibrationd reports
  # calStatus=calibrated with rpyCalib=[0,0,0] whatever it measures. It reads CarParams once at startup, so the logged notCar
  # flag is a proxy for which branch produced the updates.
  not_car = {bool(event.carParams.notCar) for event in events if event.which() == "carParams" and event.valid}
  if not not_car:
    for event in events:
      if event.which() != "initData":
        continue
      for entry in event.initData.params.entries:
        if entry.key == "CarParams" and len(entry.value):
          try:
            with car.CarParams.from_bytes(bytes(entry.value)) as params:
              not_car.add(bool(params.notCar))
          except (capnp.KjException, ValueError):
            continue
  if not_car == {True}:
    return "notcar_placeholder"
  if not_car == {False}:
    return "calibrationd"
  return "unknown"


def calibration_messages(
  events: list, cameras: Collection[str], skipped: Counter[str], *,
  allow_uncalibrated: bool = False, used_calibrations: Counter[str] | None = None,
  basis: str = "unknown", used_bases: Counter[str] | None = None, reason: str = "camera-overlays",
) -> Iterator[TimedMessage]:
  frames = [CAMERAS[signal][2] for signal in cameras]
  for event in events:
    if event.which() != "extrinsicsCalibration":
      continue
    status = event.extrinsicsCalibration.calStatus
    allowed_status = status == log.ExtrinsicsCalibration.Status.calibrated or (
      allow_uncalibrated and status == log.ExtrinsicsCalibration.Status.uncalibrated
    )
    if not event.valid or not allowed_status:
      skipped[f"{reason}: unavailable calibration"] += 1
      continue
    transforms = camera_transforms(event.extrinsicsCalibration, event.logMonoTime, frames)
    if used_calibrations is not None:
      used_calibrations[str(status)] += 1
    if used_bases is not None:
      used_bases[basis] += 1
    yield event.logMonoTime, "/tf", transforms


def overlay_messages(messages: Iterator[TimedMessage]) -> Iterator[TimedMessage]:
  for timestamp, topic, message in messages:
    if isinstance(message, PathMessage):
      yield timestamp, f"{topic}/marker", path_marker(message)
    yield timestamp, topic, message


def route_segments(identifier: str) -> tuple[Route, list[Segment]]:
  segment_range = SegmentRange(parse_indirect(identifier))
  if segment_range.selector not in (None, "r"):
    raise ValueError("ROS 2 export requires rlogs; omit the log selector or use /r")
  route = Route(segment_range.route_name)
  by_number = {segment.name.segment_num: segment for segment in route.segments}
  numbers = segment_range.seg_idxs if segment_range.slice else list(range(route.max_seg_number + 1))
  if not numbers:
    raise ValueError("No segments selected")
  missing = sorted(set(numbers) - by_number.keys())
  if missing:
    raise ValueError(f"Route is missing segments: {missing}")
  segments = [by_number[number] for number in numbers]
  for segment in segments:
    if not segment.log_path:
      raise ValueError(f"Segment {segment.name.segment_num} has no rlog")
  return route, segments


def read_events(segment: Segment) -> list:
  with warnings.catch_warnings(action="error", category=RuntimeWarning):
    events = list(LogReader(segment.log_path, sort_by_time=True))
  if not events:
    raise ValueError(f"Segment {segment.name.segment_num} contains no log events")
  return events


def utc_time_offset(events: list) -> tuple[int, str]:
  for source in ("clocks", "initData"):
    for event in events:
      if event.which() != source or not event.valid:
        continue
      wall_time = getattr(event, source).wallTimeNanos
      if wall_time > 0:
        return wall_time - event.logMonoTime, f"{source}.wallTimeNanos"
  raise ValueError("No valid UTC clock anchor (clocks.wallTimeNanos or initData.wallTimeNanos) in the first selected segment")


def convert_route(
  identifier: str, output: Path, *, topics: Collection[str], camera_overlays: bool = False, allow_uncalibrated_overlays: bool = False,
  camera_info: bool = False,
) -> Counter[str]:
  selected = set(topics)
  if not selected or selected - SIGNALS.keys():
    raise ValueError(f"Select supported signal names with --topics; unknown names: {sorted(selected - SIGNALS.keys())}")
  if allow_uncalibrated_overlays and not (camera_overlays or camera_info):
    raise ValueError("--allow-uncalibrated-overlays requires --camera-overlays or --camera-info")
  cameras = sorted(selected & CAMERAS.keys())
  selected_road_cameras = sorted(selected & ROAD_CAMERAS.keys())
  if camera_overlays and not selected_road_cameras:
    raise ValueError("--camera-overlays requires at least one selected road camera")
  if camera_info and not cameras:
    raise ValueError("--camera-info requires at least one selected camera")
  if allow_uncalibrated_overlays and not selected_road_cameras:
    raise ValueError("--allow-uncalibrated-overlays requires a selected road camera; the cabin camera has no logged extrinsics")
  # Road cameras whose logged extrinsic calibration is exported as /tf; the cabin camera has no logged extrinsics.
  road_cameras = selected_road_cameras if camera_overlays or camera_info else []
  # Cameras that get CameraInfo: every selected camera with --camera-info, plus the road cameras required by overlays.
  info_cameras = sorted(set(cameras if camera_info else ()) | set(selected_road_cameras if camera_overlays else ()))
  route, segments = route_segments(identifier)
  for segment in segments:
    for signal in cameras:
      if not getattr(segment, CAMERAS[signal][1]):
        raise ValueError(f"Segment {segment.name.segment_num} has no {CAMERAS[signal][0]} video")

  counts: Counter[str] = Counter()
  skipped: Counter[str] = Counter()
  source_counts: Counter[str] = Counter()
  source_valid_counts: Counter[str] = Counter()
  used_calibrations: Counter[str] = Counter()
  used_bases: Counter[str] = Counter()
  gps_covariance_counts: Counter[str] = Counter()
  intrinsics: dict[str, set[str]] = {}
  channels: dict[str, int] = {}
  schemas: dict[str, int] = {}
  with atomic_write(str(output), mode="wb") as stream:
    events = read_events(segments[0])
    time_offset, time_anchor = utc_time_offset(events)
    writer = Writer(stream, enable_data_crcs=True)
    writer.start(profile="ros2", library="openpilot")
    writer.add_metadata(
      "openpilot",
      {
        "route": route.name.canonical_name,
        "segments": ",".join(str(segment.name.segment_num) for segment in segments),
        "selected_signals": ",".join(sorted(selected)),
        "time_basis": "Unix epoch nanoseconds (UTC)",
        "time_offset_ns": str(time_offset),
        "time_anchor": time_anchor,
        "camera_encoding": "JPEG quality 90, original resolution and frame sampling",
        "raw_imu_frame": "sensor axes as logged; no invented transform to the camera/device",
        "device_frame": "device_flu: forward, left, up",
        "model_frame": "model_flu: model coordinates with right/down axes flipped",
        "camera_overlays": str(camera_overlays).lower(),
        "camera_info": str(camera_info).lower(),
        "camera_calibration_policy": ("allow_uncalibrated" if allow_uncalibrated_overlays else "calibrated_only") if road_cameras else "disabled",
        "camera_projection": (
          ("APPROXIMATE: uncalibrated logged rotations allowed; " if allow_uncalibrated_overlays else "") +
          "openpilot pinhole intrinsics and rotation-only projection frames; not surveyed physical camera translations"
        ),
        "path_orientation": "identity quaternions; these topics contain position geometry, not an attitude estimate",
        "gps_fix_covariance": GPS_FIX_COVARIANCE,
        "gps_velocity_frame": "gps_enu: east, north, up; linear velocity only",
        "steering_torque_units": "native vehicle-specific CAN units, not Nm",
      },
    )
    for index, segment in enumerate(tqdm(segments, desc="Converting ROS 2", unit="segment")):
      if index:
        events = read_events(segment)
      source_counts.update(event.which() for event in events)
      source_valid_counts.update(event.which() for event in events if event.valid)
      hardware = camera_hardware(events, info_cameras) if info_cameras else {}
      configs = {signal: camera_config(signal, key) for signal, key in hardware.items()}
      for signal, key in hardware.items():
        intrinsics.setdefault(CAMERAS[signal][0], set()).add(intrinsics_description(key, configs[signal]))
      videos = []
      for signal in cameras:
        path = getattr(segment, CAMERAS[signal][1])
        if signal in configs:
          videos.append(video_messages(path, signal, segment.name.segment_num, events, camera_config=configs[signal]))
        else:
          videos.append(video_messages(path, signal, segment.name.segment_num, events))
      telemetry = log_messages(events, selected - CAMERAS.keys(), skipped)
      calibration = calibration_messages(
        events, road_cameras, skipped, allow_uncalibrated=allow_uncalibrated_overlays, used_calibrations=used_calibrations,
        basis=calibration_basis(events), used_bases=used_bases, reason="camera-overlays" if camera_overlays else "camera-info",
      ) if road_cameras else iter(())
      messages = heapq.merge(telemetry, calibration, *videos, key=lambda item: item[0])
      for timestamp, topic, message in overlay_messages(messages) if camera_overlays else messages:
        timestamp += time_offset
        shift_timestamps(message, time_offset)
        datatype = message.__msgtype__
        if datatype == "sensor_msgs/msg/NavSatFix":
          gps_covariance_counts[NAV_SAT_COVARIANCE_TYPES[message.position_covariance_type]] += 1
        if topic not in channels:
          if datatype not in schemas:
            definition, _ = TYPESTORE.generate_msgdef(datatype, ros_version=2)
            schemas[datatype] = writer.register_schema(name=datatype, encoding="ros2msg", data=definition.encode())
          channels[topic] = writer.register_channel(topic=topic, message_encoding="cdr", schema_id=schemas[datatype], metadata={"offered_qos_profiles": "[]"})
        writer.add_message(
          channel_id=channels[topic], log_time=timestamp, publish_time=timestamp, sequence=counts[topic], data=bytes(TYPESTORE.serialize_cdr(message, datatype))
        )
        counts[topic] += 1
    missing_outputs = [topic for signal in sorted(selected) for topic in SIGNALS[signal].topics if not counts[topic]]
    # Overlays cannot be drawn without extrinsics, so they require /tf; --camera-info alone exports whatever calibration exists.
    required = (["/tf"] if camera_overlays else []) + [f"/camera/{CAMERAS[signal][0]}/camera_info" for signal in info_cameras]
    missing_outputs.extend(topic for topic in required if not counts[topic])
    if skipped:
      cloudlog.warning(f"Skipped unavailable ROS 2 samples: {dict(skipped)}")
    if missing_outputs:
      raise ValueError(f"Selected topics produced no available samples: {missing_outputs}")
    if used_calibrations["uncalibrated"]:
      cloudlog.warning(
        f"APPROXIMATE camera {'overlays' if camera_overlays else 'calibration'}: used {used_calibrations['uncalibrated']} valid but " +
        "uncalibrated rotation updates; camera alignment is not guaranteed."
      )
    if used_bases["notcar_placeholder"]:
      cloudlog.warning(
        f"PLACEHOLDER camera calibration: {used_bases['notcar_placeholder']} extrinsics updates come from a notCar route, where calibrationd " +
        "reports calStatus=calibrated with a zero rotation without measuring; /tf matches openpilot's model frame, not the device mounting."
      )
    if road_cameras and not counts["/tf"]:
      cloudlog.warning("No usable extrinsic camera calibration was logged; camera optical frames are not connected to device_flu.")
    if road_cameras:
      writer.add_metadata("camera_calibration_counts", {status: str(count) for status, count in sorted(used_calibrations.items())})
      writer.add_metadata("camera_calibration_basis", {basis: str(count) for basis, count in sorted(used_bases.items())})
    if info_cameras:
      writer.add_metadata(
        "camera_intrinsics",
        {
          "source": CAMERA_INTRINSICS_SOURCE,
          "distortion": CAMERA_DISTORTION,
          "principal_point": "openpilot's nominal (width/2, height/2), not a measured optical center",
          **{camera: "; ".join(sorted(descriptions)) for camera, descriptions in sorted(intrinsics.items())},
        },
      )
    if "gps-fix" in selected:
      writer.add_metadata("gps_fix_covariance_counts", {kind: str(count) for kind, count in sorted(gps_covariance_counts.items())})
    writer.add_metadata("source_event_counts", {source: str(count) for source, count in sorted(source_counts.items())})
    writer.add_metadata("source_valid_event_counts", {source: str(source_valid_counts[source]) for source in sorted(source_counts)})
    writer.add_metadata("skipped_samples", {reason: str(count) for reason, count in sorted(skipped.items())})
    writer.finish()
  return counts


def main() -> None:
  parser = argparse.ArgumentParser(description="Export selected openpilot signals as standard ROS 2/CDR messages in MCAP.")
  parser.add_argument("route", nargs="?", help="Route or segment range, e.g. dongle/route/0:2")
  parser.add_argument("-o", "--output", type=Path, help="Destination MCAP file (must not already exist)")
  parser.add_argument("--topics", nargs="+", choices=sorted(SIGNALS), help="Explicitly selected signal groups; see --list-topics")
  parser.add_argument("--camera-overlays", action="store_true", help="Include road-camera CameraInfo, calibrated /tf, and model geometry markers")
  parser.add_argument(
    "--camera-info", action="store_true",
    help="Include CameraInfo (openpilot's nominal pinhole intrinsics, distortion not modeled) for every selected camera, " +
    "plus the logged extrinsic calibration as /tf for selected road cameras",
  )
  parser.add_argument(
    "--allow-uncalibrated-overlays", action="store_true",
    help="Allow valid logged uncalibrated rotations for approximate alignment; requires --camera-overlays or --camera-info with a road camera",
  )
  listing = parser.add_mutually_exclusive_group()
  listing.add_argument("--list-topics", action="store_true", help="List implemented signal mappings and standard ROS 2 types")
  listing.add_argument("--list-sources", action="store_true", help="Count all source log message types in the given route")
  args = parser.parse_args()
  if args.list_topics:
    for name, signal in SIGNALS.items():
      print(f"{name}: {signal.source} -> {signal.datatype} -> {', '.join(signal.topics)}")
    return
  if not args.route:
    parser.error("a route is required")
  if args.list_sources:
    _, segments = route_segments(args.route)
    counts: Counter[str] = Counter()
    valid_counts: Counter[str] = Counter()
    for segment in tqdm(segments, desc="Listing source messages", unit="segment"):
      for event in read_events(segment):
        counts[event.which()] += 1
        valid_counts[event.which()] += bool(event.valid)
    for name, count in sorted(counts.items()):
      print(f"{name}: {count:,} total, {valid_counts[name]:,} valid")
    available = [camera for camera, attribute, _ in CAMERAS.values() if any(getattr(segment, attribute) for segment in segments)]
    print(f"Available camera streams: {', '.join(available)}")
    return
  if args.output is None or not args.topics:
    parser.error("--output and --topics are required for conversion")
  counts = convert_route(
    args.route, args.output, topics=args.topics, camera_overlays=args.camera_overlays,
    allow_uncalibrated_overlays=args.allow_uncalibrated_overlays, camera_info=args.camera_info,
  )
  for topic, count in sorted(counts.items()):
    print(f"{topic}: {count:,}")
  print(f"Saved {sum(counts.values()):,} ROS 2 messages to {args.output}")


if __name__ == "__main__":
  main()
