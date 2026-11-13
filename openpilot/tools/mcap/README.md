# Standard ROS 2 MCAP export

Export explicitly selected openpilot signals as **standard ROS 2 messages**:
MCAP profile `ros2`, CDR serialization, embedded `ros2msg` definitions and QoS
metadata. No ROS installation is required. Definitions come from the ROS 2
Jazzy common interfaces supplied by `rosbags`; no custom openpilot or
Foxglove-specific message definitions are used.

```sh
uv sync --extra tools

# List implemented signal groups, source messages, output topics and ROS types.
uv run python -m openpilot.tools.mcap.convert --list-topics

# Inventory every original log message type (total/valid counts) and camera streams.
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>' --list-sources

# Convert only the selected groups. Use /0:1 for a first-segment trial.
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>/0:1' --output route-ros2.mcap \
  --topics narrow-road-camera wide-road-camera gps-fix gps-velocity \
    raw-acceleration raw-angular-velocity vehicle-speed steering-angle
```

`--topics` is required. Unselected fields and original event envelopes are not
exported. Unsupported selections, missing requested camera files/rlogs, corrupt
logs, and selections producing no output fail explicitly. Existing output files
are never overwritten; unsuccessful exports are removed atomically.

The existing comma API authentication, route/segment-range parsing, and download
cache are reused. Reduced qlogs are not substituted for rlogs.

## Available mappings

| Signal group | Standard ROS 2 type | Output |
|---|---|---|
| `narrow-road-camera`, `wide-road-camera`, `cabin-camera` | `sensor_msgs/msg/CompressedImage` | `/camera/<name>/image_raw/compressed` |
| `gps-fix` | `sensor_msgs/msg/NavSatFix` | `/gps/fix` |
| `gps-velocity` | `geometry_msgs/msg/Vector3Stamped` | `/gps/velocity` |
| `raw-acceleration`, `raw-angular-velocity` | `sensor_msgs/msg/Imu` | `/imu/acceleration`, `/imu/angular_velocity` |
| `temperature` | `sensor_msgs/msg/Temperature` | `/imu/temperature` |
| `estimated-imu` | `sensor_msgs/msg/Imu` | `/motion/imu` |
| `device-velocity`, `camera-velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `/motion/velocity`, `/motion/camera_velocity` |
| `vehicle-speed`, `steering-angle`, `steering-torque`, `vehicle-yaw-rate` | `std_msgs/msg/Float64` | `/vehicle/speed`, `/vehicle/steering_angle`, `/vehicle/steering_torque`, `/vehicle/yaw_rate` |
| `autonomy-flags` | `std_msgs/msg/Bool` | `/autonomy/enabled`, `/autonomy/active`, `/autonomy/engageable` |
| `predicted-path` | `nav_msgs/msg/Path` | `/model/path` |
| `lane-lines`, `road-edges` | `nav_msgs/msg/Path` | Four `/model/lane_lines/line_<index>` and two `/model/road_edges/edge_<index>` topics |
| `lead-vehicles` | `visualization_msgs/msg/MarkerArray` | `/model/leads`: current model lead ground indicator and distance label |
| `target-acceleration` | `std_msgs/msg/Float64` | `/planning/target_acceleration` |

## Camera overlays in Foxglove

Opt in with `--camera-overlays` to add the standard ROS 2 calibration and
visualization messages needed to project model geometry into either road camera:

```sh
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>/0:1' --output route-overlays.mcap \
  --topics narrow-road-camera wide-road-camera lane-lines road-edges lead-vehicles \
  --camera-overlays
```

The flag requires at least one selected road camera. Existing exports are not
modified, and exports without this flag retain their original camera behavior.
Overlay mode adds:

- `/camera/narrowRoad/camera_info` and/or `/camera/wideRoad/camera_info`:
  `sensor_msgs/msg/CameraInfo`, at every image's capture timestamp and resolution.
- `/tf`: `tf2_msgs/msg/TFMessage`, with `device_flu` as the parent of `model_flu`
  and the selected road optical frames. Model and wide-camera rotations come
  from valid, calibrated `extrinsicsCalibration.rpyCalib` and
  `wideFromDeviceEuler`, with the ROS FLU/optical axis conversions applied.
- A `visualization_msgs/msg/Marker` topic named `<path topic>/marker` for each
  selected lane line, road edge, or predicted path. The original `nav_msgs/Path`
  topics remain available.

In Foxglove, add one **Image** panel per road camera. Choose its
`/camera/<name>/image_raw/compressed` topic and corresponding
`/camera/<name>/camera_info` under **Calibration**, then enable **Rectify**.
Under **Topics**, enable `/model/lane_lines/line_<index>/marker`, any selected
road-edge/path markers, and `/model/leads`. Keep **Sync timestamps** off:
images use exposure-start timestamps while model geometry uses the logged
exposure-end timestamp. `/tf` supplies the connection to the optical frame;
the original `model_flu` geometry does not need to be relabeled as camera data.
See the [Foxglove Image panel documentation](https://docs.foxglove.dev/docs/visualization/panels/image).

Intrinsics come from openpilot's shared `DEVICE_CAMERAS` profiles, selected by
the logged device type and camera sensor. This is the same zero-distortion
pinhole approximation used by openpilot's renderer, not a newly measured lens
calibration. Wide-camera edge alignment may therefore be approximate. Unknown
hardware, absent hardware metadata, mismatched image sizes, malformed rotations,
or an export with no eligible calibration updates fail explicitly. By default,
invalid or uncalibrated updates are counted in `skipped_samples`; no transform
is invented or backdated to cover the initial calibration warm-up.

These are rotation-only **projection frames**, with coincident origins as in
openpilot's renderer, not surveyed physical camera positions. Raw IMU and GNSS
frames remain disconnected because their spatial extrinsics are not established.

### Explicitly allowing uncalibrated overlays

Some recordings never complete calibration. To inspect approximate overlays
using their logged rotations, explicitly add `--allow-uncalibrated-overlays`:

```sh
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>' --output route-overlays-uncalibrated.mcap \
  --topics narrow-road-camera wide-road-camera lane-lines lead-vehicles \
  --camera-overlays --allow-uncalibrated-overlays
```

This requires `--camera-overlays` and additionally accepts **valid events**
whose calibration status is `uncalibrated`. Invalid events and `invalid` or
`recalibrating` statuses remain excluded. Missing, malformed, or nonfinite
rotation vectors still fail; no zero-angle fallback or calibration from
another route is substituted.

These overlays are **uncalibrated and approximate; alignment is not guaranteed**.
An aggregate warning reports accepted uncalibrated updates. MCAP metadata marks
`camera_calibration_policy=allow_uncalibrated`, prefixes `camera_projection`
with `APPROXIMATE`, and records accepted status counts in
`camera_calibration_counts`. Accepted uncalibrated updates are not mislabeled
as skipped samples. Without this flag, calibrated-only behavior is unchanged.

### Lead indicators

Select `lead-vehicles` to export the current model lead when its probability is
greater than 0.5. The `probTime=2` and `probTime=4` hypotheses describe future lead
selection, not additional vehicles, and are not shown as separate detections.
There are no measured lead heights or 3D boxes in these predictions, so the
export draws a red ground-position indicator and a distance label, not a vehicle
bounding box. Ground height is interpolated from the two inner lane lines;
positions outside their logged range are omitted and counted rather than
extrapolated. Marker sizes are visualization styling, not vehicle dimensions.

Each update clears the previous lead markers, including when confidence drops.
Markers also expire after 0.25 seconds so missing/invalid source events do not
leave an indefinitely stale detection.

## Cameras

H.265 is decoded and **each original frame** becomes a JPEG image (quality 90,
4:2:0 chroma sampling) in `sensor_msgs/msg/CompressedImage`, at the original
resolution. This is a lossy re-encoding, not original H.265 packets mislabeled as
images. JPEG playback does not require H.265 support in the viewer.

Capture stamps come from `EncodeIndex.timestampSof`, with matching segment/frame
indexes and optical frame IDs. Missing, duplicate, noncontiguous, or reordered
indexes and decoded-frame-count mismatches fail explicitly. Cabin video is only
available when its file was actually uploaded.

## Timestamps, frames, and units

- All exported timestamps use Unix epoch nanoseconds (UTC), so viewers show
  the recording's actual date instead of 1970. A fixed offset is derived from
  the first valid, nonzero `clocks.wallTimeNanos` and its `Event.logMonoTime`
  in the first selected segment, falling back to valid, nonzero
  `initData.wallTimeNanos`. Export fails explicitly if neither is available;
  the route name and the computer's current clock are not used to invent a date.
- The same offset is applied to MCAP log/publish times, camera capture stamps,
  sensor/state headers, TF/marker headers, and every path pose timestamp. Acquisition delays and
  predicted-path future offsets are preserved, even if the recorded wall clock
  changes later. GNSS headers use the converted source log timestamp so all
  topics share one clock. The offset and anchor source are stored in MCAP
  metadata as `time_offset_ns` and `time_anchor`.
- Raw IMU vectors stay in the physical sensor's logged axes,
  `imu_<sensor source>`. These are **not** assumed to be device/camera axes.
  Unmeasured IMU components have covariance element zero set to `-1`;
  measured components without covariance have zero/unknown covariance.
- Estimated device and camera velocities are rotated from forward/right/down
  into `device_flu` (forward/left/up), including angular velocity. Known
  component variances populate covariance diagonals. Estimated IMU orientation
  is converted from NED/device coordinates to ENU/FLU; Euler-angle uncertainties
  are not falsely presented as quaternion-error covariance.
- GNSS velocity is a linear-only ENU `Vector3Stamped`, not a twist containing
  fabricated angular velocity. GNSS fixes preserve WGS84 latitude, longitude,
  ellipsoidal altitude, and fix availability. Position covariance and
  constellation service are explicitly unknown rather than inferred.
- Predicted paths, lane lines, and road edges use `model_flu`, with right/down
  axes flipped. With `--camera-overlays`, logged calibration connects this
  projection frame to `device_flu` and the road optical frames as described above.
  Path poses have identity orientation: they represent **position geometry**,
  not measured attitude. Predicted-position pose stamps include the source's
  future time offsets; lane/edge geometry retains its capture time.
- Vehicle speed and target acceleration retain m/s and m/s^2. Steering-wheel
  angles are converted from degrees to radians. Yaw rate stays as logged in
  rad/s. **Steering torque retains vehicle-specific native CAN units, not Nm.**
- Invalid source events, no-fix GNSS velocities, and unavailable device-velocity
  estimates are omitted with an explicit aggregate warning and `skipped_samples`
  MCAP metadata. Invalid individual estimated-IMU components are marked
  unavailable using the standard covariance convention.

Route, selection, source-event counts, frame conventions, and omitted-sample
counts are stored as MCAP metadata, not disguised as standard ROS messages.
Source validity counts are included in discovery and metadata, so an uploaded
topic with no valid samples is distinguishable from usable measurements.

## Tests

```sh
uv run python -m unittest openpilot.tools.mcap.test_ros2
```

Tests deserialize CDR using a fresh type store populated only from the embedded
definitions, validate MCAP CRCs/profile/QoS, exercise filtering and coordinate/
unit conversions, verify UTC clock anchoring and synchronized timestamps,
and encode/decode real HEVC-to-JPEG camera frames.
Overlay tests verify both TF directions against openpilot's projection math,
CameraInfo/image synchronization, nested UTC timestamps, standard marker
decoding, current-versus-future lead semantics, and stale-marker removal.
