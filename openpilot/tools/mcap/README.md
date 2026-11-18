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
  --topics narrow-road-camera wide-road-camera gps-fix gps-velocity gps-speed \
    raw-acceleration raw-angular-velocity

# Also add per-image CameraInfo and the logged extrinsic calibration (/tf):
# one tree rooted at ground_link with both road optical frames.
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>/0:1' --output route-calibrated.mcap \
  --topics narrow-road-camera wide-road-camera ground-link --camera-info
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
| `gps-velocity` | `geometry_msgs/msg/Vector3Stamped` | `/gps/velocity`: east/north/up velocity vector |
| `gps-speed` | `std_msgs/msg/Float64` | `/gps/speed`: magnitude of that vector, m/s |
| `raw-acceleration`, `raw-angular-velocity` | `sensor_msgs/msg/Imu` | `/imu/acceleration`, `/imu/angular_velocity` |
| `temperature` | `sensor_msgs/msg/Temperature` | `/imu/temperature` |
| `estimated-imu` | `sensor_msgs/msg/Imu` | `/motion/imu` |
| `device-velocity`, `camera-velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `/motion/velocity`, `/motion/camera_velocity` |
| `predicted-path` | `nav_msgs/msg/Path` | `/model/path` |
| `lane-lines`, `road-edges` | `nav_msgs/msg/Path` | Four `/model/lane_lines/line_<index>` and two `/model/road_edges/edge_<index>` topics |
| `lead-vehicles` | `visualization_msgs/msg/MarkerArray` | `/model/leads`: current model lead ground indicator and distance label |
| `ground-link` | `tf2_msgs/msg/TFMessage` | `/tf`: `ground_link` → `device_flu` → `model_flu` from the logged extrinsic calibration |

Vehicle (CAN) state such as speed, steering, and yaw rate, openpilot's
engagement flags, and longitudinal planning are intentionally not exported; the
converter targets the device's own sensors and camera-derived outputs.
`lead-vehicles` is kept deliberately: it is the camera model's detection of the
vehicle ahead, not CAN data from the ego car. The extrinsic calibration behind
`ground-link` and the camera transforms still depends on the car indirectly,
because calibrationd only learns it while `carState` reports more than 15 mph
(see [Ground link](#ground-link)).

Two opt-in flags add camera calibration topics for the selected cameras:
`--camera-info` (`sensor_msgs/msg/CameraInfo` on `/camera/<name>/camera_info`
plus `/tf`) and `--camera-overlays`, described below. Their transforms share one
`/tf` tree with `ground-link`.

## Ground link

Select `ground-link` to publish the logged extrinsic calibration as a road-level
base frame aligned with the calibrated direction of travel, independent of any
camera selection:

- `ground_link` → `device_flu`: `ground_link` has the calibrated (model)
  frame's axes in forward/left/up form, so x follows the calibrated direction of
  travel. As in openpilot's calibrated frame, only pitch and yaw are calibrated
  and roll follows the device, so z is normal to the road only if the device is
  mounted level across the car. The origin is `height` below the device along
  that z axis: the transform's translation is
  `(0, 0, extrinsicsCalibration.height)`, and its rotation is the inverse of the
  `rpyCalib` mounting rotation.
- `device_flu` → `model_flu`: the same `rpyCalib` rotation that the camera flags
  publish. `model_flu` shares the device's origin, so a model point at
  (x, y, z) appears at (x, y, z + height) in `ground_link`. Lane lines and road
  edges, which openpilot logs at road level, land near z = 0. `/model/path`,
  logged at camera height, floats about `height` above it (see the path note
  under Camera overlays).

Transforms are published at every accepted `extrinsicsCalibration` update (about
4 Hz) under the same calibrated-only rule as the camera transforms; add
`--allow-uncalibrated-overlays` to also accept valid `uncalibrated` updates. The
warning and `ground_frame` then mark `ground_link` as APPROXIMATE, since its
rotation and height may not have converged. A selection that yields no transform fails, and a missing, non-finite, or
non-positive `height` fails instead of being guessed. Accepted updates are
recorded in the same `camera_calibration_policy`, `camera_calibration_counts`,
and `camera_calibration_basis` metadata as the camera transforms, and
`ground_frame` describes the frame. With `--camera-info` or `--camera-overlays`,
the selected road optical frames are added to the same `/tf` messages, giving
one tree rooted at `ground_link`; the cabin optical frame has no logged
extrinsics and stays disconnected. In Foxglove's 3D panel, choose `ground_link` as the display
frame. `ground_link` is body-fixed: it moves with the device, and no `odom` or
`map` frame is exported.

calibrationd only updates the calibration while `carState` reports more than
15 mph driving straight; otherwise it keeps its startup values (the saved
`CalibrationParams` from an earlier drive if present, else a zero rotation and
1.22 m). A route without real car speed therefore carries one of three things:
- a placeholder, on notCar routes (below);
- no usable calibration, in which case `ground-link` fails unless
  `--allow-uncalibrated-overlays` accepts the startup values;
- on a device calibrated during an earlier drive, that drive's saved values,
  reported as `calibrated` and exported by default (basis `calibrationd`). They
  describe the earlier mounting.

On notCar routes (see below), the rotation is the identity placeholder and the
height usually keeps calibrationd's startup value (saved `CalibrationParams` if
present, otherwise 1.22 m). The sample comma four route never met the speed
condition (its `carState.vEgo` was 0), so every update carried 1.22 m. On that
route, the model put the road
1.23-1.34 m below the camera (per-segment medians of the inner lane lines 10 m
ahead; camera odometry's road estimate gave 1.33-1.40 m). There, `ground_link`
sits up to about 0.2 m above the model's road surface; lane lines 5-15 m ahead
were a median 7 cm below it in the first two segments.

## Camera calibration info

Opt in with `--camera-info` to add calibration for **every selected camera
stream**, including `cabin-camera`. Without it, camera topics are exported as before.

- `/camera/<name>/camera_info`: `sensor_msgs/msg/CameraInfo` for every decoded
  image, with the image's exact header (start-of-frame readout stamp
  `EncodeIndex.timestampSof` and optical `frame_id`), as ROS image/camera_info
  consumers expect.
- `/tf`: when a road camera is selected, the logged `extrinsicsCalibration` as
  transforms from `device_flu` to `model_flu` and the selected road optical
  frames. These are the same transforms, under the same calibrated-only rule,
  that `--camera-overlays` adds (see below). Unlike overlay mode, a route without
  usable calibration still exports its intrinsics (unless `ground-link` is also
  selected, since it requires `/tf`), and a warning reports that the optical
  frames are not connected to `device_flu`. Add `--allow-uncalibrated-overlays`
  to also accept valid `uncalibrated` updates; it requires a selected road camera
  or `ground-link`. The cabin camera has no logged extrinsics and stays
  disconnected.

**openpilot logs contain no lens calibration.** The only camera calibration in
an rlog is extrinsic: `extrinsicsCalibration` (mounting rotation `rpyCalib`,
wide-camera rotation `wideFromDeviceEuler`, camera `height`). Intrinsics are
therefore openpilot's nominal per-hardware pinhole values (`DEVICE_CAMERAS`),
selected by the logged `deviceState.deviceType` and `<camera>CameraState.sensor`.
These are the values modeld uses to warp frames and the onroad UI uses to draw,
not a calibration of the individual device:

- `K` uses the profile's focal length for both axes and openpilot's nominal
  principal point (width/2, height/2), not a measured optical center. A comma
  four (`mici`, `os04c10`) gives the narrow road camera 1344x760 with f=1141.5 px,
  and the wide road and cabin cameras 1344x760 with f=425.25 px.
- `distortion_model` is `plumb_bob` with five zero coefficients: openpilot's
  distortion-free pinhole approximation. ROS and Foxglove tools will treat the
  images as already undistorted (`image_proc` rectification is a no-op, and
  Foxglove's **Rectify** leaves the pixels unchanged, although it must stay on
  to draw 3D overlays), while the real lenses are distorted, strongly so for the
  fisheye wide-road and cabin cameras. Alignment degrades toward the image edges.
- `R` is identity, `P` is `[K|0]`, binning is 0, and the ROI is the full image.
- On a sample comma four route, the images implied a narrow/wide focal-length
  ratio a few percent below the nominal profiles. Absolute per-device accuracy
  is unmeasured.

The profile used for each camera, the hardware key, and these caveats are
recorded in the `camera_intrinsics` metadata record. Profiles reached through
openpilot's alias entries (for example `tici`/`unknown`, assumed AR0231) are
marked as assumed hardware. Unknown hardware, a camera missing from the profile,
absent hardware metadata, and decoded-size mismatches fail explicitly.

### Placeholder calibration on notCar routes

Routes whose logged `carParams.notCar` is set carry a placeholder extrinsic
calibration. In opendbc, `notCar` marks non-car robotics platforms; stock
openpilot sets it only for the comma body. A stock device on an unrecognized car
logs brand `mock` with `notCar=false` (dashcam mode), keeps normal calibrationd
behavior, and is labeled `calibrationd`. The sample comma four route used during
development logs a synthetic notCar CarParams (brand `mock`, no fingerprint)
that stock card does not produce, as a bench setup might.

With `notCar` set, calibrationd publishes `calStatus=calibrated`, `calPerc=100`,
`validBlocks=5`, and `rpyCalib=[0,0,0]` whatever it measures.
`wideFromDeviceEuler` and `height` keep their startup values (saved
`CalibrationParams` if present, otherwise `[0,0,0]` and 1.22 m), unless the
platform reports more than 15 mph while driving straight, when calibrationd
still updates them. modeld builds its input warp from `rpyCalib` and the nominal
intrinsics only, and the onroad UI also applies `wideFromDeviceEuler` to the
wide camera, as the exported wide optical frame does. So `model_flu` matches
openpilot's model frame and the overlays match the UI, but the transforms **do
not describe how the device is mounted**.

Each accepted update is counted in the `camera_calibration_basis` metadata
record as `notcar_placeholder`, `calibrationd`, or `unknown`, from the segment's
logged `carParams` and falling back to the `CarParams` entry in `initData`.
Placeholder updates also print a `PLACEHOLDER camera calibration` warning.
calibrationd reads CarParams only once at startup, so the label is a proxy.

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
modified, and exports that use neither `--camera-overlays` nor `--camera-info`
retain their original camera behavior.
Overlay mode adds:

- `/camera/narrowRoad/camera_info` and/or `/camera/wideRoad/camera_info`:
  `sensor_msgs/msg/CameraInfo`, at every image's capture timestamp and resolution,
  as described in [Camera calibration info](#camera-calibration-info). Add
  `--camera-info` to also get CameraInfo for a selected cabin camera.
- `/tf`: `tf2_msgs/msg/TFMessage`, with `device_flu` as the parent of `model_flu`
  and the selected road optical frames. Model and wide-camera rotations come
  from valid, calibrated `extrinsicsCalibration.rpyCalib` and
  `wideFromDeviceEuler`, with the ROS FLU/optical axis conversions applied.
- A `visualization_msgs/msg/Marker` topic named `<path topic>/marker` for each
  selected lane line, road edge, or predicted path. The original `nav_msgs/Path`
  topics remain available.

In Foxglove, add one **Image** panel per road camera. Choose its
`/camera/<name>/image_raw/compressed` topic and corresponding
`/camera/<name>/camera_info` under **Calibration**, then enable **Rectify**
(required for 3D topics; with zero distortion it does not change the pixels).
Under **Topics**, enable `/model/lane_lines/line_<index>/marker`, any selected
road-edge/path markers, and `/model/leads`. Keep **Sync timestamps** off:
images use start-of-frame (readout start) stamps (`timestampSof`), while model
geometry uses the logged end-of-frame stamp (`timestampEof`, start of frame
plus the sensor readout time, 11 ms on os04c10). `/tf` supplies the connection to the optical frame;
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

modeld rebuilds its input warp from every `extrinsicsCalibration` message
whatever its status, while the exported transforms use only calibrated updates,
like the onroad UI. During calibration warm-up or invalid periods, model
geometry can therefore be in a frame that has no exported transform. Transforms
follow the calibration rate (about 4 Hz) and are never backdated, so tools that
do not extrapolate TF (tf2, rviz) cannot place images stamped before the first
or after the last accepted update. The tail can be long if calibration later
becomes invalid or recalibrating.

`/model/path` is the device's own predicted trajectory at camera height
(z about 0 m), while lane lines and road edges sit at road level. openpilot's UI
lowers the path by `extrinsicsCalibration.height` before drawing it; the export
does not, so the path marker projects near the horizon. On notCar routes that
height is a placeholder anyway.

### Explicitly allowing uncalibrated overlays

Some recordings never complete calibration. To inspect approximate overlays
using their logged rotations, explicitly add `--allow-uncalibrated-overlays`:

```sh
uv run python -m openpilot.tools.mcap.convert \
  '<dongle-id>/<route-id>' --output route-overlays-uncalibrated.mcap \
  --topics narrow-road-camera wide-road-camera lane-lines lead-vehicles \
  --camera-overlays --allow-uncalibrated-overlays
```

This requires `--camera-overlays`, `--camera-info` with a selected road camera, or the `ground-link` signal, and additionally accepts **valid events**
whose calibration status is `uncalibrated`. Invalid events and `invalid` or
`recalibrating` statuses remain excluded. Missing, malformed, or nonfinite
rotation vectors still fail; no zero-angle fallback or calibration from
another route is substituted.

These overlays are **uncalibrated and approximate; alignment is not guaranteed**.
An aggregate warning reports accepted uncalibrated updates. MCAP metadata marks
`camera_calibration_policy=allow_uncalibrated`, prefixes `camera_projection`
with `APPROXIMATE` when road-camera transforms are exported, appends an
`APPROXIMATE` note to `ground_frame` when `ground-link` is selected, and records
accepted status counts in `camera_calibration_counts`. Accepted uncalibrated updates are not mislabeled
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

## GNSS fix, velocity and speed

All three GNSS groups read `gpsLocationExternal`, which `ubloxd` fills from the
u-blox receiver's UBX-NAV-PVT solution, and share its log timestamp.

- `/gps/fix` (`NavSatFix`, frame `gps`): WGS84 latitude, longitude and
  ellipsoidal altitude; status `STATUS_FIX` (0) with `hasFix`, otherwise
  `STATUS_NO_FIX` (-1). Constellation service stays unknown. The covariance is
  described below.
- `/gps/velocity` (`Vector3Stamped`, frame `gps_enu`): the receiver's `vNED`
  velocity reordered to east, north, up (x = vE, y = vN, z = -vD), in m/s. It is
  linear only, not a twist with made-up angular velocity.
- `/gps/speed` (`Float64`, m/s): the magnitude of the same velocity,
  sqrt(vN² + vE² + vD²), so it always equals the length of the `/gps/velocity`
  vector. It includes the vertical component. The horizontal ground speed is
  sqrt(x² + y²) of `/gps/velocity`; it agrees with u-blox's own `speed`
  (gSpeed) field to about 1 mm/s. While moving, the two speeds are practically
  equal: on route `0f0b77d795033efd/00000006--1dac2daf26` their medians were
  10.283 and 10.281 m/s. When nearly stationary, vertical velocity noise makes
  the 3D speed read higher. In that route's first three segments (speeds below
  about 1 m/s), vD ranged from -0.07 to 0.11 m/s (5th to 95th percentile), and
  the 3D speed exceeded the ground speed by a median of 0.02 m/s and at most
  0.74 m/s. `std_msgs/Float64` has no header, so the sample's MCAP log time is
  its timestamp. It is identical to the `/gps/velocity` header stamp.
- Velocity and speed are published only for fixes (`hasFix`); other samples
  are counted in `skipped_samples` as `gps-velocity: no GNSS fix` and
  `gps-speed: no GNSS fix`. The receiver's speed and heading accuracies
  (`speedAccuracy`, `bearingAccuracyDeg`) are not exported, because neither
  message type has a field for them.

### How the fix covariance is built

`/gps/fix` `position_covariance` comes from the receiver's own accuracy
estimates; the converter does not invent or tune any numbers:

1. **Receiver.** Every UBX-NAV-PVT epoch, the u-blox navigation filter reports
   its horizontal and vertical accuracy estimates, `hAcc` and `vAcc` in
   millimetres, and a `gnssFixOK` flag.
2. **openpilot.** `ubloxd` logs them in metres on `gpsLocationExternal`:
   `horizontalAccuracy = hAcc × 0.001` and `verticalAccuracy = vAcc × 0.001`. It
   sets `hasFix` from `gnssFixOK` and `source` to `ublox`.
3. **Converter.** For a sample with `hasFix` and `source` `ublox`, take
   σh = `horizontalAccuracy` and σv = `verticalAccuracy` as 1-sigma standard
   deviations and write the row-major 3×3 matrix in m², in NavSatFix's east,
   north, up order:

   ```
   position_covariance = [ σh²  0    0
                           0    σh²  0
                           0    0    σv² ]
   position_covariance_type = COVARIANCE_TYPE_APPROXIMATED (1)
   ```

4. **Otherwise.** A sample without a fix, or from any other source, gets an
   all-zero matrix with `COVARIANCE_TYPE_UNKNOWN` (0). A u-blox fix whose
   accuracy is non-finite, zero or negative stops the export instead of being
   written.

Example from route `0f0b77d795033efd/00000006--1dac2daf26`: 9,675 fixes got
`APPROXIMATED` and the 37 samples without a fix got `UNKNOWN`. The median
σh = 0.935 m gives 0.874 m² on east and on north, and the median σv = 1.611 m
gives 2.595 m² up. The 95th percentiles were 2.04 m and 3.92 m. The largest
were 88 m and 61 m: degraded estimates pass through unchanged, not clipped.

Why the type is `APPROXIMATED` and not `DIAGONAL_KNOWN`:

- **Confidence level.** u-blox documents hAcc/vAcc only as "accuracy
  estimates", without a confidence level. Like common u-blox ROS drivers, the
  converter treats them as 1-sigma per axis. That is exact if hAcc is a
  per-axis sigma, and conservative (about 2-2.3x too large in variance) if it
  is a radial 68% or DRMS figure.
- **One horizontal number.** hAcc has no east/north split or correlation, so
  both horizontal axes get σh² and the cross terms are zero. Foxglove's Map
  panel, which draws the 1-sigma ellipse, shows a circle of radius σh.
- **Self-assessment.** The values are the receiver filter's own estimate, not a
  verified error bound. In a 2D fix, vAcc describes the held altitude.

Further limits:

- Devices without a u-blox publish GNSS on `gpsLocation` (qcomgpsd, which
  stores VDOP there, not metres), and `gps-fix` does not read it. Routes
  recorded before `hasFix` existed (March 2024) also keep an unknown
  covariance; `flags` is deliberately not used as a fallback, because simulator
  logs set it with made-up accuracies.
- Covariance is not adjusted for GNSS timing. The receiver's covariance applies
  at its fix epoch, while headers carry the log time, which on sampled routes is
  about 0.1 s later for fixes (per-segment medians 94-123 ms). At speed, the
  resulting along-track error can exceed hAcc, so fusion users should account
  for this latency.

The MCAP records the method in the `openpilot` metadata record
(`gps_fix_covariance`), and the number of fixes per covariance type in
`gps_fix_covariance_counts`.

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
  are not falsely presented as quaternion-error covariance. Known limitation:
  openpilot logs `cameraOdometry` in its calibrated (model) frame, despite the
  schema's "device frame" comment, so `/motion/camera_velocity` is exactly
  `device_flu` only while the logged `rpyCalib` is zero (for example on notCar
  routes). Otherwise it is rotated by the mounting calibration.
- GNSS fix, velocity, speed and fix covariance conventions are described in
  [GNSS fix, velocity and speed](#gnss-fix-velocity-and-speed).
- Predicted paths, lane lines, and road edges use `model_flu`, with right/down
  axes flipped. With `ground-link`, or with `--camera-overlays`/`--camera-info` and a selected road camera, logged
  calibration connects this projection frame to `device_flu`. `ground-link` adds `ground_link` as the root (the
  parent of `device_flu`, `height` below it), and the camera flags add the selected road optical frames, as
  described above.
  Path poses have identity orientation: they represent **position geometry**,
  not measured attitude. Predicted-position pose stamps include the source's
  future time offsets; lane/edge geometry retains its capture time.
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
Camera-info tests check per-camera intrinsics (including the cabin camera),
header synchronization across segments, both camera flags together, no
calibration outputs in default exports and no cabin CameraInfo in overlay-only
exports, missing hardware, and notCar placeholder labeling. Ground-link tests
check that a road point in openpilot's calibrated frame lands on the
`ground_link` plane, that the transform shares a tree with the camera frames,
and that missing heights or calibrations fail. GNSS tests check the u-blox covariance mapping,
unknown covariance without a fix or for other sources, rejected malformed
accuracies, and that `/gps/speed` equals the decoded `/gps/velocity` magnitude
with the same timestamp and is skipped without a fix. Tests also pin the exact set of exported signal groups, so no
vehicle (CAN) state, engagement, or planning signal is exported.
