# How to contribute

Our software is open source so you can solve your own problems without needing help from others. And if you solve a problem and are so kind, you can upstream it for the rest of the world to use. Check out our [post about externalization](https://blog.comma.ai/a-2020-theme-externalization/).

Development is coordinated through [Discord](https://discord.comma.ai) and GitHub.

## Experimental Bench Camera Mode (comma four)

This fork has an opt-in **Bench Camera Mode** in Developer settings. It uses the
device's built-in cameras, not `USE_WEBCAM` or a USB webcam. This is **not a real
drive**, a calibration procedure, a vehicle simulator, or a supported way to
control a vehicle. Hardware operation and cloud acceptance require separate
validation; mocked tests are not hardware validation.

### Start, stop, and recovery

* Physically disconnect the device from **all vehicle/harness connections**.
  Use an appropriate standalone power supply, a stable mount, clear ventilation,
  and active cooling. Camera/model operation can draw substantial power and heat.
* With normal openpilot running **offroad**, open Settings → Developer →
  **Bench Camera Mode**, read the recording/location disclosure, and confirm.
  No separate launcher, systemd override, firmware change, or OS upgrade is needed.
* Normal registration, training/terms, storage, boot, thermal, and other startup
  gates still apply. A request does not force them to pass. A rejected request or
  a pending startup reason is shown in the Developer item.
* The persistent red **BENCH MODE — NO VEHICLE CONTROL** banner with **TAP TO STOP**
  appears above all pages, including onroad and settings. Tap it to stop. Starting is
  offroad-only; stopping does not require returning to offroad settings.
* Stop and wait for offroad before doing anything else. On connection, stale
  hardware, adapter failure, or thermal permission loss, the session stops and
  cannot auto-restart. The no-output lock remains until session publishers are
  stopped, the real device state is offroad, and fresh raw panda health is safe.
  If blocked, keep the device disconnected, cool it, and restart normal openpilot
  (or reboot while disconnected). No session auto-enables on manager start/boot.
* Do not connect a vehicle during a session to test the interlock. Hardware safety
  verification must use a controlled fixture, not a car capable of actuation.

### Safety architecture and limits

The existing manager supervises `benchd`; it replaces only `card`'s `carState` and
`carParams` publishers. The real `pandad` remains the only owner of `pandaStates`,
CAN reception and fan/IR peripherals. `hardwared` remains the only owner of
`deviceState`, including all real health, thermal and network measurements.
Only its ignition condition can be supplied by a fresh manager lease **and**
adapter heartbeat. Missing/stale either one denies bench ignition.

Admission requires one fresh, valid, known panda in `noOutput`, no controls
allowed, **raw** ignition line and CAN ignition off, harness `notConnected`, and
zero receive counters on all three CAN buses. Existing `STARTED`, `FAKESEND`,
`USE_WEBCAM`, replay/simulation, `NO_FAN_CONTROL`, `BOARDD_SKIP_FW_CHECK`,
`BOARDD_LOOPBACK`, and `NOBOARD` environments are refused, even if empty.
CAN receive counters are cumulative: previous CAN traffic can conservatively
block entry/exit until a disconnected panda/device restart. Multiple or unknown
pandas are refused. This detects exposed hardware signals, **not an absolute
proof of physical disconnection**: a quiet/unpowered vehicle, unusual adapter,
or faulty harness sensor may be undetectable. Physical disconnection remains
mandatory. Trusted software and working, normally validated panda firmware are
assumed; this is not an interlock against malicious local code.

`pandad` explicitly drops `sendcan` while locked, forces `noOutput` instead of
fingerprinting/ELM327 or vehicle safety configuration, and never sends an engaged
heartbeat. `selfdrived` stays disabled with a latched bench engagement interlock.
Controllers, joystick/maneuver tools, planners, radar, and vehicle parameter,
torque and delay learners are excluded even if their debug toggles were enabled.
The manager joins session publishers before handoff/cleanup; an unsafe exit
cannot fall through to real-ignition control. No persistent `STARTED`/`FAKESEND`
variables are set and no firmware safety logic is changed.

Synthetic CP is passive, `notCar`, `noOutput`, with an explicit
`BENCH CAMERA MODE - NOT A VEHICLE` fingerprint and no invented vehicle geometry.
Only transient `CarParams` is written. Saved driving calibration, CarParams
caches, torque/delay/vehicle parameters, location initial state, and DM handedness
are not overwritten by bench learning. Bench calibration starts from default
transforms, remains uncalibrated at stationary speed, and is not saved; unlike
the usual `notCar` shortcut it does **not** claim calibrated status. Location
estimation starts without the saved driving initial state. Normal car mode must
obtain fresh transient CP/firmware/control readiness after cleanup.

### Real versus synthetic data and model diagnostics

Expected **real** services (when supported by the hardware/configuration):

* `narrowRoadCameraState`, `wideRoadCameraState`, `cabinCameraState`
* `modelV2`, `cameraOdometry`, `driverStateV2`, `driverMonitoringState`
* `accelerometer`, `gyroscope`, `deviceMotion` (this checkout's location output)
* `gpsLocation` / `gpsLocationExternal`: monitor both, but only the selected
  backend is expected. An indoor GPS fix is not guaranteed. This mode never
  invents a fix, position, speed, or GPS message.
* `selfdriveState` is from real `selfdrived` but must remain disabled/inactive.
  `carState` is explicitly synthetic stationary data with cruise off and
  `canValid=false`; it is not successful CAN validation.

Use `managerState` to check that the allowed daemons are actually running.
Use `messaging.SubMaster` to observe the above streams, checking **seen, alive,
valid, frequency and timestamps separately**. Do not start another publisher
(including `uiview.py`) to fill missing outputs.

For missing `modelV2`/`cameraOdometry`, inspect modeld's process state and
`/data/log` logs: distinguish model asset/loading/backend errors (including
Chestnut status on configurations that use it), waiting for transient CarParams,
VisionIPC connection/frame synchronization, and missing extrinsics calibration.
`calibrationd` depends on camera odometry and publishes initial uncalibrated
transforms while starting; bench does not bypass model loading errors or mark
health/model outputs successful. Default transforms allow experimental inference
but are not a valid driving calibration. Missing `deviceMotion` additionally
requires checking camera odometry and both real IMU streams. Missing
`selfdriveState` requires checking adapter CP/carState and selfdrived startup.
Validity/frequency warnings caused by absent vehicle/control services are not
evidence that they should be faked.

### Recording, uploads, and verification matrix

Normal logger/encoder, deletion, registration, uploader and Athena/connect
communication lifecycles remain available. `DisableLogging` is respected even
with a stale real-car CP and is preserved across synthetic transitions.
Driver-camera recording/lock, metering and other privacy choices are unchanged;
real cabin inference is not a promise that cabin video will be recorded.
Logger initData captures `BenchMode`/status and transient synthetic CP where
normal metadata is supported. No route metadata, engagement or GPS is falsified
to force cloud visibility.

**Recorded bench segments automatically queue all supported recording files for
upload on unmetered networks**: `qlog[.zst]`, `rlog[.zst]`, `qcamera.ts`,
`fcamera.hevc`, `ecamera.hevc`, and `dcamera.hevc` when cabin recording is enabled.
The full rlog contains recorded device/sensor/process/journal messages; ordinary
boot/crash uploads retain their existing handling. This does not collect arbitrary
files from the device filesystem or create cameras/logs that were not recorded.
Normal non-bench routes keep their existing explicit-request behavior for full
rlogs/high-resolution video.

Loggerd marks each bench segment with the directory xattr `user.bench=1` while
locked. Eligibility survives Stop, manager restart and reboot; it does not depend
on the current BenchMode setting. Open/locked segments are never selected.
Existing boot/crash/qlog/qcamera priority is retained, with full bench recordings
following. Full bench transfers wait on metered connections; normal lightweight
upload/metering behavior is unchanged. Keep the device powered, cooled, connected
to unmetered Wi-Fi and registered until the backlog drains. Storage-pressure
deletion remains enabled, so files can still be deleted before upload if storage
runs low. Do not assume offline recordings are retained indefinitely.

For bench recordings, only HTTP 200/201 transfer responses mark nonempty files
uploaded; authorization failures, HTTP 412 backend refusal and network/server
errors remain pending and retry with normal backoff. `FAKEUPLOAD` cannot mark
bench recordings successfully transferred. A marker creation failure is logged
as `Failed to mark bench segment for full upload`; those files require diagnosis
and an explicit upload request rather than silently assuming full eligibility.
Older recordings made before this change are not retroactively tagged.
Local files, an upload flag, or a summary's `user.upload=1` do **not** prove backend
acceptance. Connect processing/listing and actual backend retention cannot be
guaranteed or verified here, even after a successful HTTP transfer.

Mocked regression coverage is in
`openpilot/system/manager/test/test_bench.py`; run
`python3 -m unittest openpilot.system.manager.test.test_bench`.
It covers startup/reset defaults, confirmation, raw signal rejection, leases,
adapter failure/staleness, stop/normal recovery, process selection/publisher
ownership, and privacy/cache preservation without requiring hardware.
Bench upload selection, retry/status handling and metering tests are in
`openpilot/system/loggerd/tests/test_bench_uploader.py`; run
`python3 -m unittest openpilot.system.loggerd.tests.test_bench_uploader`.
Loggerd integration tests also check segment tagging.

Before relying on this experimental mode, separately test on a disconnected
comma four: confirmation readability and banner/Stop on every page; actual
camera/model/IMU/GPS/location streams; model loading failures; inadequate
calibration; fan/IR/thermal shutdown; registration/storage/startup blockers;
adapter and manager crashes; stale/disconnected panda; fixture-only harness/CAN
detection; repeated start/stop and subsequent normal-car initialization; and
recording disabled/enabled, cabin privacy, metered uploads and explicit requests.
Native panda build, GUI interaction, real inference, power/thermal behavior and
backend upload acceptance cannot be established by the mocked tests alone.

### Getting Started

* Set up your [development environment](/tools/)
* Join our [Discord](https://discord.comma.ai)
* Docs are at https://docs.comma.ai and https://blog.comma.ai

## What contributions are we looking for?

**openpilot's priorities are [safety](SAFETY.md), stability, quality, and features, in that order.**
openpilot is part of comma's mission to *solve self-driving cars while delivering shippable intermediaries*, and all development is towards that goal.

### What gets merged?

The probability of a pull request being merged is a function of its value to the project and the effort it will take us to get it merged.
If a PR offers *some* value but will take lots of time to get merged, it will be closed.
Simple, well-tested bug fixes are the easiest to merge, and new features are the hardest to get merged.

All of these are examples of good PRs:
* typo fix: https://github.com/commaai/openpilot/pull/30678
* removing unused code: https://github.com/commaai/openpilot/pull/30573
* simple car model port: https://github.com/commaai/openpilot/pull/30245
* car brand port: https://github.com/commaai/openpilot/pull/23331

### What doesn't get merged?

* **style changes**: code is art, and it's up to the author to make it beautiful
* **500+ line PRs**: clean it up, break it up into smaller PRs, or both
* **PRs without a clear goal**: every PR must have a singular and clear goal
* **UI design**: we do not have a good review process for this yet
* **New features**: We believe openpilot is mostly feature-complete, and the rest is a matter of refinement and fixing bugs. As a result of this, most feature PRs will be immediately closed, however the beauty of open source is that forks can and do offer features that upstream openpilot doesn't.
* **Negative expected value**: This is a class of PRs that makes an improvement, but the risk or validation costs more than the improvement. The risk can be mitigated by first getting a failing test merged.

### First contribution

[Projects / openpilot bounties](https://github.com/orgs/commaai/projects/26/views/1?pane=info) is the best place to get started and goes in-depth on what's expected when working on a bounty.
There are a lot of bounties that don't require a comma four or a car.

## Pull Requests

Pull requests should be against the master branch.

A good pull request has all of the following:
* a clearly stated purpose
* every line changed directly contributes to the stated purpose
* verification, i.e. how did you test your PR?
* justification
  * if you've optimized something, post benchmarks to prove it's better
  * if you've improved your car's tuning, post before and after plots
* passes the CI tests

## Contributing without Code

* Report bugs in GitHub issues.
* Report driving issues in the `#driving-feedback` Discord channel.
* Consider opting into cabin camera uploads to improve the driver monitoring model.
* Connect your device to Wi-Fi regularly, so that we can pull data for training better driving models.
* Run the `nightly` branch and report issues. This branch is like `master` but it's built just like a release.
* Annotate images in the [comma10k dataset](https://github.com/commaai/comma10k).

## Contributing Training Data

### A guide for forks

In order for your fork's data to be eligible for the training set:
* **Your cereal messaging structs must be [compatible](../openpilot/cereal#custom-forks)**
* **The definitions of all the stock messaging structs must not change**: Do not change how any of the fields are set, including everything from `selfdriveState.enabled` to `carState.steeringAngleDeg`. Instead, create your own structs and set them however you'd like.
* **Do not include cars that are not supported in upstream platforms**: Instead, create new opendbc platforms for cars that you'd like to support outside of upstream, even if it's just a trim-level difference.
