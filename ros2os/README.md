# ros2os: a ROS 2 native OS image for the comma four

ros2os turns a comma four (`mici`, also the comma 3X `tizi`) into a general purpose
ROS 2 robot computer. It boots like a normal Linux distribution (Ubuntu 24.04 from
AGNOS, systemd, apt, ssh, NetworkManager), runs **ROS 2 Jazzy by design**, and starts
drivers for every on-board sensor at boot. Turn it on, join its network and
`ros2 topic list` shows the cameras, IMU, GNSS, CAN bus, microphone and device health.

It does **not** run openpilot or any driving function. It is installed into the
*other* A/B slot, so the openpilot setup you have today stays on the device and
`ros2os-rollback` boots back into it.

> **Status: untested on hardware.** Everything here was written against the comma four
> code paths in this repository and unit tested off-device (see [Testing](#testing)),
> but no image has been built and booted on a device yet. Expect to debug the first
> boot over ssh, and keep the rollback path in mind.

## What runs on the device

| Hardware | Driver (package) | Topics | Notes |
|---|---|---|---|
| Road, wide-road and driver cameras (OX03C10 / OS04C10 on the Spectra ISP) | `comma_camera` (C++) | `/camera/{narrowRoad,wideRoad,cabin}/image_raw`, `.../image_raw/compressed`, `.../camera_info` | 20 Hz. openpilot's `camerad` drives the ISP; the node maps its frames zero-copy over VisionIPC. Raw images in `bgr8`/`rgb8`/`mono8`/`nv12`, JPEG encoded straight from YUV. Work is only done for topics with subscribers. |
| IMU (LSM6DS3TR-C, I2C) | `comma_imu` | `/imu/data_raw`, `/imu/temperature` | 104 Hz (up to 1.66 kHz), interrupt driven, stamped with the kernel timestamp of the data-ready IRQ, rotated into the device frame. |
| GNSS (u-blox over UART or the Quectel modem's GNSS) | `comma_gnss` | `/gps/fix`, `/gps/velocity`, `/gps/twist`, `/gps/time_reference` | Backend picked like openpilot does. u-blox: UBX NAV-PVT at 10 Hz with pigeond's configuration. Quectel: NMEA at 1 Hz. Can set the system clock (the device has no RTC battery). |
| CAN / CAN FD (internal panda, SPI) | `comma_panda` | `/can/rx`, `/can/tx`, `/panda/state` | 3 buses, 100 Hz batches. Flashes the bundled panda firmware when it differs. Listen-only (`noOutput`) by default. |
| Fan, thermal zones, power, IR LEDs, display | `comma_hardware` | `/hardware/state`, `/hardware/fan_speed_cmd`, `/diagnostics` | Fan control identical to openpilot's. Powers off before draining a parked car's battery. |
| Microphone | `comma_audio` | `/audio/raw`, `/audio/sound_pressure_db` | 16 kHz mono, A-weighted level like openpilot's micd. |
| Sensor frames | `comma_description` | `/tf_static` | `device_flu` → `imu_link`, `gps`, `*_optical`, ... |
| Remote access | `foxglove_bridge` | `ws://comma-<serial>.local:8765` | Open it in Foxglove to see every topic. |

Also on the image: LTE via openpilot's modem manager (`ros2os-modem`), mDNS hostname
`comma-<serial>.local`, an optional rosbag2 recorder with a space-keeping deleter,
Fast DDS and Cyclone DDS (plus Zenoh when its package is available), `colcon` and `rosdep` for building on the device.

## Architecture

```
partition         contents                                             source
---------------   ---------------------------------------------------  ---------------------------------------
xbl, abl, aop,    bootloaders, firmware                                AGNOS release pinned in
devcfg, ...                                                            openpilot/system/hardware/comma/agnos.json
boot              Linux kernel + device tree (all hardware drivers)    same AGNOS release, unchanged
system            AGNOS root filesystem (kernel modules, firmware,     rebuilt by ros2os/build.sh
                  vendor libraries, NetworkManager, ssh)
                  + ROS 2 Jazzy + /opt/ros2os (drivers, camerad)
userdata (/data)  Wi-Fi, ssh keys, openpilot, ros2os settings,         kept as is
                  your workspace, recordings
```

Boot sequence (systemd):

```
ros2os-hwinit      CPU/GPU/IRQ tuning, fan supply on, device permissions (what openpilot's hardwared does)
ros2os-camerad     openpilot camerad: Spectra ISP + image sensors -> VisionIPC buffers
ros2os             ros2 launch comma_bringup comma.launch.py  (all drivers respawn on failure)
ros2os-modem       LTE (only if the modem is present)
ros2os-record      rosbag2 recorder (disabled by default)
```

AGNOS' own launcher (the unit that runs `/usr/comma/comma.sh`, which starts openpilot or its installer) is masked.
The cameras are the one place where an openpilot process stays in the loop: the Spectra
ISP has no upstream V4L2 driver, and camerad is the only open implementation (sensor
programming, ISP pipeline, auto exposure). The ROS node reads its output buffers without
copying them through any other IPC.

## Topics, frames and conventions

| Topic | Type | Rate |
|---|---|---|
| `/camera/<cam>/image_raw` | `sensor_msgs/Image` | 20 Hz |
| `/camera/<cam>/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG) | 20 Hz / `jpeg_rate_divider` |
| `/camera/<cam>/camera_info` | `sensor_msgs/CameraInfo` | 20 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | 104 Hz |
| `/imu/temperature` | `sensor_msgs/Temperature` | 1 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | 10 Hz (u-blox) / 1 Hz (Quectel) |
| `/gps/velocity` | `geometry_msgs/Vector3Stamped` (ENU) | with the fix |
| `/gps/twist` | `geometry_msgs/TwistWithCovarianceStamped` (ENU) | with the fix |
| `/gps/time_reference` | `sensor_msgs/TimeReference` | with the fix |
| `/can/rx`, `/can/tx` | `comma_msgs/CanFrameArray` | 100 Hz batches |
| `/panda/state` | `comma_msgs/PandaState` | 2 Hz |
| `/hardware/state` | `comma_msgs/DeviceState` | 2 Hz |
| `/hardware/fan_speed_cmd`, `/hardware/ir_power_cmd` | `std_msgs/UInt8` [%] | 2 Hz / on demand |
| `/audio/raw` | `comma_msgs/AudioChunk` | 20 Hz (50 ms chunks) |
| `/audio/sound_pressure_db` | `std_msgs/Float32` | 10 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1-2 Hz |

`<cam>` is `narrowRoad`, `wideRoad` or `cabin`. Topic names, optical frame names, the JPEG
format string and the camera intrinsics match the [rlog → MCAP exporter](../openpilot/tools/mcap/README.md),
so tools work the same on live data and on converted openpilot logs.

Frames (all origins coincide; like openpilot, the few-centimetre offsets between sensors are not surveyed):

- `device_flu`: device body, x forward (out of the road cameras), y left, z up (REP 103).
- `imu_link`: same axes as `device_flu`. The LSM6DS3 chip axes are rotated into it:
  forward = −z, left = −x, up = +y, derived from openpilot's sensord/locationd conventions
  (an upright device reads +9.81 m/s² on z). Set `axes: sensor` for raw chip axes.
  This differs from the exporter's `imu_<sensor>` frames, which keep openpilot's logged axes.
- `narrow_road_optical`, `wide_road_optical`, `cabin_optical`: REP 103 optical frames. The cabin camera faces backwards.
- `gps`: NavSatFix frame; `gps_enu`: ENU-aligned frame for GNSS velocity.

Stamps: camera images use the start-of-frame time (`stamp: eof` for end of frame), the IMU
the data-ready interrupt, CAN batches the time they were read. All are converted to ROS
(system) time. Until NTP or GNSS sets the clock after boot, stamps start from the image build date.

Camera intrinsics are openpilot's nominal pinhole values (principal point at the centre,
no distortion). The wide-road and cabin cameras are strong fisheyes; calibrate them
yourself if you need undistortion and set `<cam>.focal_length` or use your own CameraInfo.

## Building the image

Requirements: Linux with Docker (BuildKit), ~40 GB of disk, the repository with submodules
(`git submodule update --init --recursive`). On x86_64, enable arm64 emulation once:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Build:

```bash
ros2os/build.sh
```

It downloads the AGNOS release pinned by this openpilot revision, checks both manifest
hashes, imports its root filesystem as an arm64 base image, installs ROS 2, builds
camerad and the panda firmware with openpilot's own SCons build, builds the colcon workspace
(and runs its tests), then writes an ext4 image with the same size, ext4 features, label
and UUID as the AGNOS system image. Expect about an hour on an x86_64 machine (emulated)
and much less on an arm64 machine. Outputs in `ros2os/build/`:

| File | Use |
|---|---|
| `system-<hash>.img.xz` | the system partition image (sparse, xz) |
| `ros2os-manifest.json` | install manifest in AGNOS format: AGNOS firmware + boot entries unchanged, system replaced |
| `system.img` | raw image, for flashing with EDL tools |

Options (environment variables):

- `ROS2OS_BASE_URL=https://example.com/ros2os`: put a download URL in the manifest, so devices can install from a web server.
- `EXTRA_APT_PACKAGES="ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox"`: add packages to the image.
- `RUN_TESTS=0`: skip the unit tests during the build.
- `--clean`: start from scratch. Without it, finished steps (AGNOS download, base image) are reused.

Image size: the system image is the AGNOS partition image size (4.4 GiB). The build prints
the free space left. If extra packages do not fit, install them into `/data` instead (for
example build them in your workspace) or trim the package list in `builder/scripts/install_ros.sh`.

## Installing

On a device that runs AGNOS (openpilot installed, ssh enabled with your GitHub key):

```bash
scp ros2os/build/ros2os-manifest.json ros2os/build/system-*.img.xz ros2os/installer/install.py comma@<device>:/data/tmp/
ssh comma@<device>
cd /data/tmp && python3 install.py ros2os-manifest.json --reboot
```

`install.py` writes every partition of the manifest into the inactive slot, verifies the
hashes the same way openpilot's AGNOS updater does, copies your ssh keys to
`/data/ros2os/authorized_keys` and switches the active slot. The AGNOS firmware and boot
partitions come straight from comma's servers (their URLs are in the manifest); only the
system image is local. With `ROS2OS_BASE_URL` set you can install from a URL instead:
`python3 install.py https://example.com/ros2os/ros2os-manifest.json`.

After the reboot: `ssh comma@comma-<serial>.local` and run `ros2os-status`.

**Rolling back:** `ros2os-rollback` (or `sudo abctl --set_active <0|1> && sudo reboot`)
boots the other slot, i.e. the AGNOS + openpilot you had. Be aware that openpilot's
updater installs AGNOS updates into the *inactive* slot, which would overwrite ros2os;
re-run the installer afterwards if you switch back and forth.

**Recovery:** if a device does not boot at all, restore stock AGNOS with comma's EDL flasher
(flash.comma.ai) and install again. `system.img` can also be written to `system_a`/`system_b`
with any Qualcomm EDL/fastboot tool, together with the AGNOS boot and firmware images of the same slot.

## Using it

```bash
ssh comma@comma-<serial>.local
ros2 topic list
ros2 topic hz /imu/data_raw
ros2 topic echo /gps/fix --once
journalctl -u ros2os -f            # driver logs
sudo systemctl restart ros2os      # restart all drivers
```

From another machine on the same network (same `ROS_DOMAIN_ID` and RMW), the topics show up
directly, or connect Foxglove to `ws://comma-<serial>.local:8765`.

Everything you customise lives in `/data/ros2os/` and survives image updates:

| File | Effect |
|---|---|
| `/data/ros2os/params.yaml` | overrides [`comma.yaml`](ros2_ws/src/comma_bringup/config/comma.yaml) (same structure) |
| `/data/ros2os/ros2os.env` | overrides [`ros2os.env`](builder/rootfs/etc/ros2os/ros2os.env): `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, launch arguments, recorder topics |
| `/data/ros2os/launch.py` | replaces the default launch file; include `comma_bringup/launch/comma.launch.py` to keep the drivers and add your nodes |
| `/data/ros2_ws` | your colcon workspace: `ros2os-ws init`, put packages in `src/`, `ros2os-ws build`. It is sourced automatically |

Examples:

```bash
# disable the microphone and use Cyclone DDS
echo 'ROS2OS_LAUNCH_ARGS="audio:=false"' >> /data/ros2os/ros2os.env
echo 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /data/ros2os/ros2os.env
sudo systemctl restart ros2os

# record everything (MCAP + zstd, 1 GB splits) to /data/media/ros2_bags
sudo systemctl enable --now ros2os-record
```

The root filesystem is the read-only system image, as on AGNOS. `ros2os-rw` makes it
writable until the next reboot (for a quick `apt install`), but changes are lost with
the next image update; bake packages into the image with `EXTRA_APT_PACKAGES` or keep
them in `/data`.

Networking: DDS only announces the network interfaces that exist when a node starts.
When Wi-Fi connects after boot, a NetworkManager hook restarts the drivers (at most every
10 minutes) so other machines can discover them; set `ROS2OS_RESTART_ON_NETWORK=0` to turn that off.

### CAN

```bash
ros2 topic echo /can/rx --field frames
ros2 param set /panda safety_mode elm327     # diagnostics (UDS/OBD) only
```

The panda enforces its safety model in firmware: `noOutput` (default) receives and
acknowledges but never transmits, `silent` is a pure bus monitor, `elm327` allows
diagnostic requests, and the opendbc car models allow exactly what openpilot would send
for that car. `allOutput` puts anything on the bus and is for bench work only: sending
arbitrary frames on a vehicle bus can disable safety systems.

## Safety notes

- This is a development platform. It contains no driver monitoring, no driving logic and
  none of openpilot's safety checks beyond the panda firmware's safety model.
- Power protection is on by default when the device is harnessed in a car: after 30 hours
  parked, or when the car battery stays below 11.8 V, the device powers off (`power_protection`, `max_offroad_hours`).
- The fan follows openpilot's controller. `fan_policy: openpilot` caps the fan at 30 % while the
  ignition is off, as openpilot does; the default `performance` does not, because ROS workloads
  run regardless of the ignition.

## Testing

The hardware-facing logic of every driver is plain Python or C++ separated from the ROS
wrappers, so it is tested without a device:

```bash
cd ros2os/ros2_ws/src
for p in comma_imu comma_gnss comma_panda comma_hardware comma_audio; do
  PYTHONPATH=comma_hal:$p python3 -m pytest -q $p/test
done
python3 -m pytest -q ../../installer/test_install.py
```

What these cover: LSM6DS3 register configuration, scaling and axis mapping (checked against
openpilot's sensord→locationd chain); NMEA and UBX decoding, and the u-blox configuration
frames byte-for-byte against openpilot's pigeond; CAN frame conversion and panda health mapping;
the fan controller (identical output to openpilot's over 20,000 random steps); A-weighted sound
pressure (identical to micd); NV12 conversion and JPEG encoding (`comma_camera` gtests); the installer's
sparse-image flashing (against real `img2simg` output and openpilot's `agnos.py`); the manifest tooling.
The disk image scripts (`extract_agnos.sh`, `make_image.sh`) were run end to end on a synthetic
AGNOS image, and the result flashed and verified with the installer.

Not verified yet: the Docker image build itself (it needs packages.ros.org and comma's AGNOS
download server), compiling `comma_camera`'s node against msgq, and everything on a real device.

## Layout

```
ros2os/
  build.sh                    host entry point
  builder/
    Dockerfile                arm64 image: AGNOS + ROS 2 + drivers
    Dockerfile.tools          host-arch helper for the disk image steps
    scripts/                  extract_agnos, install_ros, build_openpilot, build_workspace, finalize, make_image, manifest
    rootfs/                   files copied into the image (systemd units, env, helper commands)
  installer/install.py        on-device installer (stdlib only)
  ros2_ws/src/
    comma_msgs                CanFrame(Array), PandaState, CanBusHealth, DeviceState, AudioChunk
    comma_hal                 I2C, GPIO, thermal zones, device identity, clocks (no ROS)
    comma_imu  comma_gnss  comma_panda  comma_hardware  comma_audio  comma_camera
    comma_description         URDF / xacro macro of the sensor frames
    comma_bringup             launch file + default parameters
```
