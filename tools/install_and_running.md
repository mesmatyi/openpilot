# openpilot tools

## System Requirements

openpilot is developed and tested on **Ubuntu 20.04**, which is the primary development target aside from the [supported embedded hardware](https://github.com/commaai/openpilot#running-on-a-dedicated-device-in-a-car).


## Native setup on Ubuntu 20.04

**1. Clone openpilot**

NOTE: This repository uses Git LFS for large files. Ensure you have [Git LFS](https://git-lfs.com/) installed and set up before cloning or working with it.


Do a full clone of the whole repostitory

``` bash
git clone --recurse-submodules https://github.com/commaai/openpilot.git
```

**2. Run the setup script**

``` bash
cd openpilot
git lfs pull
tools/ubuntu_setup.sh
```

Activate a shell with the Python dependencies installed:
``` bash
poetry shell
```

**3. Build openpilot**

``` bash
scons -u -j$(nproc)
```


## WSL on Windows

[Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/about) should provide a similar experience to native Ubuntu. [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/compare-versions) specifically has been reported by several users to be a seamless experience.

Follow [these instructions](https://docs.microsoft.com/en-us/windows/wsl/install) to setup the WSL and install the `Ubuntu-20.04` distribution. Once your Ubuntu WSL environment is setup, follow the Linux setup instructions to finish setting up your environment. See [these instructions](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) for running GUI apps.

**NOTE**: If you are running WSL and any GUIs are failing (segfaulting or other strange issues) even after following the steps above, you may need to enable software rendering with `LIBGL_ALWAYS_SOFTWARE=1`, e.g. `LIBGL_ALWAYS_SOFTWARE=1 selfdrive/ui/ui`. -> This is needed for the current setup since the PC uses the Xorg as a GUI alternative. 

## Set Up the Simulator

Due to the lack of Audio support WSL is not able to produce any sound this causes the **soundd** to crash. Due to this behavior we need eliminate **soundd** service from the list of started services. This can be achived by modifiing the following lines: 

*openpilot/system/manager/process_config.py*

```Python
 NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], notcar),
NativeProcess("loggerd", "system/loggerd", ["./loggerd"], logging),
NativeProcess("modeld", "selfdrive/modeld", ["./modeld"], only_onroad),
NativeProcess("sensord", "system/sensord", ["./sensord"], only_onroad, enabled=not PC),
NativeProcess("ui", "selfdrive/ui", ["./ui"], always_run, watchdog_max_dt=(5 if not PC else None)),
#PythonProcess("soundd", "selfdrive.ui.soundd", only_onroad), # comment out this line
NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], only_onroad),
NativeProcess("pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  ```


### Launching openpilot
First, start openpilot. Note that you will either need a [mapbox token](https://docs.mapbox.com/help/getting-started/access-tokens/#how-access-tokens-work) (set with ```export MAPBOX_TOKEN="1234"```), or to disable mapsd with ```export BLOCK=mapsd```\
Before typeing command in new terminal not forget to entry in the openpilot shell by using the following command:
```bash
poetry shell
```


``` bash
# Run locally
export BLOCK=mapsd
LIBGL_ALWAYS_SOFTWARE=1 ./tools/sim/launch_openpilot.sh
```

Thenn launch the MetaDrive simulator in a new terminal

```bash
./run_bridge.py
```
You can use the following commands to control the car manually 

#### Bridge Controls:
- To engage openpilot press 2, then press 1 to increase the speed and 2 to decrease.
- To disengage, press "S" (simulates a user brake)

#### All inputs:

```
| key  |   functionality       |
|------|-----------------------|
|  1   | Cruise Resume / Accel |
|  2   | Cruise Set    / Decel |
|  3   | Cruise Cancel         |
|  r   | Reset Simulation      |
|  i   | Toggle Ignition       |
|  q   | Exit all              |
| wasd | Control manually      |
```

Once all done you should see something like this in the GUI display: \
![alt text](image.png)


### Debug Options for the Simulator

In Openpilot Plotjugger is used to visualize for most of the different kind of datastreams, this tool is also compatibile with the Simulator we can subscribe and plot all the data found in Cabana (Pub Sub messaging service, it's like an alternative to ROS)

#### Installation

Once you've [set up the openpilot environment](../README.md), this command will download PlotJuggler and install our plugins:

`cd tools/plotjuggler && ./juggle.py --install`

## Usage

```
$ ./juggle.py -h
usage: juggle.py [-h] [--demo] [--can] [--stream] [--layout [LAYOUT]] [--install] [--dbc DBC]
                 [route_or_segment_name] [segment_count]

A helper to run PlotJuggler on openpilot routes

positional arguments:
  route_or_segment_name
                        The route or segment name to plot (cabana share URL accepted) (default: None)
  segment_count         The number of segments to plot (default: None)

optional arguments:
  -h, --help            show this help message and exit
  --demo                Use the demo route instead of providing one (default: False)
  --can                 Parse CAN data (default: False)
  --stream              Start PlotJuggler in streaming mode (default: False)
  --layout [LAYOUT]     Run PlotJuggler with a pre-defined layout (default: None)
  --install             Install or update PlotJuggler + plugins (default: False)
  --dbc DBC             Set the DBC name to load for parsing CAN data. If not set, the DBC will be automatically
                        inferred from the logs. (default: None)

```

We can use it with the Simulator with the following command:\

`./juggle.py --stream`

When using the Simulator the base car is the *Honda Civic 2022* when asked about the CAN dbc file choose this option.



![alt text](image-1.png)