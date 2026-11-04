# ROS 2 and Gazebo simulation

The included rig targets Ubuntu 22.04, ROS 2 Humble, and Gazebo Classic 11.
It uses Classic plugins, not the newer `ros_gz` interface. Gazebo Classic
[reached end of life in January 2025](https://classic.gazebosim.org/).
This guide preserves that legacy environment; it does not claim support for
newer Gazebo releases.

## 1. Install ROS and simulation dependencies

Follow the official [ROS 2 Humble Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
to configure the ROS apt repository and install Humble. After that setup:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions python3-venv python3-pip \
  libgl1 libglib2.0-0
```

If the Classic packages are no longer available from your configured apt
sources, use an existing Ubuntu 22.04/Humble environment. Installing a newer
Gazebo does not provide the Classic plugins used by these world files.

## 2. Prepare the Python environment

Run from the root of this checkout in a fresh Bash terminal. Deactivate the
offline `.venv` first, if it is active. The ROS launch files execute system
`python3`, so their dependencies must be visible to that interpreter.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv-ros
.venv-ros/bin/python -m pip install -c constraints-tested.txt .
export PYTHONPATH="$PWD/.venv-ros/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export MULTIROI_DIR="$PWD"
python3 -c 'import rclpy, cv2, numpy, scipy, sklearn'
```

Keep `.venv-ros` inactive. Its installed packages are exposed through
`PYTHONPATH`, while ROS and colcon continue to use system Python 3.10. Repeat
the `source` and `export` commands in each new terminal. This arrangement is
specific to Ubuntu 22.04 and its system Python.

The two required agribot plant models are already bundled at the paths the
launch files expect. Their BSD license is retained. No separate agribot clone
is needed.

## 3. Check the camera before driving

```bash
cd tests/sim_ros2
./run_sim.sh --probe --out /tmp/multiroi-probe
```

Inspect the saved images and check that both flanking crop rows are visible.
The wrapper builds the workspace with colcon before launching. Its completion
message alone does not prove success: the preserved wrapper tolerates launch
timeouts. Inspect the ROS log and confirm that output files were written.

## 4. Run a bounded simulation

```bash
./run_sim.sh --straight 5 --seconds 30 --out /tmp/multiroi-straight
```

For a GUI, use the companion wrapper on a machine with a display:

```bash
./run_sim_gui.sh
./run_sim_gui.sh --auto
```

The first command opens teleoperation. The second enables autonomous control.
Do not run them together. Close one session before starting another.

## Fields and control options

| Option | Purpose |
| --- | --- |
| `--circle` | Circular field, default preset |
| `--straight 5` | Five straight crop rows |
| `--curve 5` | Five curved crop rows |
| `--zigzag 5` | Five zigzag crop rows |
| `--spawn 2` | Start in the second furrow from the left |
| `--rows-change 0` | Disable automatic row changing |
| `--seconds 30` | Bound the run by simulated time |
| `--out /tmp/run-name` | Select a separate log directory |
| `--tof` | Enable the experimental range-sensor command guard |
| `--lookahead` | Enable lookahead corridor memory (launch `lookahead:=true`) |
| `--camera-timeout 1.0` | Command a stop after 1.0 s without camera frames (launch `camera_timeout:=1.0`) |

Non-snapshot row counts generate cached fields under
`~/.cache/crop-row-fields`. Row changing is intended for straight fields and
is enabled automatically where multiple furrows are available. Inspect the
wrapper comments for the full option list.

## Manual launch

From `tests/sim_ros2`, with the environment above configured:

```bash
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 launch mrsim farm_probe.launch.py out_dir:=/tmp/multiroi-probe
ros2 launch mrsim farm.launch.py field:=straight5 max_seconds:=30 log_dir:=/tmp/multiroi-run
```

Run the launch commands separately. Stop the first with Ctrl-C before starting
the second. The probe records images without driving.

## Results and configuration

Logs contain commands, image features, odometry, lane cross-track error, the
control interval, whether the frame produced an observation, the track source
and the map status.
The simulator adapter uses paper morphology and temporal filtering. Lookahead
memory is off by default to keep the historical adapter configuration;
`lookahead:=true` runs the complete design with search-window seeding and the
remembered corridor. The offline video CLI uses different defaults. See
[architecture](architecture.md) before comparing their results.

The node passes the interval between camera image stamps (simulation time) to
the controller. A decode or pipeline failure publishes a zero command for that
frame and the run-end checks (time, lane end, laps) still run. Frames without
crops are not failures: the established track is held. The camera watchdog is
disabled by default (`camera_timeout:=0`); a positive value publishes zero
commands while no frames arrive. Publishing `/reset_rover` also resets the
navigation state. Before the first observed track the node commands zero
velocity.

The ToF guard is an experimental simulator feature. It is not a substitute for
an independent stop mechanism. No hardware driver is included.
