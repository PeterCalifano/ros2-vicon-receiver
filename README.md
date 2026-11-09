# Vicon receiver for ROS 2

**ros2-vicon-receiver** is a ROS 2 package, written in C++, that retrieves data from Vicon software and publishes it on ROS 2 topics. The code is partly derived from a mixture of [Vicon-ROS2](https://github.com/aheuillet/Vicon-ROS2) and [Vicon bridge](https://github.com/ethz-asl/vicon_bridge).

This is NOT an official ROS 2 package, but it has been successfully tested with ROS 2 Jazzy Jalisco on the operating systems Ubuntu 24.04 Noble Numbat (with and without Docker) and macOS 26 Tahoe (with Docker).

---

## Table of Contents

- [Vicon receiver for ROS 2](#vicon-receiver-for-ros-2)
  - [Table of Contents](#table-of-contents)
  - [Requirements](#requirements)
  - [What's new (this branch, after v1.0.0)](#whats-new-this-branch-after-v100)
  - [What's new in v1.0.0 (Jazzy)](#whats-new-in-v100-jazzy)
    - [Migration notes](#migration-notes)
  - [Compatibility](#compatibility)
  - [Package layout \& executable](#package-layout--executable)
  - [Quick start](#quick-start)
    - [Option A — VS Code Dev Container (recommended for development)](#option-a--vs-code-dev-container-recommended-for-development)
    - [Option B — Manual build from source](#option-b--manual-build-from-source)
    - [Option C — Repository scripts (Ubuntu 24.04, also WSL)](#option-c--repository-scripts-ubuntu-2404-also-wsl)
  - [Node interface](#node-interface)
    - [Launch arguments](#launch-arguments)
    - [Node parameters (all required, from the parameters file)](#node-parameters-all-required-from-the-parameters-file)
    - [Published topics](#published-topics)
      - [Naming scheme](#naming-scheme)
      - [Message types](#message-types)
      - [Notes](#notes)
      - [QoS](#qos)
      - [Example topic tree](#example-topic-tree)
  - [Frames \& mapping](#frames--mapping)
    - [TF tree (template + notes)](#tf-tree-template--notes)
      - [Static transform (`map → vicon`)](#static-transform-map--vicon)
      - [Dynamic transforms](#dynamic-transforms)
      - [Tips](#tips)
  - [Example images](#example-images)
    - [TF2 frame tree](#tf2-frame-tree)
    - [RViz: TF and Pose (template)](#rviz-tf-and-pose-template)
  - [Building \& linking details](#building--linking-details)
  - [Troubleshooting / FAQ](#troubleshooting--faq)
  - [License \& attribution](#license--attribution)
  - [Contributing](#contributing)

---

## Requirements

- [Vicon Tracker](https://www.vicon.com/software/tracker/) running on another machine, with DataStream enabled and reachable over the network (hostname/IP).
- Choose one setup path:
  - VS Code Dev Container:
    - Docker Engine ([install guide](https://docs.docker.com/engine/install/))
    - [Visual Studio Code](https://code.visualstudio.com/download) + Dev Containers extension ([Dev Container starting guide](https://code.visualstudio.com/docs/devcontainers/create-dev-container))
  - Manual build (on host):
    - ROS 2 Jazzy Jalisco installed and sourced (at least *ros-jazzy-ros-base* and *ros-dev-tools* packages, [installation guide](https://docs.ros.org/en/jazzy/Installation.html))
    - *rosdep* initialized and updated for managing ROS 2 package dependencies ([installation guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html))
    - common build tools: cmake, gcc, build-essential, python3 (on Ubuntu 24.04, a Python 3 virtual environment is recommended)
    - git package for cloning the repository

> Note: you do not need system-wide Boost or the Vicon DataStream SDK; both are vendored per-architecture inside this repository.

---

## What's new (this branch, after v1.0.0)

- **`ViconFrame` topic** (`<namespace>/frame`, package `vicon_receiver_msgs`): one message per Vicon frame with every subject, segment (pose + **occlusion mask**) and marker (position + cameras that saw it), object quality, frame number, dropped frames and latency per pipeline stage.
- **Acquisition-time stamps**: messages are stamped with the estimated camera acquisition time (frame-number timeline fitted to receive time − server latency), not the receive time; the raw receive time is kept in `ViconFrame`.
- **ServerPush streaming** (`stream_mode` parameter) so that a slow consumer gets every frame late instead of losing it; the stale frames a server sends first after connecting are never published.
- **Occluded poses** are no longer published as `PoseStamped`/TF (Vicon sends an all-zero pose); they stay in `ViconFrame` with their mask.
- **YAML configuration only**: `config/vicon_client.yaml` holds every node parameter; the node has no built-in defaults and stops with a clear error on a missing or invalid entry. `client.launch.py` takes only `params_file`.
- **Scripts**: build (installs ROS 2 if needed), launch with a reachability check, probe of the raw stream, live browser viewer (G1 model in its current posture from joint states, camera frustums), MCAP recording and optional CSV/HDF5 export; live console telemetry.
- Vicon names with characters such as `-` are converted into valid ROS names.
- Unit and integration tests in `tests/` (GoogleTest + pytest).

## What's new in v1.0.0 (Jazzy)

- **ROS 2 Jazzy** compatibility
- **TF publishing:** TF information for tracked subjects is now available
- **BREAKING:** switched output from a custom message to **`geometry_msgs/msg/PoseStamped`**

### Migration notes

- Update your subscribers to consume **`geometry_msgs/msg/PoseStamped`** instead of the legacy custom message.

---

## Compatibility

- **ROS 2**: Jazzy
- **OS / arch**: Ubuntu 24.04; `x86_64` and `ARM64` tested
- **Vicon stack**: Vicon Tracker; Vicon DataStream SDK **1.12**
- **SDK & Boost**: vendored per‑arch inside this repo (no system install needed)

---

## Package layout & executable

- **`vicon_receiver`** (C++): executable **`vicon_client`** (the node), executable **`vicon_probe`** (records what the server sends, no ROS), launch file **`client.launch.py`**
- **`vicon_receiver_msgs`**: `ViconFrame`, `SubjectState`, `SegmentState`, `MarkerState`
- **`vicon_receiver_tools`** (Python): configuration helpers, live viewer, mount estimate, export
- `config/`: node parameters (`vicon_client.yaml`), viewer (`viewer.yaml`), estimated G1 marker mount (`g1_mount.yaml`), G1 motor order for joint states (`g1_joint_states.yaml`), Fast DDS profile for WSL (`fastdds_wsl.xml`)
- `scripts/`: build, launch, probe, viewer, recording, export, formatting; `tests/`: all tests

---

## Quick start

### Option A — VS Code Dev Container (recommended for development)

> TODO: the Dev Container has not been rebuilt or tested since the scripts, viewer and recording were added. Option C is the tested path.

1. Open the repo in VS Code.
2. When prompted, **Reopen in Container**. The first time this may take a while as the container image is built.
    > If the prompt does not appear, try opening the Command Palette (`Ctrl+Shift+P`) and selecting "Dev Containers: Rebuild and Reopen in Container".
3. The container will install dependencies and build the workspace automatically.
    > Important: Inside the Dev Container, use the ROS 2 shell aliases defined in ~/.bashrc. For builds, use ros2-build so the correct Python virtual environment and libraries are used (prefer this over calling *colcon build* directly).

    ```bash
    ### ROS 2 aliases examples

    # Create example packages (run from workspace root)
    cd src
    ros2-pkg-create-py demo_py_pkg
    ros2-pkg-create-cpp demo_cpp_pkg
    cd ..

    # Install dependencies from src/
    ros2-rosdep

    # Clean build artifacts
    ros2-clean

    # Build the entire workspace
    ros2-build

    # Source the workspace environment
    ros2-source
    ```

4. When the container is ready, open a new terminal and run the client:

    ```bash
    source install/setup.bash

    # Every parameter comes from the parameters file (edit config/vicon_client.yaml)
    ros2 launch vicon_receiver client.launch.py params_file:=<repo>/config/vicon_client.yaml
    ```

5. After the ros2-vicon-receiver node is running you can:

   - Visualize streamed pose data in RViz2: add a TF display and/or Pose displays for the published topics.
   - Inspect raw messages directly: `ros2 topic list`, then `ros2 topic echo /vicon/<subject_name>/<segment_name>` (adjust topic name as provided by the node).
   - Examine the TF tree: `ros2 run tf2_tools view_frames` (generates frames.pdf) or add the TF display in RViz2 to see live transforms.
   - Record data for later analysis: `ros2 bag record -a` to capture all topics, including the relevant Vicon topics.
   - Monitor performance: `ros2 topic hz /vicon/<subject_name>/<segment_name>` to check update rates.

### Option B — Manual build from source

Follow these host steps if you prefer a native (non‑container) setup. Assumes ROS 2 Jazzy is already installed and `rosdep` initialized.

1. Source ROS 2:

    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```

2. Create (or choose) a workspace directory:

    ```bash
    mkdir -p ~/vicon_receiver_ws/src
    cd ~/vicon_receiver_ws/src
    ```

3. Clone the package into `src/`:

    ```bash
    git clone https://github.com/OPT4SMART/ros2-vicon-receiver.git
    cd ..   # back to workspace root
    ```

4. Install ROS 2 dependencies (none of the vendored Boost / Vicon SDK need system install):

    ```bash
    rosdep install --from-paths src --rosdistro $ROS_DISTRO -y --ignore-src
    ```

5. Build with colcon (Python invocation helps with virtual environments):

    ```bash
    python3 -m colcon build                 \
      --symlink-install                     \
      --cmake-args                          \
        -DCMAKE_BUILD_TYPE=Release          \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ```

    Notes:

    - `--symlink-install` speeds iteration.
    - `compile_commands.json` appears in the workspace root for IDE/LSP tooling.

6. Source the overlay so your shell sees the built package:

    ```bash
    source install/setup.bash
    ```

7. Launch the client (adjust `config/vicon_client.yaml` to your setup):

    ```bash
    # Every parameter comes from the parameters file (edit config/vicon_client.yaml)
    ros2 launch vicon_receiver client.launch.py params_file:=<repo>/config/vicon_client.yaml
    ```

8. After the ros2-vicon-receiver node is running you can:

   - Visualize streamed pose data in RViz2: add a TF display and/or Pose displays for the published topics.
   - Inspect raw messages directly: `ros2 topic list`, then `ros2 topic echo /vicon/<subject_name>/<segment_name>` (adjust topic name as provided by the node).
   - Examine the TF tree: `ros2 run tf2_tools view_frames` (generates frames.pdf) or add the TF display in RViz2 to see live transforms.
   - Record data for later analysis: `ros2 bag record -a` to capture all topics, including the relevant Vicon topics.
   - Monitor performance: `ros2 topic hz /vicon/<subject_name>/<segment_name>` to check update rates.

### Option C — Repository scripts (Ubuntu 24.04, also WSL)

The repository root is the colcon workspace; every script documents its options, environment variables and examples with `--help`.

```bash
scripts/build.sh                 # installs ROS 2 Jazzy + dependencies if missing (sudo), builds, creates the conda environment
scripts/launch.sh                # checks that the Vicon server is reachable, then runs the node
                                 #   with config/vicon_client.yaml (-c FILE for another one)
scripts/probe.sh -n 500          # records what the server sends (JSON in probe_output/, no ROS)
scripts/fetch_g1_model.sh        # copies the G1 model for the viewer (ssh, see config/viewer.yaml)
scripts/estimate_mount.py -r probe_output/<probe>.json --write   # marker mount on the G1 for the viewer
scripts/start_visualizer.sh      # live viewer: open http://127.0.0.1:8080
scripts/g1_joint_states.py       # G1 LowState -> JointState, so the viewer shows the robot's posture (needs unitree_hg)
scripts/record.sh -t my_session  # records an MCAP session in recordings/ (Ctrl+C to stop; -o DIR)
scripts/export.sh recordings/<session> --format csv,hdf5   # optional conversion
tests/run_tests.sh               # all tests
source scripts/source_ros_env.sh # ROS 2 environment for your own terminal (ros2 topic ...)
```

The node prints a status line every `status_period_s` (rate, dropped frames, latency, quality, occlusion, cameras per marker).

---

## Node interface

### Launch arguments

| Launch arg    | Type   | Description |
|---|---|---|
| `params_file` | string | Parameters file of the node (required), e.g. `config/vicon_client.yaml` |

### Node parameters (all required, from the parameters file)

The node has no built-in defaults: a missing parameter, a wrong type (e.g. `0` instead of `0.0` for a double) or an invalid value stops it with a `FATAL` message naming the parameter. The values below are those of `config/vicon_client.yaml`.

| Parameter | Type | Value in config | Description |
|---|---|---|---|
| `hostname` | string | `10.250.223.40` | Vicon Tracker PC, optional `:port` (DataStream port 801) |
| `buffer_size` | int | `200` | Frames buffered by the Vicon SDK client |
| `namespace` | string | `vicon` | Namespace of every topic |
| `world_frame` | string | `map` | World frame of the static TF |
| `vicon_frame` | string | `vicon` | Vicon frame (parent of the segment frames) |
| `map_xyz` | double[3] | `[0.0, 0.0, 0.0]` | Translation world → Vicon frame [m] |
| `map_rpy` | double[3] | `[0.0, 0.0, 0.0]` | Rotation world → Vicon frame (roll, pitch, yaw) |
| `map_rpy_in_degrees` | bool | `false` | `map_rpy` in degrees instead of radians |
| `stream_mode` | string | `ServerPush` | `ServerPush` (every frame delivered, use it for recording), `ClientPull` (latest frame, skips frames if slow) or `ClientPullPreFetch` |
| `enable_marker_data` | bool | `true` | Labeled markers in `ViconFrame` |
| `enable_marker_ray_data` | bool | `true` | Cameras that saw each marker (needs marker data) |
| `acquisition_stamp_window_frames` | int | `1000` | Window (frames) of the acquisition-time estimate |
| `status_period_s` | double | `1.0` | Period of the console status line, `0.0` disables it |
| `frame_queue_depth` | int | `100` | Depth of the `ViconFrame` publisher queue (frames kept for a slow subscriber) |
| `live_stream_confirmation_frames` | int | `3` | Increasing frame numbers, arriving at a plausible pace, before frames are published after connecting (>= 2) |
| `clock_check_measurement_s` | double | `10.0` | Duration of one local-clock-rate measurement [s] |
| `clock_check_block_s` | double | `1.0` | Blocks at both ends of a measurement whose least delayed frames are compared [s] (measurement > 2 blocks) |
| `clock_rate_warning_limit` | double | `0.01` | Relative clock rate difference that triggers a warning (0.01 = 1 %) |
| `connection_timeout_s` | int | `5` | Seconds without answer from the server (TCP keepalive) before the connection counts as lost; the node then stops with exit status 2 (no reconnection) |
| `connect_retry_period_s` | double | `1.0` | Pause between two attempts to connect [s] |
| `no_frame_pause_s` | double | `0.005` | Pause before asking again when the server has no frame [s] |

### Published topics

#### Naming scheme

All topics are published under the configured `namespace` (default `vicon`) and follow this structure:

```text
/<namespace>/<subject_name>/<segment_name>
```

Examples:

- `/vicon/cf0/cf0`
- `/vicon/qcar/qcar`

#### Message types

- **Every frame:** `<namespace>/frame`, `vicon_receiver_msgs/msg/ViconFrame`: all subjects, segments (pose in metres in the Vicon frame, occlusion mask), markers (position, occluded, camera ids), quality, frame number, dropped frames, latency per stage, raw receive time. Nothing is filtered out.
- **Pose per tracked entity:** `geometry_msgs/msg/PoseStamped` (in `world_frame`), for live monitoring; occluded segments are skipped.
- **TF:** dynamic transforms for each `<subject_name>_<segment_name>` frame (see TF tree section below); occluded segments are skipped.
- **Stamps:** the estimated camera acquisition time of the frame (same stamp on all three outputs).

#### Notes

- `<subject_name>` and `<segment_name>` come from Vicon Tracker, converted into valid ROS names: characters other than letters, digits and `_` become `_`, and a `vicon` prefix is added to names that do not start with a letter (`g1-unitree-torso-cam` → `g1_unitree_torso_cam`).
- Object quality is a Vicon Tracker metric whose range is **not** 0..1 (about 1.3 on the Tracker this was tested with).
- Topic frequency depends on your Vicon system and client configuration.

#### QoS

- `PoseStamped`: reliable, depth 10. `ViconFrame`: reliable, depth 100 (1 s at 100 Hz), so that recorders get every frame.

#### Example topic tree

```bash
/vicon/
├── qcar/
│   └── qcar [geometry_msgs/PoseStamped]
└── cf0/
    └── cf0 [geometry_msgs/PoseStamped]
```

## Frames & mapping

- The Vicon frame → world frame mapping is configurable via `world_frame`, `vicon_frame`, `map_xyz` and `map_rpy`.
- `map_rpy_in_degrees` lets you specify rotations in degrees when convenient.
- Frame IDs for subjects/segments are derived from Vicon names.

> Units follow ROS conventions (positions in meters, rotations in radians) in downstream consumers; ensure your system uses consistent units end‑to‑end.

### TF tree (template + notes)

The package exposes TF for tracked subjects/segments. The expected high‑level tree is:

```bash
map (world frame)
└── vicon (vicon_frame)          [static]
    ├── <subject_1>_<segment_1>  [dynamic]
    └── <subject_2>_<segment_2>  [dynamic]
```

#### Static transform (`map → vicon`)

- Defined via node parameters:
  - `world_frame`: name of the world frame (default: `map`)
  - `vicon_frame`: name of the Vicon frame (default: `vicon`)
  - `map_xyz`: translation from `world_frame` to `vicon_frame` in meters (default: `[0.0, 0.0, 0.0]`)
  - `map_rpy`: rotation from `world_frame` to `vicon_frame` in Roll-Pitch-Yaw angles (default: `[0.0, 0.0, 0.0]`)
  - `map_rpy_in_degrees`: `true` if `map_rpy` is given in degrees
- Fixed frame used in RViz: `world_frame` (recommended) or `vicon_frame`

#### Dynamic transforms

- One child frame per `<subject_name>_<segment_name>` populated from Vicon measurements.

#### Tips

- Generate a TF report: `ros2 run tf2_tools view_frames` and inspect the produced PDF.
- Ensure your static `world_frame → vicon_frame` is correct before validating child frames.

---

## Example images

### TF2 frame tree

<img src="docs/images/tf_tree.png" alt="TF2 frame tree for vicon_receiver" width="720">

Example TF tree showing the static `world_frame → vicon_frame` transform and dynamic subject frames.

### RViz: TF and Pose (template)

<img src="docs/images/rviz_tf_pose.png" alt="RViz visualization of TF and PoseStamped" width="720">

RViz view showing TF frames and a Pose display for a tracked subject (e.g., /vicon/qcar/qcar).

---

## Building & linking details

- The package is C++17 and uses `ament_cmake`.
- The **Vicon DataStream SDK (1.12)** and **Boost 1.75** are vendored in `third_party/<arch>/` and linked directly, so you don’t need system‑wide installations.
- The install step ships the required shared libraries so that runtime lookups succeed without extra `LD_LIBRARY_PATH` setup.

## Troubleshooting / FAQ

**The node can't connect to the Vicon server**: `scripts/launch.sh` checks the TCP port of the DataStream server first and names the cause (unresolvable name, no route, connection refused = Tracker/DataStream not running, timeout), with the local subnets and the route used. A different subnet is fine if the networks are routed; on an isolated Vicon network this machine needs an address in its subnet.

**On WSL, other ROS 2 processes do not see the node** (`ros2 topic list` is empty): WSL mirrored networking routes 127.0.0.1 to Windows. `source scripts/source_ros_env.sh` (the scripts do it themselves) selects `config/fastdds_wsl.xml`; use `--no-daemon` with `ros2 topic/node list/echo`.

**"The local clock runs ... % faster (or slower) than the Vicon frame clock"** (checked over 10 s): the local clock is wrong, so every local stamp is too. On WSL the `tsc` clock source was measured 4 % fast; switch to `hyperv_clocksource_tsc_page` (see CLAUDE.md) and restart WSL.

**"Frame number jumped ... the previous frame was stale"** after connecting: normal; the Vicon server first sends frames left over from its previous session. They are not counted as dropped.

**Frames look misaligned**: adjust `map_xyz` / `map_rpy` and confirm radians vs degrees via `map_rpy_in_degrees`.

**I don’t see TF in RViz**: confirm TF display is enabled and the fixed frame matches your global frame (`world_frame`/`vicon_frame`).

**Which topics are produced?**: topics are under your `namespace` and reflect subject/segment names in Vicon.

---

## License & attribution

- **License:** GNU General Public License v3.0 (GPL‑3.0)
- **Current maintainer:** [Andrea Drudi](https://www.unibo.it/sitoweb/andrea.drudi4), [Alice Rosetti](https://www.unibo.it/sitoweb/a.rosetti) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano) ([OPT4SMART group](mailto:info@opt4smart.eu)) of the [Department of Electrical, Electronic, and Information Engineering "Guglielmo Marconi" - DEI](https://dei.unibo.it/en) at the University of Bologna, Italy
- **Former contributors:** Andrea Camisa, Andrea Testa
- **Acknowledgments:** Vicon DataStream SDK and community contributors.

---

## Contributing

Issues and PRs are welcome. Please include:

- ROS 2 distro, OS/arch
- Vicon Tracker + SDK versions
- Steps and logs to reproduce

---
