# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 Jazzy packages that pull frames from a Vicon Tracker DataStream server and publish one `geometry_msgs/msg/PoseStamped` topic per subject/segment plus TF:

- `vicon_receiver` (C++17, `ament_cmake`): the node. Executable: `vicon_client`; launch file: `client.launch.py`.
- `vicon_receiver_msgs` (`ament_cmake`, rosidl): `ViconFrame` (one per Vicon frame: acquisition-time stamp, raw receive stamp, frame number, frames dropped, rate, latency total + stages, any_occluded) with `SubjectState` (name, ros_name, quality), `SegmentState` (pose in metres, occlusion mask) and `MarkerState` (position, occluded, camera_ids). Field layout pinned by `tests/integration/test_messages.py`.
- `vicon_receiver_tools` (`ament_python`): Python tools used by the scripts (`config.py`: read the parameters file with PyYAML; `network.py`: local-subnet report). Future Python tooling (e.g. recording export) belongs here, not in `scripts/`.

The scripts use the **repo root as the colcon workspace** (`colcon build --base-paths vicon_receiver_msgs vicon_receiver vicon_receiver_tools`); `build/`, `install/`, `log/` land at the root and are git-ignored. The Dev Container instead bind-mounts the whole repository to `/home/ubuntu/ros2_ws/src/ros2-vicon-receiver` and runs colcon from `/home/ubuntu/ros2_ws` (the host's `build/`/`install/` contain `COLCON_IGNORE`).

## Build and run

```bash
scripts/build.sh            # installs ROS 2 Jazzy + deps via apt/rosdep if missing (sudo), then colcon build
scripts/launch.sh           # checks Vicon reachability, then runs the node with config/vicon_client.yaml
scripts/launch.sh -c FILE --no-check -- --log-level debug
scripts/format.sh [--check] # clang-format (CLANG_FORMAT=... to pick the binary)
scripts/probe.sh [--host HOST[:PORT]] [-n FRAMES] [-o FILE]   # record what the server sends (JSON, no ROS)
```

Manual equivalent (ROS sourced, from the repo root): `python3 -m colcon build --base-paths vicon_receiver_msgs vicon_receiver vicon_receiver_tools --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, then `ros2 run vicon_receiver vicon_client --ros-args --params-file config/vicon_client.yaml`.

Inside the Dev Container, use the `.bashrc` aliases instead (`ros2-build`, `ros2-source`, `ros2-clean`, `ros2-rosdep`).

Running the node needs a reachable Vicon Tracker; without one, `connect()` loops on reconnect attempts (5 s SDK connect timeout each) until Ctrl+C. colcon installs per package: the node binary is `install/vicon_receiver/lib/vicon_receiver/vicon_client`.

## Tests

```bash
tests/run_tests.sh                       # colcon build (BUILD_TESTING=ON) + colcon test + pytest
tests/run_tests.sh --cpp | --python      # one side only; --no-build to skip the build
python3 -m pytest tests/unit/test_common_sh.py -k refused   # single Python test (no ROS needed for tests/unit)
source install/setup.bash && build/vicon_receiver/test_vicon_receiver_node_params --gtest_filter='*Missing*'   # single C++ test (needs the overlay for the vendored Boost/SDK libs)
```

- All tests live in `tests/` (see `tests/README.md`). C++ tests (GoogleTest) are in `tests/unit/cpp/` but compiled by `vicon_receiver/CMakeLists.txt` under `BUILD_TESTING` (skipped if the folder is absent); `scripts/build.sh` builds with `BUILD_TESTING=OFF`. `colcon test` runs only `vicon_receiver`; the tools package is tested by pytest, importing it from its sources (`conftest.py` adds it to `sys.path`).
- No Vicon needed: the network is faked with a local TCP listener, the unassigned 192.0.2.1, or the fake `ip`/`timeout` in `tests/fixtures/fake_bin/` (driven by `FAKE_*` env vars).
- Integration tests (`-m integration`) need ROS + the built workspace and skip otherwise. Tests marked `vicon_live` need a real server: `VICON_LIVE_HOST=10.250.223.40 python3 -m pytest tests -m vicon_live`. `tests/unit/test_static_checks.py` runs shellcheck/clang-format if installed (also looks in `build/.devtools/bin`; never put that venv on `PATH`, its `python3` lacks colcon).

## Configuration

- `config/vicon_client.yaml` (repo root, outside the package) is the single source of truth. The node declares every parameter **without a default**; a missing entry, wrong type (e.g. `0` for a double) or wrong vector size makes `main` log FATAL and exit 1. Don't add defaults in C++.
- `scripts/lib/common.sh` holds the shared shell helpers: ROS/workspace checks, reading a parameter from the YAML, and `check_vicon_reachable` (TCP connect to the DataStream port 801 with cause-specific errors: unresolvable name, no route, refused, timeout + local-subnet report). Anything needing Python goes through `run_tools_module <module>`, which runs `vicon_receiver_tools` **from its sources** (`PYTHONPATH`), so the helpers work before the build; no inline Python in bash.
- On this WSL machine (`networkingMode=mirrored`), a closed loopback port times out instead of refusing, so the "connection refused" branch can't be reproduced locally.

## WSL mirrored networking and DDS

WSL mirrored networking adds `ip rule ... ipproto tcp|udp lookup 127`, routing **all TCP/UDP to 127.0.0.1 to Windows** (`via ... dev loopback0`). With default settings, ROS 2 processes on this machine never discover each other, and the ros2 CLI daemon (TCP localhost) hangs.

- Fix (no sudo): `config/fastdds_wsl.xml` restricts Fast DDS to the second `lo` address `10.255.255.254` (not redirected) + shared memory, with that address as initial peer. `scripts/lib/dds.sh` (`vicon_configure_dds`) detects the redirection (`ip -4 route get 127.0.0.1` not via `dev lo`), exports `FASTRTPS_/FASTDDS_DEFAULT_PROFILES_FILE` and sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` (LOCALHOST/OFF make rmw override the profile with 127.0.0.1 peers). A profile already set by the user is kept.
- Used by `launch.sh`, `tests/run_tests.sh`, the pytest `ros_env` fixture. In a terminal: `source scripts/source_ros_env.sh`, then `ros2 topic list --no-daemon` / `ros2 topic echo --no-daemon ...` (`ros2 topic hz` has no daemon option and works).
- Scope: ROS 2 traffic stays on this machine. `tests/integration/test_dds.py` guards it (two processes must exchange messages).

## Conventions

- C++: `.clang-format` at the root (from gtsam_spaceNav: LLVM, Allman, 4 spaces, 100 cols, namespace bodies indented, no bin-packing). Doxygen file headers (`@file/@author/@brief/@version/@date`) and doc comments; one statement per line.
- Bash: `set -Eeuo pipefail`, `usage()`, small commented functions, errors via `die` to stderr; source `scripts/lib/common.sh`.
- Python (gtsam_spaceNav conventions): type hints, docstrings, dataclasses over dicts, top-level functions capitalised (`Read_parameter`), methods lowercase, local variables suffixed `_`. Modules expose a `Main(argv) -> int` CLI for `python3 -m`.
- Commits: only after the user reviews the batch; gtsam_spaceNav style (imperative subject, `[MAJOR]`/`[BUGFIX]` prefixes, `- ` bullet bodies).

## Architecture

Two classes in `vicon_receiver/src/`, compiled into the static library `vicon_receiver_core` (linked by the executable and the tests); `main.cpp` holds `main()` and the FATAL reporting of parameter errors.

- **`ViconReceiverNode`** (`vicon_receiver_node.cpp`) is the `rclcpp::Node`. It owns the `ViconDataStreamSDK::CPP::Client`, reads and validates parameters, publishes the static `world_frame → vicon_frame` TF once in the constructor, and runs the frame loop. The constructor takes `rclcpp::NodeOptions` (tests inject parameter overrides) and never connects.
- **`SegmentPosePublisher`** (`segment_pose_publisher.cpp`) is a thin wrapper around one `PoseStamped` publisher (QoS depth 10).

Runtime flow:

1. `main` constructs the node, calls `connect()` (blocking retry loop, `ClientPull` stream mode), then calls `get_frame()` in a tight `while (rclcpp::ok())` loop. **There is no `rclcpp::spin`/executor**, so subscriptions, timers, services, and runtime parameter changes are never serviced. Parameters are read once in the constructor.
2. `get_frame()` iterates subjects × segments. For each segment it builds a `TransformStamped` (`vicon_frame` → `<subject>_<segment>`, translation converted from Vicon millimetres to metres, quaternion copied as x,y,z,w) and broadcasts it on TF.
3. The same pose is then transformed with `tf2::doTransform` through the stored static TF, so **the `PoseStamped` topic is expressed in `world_frame`, while dynamic TF children are parented to `vicon_frame`**. Keep both paths consistent when changing frame handling.
4. Publishers are created lazily. The first time a `subject/segment` key is seen, it is added to `pending_publishers` and a detached `boost::thread` runs `create_publisher_thread`, which inserts into `pub_map` under `mutex`. `get_frame()` uses a `scoped_try_lock`, so pose publishing for a segment is skipped (TF is still sent) on frames where the lock is contended or the publisher isn't ready yet.

Topic name: `<namespace>/<subject>/<segment>`; TF child frame: `<subject>_<segment>`. Vicon names are converted by `vicon_receiver::to_ros_name_token()` (`name_conversion.hpp`): characters other than `[A-Za-z0-9_]` become `_`, names not starting with a letter get a `vicon` prefix (e.g. `g1-unitree-torso-cam` → `g1_unitree_torso_cam`). A publisher that cannot be created is logged and skipped (never retried); of two segments mapping to the same topic, the second is refused. `client.launch.py` has a single required argument, `params_file` (no parameter defaults of its own): `ros2 launch vicon_receiver client.launch.py params_file:=config/vicon_client.yaml`; `scripts/launch.sh` uses `ros2 run` with the same YAML after the reachability check.

## Shared code (`vicon_common`)

Static library without ROS, linked by the node and the probe (namespace `vicon_receiver`, headers in `include/vicon_receiver/`): `DroppedFrameCounter` (missed frames from frame-number gaps), SDK enum names (`result_name`, ...), and `AcquisitionClock`: estimated camera acquisition time on the local clock = `t_ref + (frame - frame_ref)/rate`, with `t_ref` = minimum over a sliding window (in frames) of `receive - latency_total - (frame - frame_ref)/rate` (transport delays are positive, so the minimum is the least delayed frame; the window follows clock drift). Restarts on non-increasing frame numbers or a rate change. The hardware frame number is always 0 on the lab Tracker: don't use it.

## Probe (`vicon_probe`)

Plain C++ executable (no rclcpp) in the same package, built from `vicon_probe_core` (`src/probe/`, `include/vicon_receiver/probe/`, namespace `vicon_probe`) on top of `vicon_common`. It uses the **same vendored SDK and calls as the node**, so what it sees is what the node can get. It connects once (`ClientPull`), enables every stream, and writes one JSON document: probe/connection/stream/server/cameras/devices/subject structure, then per frame the frame numbers, receive time, latency total + per stage, timecode, per subject quality, segment poses with occlusion flags, markers with the camera IDs of their rays, labeled/unlabeled markers; SDK calls that fail are listed in `errors` arrays instead of being dropped. Units stay SDK-native (mm, quaternion x,y,z,w). Exit codes: 0 ok, 1 args, 2 connect failed, 3 output file, 4 stream lost. `scripts/probe.sh` adds the reachability check and writes to `probe_output/` (git-ignored). The SDK `.so` has no runpath and needs the vendored Boost: link `Boost::thread Boost::chrono` with anything linking only the SDK, and use `APPEND_LIBRARY_DIRS` for its gtests.

What a live recording (2026-10-30, lab Tracker at 10.250.223.40) showed — keep in mind for message design:
- Subject/segment/marker names contain `-` (`g1-unitree-torso-cam`): convert them with `vicon_receiver::to_ros_name_token()` before using them in ROS names.
- `GetObjectQuality` is ~1.3, **not** the 0..1 the SDK docs claim; treat it as an opaque Tracker metric.
- An occluded segment comes with an all-zero pose (quaternion `[0,0,0,0]`): never publish it as a valid pose.
- Each marker is seen by 3–7 cameras (ray contributions): a usable per-marker quality signal.
- Latency total ~31 ms (mostly "Data Delivered"); 100 Hz; no timecode, no camera calibration, no devices, no unlabeled markers from this Tracker.

## Vendored dependencies

`vicon_receiver/third_party/{x86_64,aarch64}/` holds the Vicon DataStream SDK 1.12 (`libViconDataStreamSDK_CPP.so` + headers) and Boost 1.75 (headers + `system`/`thread`/`chrono`/`timer` shared libs). This is most of the repo's ~6800 tracked files; ignore it when searching.

`CMakeLists.txt` selects the arch from `CMAKE_SYSTEM_PROCESSOR` (fails on anything else), wraps the vendored libraries as IMPORTED targets (`ViconDataStreamSDK::ViconDataStreamSDK_CPP`, `Boost::system`, etc.) without `find_package(Boost)`, and installs the `.so` files into `install/lib` alongside `INSTALL_RPATH $ORIGIN/../lib`. System Boost or SDK installs are neither needed nor used. If you add a Boost component, you must add both the `find_library` + IMPORTED target and the install rule, and the library must exist for both architectures.
