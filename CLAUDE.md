# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 Jazzy packages that pull frames from a Vicon Tracker DataStream server and publish one `geometry_msgs/msg/PoseStamped` topic per subject/segment plus TF:

- `vicon_receiver` (C++17, `ament_cmake`): the node. Executable: `vicon_client`; launch file: `client.launch.py`.
- `vicon_receiver_tools` (`ament_python`): Python tools used by the scripts (`config.py`: read the parameters file with PyYAML; `network.py`: local-subnet report). Future Python tooling (e.g. recording export) belongs here, not in `scripts/`.

The scripts use the **repo root as the colcon workspace** (`colcon build --base-paths vicon_receiver vicon_receiver_tools`); `build/`, `install/`, `log/` land at the root and are git-ignored. The Dev Container instead bind-mounts `vicon_receiver/` to `/home/ubuntu/ros2_ws/src/ros2-vicon-receiver` and runs colcon from `/home/ubuntu/ros2_ws`.

## Build and run

```bash
scripts/build.sh            # installs ROS 2 Jazzy + deps via apt/rosdep if missing (sudo), then colcon build
scripts/launch.sh           # checks Vicon reachability, then runs the node with config/vicon_client.yaml
scripts/launch.sh -c FILE --no-check -- --log-level debug
scripts/format.sh [--check] # clang-format (CLANG_FORMAT=... to pick the binary)
```

Manual equivalent (ROS sourced, from the repo root): `python3 -m colcon build --base-paths vicon_receiver vicon_receiver_tools --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, then `ros2 run vicon_receiver vicon_client --ros-args --params-file config/vicon_client.yaml`.

Inside the Dev Container, use the `.bashrc` aliases instead (`ros2-build`, `ros2-source`, `ros2-clean`, `ros2-rosdep`).

There are no tests yet. Compiler warnings (`-Wall -Wextra -Wpedantic`) are the only static check. Running the node needs a reachable Vicon Tracker; without one, `connect()` loops on reconnect attempts (5 s SDK connect timeout each) until Ctrl+C. colcon installs per package: the node binary is `install/vicon_receiver/lib/vicon_receiver/vicon_client`.

## Configuration

- `config/vicon_client.yaml` (repo root, outside the package) is the single source of truth. The node declares every parameter **without a default**; a missing entry, wrong type (e.g. `0` for a double) or wrong vector size makes `main` log FATAL and exit 1. Don't add defaults in C++.
- `scripts/lib/common.sh` holds the shared shell helpers: ROS/workspace checks, reading a parameter from the YAML, and `check_vicon_reachable` (TCP connect to the DataStream port 801 with cause-specific errors: unresolvable name, no route, refused, timeout + local-subnet report). Anything needing Python goes through `run_tools_module <module>`, which runs `vicon_receiver_tools` **from its sources** (`PYTHONPATH`), so the helpers work before the build; no inline Python in bash.
- On this WSL machine (`networkingMode=mirrored`), a closed loopback port times out instead of refusing, so the "connection refused" branch can't be reproduced locally.

## Conventions

- C++: `.clang-format` at the root (from gtsam_spaceNav: LLVM, Allman, 4 spaces, 100 cols, namespace bodies indented, no bin-packing). Doxygen file headers (`@file/@author/@brief/@version/@date`) and doc comments; one statement per line.
- Bash: `set -Eeuo pipefail`, `usage()`, small commented functions, errors via `die` to stderr; source `scripts/lib/common.sh`.
- Python (gtsam_spaceNav conventions): type hints, docstrings, dataclasses over dicts, top-level functions capitalised (`Read_parameter`), methods lowercase, local variables suffixed `_`. Modules expose a `Main(argv) -> int` CLI for `python3 -m`.
- Commits: only after the user reviews the batch; gtsam_spaceNav style (imperative subject, `[MAJOR]`/`[BUGFIX]` prefixes, `- ` bullet bodies).

## Architecture

Two classes, both in `vicon_receiver/src/`:

- **`Communicator`** (`communicator.cpp`, also contains `main` and the FATAL reporting of parameter errors) is the `rclcpp::Node`. It owns the `ViconDataStreamSDK::CPP::Client`, reads and validates parameters, publishes the static `world_frame → vicon_frame` TF once in the constructor, and runs the frame loop.
- **`Publisher`** (`publisher.cpp`) is a thin wrapper around one `PoseStamped` publisher (QoS depth 10).

Runtime flow:

1. `main` constructs the node, calls `connect()` (blocking retry loop, `ClientPull` stream mode), then calls `get_frame()` in a tight `while (rclcpp::ok())` loop. **There is no `rclcpp::spin`/executor**, so subscriptions, timers, services, and runtime parameter changes are never serviced. Parameters are read once in the constructor.
2. `get_frame()` iterates subjects × segments. For each segment it builds a `TransformStamped` (`vicon_frame` → `<subject>_<segment>`, translation converted from Vicon millimetres to metres, quaternion copied as x,y,z,w) and broadcasts it on TF.
3. The same pose is then transformed with `tf2::doTransform` through the stored static TF, so **the `PoseStamped` topic is expressed in `world_frame`, while dynamic TF children are parented to `vicon_frame`**. Keep both paths consistent when changing frame handling.
4. Publishers are created lazily. The first time a `subject/segment` key is seen, it is added to `pending_publishers` and a detached `boost::thread` runs `create_publisher_thread`, which inserts into `pub_map` under `mutex`. `get_frame()` uses a `scoped_try_lock`, so pose publishing for a segment is skipped (TF is still sent) on frames where the lock is contended or the publisher isn't ready yet.

Topic name: `<namespace>/<subject>/<segment>`. The launch argument is `topic_namespace` but the node parameter is `namespace`. `client.launch.py` still carries its own argument defaults (hostname `192.168.10.1`); `scripts/launch.sh` bypasses it and uses `ros2 run` with the YAML.

## Vendored dependencies

`vicon_receiver/third_party/{x86_64,aarch64}/` holds the Vicon DataStream SDK 1.12 (`libViconDataStreamSDK_CPP.so` + headers) and Boost 1.75 (headers + `system`/`thread`/`chrono`/`timer` shared libs). This is most of the repo's ~6800 tracked files; ignore it when searching.

`CMakeLists.txt` selects the arch from `CMAKE_SYSTEM_PROCESSOR` (fails on anything else), wraps the vendored libraries as IMPORTED targets (`ViconDataStreamSDK::ViconDataStreamSDK_CPP`, `Boost::system`, etc.) without `find_package(Boost)`, and installs the `.so` files into `install/lib` alongside `INSTALL_RPATH $ORIGIN/../lib`. System Boost or SDK installs are neither needed nor used. If you add a Boost component, you must add both the `find_library` + IMPORTED target and the install rule, and the library must exist for both architectures.
