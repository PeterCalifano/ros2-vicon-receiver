# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 Jazzy packages that pull frames from a Vicon Tracker DataStream server and publish one `geometry_msgs/msg/PoseStamped` topic per subject/segment plus TF:

- `vicon_receiver` (C++17, `ament_cmake`): the node. Executable: `vicon_client`; launch file: `client.launch.py`.
- `vicon_receiver_msgs` (`ament_cmake`, rosidl): `ViconFrame` (one per Vicon frame: acquisition-time stamp, raw receive stamp, frame number, frames dropped, rate, latency total + stages, any_occluded) with `SubjectState` (name, ros_name, quality), `SegmentState` (pose in metres, occlusion mask) and `MarkerState` (position, occluded, camera_ids). Field layout pinned by `tests/integration/test_messages.py`.
- `vicon_receiver_tools` (`ament_python`): Python tools used by the scripts (`config.py`: read the parameters file with PyYAML; `network.py`: local-subnet report; viewer, mount estimate, export). Runtime deps (numpy, scipy, rclpy, rosbag2_py + MCAP storage, ros2bag, the msgs package) are declared in its `package.xml` for rosdep; the Python environment of the tools is `environment.yml`. Python tooling belongs here, not in `scripts/`.

The scripts use the **repo root as the colcon workspace** (`colcon build --base-paths vicon_receiver_msgs vicon_receiver vicon_receiver_tools`); `build/`, `install/`, `log/` land at the root and are git-ignored. The Dev Container instead bind-mounts the whole repository to `/home/ubuntu/ros2_ws/src/ros2-vicon-receiver` and runs colcon from `/home/ubuntu/ros2_ws` (the host's `build/`/`install/` contain `COLCON_IGNORE`).

## Build and run

```bash
scripts/build.sh            # installs ROS 2 Jazzy + deps via apt/rosdep if missing (sudo), then colcon build
scripts/launch.sh           # checks Vicon reachability, then runs the node with config/vicon_client.yaml
scripts/launch.sh -c FILE --no-check -- --log-level debug
scripts/format.sh [--check] # clang-format (CLANG_FORMAT=... to pick the binary)
scripts/probe.sh [--host HOST[:PORT]] [-n FRAMES] [-o FILE]   # record what the server sends (JSON, no ROS)
scripts/fetch_g1_model.sh             # copy the G1 model (URDF + meshes) from pcalifano-u into models/g1 (git-ignored)
scripts/estimate_mount.sh -r probe.json [--write]   # rough marker mount on the G1 -> config/g1_mount.yaml
scripts/viz.sh                        # live viewer in the browser (viser), http://127.0.0.1:8080
source scripts/source_ros_env.sh      # ROS + workspace + WSL DDS profile in an interactive shell
scripts/record.sh [-t TAG] [-o DIR]   # record a session (MCAP) of the running node; Ctrl+C stops
scripts/export.sh SESSION --format csv,hdf5   # optional conversion of a session
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
- No Vicon needed: the network is faked with a local TCP listener, the unassigned 192.0.2.1, or the fake `ip`/`timeout`/`conda` in `tests/fixtures/fake_bin/` (driven by `FAKE_*` env vars).
- Integration tests (`-m integration`) need ROS + the built workspace and skip otherwise. Tests marked `vicon_live` need a real server: `VICON_LIVE_HOST=10.250.223.40 python3 -m pytest tests -m vicon_live`. `tests/unit/test_static_checks.py` runs shellcheck/clang-format if installed (pinned in the conda environment; never build with it activated, its `python3` lacks colcon).

## Configuration

- `config/vicon_client.yaml` (repo root, outside the package) is the single source of truth. The node declares every parameter **without a default**; a missing entry, wrong type (e.g. `0` for a double) or wrong vector size makes `main` log FATAL and exit 1. Don't add defaults in C++.
- Streaming/stamping/telemetry parameters (all required): `stream_mode` (`ServerPush` | `ClientPull` | `ClientPullPreFetch`; ServerPush delivers every frame, late rather than lost — use it for recording; ClientPull asks for the latest frame and skips frames if slow), `enable_marker_data`, `enable_marker_ray_data` (needs marker data), `acquisition_stamp_window_frames` (>= 1), `status_period_s` (>= 0, 0 disables). Invalid values (unknown mode, window 0, negative period, rays without markers) and an invalid `namespace` are rejected at startup with `std::invalid_argument` → FATAL.
- `scripts/lib/common.sh` holds the shared shell helpers: ROS/workspace checks, reading a parameter from the YAML, and `check_vicon_reachable` (TCP connect to the DataStream port 801 with cause-specific errors: unresolvable name, no route, refused, timeout + local-subnet report). Anything needing Python goes through `run_tools_module <module>`, which runs `vicon_receiver_tools` **from its sources** (`PYTHONPATH`), so the helpers work before the build; no inline Python in bash.
- On this WSL machine (`networkingMode=mirrored`), a closed loopback port times out instead of refusing, so the "connection refused" branch can't be reproduced locally.

## WSL mirrored networking and DDS

WSL mirrored networking adds `ip rule ... ipproto tcp|udp lookup 127`, routing **all TCP/UDP to 127.0.0.1 to Windows** (`via ... dev loopback0`). With default settings, ROS 2 processes on this machine never discover each other, and the ros2 CLI daemon (TCP localhost) hangs.

- Fix (no sudo): `config/fastdds_wsl.xml` restricts Fast DDS to the second `lo` address `10.255.255.254` (not redirected) + shared memory, with that address as initial peer. `scripts/lib/dds.sh` (`vicon_configure_dds`) detects the redirection (`ip -4 route get 127.0.0.1` not via `dev lo`), exports `FASTRTPS_/FASTDDS_DEFAULT_PROFILES_FILE` and sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` (LOCALHOST/OFF make rmw override the profile with 127.0.0.1 peers). A profile already set by the user is kept.
- Used by `launch.sh`, `tests/run_tests.sh`, the pytest `ros_env` fixture. In a terminal: `source scripts/source_ros_env.sh`, then `ros2 topic list --no-daemon` / `ros2 topic echo --no-daemon ...` (`ros2 topic hz` has no daemon option and works).
- Scope: ROS 2 traffic stays on this machine. `tests/integration/test_dds.py` guards it (two processes must exchange messages).

## WSL clock (measured 2026-10-30)

On this laptop the WSL clocksource is `tsc` and it runs **~4.2 % fast** (NTP check: monotonic +42 000 ppm; realtime gains ~200 ms / 5 s and systemd-timesyncd steps it back by ~1 s). Every local stamp (receive times, ROS time, recordings) is then wrong, and `make` reports clock skew. The node detects it (the warning above); `build/diag/clock_rate.py` measures it against NTP. Fix (root, user decision): `echo hyperv_clocksource_tsc_page | sudo tee /sys/devices/system/clocksource/clocksource0/current_clocksource` (available here), persistent via `kernelCommandLine = clocksource=hyperv_clocksource_tsc_page` in the Windows `.wslconfig` `[wsl2]` section + `wsl --shutdown`. Until fixed, don't trust live timing measurements on this machine.

## Conventions

- C++: `.clang-format` at the root (from gtsam_spaceNav: LLVM, Allman, 4 spaces, 100 cols, namespace bodies indented, no bin-packing). Doxygen file headers (`@file/@author/@brief/@version/@date`) and doc comments; one statement per line.
- Bash: `set -Eeuo pipefail`, `usage()`, small commented functions, errors via `die` to stderr; source `scripts/lib/common.sh`.
- Python (gtsam_spaceNav conventions): type hints, docstrings, dataclasses over dicts, top-level functions capitalised (`Read_parameter`), methods lowercase, local variables suffixed `_`. Modules expose a `Main(argv) -> int` CLI for `python3 -m`.
- Commits: only after the user reviews the batch; gtsam_spaceNav style (imperative subject, `[MAJOR]`/`[BUGFIX]` prefixes, `- ` bullet bodies).

## Architecture

The node code in `vicon_receiver/src/` is compiled into the static library `vicon_receiver_core` (linked by the executable and the tests); `main.cpp` holds `main()` and the FATAL reporting of parameter errors.

- **`ViconReceiverNode`** (`vicon_receiver_node.cpp`) is the `rclcpp::Node`. It owns the `ViconDataStreamSDK::CPP::Client`, reads and validates parameters, publishes the static `world_frame → vicon_frame` TF once in the constructor, creates the `<namespace>/frame` publisher (`ViconFrame`, reliable, depth 100), and runs the frame loop. The constructor takes `rclcpp::NodeOptions` (tests inject parameter overrides) and never connects.
- **`ros_conversions.cpp`**: Vicon names → ROS names (`to_ros_name_token`) and pure SDK-result → message conversions (mm → m, occlusion flags; a failed SDK call counts as occluded), `is_valid_pose` (not occluded and unit quaternion), TF transform, time conversions. Unit-tested with hand-built `Output_*` structs.
- **`SegmentPosePublisher`** (in `vicon_receiver_node.hpp/.cpp`, DEVNOTE: location may be reevaluated) is a thin wrapper around one `PoseStamped` publisher (QoS depth 10).

Runtime flow:

1. `main` constructs the node, calls `connect()` (blocking retry loop; enables segment data, plus marker/ray data per parameters; sets `stream_mode`, `ServerPush` in the config), then calls `get_frame()` in a tight `while (rclcpp::ok())` loop. **There is no `rclcpp::spin`/executor**, so subscriptions, timers, services, and runtime parameter changes are never serviced. Parameters are read once in the constructor.
2. `get_frame()`: `GetFrame()`, then the receive time is taken immediately (system clock for stamps, steady clock for the clock-rate check). `NotConnected` → reconnect; every successful `connect()` calls `reset_stream_state()` (gate, gaps, acquisition clock, clock check, status period all restart; frames missed during the outage are not counted). **`StartupLeftoverFrameFilter`** holds back frames after connecting until a run of 3 consecutive frame numbers has arrived over at least half its nominal duration (leftovers come back to back and can be consecutive; they and the first live frames are never published). `NoFrame` (Tracker not streaming) is silent except for the status line, which then reads `no frames in the last 1.0 s`. `read_frame_header` fills frame number, rate, latency total + stages (a non-finite or negative latency counts as not reported), `frames_dropped` (`DroppedFrameCounter`), and `header.stamp` = estimated acquisition time (`AcquisitionClock`; falls back to receive time, with one warning, if frame number or rate is missing). A later forward frame-number jump larger than rate × (receive interval + 1 s) is a **stale frame**: logged, gap/clock/clock check restarted, no drops counted. `LocalClockRateMonitor` compares the steady clock with frame numbers / rate over 10 s measurements (least delayed frame of the first and last 1 s blocks, so jitter and stalls don't bias it) and warns when the local clock runs > 1 % faster or slower (independent of `status_period_s`).
3. `read_subjects` puts **every** subject/segment into the `ViconFrame` (all-zero occluded poses included, with the occlusion mask; `any_occluded`), markers + camera ids if enabled. Only `is_valid_pose` segments go to `publish_segment_pose`: TF (`vicon_frame` → `<subject>_<segment>`) and the `PoseStamped`, transformed through the static TF so **the `PoseStamped` topic is expressed in `world_frame`, while TF children are parented to `vicon_frame`**; both stamped with the acquisition time. Keep both paths consistent when changing frame handling.
4. Publishers are created lazily. The first time a `subject/segment` key is seen, it is added to `pending_publishers` and a detached `boost::thread` runs `create_publisher_thread`, which inserts into `pub_map` under `mutex`. `publish_segment_pose()` uses a `scoped_try_lock`, so pose publishing for a segment is skipped (TF is still sent) on frames where the lock is contended or the publisher isn't ready yet.
5. `update_status` feeds `ConsoleStatusLine` and `print_status_if_due` prints one console line per `status_period_s`, also while no frame arrives (`95.9 Hz | dropped 0 | latency 30.6 ms (30.1-31.3) | <subject> q=1.27 occl 0% cams 3-8`).

Topic name: `<namespace>/<subject>/<segment>`; TF child frame: `<subject>_<segment>`. Vicon names are converted by `vicon_receiver::to_ros_name_token()` (`ros_conversions.hpp`): characters other than `[A-Za-z0-9_]` become `_`, names not starting with a letter get a `vicon` prefix (e.g. `g1-unitree-torso-cam` → `g1_unitree_torso_cam`). A publisher that cannot be created is logged and skipped (never retried); of two segments mapping to the same topic, the second is refused. `client.launch.py` has a single required argument, `params_file` (no parameter defaults of its own): `ros2 launch vicon_receiver client.launch.py params_file:=config/vicon_client.yaml`; `scripts/launch.sh` uses `ros2 run` with the same YAML after the reachability check.

## Shared code (`vicon_common`)

Static library without ROS, linked by the node and the probe (namespace `vicon_receiver`, headers in `include/vicon_receiver/`; few files by design: `frame_timing`, `utilities`, `console_status_line`): `DroppedFrameCounter` (missed frames from frame-number gaps), `StartupLeftoverFrameFilter` (startup leftovers), `LocalClockRateMonitor` (local clock rate vs frame numbers), `ConsoleStatusLine`, SDK enum names (`result_name`, ...), and `AcquisitionClock`: estimated camera acquisition time on the local clock = `t_ref + (frame - frame_ref)/rate`, with `t_ref` = minimum over a sliding window (in frames) of `receive - latency_total - (frame - frame_ref)/rate` (transport delays are positive, so the minimum is the least delayed frame; the window follows clock drift). Restarts on non-increasing frame numbers or a rate change. The hardware frame number is always 0 on the lab Tracker: don't use it. **TODO** (in `frame_timing.hpp`): improve the acquisition-time estimate; first find out whether the limit is what the Vicon stream provides (latency covers the Tracker pipeline only, no timecode, no hardware frame time; UDP multicast mode untested) or the receive side.

## Probe (`vicon_probe`)

Plain C++ executable (no rclcpp) in the same package, built from `vicon_probe_core` (`src/probe/`, `include/vicon_receiver/probe/`, namespace `vicon_probe`) on top of `vicon_common`. It uses the **same vendored SDK and calls as the node**, so what it sees is what the node can get. It connects once with the required `--stream-mode` (`probe.sh` passes the YAML `stream_mode`, `--stream-mode` overrides), enables every stream, and writes one JSON document: probe/connection/stream/server/cameras/devices/subject structure, then per frame the frame numbers, receive time, latency total + per stage, timecode, per subject quality, segment poses with occlusion flags, markers with the camera IDs of their rays, labeled/unlabeled markers; SDK calls that fail are listed in `errors` arrays instead of being dropped. Units stay SDK-native (mm, quaternion x,y,z,w). Exit codes: 0 ok, 1 args, 2 connect failed, 3 output file, 4 stream lost. Frames left over from the server's previous session are flagged `previous_frame_stale` (same rule as the node) and not counted as missed. `scripts/probe.sh` adds the reachability check and writes to `probe_output/` (git-ignored). The SDK `.so` has no runpath and needs the vendored Boost: link `Boost::thread Boost::chrono` with anything linking only the SDK, and use `APPEND_LIBRARY_DIRS` for its gtests.

What a live recording (2026-10-30, lab Tracker at 10.250.223.40) showed — keep in mind for message design:
- Subject/segment/marker names contain `-` (`g1-unitree-torso-cam`): convert them with `vicon_receiver::to_ros_name_token()` before using them in ROS names.
- `GetObjectQuality` is ~1.3, **not** the 0..1 the SDK docs claim; treat it as an opaque Tracker metric.
- An occluded segment comes with an all-zero pose (quaternion `[0,0,0,0]`): never publish it as a valid pose.
- Each marker is seen by 3–7 cameras (ray contributions): a usable per-marker quality signal.
- Latency total ~31 ms (mostly "Data Delivered"); 100 Hz; no timecode, no camera calibration, no devices, no unlabeled markers from this Tracker.
- After connecting, the server first sends **1–2 frames left over from its previous session**, received back to back, usually followed by a frame-number jump; with the Tracker not streaming (seen 2026-10-30 afternoon: `GetFrame` → `NoFrame` after them) two leftovers were consecutive. The node holds them back with `StartupLeftoverFrameFilter` (3 consecutive frame numbers spread over ≥ half their nominal duration); the probe records them and flags them with `is_stale_previous_frame` (any jump within the first `kConnectionStartupFrames` (10) frames is stale leftovers, later only implausible jumps (> rate × (interval + 1 s))).
- Stream modes (2000 frames each, probe, after removing the local clock skew and a 1.29 s timesync step): both delivered every frame; arrival delay above the best case median ~4 ms, p95 ~7 ms, max ~10 ms in both ClientPull and ServerPush. ServerPush is used because it never skips frames when the consumer is slow.

## Live viewer (`vicon_receiver_tools.viewer`, viser)

Browser viewer instead of RViz (the user wanted low CPU): `scripts/viz.sh` → the conda environment's `python -m vicon_receiver_tools.viewer`. It subscribes to `ViconFrame` **raw** (serialized; the callback only stores bytes) and decodes the latest message once per scene update (`update_rate_hz`, 30): subject axes (hidden when occluded), trails (sampled at the scene rate, redrawn at `trail_update_rate_hz`), markers coloured by camera count (green ≥ `good_camera_count`, yellow fewer; occluded markers have no position and are hidden, as is a subject's whole cloud when none is visible), rate panel falling to 0 when the stream stops (no scene update without a new frame), segments with a non-unit quaternion treated as occluded, the G1 URDF (`ViserUrdf` under `/robot`) placed at `T_vicon_body · mount · T_mount_root`, camera frustums (FOV from the intrinsics in `config/viewer.yaml`, read from the robot's `camera_info`), telemetry panel, and mount sliders + "Save mount". Measured steady state: viewer ~14 %, node ~5 % of one core.

- Config: `config/viewer.yaml` (every entry required, `viewer_config.py`). Mount: `config/g1_mount.yaml` = pose of `torso_link` in the Vicon body frame; **estimated** by `mount_estimate.py` (marker constellation from a probe recording fitted to the meshes rigidly attached to `torso_link`, 1 cm surface offset, the 2 highest markers forced onto `head_link` per the user; 12 yaw starts; ~2.5 min) — RMS 5.9 mm but surface fits are ambiguous: verify visually and adjust with the sliders. A proper body→camera calibration is a separate, later task.
- Conda environment `vicon_receiver` (`environment.yml`; no local venv, user decision): Python 3.12 + NumPy 1.26 to match the compiled modules of the system ROS 2, viser 1.1.1, yourdfpy 0.0.60, trimesh 5.2.0, h5py, pytest, clang-format, shellcheck. Created or updated by `build.sh` if conda is installed. Scripts find its interpreter with `conda env list` (`tools_env_python` in `common.sh`), so it needs no activation; ROS 2 comes through the `PYTHONPATH` of its `setup.bash`. `tests/run_tests.sh` runs pytest with it. yourdfpy calls the mesh resolver with the keyword `fname`; the G1 URDF uses `package://legged_nav_system/...` paths resolved by tail matching.
- On WSL, a web server bound to 127.0.0.1 is reachable from Linux and from the Windows browser (`localhost`); only *closed* local ports hang.

## Recording and export

- `scripts/record.sh`: requires the node (`ros2 node list --no-daemon`), creates `<output>/<UTC>_<tag>/` (default `recordings/`, git-ignored; `-o` for an external disk) with `bag/` (`ros2 bag record -s mcap -e '^(/<ns>/.*|/tf|/tf_static)$'`), `config/` copies (node, viewer, mount), `meta.yaml` (start/stop UTC, machine, git rev + dirty, vicon host, stream mode, **clock_source**) and `bag_info.txt`. **MCAP is the only output.**
- Stopping: the recorder runs as a child; `record.sh` traps INT and TERM and forwards **SIGTERM** (the recorder closes the bag cleanly on it). SIGINT is not forwarded because a background child of a non-interactive shell inherits SIGINT as ignored (POSIX); for the same reason a `record.sh` started in the background by a script must be stopped with SIGTERM, not SIGINT. `record.sh` exits non-zero if the recorder stopped by itself with an error or left no `bag/metadata.yaml` (bag not closed); config entries are read before the session directory is created.
- `scripts/export.sh SESSION --format csv[,hdf5]` (format required, nothing implicit) → `vicon_receiver_tools.export`: reads `/<ns>/frame` (namespace from the session's copied config) with `rosbag2_py`, feeds `CsvExporter` (`frames.csv`, `poses_<frame_id>.csv`, `markers_<subject>.csv`) and `Hdf5Exporter` (`session.h5`, arrays aligned on the frame index, NaN/occluded=true where missing, meta + configs as attributes). Add a format = add a class with `add`/`finish`/`abort` to `EXPORTERS` (`abort` runs when reading fails half-way: the CSV exporter deletes its partial files).
- Tested end to end with `tests/fixtures/fake_vicon_frames.py` (node named `vicon_client`, injected occlusions every 50 frame numbers and a 3-frame gap every 300 frames; the test waits for the MCAP file before timing the recording) in `tests/integration/test_record_export.py` (own ROS domain 89). Live check: 33.5 s → 3340 ViconFrame/PoseStamped/TF messages, exported to CSV/HDF5.

## Vendored dependencies

`vicon_receiver/third_party/{x86_64,aarch64}/` holds the Vicon DataStream SDK 1.12 (`libViconDataStreamSDK_CPP.so` + headers) and Boost 1.75 (headers + `system`/`thread`/`chrono`/`timer` shared libs). This is most of the repo's ~6800 tracked files; ignore it when searching.

`CMakeLists.txt` selects the arch from `CMAKE_SYSTEM_PROCESSOR` (fails on anything else), wraps the vendored libraries as IMPORTED targets (`ViconDataStreamSDK::ViconDataStreamSDK_CPP`, `Boost::system`, etc.) without `find_package(Boost)`, and installs the `.so` files into `install/lib` alongside `INSTALL_RPATH $ORIGIN/../lib`. System Boost or SDK installs are neither needed nor used. If you add a Boost component, you must add both the `find_library` + IMPORTED target and the install rule, and the library must exist for both architectures.
