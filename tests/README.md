# Tests

All the tests of the repository live in this folder.

| Framework | Covers | Runs with |
|---|---|---|
| **GoogleTest** (`ament_cmake_gtest`) | C++ unit tests of the node code | `colcon test` |
| **pytest** | Python tools, shell helpers, script CLIs, static checks, node and launch integration | `python3 -m pytest tests` |

The `vicon_receiver_tools` Python package is imported from its sources (`conftest.py` puts it on
`sys.path`), so `tests/unit` runs without building the workspace.

## Running

```bash
tests/run_tests.sh                 # build with tests, run C++ then Python tests
tests/run_tests.sh --cpp           # C++ unit tests only
tests/run_tests.sh --python        # Python tests only
tests/run_tests.sh --no-build      # skip the colcon build
tests/run_tests.sh --python -- -k reachable -v   # extra arguments go to pytest
```

`run_tests.sh` needs ROS 2 Jazzy installed (`scripts/build.sh`). The Python unit tests also run on
their own without ROS: `python3 -m pytest tests/unit`.

None of the tests need a Vicon system. The network is replaced by a local TCP listener, a
documentation address that is never assigned (192.0.2.1), or fake `ip`/`timeout` commands.

## Layout

```
tests/
  run_tests.sh                  single entry point (colcon build/test + pytest)
  pytest.ini                    pytest configuration (testpaths, markers)
  conftest.py                   shared fixtures: script runner, config factory, TCP listener,
                                ROS environment, long-running process helper
  fixtures/
    fake_bin/ip                 fake 'ip' (addresses, routes) driven by FAKE_IP_* variables
    fake_bin/timeout            fake 'timeout' reproducing TCP probe outcomes (FAKE_TIMEOUT_*)
    fake_vicon_frames.py        synthetic ViconFrame publisher named vicon_client (record/export)
    replay_probe.py             replays a probe recording as ViconFrame (viewer checks without a Tracker)
  unit/
    cpp/test_vicon_receiver_node_params.cpp   GoogleTest: parameter contract of the node (incl. invalid values)
    cpp/test_frame_timing.cpp          GoogleTest: acquisition time, dropped/stale frames, leftover frames, local clock rate
    cpp/test_utilities.cpp             GoogleTest: running statistics, SDK names, TCP keepalive
    cpp/test_ros_conversions.cpp       GoogleTest: Vicon names -> ROS names, SDK results -> messages, TF, times
    cpp/test_console_status_line.cpp       GoogleTest: console telemetry line
    cpp/test_vicon_receiver_node_publishers.cpp  GoogleTest: publisher creation with Vicon names
    cpp/test_probe_*.cpp               GoogleTest: probe JSON writer and options
    test_common_sh.py                  scripts/lib/common.sh functions
    test_ros_env_sh.py                 scripts/lib/ros_env.sh (setup files, WSL mirrored networking detection/profile), source_ros_env.sh
    test_scripts_cli.py                command line and usage layout of scripts/* and tests/run_tests.sh
    test_static_checks.py              shellcheck and clang-format (skipped if not installed)
    test_tools_config.py               vicon_receiver_tools.config (parameters file access)
    test_tools_network.py              vicon_receiver_tools.network (local subnet report)
    test_tools_viewer.py               viewer data views, colours, trails, telemetry, frame buffer, configs
    test_tools_mount.py                robot model access and mount estimate (head/shoulder pairs, synthetic robot; needs the conda environment)
    test_tools_export.py               export to CSV without ROS (fake reader), partial-output cleanup
  integration/
    test_dds.py                 two ROS 2 processes exchange messages (WSL DDS fix guard)
    test_messages.py            vicon_receiver_msgs field names and types (interface pin)
    test_node_live.py           vicon_client against a real server: ViconFrame content (vicon_live)
    test_node_params.py         vicon_client with YAML parameter files (FATAL messages, exit codes)
    test_launch_file.py         client.launch.py: only a required params_file argument
    test_launch_sh.py           launch.sh: reachability gate and node start-up
    test_probe.py               vicon_probe and probe.sh: CLI, connection failures, live recording
    test_record_export.py       record.sh (MCAP, metadata, SIGINT stop, missing entry) and export.sh (CSV, HDF5)
```

## How the C++ tests are built

The test sources are in `tests/unit/cpp`, but they are compiled by the `vicon_receiver` package:
its `CMakeLists.txt` adds them when `BUILD_TESTING` is on and the folder exists. The node code is
built as the static library `vicon_receiver_core`, linked by both the `vicon_client` executable and
the tests. The tests construct `ViconReceiverNode` with `rclcpp::NodeOptions` parameter overrides and
never connect to a Vicon server.

`scripts/build.sh` builds with `BUILD_TESTING=OFF`, and `run_tests.sh` rebuilds with it `ON`.

## Integration tests

Tests marked `integration` need ROS 2 and the built workspace. They are skipped (with the reason)
when either is missing. Run only those with `python3 -m pytest tests -m integration`.

Tests marked `vicon_live` record from a real Vicon DataStream server and are skipped unless
`VICON_LIVE_HOST` is set:

```bash
VICON_LIVE_HOST=10.250.223.40 python3 -m pytest tests -m vicon_live
```

## Developer tools

`tests/run_tests.sh` runs pytest in the conda environment of `environment.yml`
(`conda env create -f environment.yml`), which also pins `shellcheck` and `clang-format` for the
static checks. Without it, the system Python runs the tests that do not need the environment, with
the tools found on `PATH`. Don't build with the environment activated: colcon needs the system
Python of ROS 2.
