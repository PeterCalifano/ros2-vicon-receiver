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
  unit/
    cpp/test_communicator_params.cpp   GoogleTest: parameter contract of the node
    cpp/test_communicator_publishers.cpp  GoogleTest: publisher creation with Vicon names
    cpp/test_name_conversion.cpp       GoogleTest: Vicon name -> ROS name conversion
    cpp/test_probe_*.cpp               GoogleTest: probe JSON writer, options, SDK names, frame gaps
    test_common_sh.py                  scripts/lib/common.sh functions
    test_scripts_cli.py                command-line handling of scripts/*.sh
    test_static_checks.py              shellcheck and clang-format (skipped if not installed)
    test_tools_config.py               vicon_receiver_tools.config (parameters file access)
    test_tools_network.py              vicon_receiver_tools.network (local subnet report)
  integration/
    test_node_params.py         vicon_client with YAML parameter files (FATAL messages, exit codes)
    test_launch_sh.py           launch.sh: reachability gate and node start-up
    test_probe.py               vicon_probe and probe.sh: CLI, connection failures, live recording
```

## How the C++ tests are built

The test sources are in `tests/unit/cpp`, but they are compiled by the `vicon_receiver` package:
its `CMakeLists.txt` adds them when `BUILD_TESTING` is on and the folder exists. The node code is
built as the static library `vicon_receiver_core`, linked by both the `vicon_client` executable and
the tests. The tests construct `Communicator` with `rclcpp::NodeOptions` parameter overrides and
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

The static checks use `shellcheck` and `clang-format` from `PATH`, or from the optional virtual
environment `build/.devtools` (`python3 -m venv build/.devtools && build/.devtools/bin/pip install
clang-format==18.1.8 shellcheck-py`). Don't put that environment's `bin/` on `PATH`: its
`python3` has no colcon.
