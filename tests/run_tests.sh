#!/usr/bin/env bash
# =============================================================================
# run_tests.sh - Single entry point for all the tests of the repository.
#
# Runs, in order:
#   1. a colcon build with BUILD_TESTING=ON (the C++ tests are compiled by the
#      vicon_receiver package from tests/unit/cpp);
#   2. the C++ unit tests (GoogleTest) through `colcon test`;
#   3. the Python tests (pytest) in tests/unit and tests/integration.
#
# Options: tests/run_tests.sh --help
# =============================================================================

set -Eeuo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${TESTS_DIR}/../scripts/lib/common.sh"

run_cpp=true
run_python=true
run_build=true
pytest_args=()

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} [--cpp | --python] [--no-build] [-h] [-- PYTEST_ARGS...]

Build the workspace with the C++ tests (BUILD_TESTING=ON), then run the C++
tests (GoogleTest, via colcon test) and the Python tests (pytest, with the
Python of the conda environment if it exists). No Vicon server is needed:
integration tests skip without ROS 2 or the build, live tests without
VICON_LIVE_HOST.

Options:
  --cpp       Run only the C++ unit tests
  --python    Run only the Python tests
  --no-build  Do not rebuild before testing
  -h, --help  Show this help and exit

Arguments after '--' are passed to pytest.

Environment:
  VICON_LIVE_HOST  host[:port] of a real Vicon server for the tests marked
                   vicon_live (skipped when not set)

Examples:
  tests/run_tests.sh
  tests/run_tests.sh --python -- -k reachable -v
  tests/run_tests.sh --cpp --no-build
  VICON_LIVE_HOST=192.0.2.10 tests/run_tests.sh --python -- -m vicon_live
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --cpp)
                run_python=false
                ;;
            --python)
                run_cpp=false
                ;;
            --no-build)
                run_build=false
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            --)
                shift
                pytest_args=("$@")
                break
                ;;
            *)
                die_with_usage "Unknown option: $1"
                ;;
        esac
        shift
    done

    if [[ "${run_cpp}" == false && "${run_python}" == false ]]; then
        die_with_usage "--cpp and --python exclude each other."
    fi
}

# Run the C++ unit tests. Returns non-zero if any test failed.
run_cpp_tests()
{
    # colcon test-result reads every result file of the folder: remove those of earlier runs,
    # which include the results of deleted test targets
    rm -rf "${REPO_ROOT}/build/vicon_receiver/test_results"

    # Only vicon_receiver registers tests with colcon; the Python tools are tested by pytest
    info "Running C++ unit tests (colcon test)"
    python3 -m colcon test \
        --base-paths "${PACKAGE_DIRS[@]}" \
        --packages-select vicon_receiver \
        --ctest-args --output-on-failure

    # colcon test does not fail on test failures: test-result does
    python3 -m colcon test-result --test-result-base build/vicon_receiver --verbose
}

# Run the Python tests. Returns non-zero if any test failed.
run_python_tests()
{
    # The conda environment has pytest, the viewer packages and the code checks; without it the
    # system Python runs the tests that do not need them
    local python=""
    python="$(tools_env_bin python)"
    python="${python:-python3}"
    info "Running Python tests (pytest, ${python})"
    "${python}" -m pytest "${TESTS_DIR}" "${pytest_args[@]}"
}

main()
{
    parse_args "$@"
    cd "${REPO_ROOT}"
    require_ros
    require_colcon
    # Integration tests run several ROS 2 processes, which must see each other
    vicon_configure_dds

    if [[ "${run_build}" == true ]]; then
        colcon_build_workspace Release ON
    fi

    local failures=0
    if [[ "${run_cpp}" == true ]]; then
        run_cpp_tests || failures=$((failures + 1))
    fi
    if [[ "${run_python}" == true ]]; then
        run_python_tests || failures=$((failures + 1))
    fi

    if [[ ${failures} -gt 0 ]]; then
        die "${failures} test suite(s) failed."
    fi
    info "All tests passed."
}

main "$@"
