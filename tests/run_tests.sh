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
# Usage:
#   tests/run_tests.sh [--cpp | --python] [--no-build] [-- PYTEST_ARGS...]
#
# Examples:
#   tests/run_tests.sh
#   tests/run_tests.sh --python -- -k reachable -v
# =============================================================================

set -Eeuo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${TESTS_DIR}/../scripts/lib/common.sh"

# Packages built and tested by colcon (same list as scripts/build.sh)
readonly PACKAGE_DIRS=(
    "vicon_receiver"
    "vicon_receiver_tools"
)

run_cpp=true
run_python=true
run_build=true
pytest_args=()

usage()
{
    cat <<EOF
Usage: $(basename "$0") [options] [-- PYTEST_ARGS...]

Build the workspace with tests enabled, then run the C++ (GoogleTest, via
colcon test) and Python (pytest) tests.

Options:
  --cpp       Run only the C++ unit tests
  --python    Run only the Python tests
  --no-build  Do not rebuild before testing
  -h, --help  Show this help and exit

Arguments after '--' are passed to pytest.
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
                usage >&2
                die "Unknown option: $1"
                ;;
        esac
        shift
    done
}

build_with_tests()
{
    info "Building ${PACKAGE_DIRS[*]} with BUILD_TESTING=ON"
    python3 -m colcon build \
        --base-paths "${PACKAGE_DIRS[@]}" \
        --symlink-install \
        --cmake-args \
        "-DCMAKE_BUILD_TYPE=Release" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        "-DBUILD_TESTING=ON"
}

# Run the C++ unit tests. Returns non-zero if any test failed.
run_cpp_tests()
{
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
    info "Running Python tests (pytest)"
    python3 -m pytest "${TESTS_DIR}" "${pytest_args[@]}"
}

main()
{
    parse_args "$@"
    cd "${REPO_ROOT}"
    require_ros
    require_colcon

    if [[ "${run_build}" == true ]]; then
        build_with_tests
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
