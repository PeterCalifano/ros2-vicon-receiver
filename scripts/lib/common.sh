#!/usr/bin/env bash
# =============================================================================
# common.sh - Shared helpers of the scripts in scripts/ and tests/run_tests.sh.
#
# Sourced (not executed) by every script. Provides:
#   - logging and errors (info, warn, die, die_with_usage, require_option_value)
#   - the ROS 2 environment (require_ros, require_workspace; scripts/lib/ros_env.sh)
#   - the colcon build of the packages of this repository (colcon_build_workspace)
#   - the Python tools: from their sources (run_tools_module) or in their conda
#     environment (tools_env_python, tools_env_bin, exec_tools_env_module)
#   - reading values from the YAML config files (read_config_param, read_config_key_path)
#   - a reachability check of the Vicon DataStream server with clear errors
#
# Environment:
#   VICON_CHECK_TIMEOUT_S   TCP connect timeout of the reachability check [s] (default: 3)
# =============================================================================

# Constants defined here are used by the scripts that source this file.
# shellcheck disable=SC2034

# Guard against double sourcing
if [[ -n "${VICON_COMMON_SH_SOURCED:-}" ]]; then
    return 0
fi
readonly VICON_COMMON_SH_SOURCED=1

# ROS distribution, setup files and DDS configuration, shared with scripts/source_ros_env.sh
# shellcheck source=scripts/lib/ros_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/ros_env.sh"

# Repository layout (resolved by ros_env.sh, so the scripts can run from any directory)
readonly REPO_ROOT="${VICON_REPO_ROOT}"
readonly DEFAULT_CONFIG_FILE="${REPO_ROOT}/config/vicon_client.yaml"
readonly DEFAULT_VIEWER_CONFIG_FILE="${REPO_ROOT}/config/viewer.yaml"

# Packages of this repository, relative to the repository root (colcon workspace)
readonly -a PACKAGE_DIRS=(
    "vicon_receiver_msgs"
    "vicon_receiver"
    "vicon_receiver_tools"
)

# Sources of the vicon_receiver_tools Python package. The helpers below run its modules
# from the sources, so they also work before the workspace is built.
readonly TOOLS_PYTHON_DIR="${REPO_ROOT}/vicon_receiver_tools"

# Conda environment of the Python tools (viewer, mount estimate, export, code checks), created by
# scripts/build.sh. Its name is only written in this file.
readonly TOOLS_ENVIRONMENT_FILE="${REPO_ROOT}/environment.yml"

# Name of the node, which is also the top-level key of the parameters file
readonly VICON_NODE_NAME="vicon_client"

# Port of the Vicon DataStream server (fixed by the Vicon protocol, not a tuning parameter)
readonly VICON_DATASTREAM_PORT=801

# Name of the calling script, used in the usage and as prefix of every message
SCRIPT_NAME="$(basename "$0")"

# -----------------------------------------------------------------------------
# Logging and errors
# -----------------------------------------------------------------------------

# Print an informational message to stdout.
info()
{
    echo "[${SCRIPT_NAME}] $*"
}

# Print a warning to stderr.
warn()
{
    echo "[${SCRIPT_NAME}] WARNING: $*" >&2
}

# Print an error to stderr and terminate with exit code 1.
# Additional arguments after the first are printed as indented detail lines.
die()
{
    echo "[${SCRIPT_NAME}] ERROR: $1" >&2
    shift
    local line=""
    for line in "$@"; do
        echo "    ${line}" >&2
    done
    exit 1
}

# Print the usage of the calling script (its 'usage' function) to stderr, then stop like die.
# For command lines that cannot be understood: unknown option, missing argument.
die_with_usage()
{
    usage >&2
    die "$@"
}

# Stop if an option that takes a value has none.
#   $1  the option (e.g. --config)
#   $2  number of arguments left, the option included ('$#' in parse_args)
require_option_value()
{
    local option="$1"
    local remaining="$2"
    if [[ "${remaining}" -lt 2 ]]; then
        die "Option ${option} requires a value."
    fi
}

# -----------------------------------------------------------------------------
# ROS 2 environment and build
# -----------------------------------------------------------------------------

# Make sure ROS 2 is installed and source it.
require_ros()
{
    if [[ ! -f "${VICON_ROS_SETUP_FILE}" ]]; then
        die "ROS 2 ${VICON_ROS_DISTRO} not found (${VICON_ROS_SETUP_FILE} is missing)." \
            "Run scripts/build.sh first: it installs ROS 2 and builds the workspace."
    fi
    vicon_source_setup_file "${VICON_ROS_SETUP_FILE}"
}

# Make sure ROS 2 is installed and the workspace has been built, and source both.
require_workspace()
{
    require_ros
    if [[ ! -f "${VICON_WORKSPACE_SETUP_FILE}" ]]; then
        die "Workspace not built (${VICON_WORKSPACE_SETUP_FILE} is missing)." \
            "Run scripts/build.sh first."
    fi
    vicon_source_setup_file "${VICON_WORKSPACE_SETUP_FILE}"
}

# Make sure colcon is importable by the python3 in use. An active virtual
# environment without system site packages hides it.
require_colcon()
{
    if ! python3 -c "import colcon_core" >/dev/null 2>&1; then
        die "colcon is not available to $(command -v python3)." \
            "If a conda environment is active, deactivate it (conda deactivate): the workspace" \
            "is built with the system Python of ROS 2. Then run this script again."
    fi
}

# Build the packages of this repository with colcon. build/, install/ and log/ are created at
# the repository root.
#   $1  CMake build type (Release or Debug)
#   $2  BUILD_TESTING: ON compiles the C++ tests (tests/unit/cpp), OFF skips them
colcon_build_workspace()
{
    local build_type="$1"
    local build_testing="$2"
    info "Building ${PACKAGE_DIRS[*]} (${build_type}, BUILD_TESTING=${build_testing})"
    (
        cd "${REPO_ROOT}" || exit 1
        python3 -m colcon build \
            --base-paths "${PACKAGE_DIRS[@]}" \
            --symlink-install \
            --cmake-args \
            "-DCMAKE_BUILD_TYPE=${build_type}" \
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
            "-DBUILD_TESTING=${build_testing}"
    )
}

# -----------------------------------------------------------------------------
# Python tools
# -----------------------------------------------------------------------------

# Print the PYTHONPATH that imports vicon_receiver_tools from its sources.
tools_pythonpath()
{
    printf '%s\n' "${TOOLS_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
}

# Run a module of the vicon_receiver_tools Python package from its sources, with the python3
# in use (for the modules that need only the standard library and PyYAML: config, network).
#   $1   module name (e.g. 'config' for vicon_receiver_tools.config)
#   $2.. arguments of the module
run_tools_module()
{
    local module="$1"
    shift
    PYTHONPATH="$(tools_pythonpath)" python3 -m "vicon_receiver_tools.${module}" "$@"
}

# Print the name of the conda environment of the Python tools.
tools_env_name()
{
    sed -n 's/^name:[[:space:]]*//p' "${TOOLS_ENVIRONMENT_FILE}"
}

# Print the conda command, or nothing if conda is not installed.
conda_command()
{
    echo "${CONDA_EXE:-$(command -v conda || true)}"
}

# Print the directory of the conda environment of the Python tools; nothing if it does not exist.
# The environment does not have to be activated.
tools_env_prefix()
{
    local conda_bin=""
    conda_bin="$(conda_command)"
    if [[ -n "${conda_bin}" ]]; then
        "${conda_bin}" env list 2>/dev/null |
            awk -v name="$(tools_env_name)" '$1 == name { print $NF }' || true
    fi
}

# Print the path of a program of the tools environment (e.g. python, clang-format); nothing if
# the environment or the program does not exist.
#   $1  program name
tools_env_bin()
{
    local program="$1"
    local prefix=""
    prefix="$(tools_env_prefix)"
    if [[ -n "${prefix}" && -x "${prefix}/bin/${program}" ]]; then
        printf '%s\n' "${prefix}/bin/${program}"
    fi
}

# Print the Python interpreter of the tools environment; error if the environment does not exist.
tools_env_python()
{
    local python=""
    python="$(tools_env_bin python)"
    if [[ -z "${python}" ]]; then
        die "Conda environment '$(tools_env_name)' not found." \
            "Create it with: conda env create -f ${TOOLS_ENVIRONMENT_FILE}"
    fi
    printf '%s\n' "${python}"
}

# Replace the shell with a module of vicon_receiver_tools, run from its sources by the Python of
# the tools environment (viewer, export: they need its packages).
#   $1   module name (e.g. 'viewer' for vicon_receiver_tools.viewer)
#   $2.. arguments of the module
exec_tools_env_module()
{
    local module="$1"
    shift
    local python=""
    python="$(tools_env_python)"
    PYTHONPATH="$(tools_pythonpath)" exec "${python}" -m "vicon_receiver_tools.${module}" "$@"
}

# -----------------------------------------------------------------------------
# Config files
# -----------------------------------------------------------------------------

# Make sure a config file exists.
#   $1  config file
require_config_file()
{
    local config_file="$1"
    if [[ ! -f "${config_file}" ]]; then
        die "Config file not found: ${config_file}"
    fi
}

# Print one value of a YAML config file (see vicon_receiver_tools/vicon_receiver_tools/config.py).
# Terminates with a clear error if the file cannot be parsed or the key is missing.
#   $1   config file
#   $2.. key and options of the config module
read_config_value()
{
    local config_file="$1"
    shift
    # On success the module prints only the value; on failure only the error message.
    local output=""
    if ! output="$(run_tools_module config "${config_file}" "$@" 2>&1)"; then
        die "Invalid config file: ${output}"
    fi
    printf '%s\n' "${output}"
}

# Print the value of one parameter of the vicon_client node from a parameters file.
#   $1  parameters file (YAML)
#   $2  parameter name (key under vicon_client.ros__parameters)
# Call it as 'value="$(read_config_param ...)"' with 'value' declared beforehand:
# 'local value="$(...)"' would hide the failure from 'set -e'. Same for read_config_key_path.
read_config_param()
{
    read_config_value "$1" "$2" --node "${VICON_NODE_NAME}"
}

# Print one value of a YAML config file by its dotted key path (e.g. robot.source).
#   $1  config file (YAML)
#   $2  dotted key path
read_config_key_path()
{
    read_config_value "$1" "$2" --key-path
}

# -----------------------------------------------------------------------------
# Vicon reachability
# -----------------------------------------------------------------------------

# Split a "host[:port]" string into the globals VICON_HOST and VICON_PORT.
# The port defaults to the DataStream port when not given.
split_host_port()
{
    local host_spec="$1"

    if [[ "${host_spec}" == *";"* ]]; then
        die "hostname '${host_spec}' lists several servers (';')." \
            "The reachability check supports a single server; use --no-check to skip it."
    fi

    if [[ "${host_spec}" == *:* ]]; then
        VICON_HOST="${host_spec%%:*}"
        VICON_PORT="${host_spec##*:}"
    else
        VICON_HOST="${host_spec}"
        VICON_PORT="${VICON_DATASTREAM_PORT}"
    fi

    if [[ -z "${VICON_HOST}" ]]; then
        die "Empty hostname in '${host_spec}'."
    fi
    if [[ ! "${VICON_PORT}" =~ ^[0-9]+$ ]]; then
        die "Invalid port '${VICON_PORT}' in hostname '${host_spec}'."
    fi
}

# Return 0 if the argument is a dotted IPv4 address.
is_ipv4_address()
{
    [[ "$1" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]
}

# Print the IPv4 address of a host name (or the address itself), or nothing.
resolve_ipv4()
{
    local host="$1"
    if is_ipv4_address "${host}"; then
        printf '%s\n' "${host}"
        return 0
    fi
    # getent fails for unknown names: tolerate it so the caller can report the problem
    getent ahostsv4 "${host}" 2>/dev/null | awk 'NR == 1 { print $1 }' || true
}

# Print a one-line description of the kernel route used to reach an address.
describe_route()
{
    local address="$1"
    local route=""
    route="$(ip -4 route get "${address}" 2>/dev/null | head -n 1 || true)"
    if [[ -z "${route}" ]]; then
        printf 'no route\n'
        return 0
    fi

    # A route through a gateway ("via") means the address is not on a directly
    # connected network, which is unusual for a Vicon system on a lab LAN.
    if [[ "${route}" == *" via "* ]]; then
        printf '%s (through a gateway: not a directly connected network)\n' "${route}"
    else
        printf '%s\n' "${route}"
    fi
}

# Print a one-line report saying whether a local network interface is in the same
# IPv4 subnet as the given address (the usual setup for a Vicon network), listing
# the local addresses otherwise (see vicon_receiver_tools/vicon_receiver_tools/network.py).
describe_local_subnet()
{
    local address="$1"
    ip -o -4 addr show 2>/dev/null | run_tools_module network "${address}" || true
}

# Check that the Vicon DataStream server accepts TCP connections.
#   $1  host name or IPv4 address of the Vicon Tracker PC
#   $2  TCP port of the DataStream server
# Terminates with an error that names the most likely cause if it does not.
check_vicon_reachable()
{
    local host="$1"
    local port="$2"
    local timeout_s="${VICON_CHECK_TIMEOUT_S:-3}"

    # 1) Name resolution (only relevant for host names)
    local address=""
    address="$(resolve_ipv4 "${host}")"
    if [[ -z "${address}" ]]; then
        die "Cannot resolve Vicon host name '${host}'." \
            "Use the IP address of the Vicon Tracker PC in the config file (hostname)."
    fi

    local route=""
    route="$(describe_route "${address}")"
    if [[ "${route}" == "no route" ]]; then
        die "No network route to the Vicon Tracker PC at ${address}." \
            "Is this machine connected to the Vicon network, with an address in the same" \
            "subnet (e.g. 192.168.10.x) on the network card cabled to the Vicon switch?"
    fi

    # 2) TCP connection to the DataStream server. The address and port are passed as
    #    positional arguments to the inner shell instead of being interpolated.
    info "Checking Vicon DataStream server at ${address}:${port} (timeout ${timeout_s} s) ..."
    local tcp_error=""
    local tcp_status=0
    # shellcheck disable=SC2016  # $0/$1 are expanded by the inner shell on purpose
    tcp_error="$(timeout "${timeout_s}" bash -c 'exec 3<>"/dev/tcp/$0/$1"' \
        "${address}" "${port}" 2>&1)" || tcp_status=$?

    if [[ ${tcp_status} -eq 0 ]]; then
        info "Vicon DataStream server reachable at ${address}:${port}."
        return 0
    fi

    # 3) Translate the failure into the most likely cause
    if [[ "${tcp_error}" == *"Connection refused"* ]]; then
        die "The Vicon Tracker PC at ${address} is up, but nothing accepts connections on TCP ${port}." \
            "Is Vicon Tracker running, with the DataStream server enabled?" \
            "Is ${port} the right DataStream port (hostname \"<ip>:<port>\" in the config)?"
    fi

    local cause="no answer within ${timeout_s} s"
    if [[ "${tcp_error}" == *"No route to host"* ]]; then
        cause="no route to host (the address does not answer on the local network)"
    elif [[ "${tcp_error}" == *"Network is unreachable"* ]]; then
        cause="network unreachable"
    elif [[ ${tcp_status} -ne 124 && -n "${tcp_error}" ]]; then
        cause="${tcp_error##*: }"
    fi

    local subnet=""
    subnet="$(describe_local_subnet "${address}")"

    die "Vicon Tracker PC not reachable at ${address}:${port}: ${cause}." \
        "Local network: ${subnet}" \
        "Route used:    ${route}" \
        "Check that:" \
        "  - this machine has an address in the Vicon network (e.g. 192.168.10.x) on the" \
        "    network card cabled to the Vicon switch;" \
        "  - the Vicon Tracker PC is on and its IP matches 'hostname' in the config file;" \
        "  - the firewall of the Tracker PC allows inbound TCP ${port}."
}
