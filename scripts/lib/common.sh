#!/usr/bin/env bash
# =============================================================================
# common.sh - Shared helpers for the scripts in scripts/.
#
# Sourced (not executed) by launch.sh, probe.sh and the other entry points.
# Provides:
#   - logging helpers (info, warn, die)
#   - environment checks (ROS 2 installation, built workspace)
#   - reading values from the node parameters file (YAML)
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

# Repository layout (paths are resolved from this file, so callers can run from any directory)
COMMON_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${COMMON_SH_DIR}/../.." && pwd)"
readonly COMMON_SH_DIR REPO_ROOT

readonly ROS_DISTRO_NAME="jazzy"
readonly ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
readonly WORKSPACE_SETUP_FILE="${REPO_ROOT}/install/setup.bash"
readonly DEFAULT_CONFIG_FILE="${REPO_ROOT}/config/vicon_client.yaml"

# Sources of the vicon_receiver_tools Python package. The helpers below run its modules
# from the sources, so they also work before the workspace is built.
readonly TOOLS_PYTHON_DIR="${REPO_ROOT}/vicon_receiver_tools"

# Name of the node, which is also the top-level key of the parameters file
readonly VICON_NODE_NAME="vicon_client"

# Port of the Vicon DataStream server (fixed by the Vicon protocol, not a tuning parameter)
readonly VICON_DATASTREAM_PORT=801

# Name of the calling script, used as prefix of every message
SCRIPT_NAME="$(basename "$0")"

# -----------------------------------------------------------------------------
# Logging
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

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------

# Source a ROS setup file. ROS setup scripts reference unset variables, so
# 'nounset' is disabled while sourcing them.
source_setup_file()
{
    local setup_file="$1"
    set +u
    # shellcheck disable=SC1090  # path is only known at runtime
    source "${setup_file}"
    set -u
}

# Make sure ROS 2 is installed and source it.
require_ros()
{
    if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
        die "ROS 2 ${ROS_DISTRO_NAME} not found (${ROS_SETUP_FILE} is missing)." \
            "Run scripts/build.sh first: it installs ROS 2 and builds the workspace."
    fi
    source_setup_file "${ROS_SETUP_FILE}"
}

# Make sure the workspace has been built and source its overlay.
require_install()
{
    if [[ ! -f "${WORKSPACE_SETUP_FILE}" ]]; then
        die "Workspace not built (${WORKSPACE_SETUP_FILE} is missing)." \
            "Run scripts/build.sh first."
    fi
    source_setup_file "${WORKSPACE_SETUP_FILE}"
}

# Make sure colcon is importable by the python3 in use. An active virtual
# environment without system site packages hides it.
require_colcon()
{
    if ! python3 -c "import colcon_core" >/dev/null 2>&1; then
        die "colcon is not available to $(command -v python3)." \
            "If a Python virtual environment is active, deactivate it or recreate it" \
            "with --system-site-packages, then run this script again."
    fi
}

# Run a module of the vicon_receiver_tools Python package from its sources.
#   $1   module name (e.g. 'config' for vicon_receiver_tools.config)
#   $2.. arguments of the module
run_tools_module()
{
    local module="$1"
    shift
    PYTHONPATH="${TOOLS_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
        python3 -m "vicon_receiver_tools.${module}" "$@"
}

# Make sure a parameters file exists.
require_config_file()
{
    local config_file="$1"
    if [[ ! -f "${config_file}" ]]; then
        die "Config file not found: ${config_file}"
    fi
}

# -----------------------------------------------------------------------------
# Parameters file
# -----------------------------------------------------------------------------

# Print the value of one parameter of the vicon_client node from a parameters file
# (see vicon_receiver_tools/vicon_receiver_tools/config.py).
#   $1  parameters file (YAML)
#   $2  parameter name (key under vicon_client.ros__parameters)
# Terminates with a clear error if the file cannot be parsed or the key is missing.
# Call it as 'value="$(read_config_param ...)"' with 'value' declared beforehand:
# 'local value="$(...)"' would hide the failure from 'set -e'.
read_config_param()
{
    local config_file="$1"
    local param_name="$2"

    # On success the module prints only the value; on failure only the error message.
    local output=""
    if ! output="$(run_tools_module config \
        "${config_file}" "${param_name}" --node "${VICON_NODE_NAME}" 2>&1)"; then
        die "Invalid config file: ${output}"
    fi
    printf '%s\n' "${output}"
}

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

# -----------------------------------------------------------------------------
# Vicon reachability
# -----------------------------------------------------------------------------

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
