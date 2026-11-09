#!/usr/bin/env bash
# =============================================================================
# ros_env.sh - ROS 2 environment of this repository: ROS distribution, setup
# files, and the DDS configuration of this machine.
#
# On WSL with mirrored networking, every TCP/UDP packet for 127.0.0.1 is routed
# to Windows, so ROS 2 processes on this machine cannot discover each other.
# vicon_configure_dds detects this and selects config/fastdds_wsl.xml, which
# keeps DDS traffic inside Linux.
#
# Sourced by scripts/lib/common.sh (scripts, which stop on errors) and by
# scripts/source_ros_env.sh (interactive shells). It only defines variables and
# functions prefixed with VICON_/vicon_, leaves the shell options as they were
# and never exits, so it is safe to source in an interactive shell.
# =============================================================================

# The variables are used by the files that source this one.
# shellcheck disable=SC2034

# Repository root (resolved from this file, so it works from any directory)
VICON_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# ROS 2 distribution of the workspace, and the setup files of ROS 2 and of the workspace overlay
VICON_ROS_DISTRO="jazzy"
VICON_ROS_SETUP_FILE="/opt/ros/${VICON_ROS_DISTRO}/setup.bash"
VICON_WORKSPACE_SETUP_FILE="${VICON_REPO_ROOT}/install/setup.bash"

# Address of the Linux loopback device that WSL does not redirect; it must match
# the address in config/fastdds_wsl.xml.
VICON_WSL_LOOPBACK_ADDRESS="10.255.255.254"

# Source a ROS setup file. ROS setup scripts reference unset variables, so 'nounset' is
# disabled while sourcing them and restored afterwards if it was set.
#   $1  setup file
vicon_source_setup_file()
{
    local setup_file="$1"
    local nounset_was_set=false
    if [[ $- == *u* ]]; then
        nounset_was_set=true
    fi
    set +u
    # shellcheck disable=SC1090  # path is only known at runtime
    source "${setup_file}"
    local status=$?
    if [[ "${nounset_was_set}" == true ]]; then
        set -u
    fi
    return "${status}"
}

# Print the path of the repository Fast DDS profile for WSL.
vicon_dds_profile_file()
{
    printf '%s\n' "${VICON_REPO_ROOT}/config/fastdds_wsl.xml"
}

# Return 0 if traffic to 127.0.0.1 does not stay on the Linux loopback device.
# Normally the route is "local 127.0.0.1 dev lo table local ..."; WSL mirrored
# networking gives "127.0.0.1 via ... dev loopback0 table 127 ...".
vicon_localhost_redirected()
{
    local route
    route="$(ip -4 route get 127.0.0.1 2>/dev/null || true)"
    if [[ -z "${route}" ]]; then
        return 1
    fi
    [[ "${route}" != *" dev lo "* ]]
}

# Return 0 if the loopback device has the address used by the WSL profile.
vicon_wsl_loopback_address_present()
{
    local addresses
    addresses="$(ip -4 addr show dev lo 2>/dev/null || true)"
    [[ "${addresses}" == *"inet ${VICON_WSL_LOOPBACK_ADDRESS}/"* ]]
}

# Select the Fast DDS profile for this machine and print what was chosen.
#   - a profile already set in FASTRTPS_DEFAULT_PROFILES_FILE (Fast DDS 2.x) or
#     FASTDDS_DEFAULT_PROFILES_FILE (3.x) is kept;
#   - if localhost traffic is redirected (WSL mirrored networking), the
#     repository profile is exported (both Fast DDS 2.x and 3.x variables) and
#     ROS_AUTOMATIC_DISCOVERY_RANGE is set to SYSTEM_DEFAULT, because LOCALHOST
#     and OFF would override the profile with (redirected) 127.0.0.1 peers;
#   - otherwise nothing changes (default DDS configuration).
# Always returns 0: a missing fix shows up as a warning, not as a failure.
vicon_configure_dds()
{
    local user_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_DEFAULT_PROFILES_FILE:-}}"
    if [[ -n "${user_profile}" ]]; then
        echo "[dds] Keeping the Fast DDS profile already set: ${user_profile}"
        return 0
    fi

    if ! vicon_localhost_redirected; then
        return 0
    fi

    local profile
    profile="$(vicon_dds_profile_file)"
    if ! vicon_wsl_loopback_address_present; then
        echo "[dds] WARNING: localhost is redirected (WSL mirrored networking) but lo has no" \
            "${VICON_WSL_LOOPBACK_ADDRESS}: ${profile} will not work; ROS 2 processes on this" \
            "machine may not see each other." >&2
        return 0
    fi

    export FASTRTPS_DEFAULT_PROFILES_FILE="${profile}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${profile}"
    echo "[dds] localhost is redirected to Windows (WSL mirrored networking):" \
        "using ${profile}"

    # With LOCALHOST or OFF the ROS middleware replaces the discovery settings of the profile
    # with 127.0.0.1 peers, which are redirected again: let the profile decide instead.
    if [[ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-SYSTEM_DEFAULT}" != "SYSTEM_DEFAULT" ]]; then
        echo "[dds] ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE} replaced by" \
            "SYSTEM_DEFAULT (discovery is set by the profile; it stays on this machine)"
    fi
    export ROS_AUTOMATIC_DISCOVERY_RANGE="SYSTEM_DEFAULT"
}
