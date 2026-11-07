#!/usr/bin/env bash
# =============================================================================
# record.sh - Record a session of the vicon_client node to an MCAP bag.
#
# Run it while the node is running (scripts/launch.sh). Every session gets its
# own directory:
#
#   <output>/<YYYYmmddTHHMMSSZ>[_<tag>]/
#     bag/             rosbag2, MCAP storage: ViconFrame, PoseStamped, /tf, /tf_static
#     config/          copies of the node, viewer and mount configuration
#     meta.yaml        start/stop time, machine, git revision, clock source, ...
#     bag_info.txt     ros2 bag info at the end of the session
#
# MCAP is the only output: conversion to CSV/HDF5 is a separate, optional step
# (scripts/export.sh). Stop the recording with Ctrl+C.
#
# Usage:
#   scripts/record.sh [-c FILE] [-t TAG] [-o DIR]
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

config_file="${DEFAULT_CONFIG_FILE}"
output_dir="${REPO_ROOT}/recordings"
tag=""

usage()
{
    cat <<USAGE
Usage: $(basename "$0") [options]

Record the vicon_client topics (ViconFrame, PoseStamped, TF) of a running node
to an MCAP bag, in a new session directory. Stop with Ctrl+C.

Options:
  -c, --config FILE  Node parameters file (default: config/vicon_client.yaml);
                     its namespace selects the topics, and it is copied
  -t, --tag TAG      Name appended to the session directory (letters, digits, - _)
  -o, --output DIR   Where sessions are created (default: recordings/), e.g. an
                     external disk (/mnt/<drive>/... on WSL)
  -h, --help         Show this help and exit
USAGE
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config | -t | --tag | -o | --output)
                if [[ $# -lt 2 ]]; then
                    die "Option $1 requires a value."
                fi
                case "$1" in
                    -c | --config) config_file="$2" ;;
                    -t | --tag) tag="$2" ;;
                    *) output_dir="$2" ;;
                esac
                shift
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            *)
                usage >&2
                die "Unknown option: $1"
                ;;
        esac
        shift
    done

    if [[ -n "${tag}" && ! "${tag}" =~ ^[A-Za-z0-9_-]+$ ]]; then
        die "Invalid tag '${tag}': use letters, digits, '-' and '_' only."
    fi
}

# Fail early if the node is not running: the bag would only contain nothing.
require_node_running()
{
    local nodes=""
    # --no-daemon: the ros2 CLI daemon is unreachable on WSL mirrored networking
    nodes="$(timeout 20 ros2 node list --no-daemon 2>/dev/null || true)"
    if ! grep -qx "/${VICON_NODE_NAME}" <<<"${nodes}"; then
        die "Node /${VICON_NODE_NAME} not found: start it first (scripts/launch.sh)."
    fi
}

# Write the session metadata known at the start.
write_start_metadata()
{
    local session_dir="$1"
    local namespace="$2"
    local git_revision=""
    git_revision="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    local git_dirty="false"
    if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain 2>/dev/null)" ]]; then
        git_dirty="true"
    fi
    local clock_source=""
    clock_source="$(cat /sys/devices/system/clocksource/clocksource0/current_clocksource 2>/dev/null || echo unknown)"

    cat >"${session_dir}/meta.yaml" <<META
# Session recorded by scripts/record.sh
tag: "${tag}"
start_utc: "$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
machine: "$(hostname)"
git_revision: "${git_revision}"
git_dirty: ${git_dirty}
ros_distro: "${ROS_DISTRO:-unknown}"
vicon_host: "$(read_config_param "${config_file}" hostname)"
stream_mode: "$(read_config_param "${config_file}" stream_mode)"
topics_regex: "^(/${namespace}/.*|/tf|/tf_static)$"
storage: "mcap"
# Local clock source: on WSL, 'tsc' was measured running ~4 % fast (see CLAUDE.md)
clock_source: "${clock_source}"
META
}

# Complete the metadata and the bag summary at the end of the session.
finish_session()
{
    local session_dir="$1"
    echo "stop_utc: \"$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)\"" >>"${session_dir}/meta.yaml"
    if [[ -d "${session_dir}/bag" ]]; then
        ros2 bag info "${session_dir}/bag" >"${session_dir}/bag_info.txt" 2>&1 || true
    fi
    info "Session saved in ${session_dir}"
}

main()
{
    parse_args "$@"
    require_config_file "${config_file}"
    config_file="$(realpath "${config_file}")"
    require_ros
    require_install
    vicon_configure_dds
    require_node_running

    local namespace=""
    namespace="$(read_config_param "${config_file}" namespace)"
    namespace="${namespace#/}"

    # One directory per session; never overwrite an existing one
    mkdir -p "${output_dir}"
    local session_name=""
    session_name="$(date -u +%Y%m%dT%H%M%SZ)${tag:+_${tag}}"
    local session_dir="${output_dir}/${session_name}"
    if [[ -e "${session_dir}" ]]; then
        die "Session directory already exists: ${session_dir}"
    fi
    mkdir -p "${session_dir}/config"

    # Configuration used for this session
    cp "${config_file}" "${session_dir}/config/vicon_client.yaml"
    for extra_config in "${REPO_ROOT}/config/viewer.yaml" "${REPO_ROOT}/config/g1_mount.yaml"; do
        if [[ -f "${extra_config}" ]]; then
            cp "${extra_config}" "${session_dir}/config/"
        fi
    done
    write_start_metadata "${session_dir}" "${namespace}"

    info "Recording to ${session_dir}/bag (Ctrl+C to stop)"
    ros2 bag record -s mcap -o "${session_dir}/bag" \
        -e "^(/${namespace}/.*|/tf|/tf_static)$" &
    local recorder_pid=$!

    # Ctrl+C (SIGINT) or SIGTERM stop the recording: the recorder gets SIGTERM, on which it
    # closes the bag cleanly. SIGINT is not forwarded because a recorder started by a script
    # in the background inherits SIGINT as ignored (POSIX), e.g. under a service manager.
    trap 'kill -TERM "${recorder_pid}" 2>/dev/null || true' INT TERM

    # wait returns early when a trapped signal arrives: wait again until the recorder is done
    while kill -0 "${recorder_pid}" 2>/dev/null; do
        wait "${recorder_pid}" || true
    done
    trap - INT TERM

    finish_session "${session_dir}"
}

main "$@"
