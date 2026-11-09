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
# (scripts/export.sh).
#
# Options: scripts/record.sh --help
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
    cat <<EOF
Usage: ${SCRIPT_NAME} [-c FILE] [-t TAG] [-o DIR] [-h]

Record the topics of a running ${VICON_NODE_NAME} node (ViconFrame, PoseStamped, TF)
to an MCAP bag, in a new session directory <DIR>/<UTC time>[_TAG]/ with the bag,
copies of the configuration, meta.yaml and the bag summary.

Stop the recording with Ctrl+C, or with SIGTERM (kill -TERM <pid>) when another
script started it in the background: there it ignores SIGINT. Closing the
terminal (SIGHUP) also closes the bag cleanly.

Options:
  -c, --config FILE  Node parameters file (default: config/vicon_client.yaml);
                     its namespace selects the topics, and it is copied
  -t, --tag TAG      Suffix of the session directory (letters, digits, - _)
  -o, --output DIR   Where sessions are created (default: recordings/), e.g. an
                     external disk (/mnt/<drive>/... on WSL)
  -h, --help         Show this help and exit

Exit status: 0 when the bag was closed cleanly, 1 otherwise (node not running,
empty namespace, recorder error, or no closed bag).

Examples:
  scripts/record.sh
  scripts/record.sh -t walk_slow
  scripts/record.sh -t calibration -o /mnt/d/vicon_sessions
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config | -t | --tag | -o | --output)
                require_option_value "$1" "$#"
                case "$1" in
                    -c | --config) config_file="$2" ;;
                    -t | --tag) tag="$2" ;;
                    -o | --output) output_dir="$2" ;;
                esac
                shift
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            *)
                die_with_usage "Unknown option: $1"
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

# Ask the recorder to close the bag and stop.
#   $1  process id of the recorder; empty while it has not been started
stop_recorder()
{
    local recorder_pid="$1"
    if [[ -n "${recorder_pid}" ]]; then
        kill -TERM "${recorder_pid}" 2>/dev/null || true
    fi
}

# Write the session metadata known at the start.
write_start_metadata()
{
    local session_dir="$1"
    local namespace="$2"
    local vicon_host="$3"
    local stream_mode="$4"
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
vicon_host: "${vicon_host}"
stream_mode: "${stream_mode}"
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
}

# Fail if the recorder did not produce a closed bag: it stopped by itself with an error, or it
# never wrote the bag metadata (written when the bag is closed).
check_recording()
{
    local session_dir="$1"
    local recorder_status="$2"
    local stop_requested="$3"

    if [[ "${stop_requested}" != "true" && "${recorder_status}" -ne 0 ]]; then
        die "ros2 bag record stopped with exit status ${recorder_status} (see its messages" \
            "above); session left in ${session_dir}"
    fi
    if [[ ! -f "${session_dir}/bag/metadata.yaml" ]]; then
        die "No closed bag in ${session_dir}/bag (ros2 bag record exit status" \
            "${recorder_status}): see its messages above."
    fi
}

main()
{
    parse_args "$@"
    require_config_file "${config_file}"
    config_file="$(realpath "${config_file}")"
    require_workspace
    vicon_configure_dds

    # Read before creating anything: a missing entry stops here
    local namespace=""
    namespace="$(read_config_param "${config_file}" namespace)"
    namespace="${namespace#/}"
    if [[ -z "${namespace}" ]]; then
        die "The namespace in ${config_file} is empty: the topics to record cannot be selected."
    fi
    local vicon_host=""
    vicon_host="$(read_config_param "${config_file}" hostname)"
    local stream_mode=""
    stream_mode="$(read_config_param "${config_file}" stream_mode)"
    local extra_configs=()
    if [[ -f "${DEFAULT_VIEWER_CONFIG_FILE}" ]]; then
        local mount_file=""
        mount_file="$(read_config_key_path "${DEFAULT_VIEWER_CONFIG_FILE}" robot.mount_file)"
        extra_configs=("${DEFAULT_VIEWER_CONFIG_FILE}" "$(absolute_repo_path "${mount_file}")")
    fi

    require_node_running

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
    local extra_config=""
    for extra_config in "${extra_configs[@]}"; do
        if [[ -f "${extra_config}" ]]; then
            cp "${extra_config}" "${session_dir}/config/"
        fi
    done
    write_start_metadata "${session_dir}" "${namespace}" "${vicon_host}" "${stream_mode}"

    # Ctrl+C (SIGINT), SIGTERM or a closed terminal (SIGHUP) stop the recording: the recorder gets
    # SIGTERM, on which it closes the bag cleanly. SIGINT is not forwarded because a recorder
    # started by a script in the background inherits SIGINT as ignored (POSIX), e.g. under a
    # service manager. Installed before the recorder starts, so that no signal can orphan it.
    local recorder_pid=""
    local stop_requested="false"
    trap 'stop_requested="true"; stop_recorder "${recorder_pid}"' INT TERM HUP

    info "Recording to ${session_dir}/bag (Ctrl+C to stop)"
    # A closed terminal also sends SIGHUP to the recorder: it must wait for our SIGTERM instead
    (
        trap '' HUP
        exec ros2 bag record -s mcap -o "${session_dir}/bag" \
            -e "^(/${namespace}/.*|/tf|/tf_static)$"
    ) &
    recorder_pid=$!
    # A signal that arrived while the recorder was being started
    if [[ "${stop_requested}" == "true" ]]; then
        stop_recorder "${recorder_pid}"
    fi

    # wait returns early when a trapped signal arrives: wait again until the recorder is done
    while kill -0 "${recorder_pid}" 2>/dev/null; do
        wait "${recorder_pid}" || true
    done
    trap - INT TERM HUP

    # Exit status of the recorder (bash keeps it after the process has ended)
    local recorder_status=0
    wait "${recorder_pid}" || recorder_status=$?

    finish_session "${session_dir}"
    check_recording "${session_dir}" "${recorder_status}" "${stop_requested}"
    info "Session saved in ${session_dir}"
}

main "$@"
