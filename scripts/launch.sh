#!/usr/bin/env bash
# =============================================================================
# launch.sh - Start the vicon_receiver node from a parameters file.
#
# Steps:
#   1. check that ROS 2 and the built workspace are available;
#   2. read the Vicon server address ('hostname') from the parameters file;
#   3. check that the Vicon DataStream server accepts connections, and stop with
#      a clear error naming the likely cause if it does not;
#   4. run the node with the parameters file.
#
# Options: scripts/launch.sh --help
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

config_file="${DEFAULT_CONFIG_FILE}"
run_check=true
extra_ros_args=()

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} [-c FILE] [--no-check] [-h] [-- ROS_ARGS...]

Start the vicon_receiver node (${VICON_NODE_NAME}) with a parameters file, after
checking that the Vicon DataStream server of its 'hostname' accepts connections.
The node runs in the foreground until Ctrl+C. It exits with status 1 when a
parameter is missing or invalid, and with status 2 when the connection to the
Vicon server is lost (no reconnection).

Options:
  -c, --config FILE  Node parameters file (default: config/vicon_client.yaml)
      --no-check     Skip the reachability check of the Vicon server
  -h, --help         Show this help and exit

Arguments after '--' are appended to the node's --ros-args.

Environment:
  VICON_CHECK_TIMEOUT_S  Timeout of the reachability check [s] (default: 3)

Examples:
  scripts/launch.sh
  scripts/launch.sh -c config/lab_b.yaml
  scripts/launch.sh -- --log-level debug
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config)
                require_option_value "$1" "$#"
                config_file="$2"
                shift
                ;;
            --no-check)
                run_check=false
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            --)
                shift
                extra_ros_args=("$@")
                break
                ;;
            *)
                die_with_usage "Unknown option: $1"
                ;;
        esac
        shift
    done
}

main()
{
    parse_args "$@"

    require_config_file "${config_file}"
    config_file="$(realpath "${config_file}")"
    require_workspace
    # So that other ROS 2 processes on this machine (ros2 topic, recorder, viewer) see the node
    vicon_configure_dds

    if [[ "${run_check}" == true ]]; then
        local host_spec=""
        host_spec="$(read_config_param "${config_file}" "hostname")"
        split_host_port "${host_spec}"
        check_vicon_reachable "${VICON_HOST}" "${VICON_PORT}"
    else
        warn "Skipping the Vicon reachability check (--no-check)."
    fi

    info "Starting ${VICON_NODE_NAME} with ${config_file}"
    exec ros2 run vicon_receiver vicon_client \
        --ros-args --params-file "${config_file}" "${extra_ros_args[@]}"
}

main "$@"
