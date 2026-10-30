#!/usr/bin/env bash
# =============================================================================
# probe.sh - Record what a Vicon DataStream server actually sends.
#
# Runs vicon_probe (no ROS node involved): it connects once to the server,
# records a number of frames of everything available (subjects, segments,
# markers, quality, occlusion, latency, cameras) and writes them as JSON.
#
# Steps:
#   1. take the server address from the parameters file (or --host);
#   2. check that the DataStream server is reachable (same check as launch.sh);
#   3. run vicon_probe and write the recording to probe_output/ (or -o FILE).
#
# Usage:
#   scripts/probe.sh [-c FILE | --host HOST[:PORT]] [-n FRAMES] [-o FILE] [options]
#
# Examples:
#   scripts/probe.sh
#   scripts/probe.sh --host 10.250.223.40 -n 500 -o session.json
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

readonly PROBE_EXECUTABLE="${REPO_ROOT}/install/vicon_receiver/lib/vicon_receiver/vicon_probe"
readonly PROBE_OUTPUT_DIR="${REPO_ROOT}/probe_output"

# Number of frames recorded when -n is not given (about one second at 100 Hz)
readonly DEFAULT_FRAME_COUNT=100

config_file="${DEFAULT_CONFIG_FILE}"
host_override=""
frame_count="${DEFAULT_FRAME_COUNT}"
output_file=""
run_check=true
probe_options=()

usage()
{
    cat <<EOF
Usage: $(basename "$0") [options]

Record what the Vicon DataStream server sends (subjects, markers, quality,
occlusion, latency, cameras) with vicon_probe, and write it as JSON.

Options:
  -c, --config FILE     Parameters file to take 'hostname' from
                        (default: config/vicon_client.yaml)
      --host HOST[:PORT]
                        Server address, instead of the parameters file
  -n, --frames N        Number of frames to record (default: ${DEFAULT_FRAME_COUNT})
  -o, --output FILE     JSON output (default: probe_output/vicon_probe_<UTC time>.json)
      --timeout-ms MS   Connection timeout of the SDK (default: SDK, 5000 ms)
      --centroids       Also record the 2D centroids of every camera (heavy)
      --no-check        Skip the reachability check
  -h, --help            Show this help and exit

Environment:
  VICON_CHECK_TIMEOUT_S  Timeout of the reachability check in seconds (default: 3)
EOF
}

# Return the value following an option, or stop if it is missing.
require_value()
{
    local option="$1"
    local remaining="$2"
    if [[ "${remaining}" -lt 2 ]]; then
        die "Option ${option} requires a value."
    fi
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config)
                require_value "$1" "$#"
                config_file="$2"
                shift
                ;;
            --host)
                require_value "$1" "$#"
                host_override="$2"
                shift
                ;;
            -n | --frames)
                require_value "$1" "$#"
                frame_count="$2"
                shift
                ;;
            -o | --output)
                require_value "$1" "$#"
                output_file="$2"
                shift
                ;;
            --timeout-ms)
                require_value "$1" "$#"
                probe_options+=("--timeout-ms" "$2")
                shift
                ;;
            --centroids)
                probe_options+=("--centroids")
                ;;
            --no-check)
                run_check=false
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
}

# Print the server address: --host if given, else 'hostname' from the parameters file.
resolve_host_spec()
{
    if [[ -n "${host_override}" ]]; then
        printf '%s\n' "${host_override}"
        return 0
    fi

    require_config_file "${config_file}"
    local host_spec=""
    host_spec="$(read_config_param "${config_file}" "hostname")"
    printf '%s\n' "${host_spec}"
}

main()
{
    parse_args "$@"

    require_ros
    require_install
    if [[ ! -x "${PROBE_EXECUTABLE}" ]]; then
        die "vicon_probe not found (${PROBE_EXECUTABLE})." "Run scripts/build.sh first."
    fi

    local host_spec=""
    host_spec="$(resolve_host_spec)"
    split_host_port "${host_spec}"

    if [[ "${run_check}" == true ]]; then
        check_vicon_reachable "${VICON_HOST}" "${VICON_PORT}"
    else
        warn "Skipping the Vicon reachability check (--no-check)."
    fi

    if [[ -z "${output_file}" ]]; then
        mkdir -p "${PROBE_OUTPUT_DIR}"
        output_file="${PROBE_OUTPUT_DIR}/vicon_probe_$(date -u +%Y%m%dT%H%M%SZ).json"
    fi

    info "Recording ${frame_count} frame(s) from ${VICON_HOST}:${VICON_PORT}"
    exec "${PROBE_EXECUTABLE}" \
        --host "${VICON_HOST}:${VICON_PORT}" \
        --frames "${frame_count}" \
        --json "${output_file}" \
        "${probe_options[@]}"
}

main "$@"
