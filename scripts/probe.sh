#!/usr/bin/env bash
# =============================================================================
# probe.sh - Record what a Vicon DataStream server actually sends.
#
# Runs vicon_probe (no ROS node involved): it connects once to the server,
# records a number of frames of everything available (subjects, segments,
# markers, quality, occlusion, latency, cameras) and writes them as JSON.
#
# Steps:
#   1. take the server address and stream mode from the parameters file (or the options);
#   2. check that the DataStream server is reachable (same check as launch.sh);
#   3. run vicon_probe and write the recording to probe_output/ (or -o FILE).
#
# Options: scripts/probe.sh --help
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
host_option=""
stream_mode_option=""
frame_count="${DEFAULT_FRAME_COUNT}"
output_file=""
run_check=true
probe_options=()

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} [-c FILE] [--host HOST[:PORT]] [--stream-mode MODE] [-n N]
                [-o FILE] [--timeout-ms MS] [--centroids] [--no-check] [-h]

Record what the Vicon DataStream server sends (subjects, segments, markers,
quality, occlusion, latency, cameras) with vicon_probe, and write it as JSON.
The probe uses the same SDK calls as the node, without ROS.

Options:
  -c, --config FILE     Parameters file giving 'hostname' and 'stream_mode'
                        (default: config/vicon_client.yaml)
      --host HOST[:PORT]
                        Server address instead of 'hostname' (default port: ${VICON_DATASTREAM_PORT})
      --stream-mode MODE
                        ServerPush, ClientPull or ClientPullPreFetch instead of
                        'stream_mode' (e.g. to compare the modes)
  -n, --frames N        Number of frames to record (default: ${DEFAULT_FRAME_COUNT})
  -o, --output FILE     JSON output file
                        (default: probe_output/vicon_probe_<UTC time>.json)
      --timeout-ms MS   SDK connection timeout [ms] (default: 5000, the SDK's)
      --centroids       Also record the 2D centroids of every camera (heavy)
      --no-check        Skip the reachability check of the Vicon server
  -h, --help            Show this help and exit

Environment:
  VICON_CHECK_TIMEOUT_S  Timeout of the reachability check [s] (default: 3)

Exit status (of vicon_probe): 0 recorded, 1 invalid arguments, 2 connection
failed, 3 output file not writable, 4 stream lost while recording.

Examples:
  scripts/probe.sh
  scripts/probe.sh --host 192.0.2.10 -n 500 -o session.json
  scripts/probe.sh --stream-mode ClientPull -n 2000
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config | --host | --stream-mode | -n | --frames | -o | --output | --timeout-ms)
                require_option_value "$1" "$#"
                case "$1" in
                    -c | --config) config_file="$2" ;;
                    --host) host_option="$2" ;;
                    --stream-mode) stream_mode_option="$2" ;;
                    -n | --frames) frame_count="$2" ;;
                    -o | --output) output_file="$2" ;;
                    --timeout-ms) probe_options+=("--timeout-ms" "$2") ;;
                esac
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
                die_with_usage "Unknown option: $1"
                ;;
        esac
        shift
    done
}

# Print the value given on the command line, or else the parameter of the parameters file, so
# that the probe measures what the node gets.
#   $1  value of the command-line option (empty if not given)
#   $2  parameter name in the parameters file
option_or_config_param()
{
    local option_value="$1"
    local param_name="$2"
    if [[ -n "${option_value}" ]]; then
        printf '%s\n' "${option_value}"
        return 0
    fi
    require_config_file "${config_file}"
    read_config_param "${config_file}" "${param_name}"
}

main()
{
    parse_args "$@"

    require_workspace
    if [[ ! -x "${PROBE_EXECUTABLE}" ]]; then
        die "vicon_probe not found (${PROBE_EXECUTABLE})." "Run scripts/build.sh first."
    fi

    local host_spec=""
    host_spec="$(option_or_config_param "${host_option}" hostname)"
    split_host_port "${host_spec}"
    local stream_mode=""
    stream_mode="$(option_or_config_param "${stream_mode_option}" stream_mode)"

    if [[ "${run_check}" == true ]]; then
        check_vicon_reachable "${VICON_HOST}" "${VICON_PORT}"
    else
        warn "Skipping the Vicon reachability check (--no-check)."
    fi

    if [[ -z "${output_file}" ]]; then
        mkdir -p "${PROBE_OUTPUT_DIR}"
        output_file="${PROBE_OUTPUT_DIR}/vicon_probe_$(date -u +%Y%m%dT%H%M%SZ).json"
    fi

    info "Recording ${frame_count} frame(s) from ${VICON_HOST}:${VICON_PORT} (${stream_mode})"
    exec "${PROBE_EXECUTABLE}" \
        --host "${VICON_HOST}:${VICON_PORT}" \
        --frames "${frame_count}" \
        --stream-mode "${stream_mode}" \
        --json "${output_file}" \
        "${probe_options[@]}"
}

main "$@"
