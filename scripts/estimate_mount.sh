#!/usr/bin/env bash
# =============================================================================
# estimate_mount.sh - Rough estimate of where the Vicon markers sit on the G1.
#
# Fits the marker constellation of a probe recording (scripts/probe.sh) onto the
# G1 meshes and, with --write, saves the pose of the mount link in the Vicon body
# frame to the mount file of config/viewer.yaml. A starting point for the live
# viewer (scripts/viz.sh), where it can be adjusted; not a calibration.
#
# Usage:
#   scripts/estimate_mount.sh -r RECORDING.json [-c FILE] [--write]
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

viewer_config="${REPO_ROOT}/config/viewer.yaml"
recording=""
write_option=()

usage()
{
    cat <<USAGE
Usage: $(basename "$0") -r RECORDING.json [options]

Estimate the pose of the robot mount link in the Vicon body frame from a probe
recording (markers fitted to the robot meshes; takes a few minutes).

Options:
  -r, --recording FILE  Probe recording (scripts/probe.sh), with markers
  -c, --config FILE     Viewer config (default: config/viewer.yaml)
      --write           Save the result to the mount file of the viewer config
  -h, --help            Show this help and exit
USAGE
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -r | --recording | -c | --config)
                if [[ $# -lt 2 ]]; then
                    die "Option $1 requires a file argument."
                fi
                if [[ "$1" == "-r" || "$1" == "--recording" ]]; then
                    recording="$2"
                else
                    viewer_config="$2"
                fi
                shift
                ;;
            --write)
                write_option=("--write")
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

main()
{
    parse_args "$@"
    if [[ -z "${recording}" ]]; then
        usage >&2
        die "A probe recording is required (-r)."
    fi
    require_config_file "${viewer_config}"
    require_config_file "${recording}"
    local tools_python=""
    tools_python="$(tools_env_python)"

    PYTHONPATH="${TOOLS_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
        exec "${tools_python}" -m vicon_receiver_tools.mount_estimate \
        --recording "$(realpath "${recording}")" \
        --viewer-config "$(realpath "${viewer_config}")" \
        --repo-root "${REPO_ROOT}" \
        "${write_option[@]}"
}

main "$@"
