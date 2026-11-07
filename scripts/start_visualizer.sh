#!/usr/bin/env bash
# =============================================================================
# start_visualizer.sh - Live viewer of the Vicon stream in a web browser (viser).
#
# Shows the ViconFrame topic of a running vicon_client node (scripts/launch.sh):
# subjects, trails, markers coloured by camera count, occlusion, the G1 model
# and its camera frustums, telemetry, and sliders to adjust the marker mount.
#
# Usage:
#   scripts/start_visualizer.sh [-c FILE]
# Then open the printed address (default http://127.0.0.1:8080) in a browser.
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

viewer_config="${REPO_ROOT}/config/viewer.yaml"

usage()
{
    cat <<USAGE
Usage: $(basename "$0") [options]

Start the live viewer of the ViconFrame topic (run scripts/launch.sh first).

Options:
  -c, --config FILE  Viewer config (default: config/viewer.yaml)
  -h, --help         Show this help and exit
USAGE
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config)
                if [[ $# -lt 2 ]]; then
                    die "Option $1 requires a file argument."
                fi
                viewer_config="$2"
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
}

main()
{
    parse_args "$@"
    require_config_file "${viewer_config}"
    viewer_config="$(realpath "${viewer_config}")"
    require_ros
    require_install
    local tools_python=""
    tools_python="$(tools_env_python)"
    # The viewer is a separate ROS 2 process: it must see the node (WSL: see dds.sh)
    vicon_configure_dds

    cd "${REPO_ROOT}"
    PYTHONPATH="${TOOLS_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
        exec "${tools_python}" -m vicon_receiver_tools.viewer \
        --config "${viewer_config}" --repo-root "${REPO_ROOT}"
}

main "$@"
