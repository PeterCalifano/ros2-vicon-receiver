#!/usr/bin/env bash
# =============================================================================
# start_visualizer.sh - Live viewer of the Vicon stream in a web browser (viser).
#
# Shows the ViconFrame topic of a running vicon_client node (scripts/launch.sh):
# subjects, trails, markers coloured by camera count, occlusion, the G1 model
# and its camera frustums, telemetry, and sliders to adjust the marker mount.
#
# Options: scripts/start_visualizer.sh --help
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

viewer_config="${DEFAULT_VIEWER_CONFIG_FILE}"

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} [-c FILE] [-h]

Start the live viewer of the ViconFrame topic of a running ${VICON_NODE_NAME} node
(scripts/launch.sh first), then open the printed address in a browser (default
http://127.0.0.1:8080, set by viewer.host and viewer.port in the viewer config).
Runs in the conda environment of environment.yml (scripts/build.sh creates it);
the G1 model comes from scripts/fetch_g1_model.sh.

Options:
  -c, --config FILE  Viewer config (default: config/viewer.yaml)
  -h, --help         Show this help and exit

Examples:
  scripts/start_visualizer.sh
  scripts/start_visualizer.sh -c config/viewer_lab_b.yaml
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -c | --config)
                require_option_value "$1" "$#"
                viewer_config="$2"
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
}

main()
{
    parse_args "$@"
    require_config_file "${viewer_config}"
    viewer_config="$(realpath "${viewer_config}")"
    require_workspace
    # The viewer is a separate ROS 2 process: it must see the node (WSL: see ros_env.sh)
    vicon_configure_dds

    cd "${REPO_ROOT}"
    exec_tools_env_module viewer --config "${viewer_config}" --repo-root "${REPO_ROOT}"
}

main "$@"
