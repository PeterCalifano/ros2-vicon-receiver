#!/usr/bin/env bash
# =============================================================================
# fetch_g1_model.sh - Copy the G1 Unitree model (URDF + meshes) for the viewer.
#
# Source and destination come from config/viewer.yaml (robot.source,
# robot.directory). The copy is made with rsync over ssh and is git-ignored.
#
# Options: scripts/fetch_g1_model.sh --help
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

Copy the G1 Unitree model shown by the live viewer (URDF + meshes) from
robot.source ([user@]host:path) to robot.directory, both in the viewer config,
with rsync over ssh; then check that robot.urdf is in the copy. ssh to the
source host must work without a password prompt (ssh-agent or key).

Options:
  -c, --config FILE  Viewer config (default: config/viewer.yaml)
  -h, --help         Show this help and exit

Examples:
  scripts/fetch_g1_model.sh
  scripts/fetch_g1_model.sh -c config/viewer_lab_b.yaml
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

    local source=""
    source="$(read_config_key_path "${viewer_config}" robot.source)"
    local directory=""
    directory="$(read_config_key_path "${viewer_config}" robot.directory)"
    local urdf=""
    urdf="$(read_config_key_path "${viewer_config}" robot.urdf)"

    directory="$(absolute_repo_path "${directory}")"
    mkdir -p "${directory}"

    info "Copying ${source} to ${directory}"
    # Trailing slash on the source: copy its content, not the directory itself
    # BatchMode: fail at once instead of waiting at a password prompt
    if ! rsync -a --info=stats1 -e 'ssh -o BatchMode=yes' "${source%/}/" "${directory}/"; then
        die "rsync from ${source} failed." \
            "Check that ssh to the host works without a password prompt (ssh-agent, key)."
    fi

    if [[ ! -f "${directory}/${urdf}" ]]; then
        die "The copy has no ${urdf}: check robot.source and robot.urdf in ${viewer_config}."
    fi
    info "G1 model ready: ${directory}/${urdf}"
}

main "$@"
