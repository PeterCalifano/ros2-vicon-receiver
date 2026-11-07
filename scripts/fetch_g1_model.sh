#!/usr/bin/env bash
# =============================================================================
# fetch_g1_model.sh - Copy the G1 Unitree model (URDF + meshes) for the viewer.
#
# Source and destination come from config/viewer.yaml (robot.source,
# robot.directory). The copy is made with rsync over ssh and is git-ignored.
#
# Usage:
#   scripts/fetch_g1_model.sh [-c FILE]
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

viewer_config="${REPO_ROOT}/config/viewer.yaml"

usage()
{
    cat <<EOF
Usage: $(basename "$0") [options]

Copy the G1 Unitree model used by the live viewer from robot.source to
robot.directory (both in the viewer config), with rsync over ssh.

Options:
  -c, --config FILE  Viewer config (default: config/viewer.yaml)
  -h, --help         Show this help and exit
EOF
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

# Print one value of the viewer config (dotted key path).
read_viewer_value()
{
    local key_path="$1"
    local output=""
    if ! output="$(run_tools_module config "${viewer_config}" "${key_path}" --key-path 2>&1)"; then
        die "Invalid viewer config: ${output}"
    fi
    printf '%s\n' "${output}"
}

main()
{
    parse_args "$@"
    require_config_file "${viewer_config}"

    local source=""
    source="$(read_viewer_value robot.source)"
    local directory=""
    directory="$(read_viewer_value robot.directory)"
    local urdf=""
    urdf="$(read_viewer_value robot.urdf)"

    # Relative destinations are relative to the repository root
    if [[ "${directory}" != /* ]]; then
        directory="${REPO_ROOT}/${directory}"
    fi
    mkdir -p "${directory}"

    info "Copying ${source} to ${directory}"
    # Trailing slash on the source: copy its content, not the directory itself
    if ! rsync -a --info=stats1 "${source%/}/" "${directory}/"; then
        die "rsync from ${source} failed." \
            "Check that ssh to the host works without a password prompt (ssh-agent, key)."
    fi

    if [[ ! -f "${directory}/${urdf}" ]]; then
        die "The copy has no ${urdf}: check robot.source and robot.urdf in ${viewer_config}."
    fi
    info "G1 model ready: ${directory}/${urdf}"
}

main "$@"
