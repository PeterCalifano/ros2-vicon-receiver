#!/usr/bin/env bash
# =============================================================================
# export.sh - Optional export of a recorded session (MCAP) to CSV and/or HDF5.
#
# The MCAP bag of a session (scripts/record.sh) is the stored data. This script
# only converts it, when asked, into files for analysis tools, written to
# <session>/exports/. The format is required: nothing is exported by default.
#
# Options: scripts/export.sh --help
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

session_dir=""
formats=""

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} SESSION_DIR --format FORMATS [-h]

Export the ViconFrame messages of a recorded session to <SESSION_DIR>/exports/.
Runs in the conda environment of environment.yml (scripts/build.sh creates it).

Arguments:
  SESSION_DIR          Session directory created by scripts/record.sh

Options:
  --format FORMATS     Comma-separated list, required:
                         csv   frames.csv (one row per frame),
                               poses_<frame_id>.csv (one file per segment),
                               markers_<subject>.csv (one file per subject)
                         hdf5  session.h5 (arrays aligned on the frame index)
  -h, --help           Show this help and exit

Examples:
  scripts/export.sh recordings/20261030T101500Z_walk --format csv
  scripts/export.sh recordings/20261030T101500Z_walk --format csv,hdf5
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --format)
                require_option_value "$1" "$#"
                formats="$2"
                shift
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            -*)
                die_with_usage "Unknown option: $1"
                ;;
            *)
                if [[ -n "${session_dir}" ]]; then
                    die "Only one session directory can be given."
                fi
                session_dir="$1"
                ;;
        esac
        shift
    done

    if [[ -z "${session_dir}" || -z "${formats}" ]]; then
        die_with_usage "A session directory and --format are required."
    fi
}

main()
{
    parse_args "$@"
    if [[ ! -d "${session_dir}" ]]; then
        die "Session directory not found: ${session_dir}"
    fi
    require_workspace
    exec_tools_env_module export "$(realpath "${session_dir}")" --format "${formats}"
}

main "$@"
