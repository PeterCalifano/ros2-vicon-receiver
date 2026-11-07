#!/usr/bin/env bash
# =============================================================================
# export.sh - Optional export of a recorded session (MCAP) to CSV and/or HDF5.
#
# The MCAP bag of a session (scripts/record.sh) is the stored data. This script
# only converts it, when asked, into files for analysis tools, written to
# <session>/exports/. The format is required: nothing is exported by default.
#
# Usage:
#   scripts/export.sh SESSION_DIR --format csv[,hdf5]
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

session_dir=""
formats=""

usage()
{
    cat <<USAGE
Usage: $(basename "$0") SESSION_DIR --format FORMATS

Export the ViconFrame messages of a recorded session to <session>/exports/.

Arguments:
  SESSION_DIR        Session directory created by scripts/record.sh
  --format FORMATS   Comma-separated: csv, hdf5 (required)
  -h, --help         Show this help and exit
USAGE
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --format)
                if [[ $# -lt 2 ]]; then
                    die "Option $1 requires a value."
                fi
                formats="$2"
                shift
                ;;
            -h | --help)
                usage
                exit 0
                ;;
            -*)
                usage >&2
                die "Unknown option: $1"
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
        usage >&2
        die "A session directory and --format are required."
    fi
}

main()
{
    parse_args "$@"
    if [[ ! -d "${session_dir}" ]]; then
        die "Session directory not found: ${session_dir}"
    fi
    require_ros
    require_install
    local tools_python=""
    tools_python="$(tools_env_python)"

    PYTHONPATH="${TOOLS_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
        exec "${tools_python}" -m vicon_receiver_tools.export \
        "$(realpath "${session_dir}")" --format "${formats}"
}

main "$@"
