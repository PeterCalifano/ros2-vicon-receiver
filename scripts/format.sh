#!/usr/bin/env bash
# =============================================================================
# format.sh - Apply (or check) the repository clang-format style.
#
# Formats every C++ source and header owned by this repository, using the
# .clang-format file at the repository root. Vendored third-party code is
# never touched.
#
# Options: scripts/format.sh --help
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

# Directories (relative to the repository root) that contain our own C++ code
readonly SOURCE_DIRS=(
    "vicon_receiver/src"
    "vicon_receiver/include"
    "tests/unit/cpp"
)

check_only=false

usage()
{
    cat <<EOF
Usage: ${SCRIPT_NAME} [--check] [-h]

Format the C++ files of this repository in place with clang-format and the
.clang-format file at the repository root (vendored code is never touched).
Folders: ${SOURCE_DIRS[*]}

Options:
  --check     Only check: change no file, fail if any file needs formatting
  -h, --help  Show this help and exit

Environment:
  CLANG_FORMAT  clang-format executable (default: the one pinned in the conda
                environment of environment.yml, else clang-format on PATH)

Examples:
  scripts/format.sh
  scripts/format.sh --check
  CLANG_FORMAT=clang-format-18 scripts/format.sh
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --check)
                check_only=true
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

# Print the clang-format to use: CLANG_FORMAT, the conda environment's, or the one on PATH.
find_clang_format()
{
    if [[ -n "${CLANG_FORMAT:-}" ]]; then
        echo "${CLANG_FORMAT}"
        return 0
    fi
    local environment_clang_format=""
    environment_clang_format="$(tools_env_bin clang-format)"
    echo "${environment_clang_format:-clang-format}"
}

# Collect all C++ files in SOURCE_DIRS into the global array CPP_FILES.
collect_files()
{
    CPP_FILES=()
    local dir=""
    for dir in "${SOURCE_DIRS[@]}"; do
        if [[ ! -d "${REPO_ROOT}/${dir}" ]]; then
            continue
        fi
        while IFS= read -r -d '' file; do
            CPP_FILES+=("${file}")
        done < <(find "${REPO_ROOT}/${dir}" -type f \
            \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 | sort -z)
    done
}

main()
{
    parse_args "$@"

    local clang_format=""
    clang_format="$(find_clang_format)"
    if ! command -v "${clang_format}" >/dev/null 2>&1; then
        die "'${clang_format}' not found. Create the conda environment" \
            "(conda env create -f environment.yml) or install it with: sudo apt install clang-format"
    fi

    collect_files
    if [[ ${#CPP_FILES[@]} -eq 0 ]]; then
        info "No C++ files found."
        exit 0
    fi

    if [[ "${check_only}" == true ]]; then
        # --dry-run + --Werror makes clang-format fail on the first unformatted file
        "${clang_format}" --style=file --dry-run --Werror "${CPP_FILES[@]}"
        info "${#CPP_FILES[@]} files checked, all formatted."
    else
        "${clang_format}" --style=file -i "${CPP_FILES[@]}"
        info "${#CPP_FILES[@]} files formatted."
    fi
}

main "$@"
