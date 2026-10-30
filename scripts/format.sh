#!/usr/bin/env bash
# =============================================================================
# format.sh - Apply (or check) the repository clang-format style.
#
# Formats every C++ source and header owned by this repository, using the
# .clang-format file at the repository root. Vendored third-party code is
# never touched.
#
# Usage:
#   scripts/format.sh            # format files in place
#   scripts/format.sh --check    # only report files that are not formatted
#
# Environment:
#   CLANG_FORMAT   clang-format executable to use (default: clang-format)
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

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
Usage: $(basename "$0") [--check] [-h|--help]

Format the C++ sources of this repository with clang-format.

Options:
  --check     Do not modify files; exit with an error if any file needs formatting.
  -h, --help  Show this help and exit.

The clang-format executable can be overridden with the CLANG_FORMAT variable.
EOF
}

# Print an error message to stderr and terminate.
die()
{
    echo "[format.sh] ERROR: $*" >&2
    exit 1
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
                usage >&2
                die "Unknown option: $1"
                ;;
        esac
        shift
    done
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

    local clang_format="${CLANG_FORMAT:-clang-format}"
    if ! command -v "${clang_format}" >/dev/null 2>&1; then
        die "'${clang_format}' not found. Install it with: sudo apt install clang-format"
    fi

    collect_files
    if [[ ${#CPP_FILES[@]} -eq 0 ]]; then
        echo "[format.sh] No C++ files found."
        exit 0
    fi

    if [[ "${check_only}" == true ]]; then
        # --dry-run + --Werror makes clang-format fail on the first unformatted file
        "${clang_format}" --style=file --dry-run --Werror "${CPP_FILES[@]}"
        echo "[format.sh] ${#CPP_FILES[@]} files checked, all formatted."
    else
        "${clang_format}" --style=file -i "${CPP_FILES[@]}"
        echo "[format.sh] ${#CPP_FILES[@]} files formatted."
    fi
}

main "$@"
