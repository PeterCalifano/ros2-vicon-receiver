#!/usr/bin/env bash
# =============================================================================
# build.sh - Install everything needed and build the vicon_receiver workspace.
#
# The repository root is used as the colcon workspace: build/, install/ and log/
# are created next to the packages (and are git-ignored).
#
# Steps:
#   1. check the OS (ROS 2 Jazzy binaries target Ubuntu 24.04 "noble");
#   2. install ROS 2 Jazzy from the official apt repository if missing (sudo);
#   3. initialise/update rosdep and install the package dependencies (package.xml);
#   4. build the packages with colcon.
#
# Usage:
#   scripts/build.sh [--clean] [--debug]
# =============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

# Packages of this repository, relative to the repository root. Their system
# dependencies (package.xml) are installed by rosdep.
readonly PACKAGE_DIRS=(
    "vicon_receiver"
    "vicon_receiver_tools"
)

readonly ROSDEP_SOURCES_FILE="/etc/ros/rosdep/sources.list.d/20-default.list"
readonly ROS_APT_SOURCE_REPO="ros-infrastructure/ros-apt-source"

clean_first=false
build_type="Release"

usage()
{
    cat <<EOF
Usage: $(basename "$0") [options]

Install ROS 2 ${ROS_DISTRO_NAME} (if missing) and the package dependencies, then
build this repository's packages with colcon. Steps that need root use sudo.

Options:
  --clean     Remove build/, install/ and log/ before building
  --debug     Build with CMAKE_BUILD_TYPE=Debug (default: Release)
  -h, --help  Show this help and exit

Environment:
  ROS_APT_SOURCE_VERSION  Pin the ros2-apt-source release (default: latest)
EOF
}

parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --clean)
                clean_first=true
                ;;
            --debug)
                build_type="Debug"
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

# -----------------------------------------------------------------------------
# System checks and installation
# -----------------------------------------------------------------------------

# ROS 2 Jazzy binary packages exist only for Ubuntu 24.04 (noble).
check_os()
{
    local codename=""
    if [[ -f /etc/os-release ]]; then
        codename="$(. /etc/os-release && echo "${VERSION_CODENAME:-}")"
    fi
    if [[ "${codename}" != "noble" ]]; then
        die "Unsupported OS (codename '${codename:-unknown}')." \
            "ROS 2 ${ROS_DISTRO_NAME} binaries require Ubuntu 24.04 (noble)." \
            "On other systems use the VS Code Dev Container (.devcontainer/)."
    fi
}

# Return 0 if a Debian package is installed.
is_apt_package_installed()
{
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

# Install the given apt packages that are not installed yet (uses sudo).
ensure_apt_packages()
{
    local missing=()
    local package=""
    for package in "$@"; do
        if ! is_apt_package_installed "${package}"; then
            missing+=("${package}")
        fi
    done

    if [[ ${#missing[@]} -eq 0 ]]; then
        return 0
    fi

    info "Installing system packages (sudo): ${missing[*]}"
    sudo apt-get update -q
    sudo apt-get install -y -q "${missing[@]}"
}

# Print the tag of the latest ros2-apt-source release.
# The tag is read from the redirect of the github.com "latest release" page
# (.../releases/latest -> .../releases/tag/<tag>) rather than from the GitHub REST
# API, which rejects unauthenticated clients with HTTP 403 once they exceed its rate
# limit (common behind institute NAT/proxies).
latest_ros_apt_source_tag()
{
    local latest_url="https://github.com/${ROS_APT_SOURCE_REPO}/releases/latest"
    local redirect_url=""
    if ! redirect_url="$(curl -fsS -o /dev/null -w '%{redirect_url}' "${latest_url}")"; then
        return 1
    fi

    if [[ "${redirect_url}" != */releases/tag/* ]]; then
        return 1
    fi
    printf '%s\n' "${redirect_url##*/}"
}

# Add the official ROS 2 apt repository, following the ROS 2 Jazzy installation guide.
# The release of the ros2-apt-source package can be pinned with ROS_APT_SOURCE_VERSION.
add_ros_apt_repository()
{
    if is_apt_package_installed "ros2-apt-source"; then
        info "ROS 2 apt repository already configured."
        return 0
    fi

    ensure_apt_packages locales software-properties-common curl ca-certificates

    info "Configuring the en_US.UTF-8 locale (sudo)"
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    info "Enabling the Ubuntu 'universe' repository (sudo)"
    sudo add-apt-repository -y universe

    # Determine which release of ros2-apt-source to install
    local release_tag="${ROS_APT_SOURCE_VERSION:-}"
    if [[ -z "${release_tag}" ]]; then
        if ! release_tag="$(latest_ros_apt_source_tag)"; then
            die "Could not determine the latest ros2-apt-source release from github.com." \
                "Check the network/proxy, or pin a release and retry, e.g.:" \
                "  ROS_APT_SOURCE_VERSION=1.3.0 scripts/build.sh" \
                "Releases: https://github.com/${ROS_APT_SOURCE_REPO}/releases"
        fi
    fi

    # The ros2-apt-source package installs the repository key and source list
    local deb_name="ros2-apt-source_${release_tag}.noble_all.deb"
    local deb_url="https://github.com/${ROS_APT_SOURCE_REPO}/releases/download/${release_tag}/${deb_name}"
    local download_dir=""
    download_dir="$(mktemp -d)"
    local deb_file="${download_dir}/${deb_name}"

    info "Downloading ${deb_name}"
    if ! curl -fsSL -o "${deb_file}" "${deb_url}"; then
        rm -rf "${download_dir}"
        die "Download failed: ${deb_url}" \
            "Check the network/proxy, or pin another release with ROS_APT_SOURCE_VERSION."
    fi

    info "Installing ros2-apt-source ${release_tag} (sudo)"
    sudo dpkg -i "${deb_file}"
    rm -rf "${download_dir}"
}

# Install ROS 2 (base variant + development tools) if it is not installed.
install_ros_if_missing()
{
    if [[ -f "${ROS_SETUP_FILE}" ]]; then
        info "ROS 2 ${ROS_DISTRO_NAME} found in /opt/ros/${ROS_DISTRO_NAME}."
        return 0
    fi

    info "ROS 2 ${ROS_DISTRO_NAME} not found: installing it from the official apt repository."
    add_ros_apt_repository
    ensure_apt_packages "ros-${ROS_DISTRO_NAME}-ros-base" "ros-dev-tools"
}

# Initialise rosdep once, refresh its database and install the package dependencies.
install_rosdep_dependencies()
{
    if [[ ! -f "${ROSDEP_SOURCES_FILE}" ]]; then
        info "Initialising rosdep (sudo)"
        sudo rosdep init
    fi

    info "Updating the rosdep database"
    rosdep update --rosdistro "${ROS_DISTRO_NAME}"

    info "Installing package dependencies with rosdep"
    rosdep install \
        --from-paths "${PACKAGE_DIRS[@]}" \
        --rosdistro "${ROS_DISTRO_NAME}" \
        --ignore-src \
        -y
}

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------

# Make sure every package of PACKAGE_DIRS is present (e.g. not lost in a partial checkout).
check_package_dirs()
{
    local package_dir=""
    for package_dir in "${PACKAGE_DIRS[@]}"; do
        if [[ ! -f "${REPO_ROOT}/${package_dir}/package.xml" ]]; then
            die "Package not found: ${REPO_ROOT}/${package_dir}/package.xml"
        fi
    done
}

clean_workspace()
{
    info "Removing build/, install/ and log/"
    rm -rf "${REPO_ROOT}/build" "${REPO_ROOT}/install" "${REPO_ROOT}/log"
}

build_workspace()
{
    info "Building ${PACKAGE_DIRS[*]} (${build_type})"
    python3 -m colcon build \
        --base-paths "${PACKAGE_DIRS[@]}" \
        --symlink-install \
        --cmake-args \
        "-DCMAKE_BUILD_TYPE=${build_type}" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
}

main()
{
    parse_args "$@"

    # colcon and rosdep work on paths relative to the workspace root
    cd "${REPO_ROOT}"

    check_os
    install_ros_if_missing
    require_ros

    check_package_dirs
    install_rosdep_dependencies

    if [[ "${clean_first}" == true ]]; then
        clean_workspace
    fi

    require_colcon
    build_workspace

    info "Build complete."
    info "Next: scripts/launch.sh [-c config/vicon_client.yaml]"
}

main "$@"
