#!/usr/bin/env bash
# =============================================================================
# source_ros_env.sh - Set up an interactive shell for this workspace.
#
# Source it (do not execute it):
#   source scripts/source_ros_env.sh
#
# It sources ROS 2 Jazzy and the workspace overlay, and selects the DDS
# configuration of this machine (scripts/lib/dds.sh), so that ros2 topic,
# ros2 bag, viewers, ... see the vicon_client node.
#
# On WSL with mirrored networking the ros2 CLI daemon is unreachable (it uses
# TCP on 127.0.0.1): pass --no-daemon to the ros2 commands that accept it,
# e.g. "ros2 topic list --no-daemon".
#
# This file does not change shell options and does not exit the shell on errors.
# =============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "source_ros_env.sh must be sourced: source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

# Everything runs in a function so that its variables stay local to it
vicon_ros_env_main()
{
    local repo_root
    repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

    local ros_setup_file="/opt/ros/jazzy/setup.bash"
    local workspace_setup_file="${repo_root}/install/setup.bash"

    if [[ ! -f "${ros_setup_file}" ]]; then
        echo "[source_ros_env] ROS 2 Jazzy not found (${ros_setup_file}): run scripts/build.sh" >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "${ros_setup_file}"

    if [[ -f "${workspace_setup_file}" ]]; then
        # shellcheck disable=SC1090
        source "${workspace_setup_file}"
    else
        echo "[source_ros_env] Workspace not built (${workspace_setup_file} missing): run scripts/build.sh" >&2
    fi

    # shellcheck source=scripts/lib/dds.sh
    source "${repo_root}/scripts/lib/dds.sh"
    vicon_configure_dds

    if vicon_localhost_redirected; then
        echo "[source_ros_env] Use --no-daemon with ros2 topic/node/service list|echo|info on this machine."
    fi
    echo "[source_ros_env] ROS 2 environment ready for ${repo_root}"
}

vicon_ros_env_main
