#!/usr/bin/env bash
# =============================================================================
# source_ros_env.sh - Set up an interactive shell for this workspace.
#
# Source it (do not execute it); it takes no arguments:
#   source scripts/source_ros_env.sh
#
# It sources ROS 2 and the workspace overlay, and selects the DDS configuration
# of this machine (scripts/lib/ros_env.sh), so that ros2 topic, ros2 bag,
# viewers, ... see the vicon_client node.
#
# On WSL with mirrored networking the ros2 CLI daemon is unreachable (it uses
# TCP on 127.0.0.1): pass --no-daemon to the ros2 commands that accept it,
# e.g. "ros2 topic list --no-daemon".
#
# This file does not change shell options and does not exit the shell on errors.
# =============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    cat >&2 <<EOF
Usage: source ${BASH_SOURCE[0]}

source_ros_env.sh must be sourced, not executed: it sets up the calling shell
(ROS 2, the workspace overlay and the DDS configuration of this machine).
EOF
    exit 1
fi

# Everything runs in a function so that its variables stay local to it
vicon_ros_env_main()
{
    # shellcheck source=scripts/lib/ros_env.sh
    source "$(dirname "${BASH_SOURCE[0]}")/lib/ros_env.sh"

    if [[ ! -f "${VICON_ROS_SETUP_FILE}" ]]; then
        echo "[source_ros_env] ROS 2 ${VICON_ROS_DISTRO} not found (${VICON_ROS_SETUP_FILE}):" \
            "run scripts/build.sh" >&2
        return 1
    fi
    vicon_source_setup_file "${VICON_ROS_SETUP_FILE}"

    if [[ -f "${VICON_WORKSPACE_SETUP_FILE}" ]]; then
        vicon_source_setup_file "${VICON_WORKSPACE_SETUP_FILE}"
    else
        echo "[source_ros_env] Workspace not built (${VICON_WORKSPACE_SETUP_FILE} missing):" \
            "run scripts/build.sh" >&2
    fi

    vicon_configure_dds

    if vicon_localhost_redirected; then
        echo "[source_ros_env] Use --no-daemon with ros2 topic/node/service list|echo|info on this machine."
    fi
    echo "[source_ros_env] ROS 2 environment ready for ${VICON_REPO_ROOT}"
}

vicon_ros_env_main
