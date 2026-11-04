"""Integration tests of vicon_receiver/launch/client.launch.py.

The launch file has no parameter defaults of its own: the parameters file is a required argument,
so the YAML stays the single source of truth.
"""

from __future__ import annotations

import subprocess

import pytest

from conftest import DEFAULT_TIMEOUT_S

pytestmark = pytest.mark.integration

LAUNCH_COMMAND: list[str] = ["ros2", "launch", "vicon_receiver", "client.launch.py"]


def Run_launch(args: list[str], env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    """Run ros2 launch with the given extra arguments and capture its output."""
    return subprocess.run(
        [*LAUNCH_COMMAND, *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )


def test_launch_file_declares_only_params_file(ros_env: dict[str, str]) -> None:
    """The only launch argument is the parameters file (no duplicated parameter defaults)."""
    result_ = Run_launch(["--show-args"], ros_env)
    assert result_.returncode == 0, result_.stderr
    assert "'params_file':" in result_.stdout
    for old_argument_ in ("hostname", "buffer_size", "topic_namespace", "map_xyz"):
        assert f"'{old_argument_}':" not in result_.stdout


def test_launch_file_requires_params_file(ros_env: dict[str, str]) -> None:
    """Without params_file, ros2 launch stops with an error naming the argument."""
    result_ = Run_launch([], ros_env)
    assert result_.returncode != 0
    assert "missing required argument 'params_file'" in result_.stdout + result_.stderr
