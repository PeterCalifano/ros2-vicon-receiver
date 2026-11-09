"""Integration tests of the vicon_client executable with parameter files.

The C++ unit tests check the parameter contract of the node with in-memory overrides. These tests
check the real path a user takes: a YAML parameters file passed with --params-file, the FATAL
messages printed by main() and the exit code. No Vicon server is needed: the valid-config test
points the node at a local TCP listener and stops it once it has started.
"""

from __future__ import annotations

import re
import subprocess
from collections.abc import Callable
from pathlib import Path

import pytest

from conftest import DEFAULT_TIMEOUT_S, NODE_EXECUTABLE, TcpListener, Run_until_output

pytestmark = pytest.mark.integration

# Messages printed by the node (see vicon_receiver/src/main.cpp and vicon_receiver_node.cpp)
FATAL_PREFIX: str = "Invalid node configuration:"
STATIC_TF_MESSAGE: str = "Published static transform"


def Node_command(config: Path | None) -> list[str]:
    """Command running the node, with the given parameters file if any."""
    command_ = [str(NODE_EXECUTABLE)]
    if config is not None:
        command_ += ["--ros-args", "--params-file", str(config)]
    return command_


def Run_node_to_exit(config: Path | None, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    """Run the node until it exits by itself (configuration errors), output combined."""
    return subprocess.run(
        Node_command(config),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )


def test_node_without_parameters_file_fails(ros_env: dict[str, str]) -> None:
    """Without a parameters file the node reports the first missing parameter and exits 1."""
    outcome_ = Run_node_to_exit(None, ros_env)
    assert "Statically typed parameter 'hostname' must be initialized" in outcome_.stdout
    assert "--params-file config/vicon_client.yaml" in outcome_.stdout
    assert outcome_.returncode == 1


def test_node_reports_missing_parameter_by_name(
    ros_env: dict[str, str], make_config: Callable[..., Path]
) -> None:
    """A parameter missing from the YAML is named in the error."""
    config_ = make_config(remove=["buffer_size"])
    outcome_ = Run_node_to_exit(config_, ros_env)
    assert "Statically typed parameter 'buffer_size' must be initialized" in outcome_.stdout
    assert outcome_.returncode == 1


def test_node_rejects_integer_list_for_doubles(
    ros_env: dict[str, str], make_config: Callable[..., Path]
) -> None:
    """'[0, 0, 0]' in YAML is an integer array and is rejected for map_xyz."""
    config_ = make_config(overrides={"map_xyz": [0, 0, 0]})
    outcome_ = Run_node_to_exit(config_, ros_env)
    assert "parameter 'map_xyz' has invalid type" in outcome_.stdout
    assert outcome_.returncode == 1


def test_node_rejects_wrong_vector_size(
    ros_env: dict[str, str], make_config: Callable[..., Path]
) -> None:
    """map_rpy must have exactly three elements."""
    config_ = make_config(overrides={"map_rpy": [0.0, 0.0]})
    outcome_ = Run_node_to_exit(config_, ros_env)
    assert "parameter 'map_rpy' must have 3 elements, got 2" in outcome_.stdout
    assert outcome_.returncode == 1


def test_node_starts_with_valid_config(
    ros_env: dict[str, str], make_config: Callable[..., Path], tcp_listener: TcpListener
) -> None:
    """With a valid file the node publishes the static transform and tries to connect."""
    config_ = make_config(overrides={"hostname": tcp_listener.host_spec})
    expected_ = f"Connecting to {tcp_listener.host_spec}"
    outcome_ = Run_until_output(Node_command(config_), ros_env, expected_)
    assert outcome_.found, outcome_.output
    assert STATIC_TF_MESSAGE in outcome_.output
    assert FATAL_PREFIX not in outcome_.output
    # Still running when the expected line appeared: stopped by the test, not crashed
    assert outcome_.returncode is None


def test_failed_connections_are_paced_and_name_the_sdk_result(
    ros_env: dict[str, str], make_config: Callable[..., Path]
) -> None:
    """An unknown host fails at once: attempts follow connect_retry_period_s, with the cause."""
    config_ = make_config(
        overrides={"hostname": "nonexistent.invalid", "connect_retry_period_s": 0.5}
    )
    # The text never appears: the node is observed for the whole window, then stopped
    outcome_ = Run_until_output(Node_command(config_), ros_env, "never printed", timeout_s=3.0)

    attempts_ = re.findall(r"Connect failed \((\w+)\)", outcome_.output)
    if not attempts_:
        pytest.skip("name resolution on this network took longer than the observation window")
    # At most one attempt per period; without the pause it was about 77 attempts per second
    assert len(attempts_) <= 7, outcome_.output
