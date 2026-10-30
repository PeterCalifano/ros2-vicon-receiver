"""Integration tests of scripts/launch.sh: reachability gate and node start-up.

The Vicon DataStream server is replaced by a local TCP listener (reachable case) or by an address
of the TEST-NET-1 documentation range, which is never assigned (unreachable case).
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from conftest import SCRIPTS_DIR, ShellRunner, TcpListener, Run_until_output

pytestmark = pytest.mark.integration

# RFC 5737 documentation address: guaranteed not to host a Vicon server
UNREACHABLE_ADDRESS: str = "192.0.2.1"

STARTING_MESSAGE: str = "Starting vicon_client"
STATIC_TF_MESSAGE: str = "Published static transform"


def test_launch_starts_node_when_server_reachable(
    ros_env: dict[str, str], make_config: Callable[..., Path], tcp_listener: TcpListener
) -> None:
    """The check passes against a listening port and the node starts with the given config."""
    config_ = make_config(overrides={"hostname": tcp_listener.host_spec})
    command_ = [str(SCRIPTS_DIR / "launch.sh"), "-c", str(config_)]

    outcome_ = Run_until_output(command_, ros_env, STATIC_TF_MESSAGE, timeout_s=30.0)
    assert outcome_.found, outcome_.output
    assert f"reachable at {tcp_listener.host_spec}" in outcome_.output
    assert f"{STARTING_MESSAGE} with {config_}" in outcome_.output


def test_launch_stops_when_server_unreachable(
    ros_shell: ShellRunner, make_config: Callable[..., Path]
) -> None:
    """An unreachable server stops launch.sh with an error, and the node is not started."""
    config_ = make_config(overrides={"hostname": UNREACHABLE_ADDRESS})
    result_ = ros_shell.run_script(
        "launch.sh", ["-c", str(config_)], extra_env={"VICON_CHECK_TIMEOUT_S": "1"}
    )
    assert result_.returncode == 1
    # Depending on the machine's routes: no route at all, or no answer through a gateway
    assert f"Vicon Tracker PC at {UNREACHABLE_ADDRESS}" in result_.stderr or (
        f"Vicon Tracker PC not reachable at {UNREACHABLE_ADDRESS}" in result_.stderr
    )
    assert STARTING_MESSAGE not in result_.stdout


def test_launch_no_check_starts_node_anyway(
    ros_env: dict[str, str], make_config: Callable[..., Path]
) -> None:
    """--no-check skips the gate: the node starts even if the server is unreachable."""
    config_ = make_config(overrides={"hostname": UNREACHABLE_ADDRESS})
    command_ = [str(SCRIPTS_DIR / "launch.sh"), "-c", str(config_), "--no-check"]

    outcome_ = Run_until_output(command_, ros_env, STATIC_TF_MESSAGE, timeout_s=30.0)
    assert outcome_.found, outcome_.output
    assert "Skipping the Vicon reachability check" in outcome_.output
