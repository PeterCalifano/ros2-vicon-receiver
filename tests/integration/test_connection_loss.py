"""Integration test: a dead Vicon connection is detected by the TCP keepalive of the node.

Pulling the cable or switching the Tracker off leaves the SDK's TCP connection open but silent.
The node then relies on the keepalive options of utilities.cpp (enable_tcp_keepalive) to fail the
connection after connection_timeout_s. This test reproduces a silent peer without a Tracker and
without root: it runs the GoogleTest TcpKeepalive.DeadPeerIsDetectedWithinTheTimeout inside a
private user + network namespace (unshare -rn), where the test may drop every packet on its own
loopback device (tc netem loss 100%). The rest of the stop path (the SDK reports NotConnected,
the frame loop exits with status 2) is covered by ViconReceiverNodeConnection in the C++ tests.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from conftest import REPO_ROOT

pytestmark = pytest.mark.integration

# GoogleTest binary with the keepalive tests (built by tests/run_tests.sh, BUILD_TESTING=ON)
TEST_BINARY: Path = REPO_ROOT / "build" / "vicon_receiver" / "test_common"
TEST_NAME: str = "TcpKeepalive.DeadPeerIsDetectedWithinTheTimeout"


def Network_namespace_available() -> bool:
    """Whether this kernel lets an unprivileged user create a network namespace with netem."""
    if shutil.which("unshare") is None or shutil.which("tc") is None:
        return False
    result_ = subprocess.run(
        ["unshare", "-rn", "bash", "-c", "ip link set lo up && tc qdisc add dev lo root netem loss 100%"],
        capture_output=True,
        timeout=30,
        check=False,
    )
    return result_.returncode == 0


def test_dead_peer_is_detected(ros_env: dict[str, str]) -> None:
    """The keepalive options fail a silent connection after the timeout, with ETIMEDOUT."""
    if not TEST_BINARY.is_file():
        pytest.skip(f"C++ tests not built ({TEST_BINARY}): tests/run_tests.sh")
    if not Network_namespace_available():
        pytest.skip("no unprivileged network namespace with tc netem on this machine")

    environment_ = dict(ros_env, VICON_TEST_NETWORK_NAMESPACE="1")
    result_ = subprocess.run(
        ["unshare", "-rn", "bash", "-c", f'ip link set lo up && exec "$0" --gtest_filter={TEST_NAME}', str(TEST_BINARY)],
        env=environment_,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    output_ = result_.stdout + result_.stderr
    assert result_.returncode == 0, output_
    assert "[  PASSED  ] 1 test" in output_, output_
    assert "SKIPPED" not in output_, output_
