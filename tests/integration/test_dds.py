"""Integration test: two ROS 2 processes on this machine exchange messages.

This failed on WSL with mirrored networking until scripts/lib/dds.sh selected
config/fastdds_wsl.xml (localhost traffic was routed to Windows). The ros_env fixture applies the
same DDS configuration as the scripts, so this test guards the fix (and passes trivially on
machines without the problem).
"""

from __future__ import annotations

import subprocess
import sys
import textwrap

import pytest

pytestmark = pytest.mark.integration

# Random-ish domain so that the test does not interfere with other ROS 2 systems on the machine
TEST_DOMAIN_ID: str = "87"

TALKER_SOURCE: str = textwrap.dedent(
    """
    import time
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("dds_test_talker")
    publisher = node.create_publisher(String, "/dds_test_topic", 10)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        publisher.publish(String(data="ping"))
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    """
)

LISTENER_SOURCE: str = textwrap.dedent(
    """
    import time
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("dds_test_listener")
    received = [0]
    node.create_subscription(String, "/dds_test_topic", lambda _m: received.__setitem__(0, received[0] + 1), 10)
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(received[0])
    rclpy.shutdown()
    """
)


def test_two_processes_exchange_messages(ros_env: dict[str, str]) -> None:
    """A listener in another process receives most of the talker's messages."""
    environment_ = dict(ros_env)
    environment_["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID

    talker_ = subprocess.Popen([sys.executable, "-c", TALKER_SOURCE], env=environment_)
    try:
        listener_ = subprocess.run(
            [sys.executable, "-c", LISTENER_SOURCE],
            env=environment_,
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
    finally:
        talker_.wait(timeout=30)

    received_ = int(listener_.stdout.strip().splitlines()[-1])
    # 6 s at 10 Hz is ~60 messages; discovery takes part of the first second
    assert received_ >= 30, f"only {received_} messages received across processes"
