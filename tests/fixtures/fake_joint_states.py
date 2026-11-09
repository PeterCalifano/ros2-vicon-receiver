"""Test fixture: publish a G1 posture as sensor_msgs/JointState, for checks of the live viewer.

Publishes the joints of config/g1_joint_states.yaml at 50 Hz on its joint_states_topic: arms
raised, elbows bent, and the waist turning slowly (yaw amplitude 0.4 rad, period 12 s); the other
joints stay at 0. Together with tests/fixtures/replay_probe.py it shows the robot of the viewer in
a posture without the robot.

Usage (ROS 2 + workspace sourced, e.g. source scripts/source_ros_env.sh):
    python3 tests/fixtures/fake_joint_states.py [--seconds S]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

REPO_ROOT: Path = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
import g1_joint_states  # noqa: E402

# Fixed angles of the posture [rad]
POSTURE_RAD: dict[str, float] = {
    "left_shoulder_pitch_joint": -1.2,
    "left_shoulder_roll_joint": 0.3,
    "left_elbow_joint": 0.9,
    "right_shoulder_pitch_joint": -0.4,
    "right_shoulder_roll_joint": -0.5,
    "right_elbow_joint": 1.2,
    "left_knee_joint": 0.3,
    "right_knee_joint": 0.3,
}

# Waist motion
WAIST_JOINT: str = "waist_yaw_joint"
WAIST_AMPLITUDE_RAD: float = 0.4
WAIST_PERIOD_S: float = 12.0

PUBLISH_RATE_HZ: float = 50.0


def Main(argv: list[str]) -> int:
    """Publish the posture until --seconds have passed or Ctrl+C."""
    parser_ = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser_.add_argument("--seconds", type=float, default=3600.0, help="how long to publish [s]")
    arguments_ = parser_.parse_args(argv)

    config_ = g1_joint_states.Load_config(REPO_ROOT / "config" / "g1_joint_states.yaml")
    names_ = list(config_.motor_joint_names)

    # Ctrl+C raises KeyboardInterrupt here instead of invalidating the context during a publish
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node_ = rclpy.create_node("fake_joint_states")
    publisher_ = node_.create_publisher(JointState, config_.joint_states_topic, 10)
    start_ = time.monotonic()
    try:
        while time.monotonic() - start_ < arguments_.seconds:
            elapsed_ = time.monotonic() - start_
            positions_ = dict(POSTURE_RAD)
            positions_[WAIST_JOINT] = WAIST_AMPLITUDE_RAD * math.sin(
                2.0 * math.pi * elapsed_ / WAIST_PERIOD_S
            )
            message_ = JointState()
            message_.header.stamp = node_.get_clock().now().to_msg()
            message_.name = names_
            message_.position = [positions_.get(name_, 0.0) for name_ in names_]
            publisher_.publish(message_)
            time.sleep(1.0 / PUBLISH_RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        node_.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(Main(sys.argv[1:]))
