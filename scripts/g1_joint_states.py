#!/usr/bin/env python3
"""g1_joint_states.py - Joint states of the G1 for the live viewer.

Converts the motor angles of the robot (unitree_hg/msg/LowState, by motor index) into a
sensor_msgs/msg/JointState (by URDF joint name), which the live viewer reads to draw the robot in
its current posture (robot.joint_states_topic in config/viewer.yaml). This script is the adapter
between the G1 and the generic viewer: the motor order and the topics are set-up facts of
config/g1_joint_states.yaml.

Options: scripts/g1_joint_states.py --help
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT: Path = Path(__file__).resolve().parents[1]
TOOLS_DIR: Path = REPO_ROOT / "vicon_receiver_tools"
DEFAULT_CONFIG: Path = REPO_ROOT / "config" / "g1_joint_states.yaml"
SCRIPT_NAME: str = Path(__file__).name

# Top-level key of the config file
CONFIG_SECTION: str = "g1_joint_states"

USAGE: str = f"""Usage: {SCRIPT_NAME} [-c FILE] [-h]

Publish the joint angles of the G1 for the live viewer: unitree_hg/msg/LowState
(motor angles by index) in, sensor_msgs/msg/JointState (by URDF joint name) out,
one message per LowState message, until Ctrl+C. Needs ROS 2 and the unitree_hg
messages in the environment (source scripts/source_ros_env.sh and the overlay
of the workspace that provides unitree_hg).

Options:
  -c, --config FILE  Adapter config: topics and the URDF joint of each motor
                     (default: config/g1_joint_states.yaml)
  -h, --help         Show this help and exit

Examples:
  scripts/g1_joint_states.py
  scripts/g1_joint_states.py -c config/g1_joint_states_replay.yaml
"""


@dataclass(frozen=True)
class G1JointStatesConfig:
    """Topics and motor order of the adapter.

    Attributes:
        lowstate_topic: unitree_hg/msg/LowState topic of the robot.
        joint_states_topic: sensor_msgs/msg/JointState topic published for the viewer.
        motor_joint_names: URDF joint of each motor, in motor index order.
    """

    lowstate_topic: str
    joint_states_topic: str
    motor_joint_names: tuple[str, ...]


def Fail(message: str) -> int:
    """Print an error like the shell scripts do and return the exit status 1."""
    print(f"[{SCRIPT_NAME}] ERROR: {message}", file=sys.stderr)
    return 1


def Fail_with_usage(message: str) -> int:
    """Print the usage and an error to stderr, for command lines that cannot be understood."""
    print(USAGE, end="", file=sys.stderr)
    return Fail(message)


def Parse_arguments(argv: list[str]) -> Path | int:
    """Config file of the command line, or an exit status (help shown, or an error)."""
    config_ = DEFAULT_CONFIG
    index_ = 0
    while index_ < len(argv):
        argument_ = argv[index_]
        if argument_ in ("-h", "--help"):
            print(USAGE, end="")
            return 0
        if argument_ in ("-c", "--config"):
            if index_ + 1 >= len(argv):
                return Fail(f"Option {argument_} requires a value.")
            config_ = Path(argv[index_ + 1])
            index_ += 2
            continue
        return Fail_with_usage(f"Unknown option: {argument_}")
    return config_


def Load_config(path: Path) -> G1JointStatesConfig:
    """Read and check the adapter config.

    Raises:
        ConfigError: If an entry is missing or malformed (reported by name).
    """
    from vicon_receiver_tools.config import ConfigError, Load_yaml

    document_ = Load_yaml(path)
    section_ = document_.get(CONFIG_SECTION) if isinstance(document_, dict) else None
    if not isinstance(section_, dict):
        raise ConfigError(f"{path}: {CONFIG_SECTION} not found")
    for key_ in ("lowstate_topic", "joint_states_topic"):
        if not isinstance(section_.get(key_), str) or not section_[key_].strip():
            raise ConfigError(f"{CONFIG_SECTION}.{key_} must be a non-empty string")
    names_ = section_.get("motor_joint_names")
    if not isinstance(names_, list) or not names_ or not all(isinstance(name_, str) and name_ for name_ in names_):
        raise ConfigError(f"{CONFIG_SECTION}.motor_joint_names must be a non-empty list of joint names")
    if len(set(names_)) != len(names_):
        raise ConfigError(f"{CONFIG_SECTION}.motor_joint_names lists a joint twice")
    return G1JointStatesConfig(
        lowstate_topic=section_["lowstate_topic"],
        joint_states_topic=section_["joint_states_topic"],
        motor_joint_names=tuple(names_),
    )


def Motor_angles(motor_states: Sequence[Any], joint_count: int) -> list[float]:
    """Angles [rad] of the first joint_count motors (MotorState.q), in motor index order.

    Raises:
        ValueError: If the message has fewer motors than configured joints.
    """
    if len(motor_states) < joint_count:
        raise ValueError(f"LowState has {len(motor_states)} motors, the config names {joint_count}")
    return [float(motor_states[index_].q) for index_ in range(joint_count)]


def Main(argv: list[str]) -> int:
    """Run the adapter until Ctrl+C."""
    config_path_ = Parse_arguments(argv)
    if isinstance(config_path_, int):
        return config_path_
    if not config_path_.is_file():
        return Fail(f"File not found: {config_path_}")

    # Tools from their sources, as the shell scripts do (works before the build)
    sys.path.insert(0, str(TOOLS_DIR))
    from vicon_receiver_tools.config import ConfigError

    try:
        config_ = Load_config(config_path_)
    except ConfigError as error_:
        return Fail(f"invalid config: {error_}")

    try:
        import rclpy
        from rclpy.exceptions import InvalidTopicNameException
        from rclpy.executors import ExternalShutdownException
        from sensor_msgs.msg import JointState
    except ImportError as error_:
        return Fail(f"ROS 2 not available ({error_}): source scripts/source_ros_env.sh first.")
    try:
        from unitree_hg.msg import LowState
    except ImportError:
        return Fail("unitree_hg messages not found: source the overlay of the workspace that provides them.")

    rclpy.init()
    node_ = rclpy.create_node("g1_joint_states")
    joint_count_ = len(config_.motor_joint_names)
    try:
        publisher_ = node_.create_publisher(JointState, config_.joint_states_topic, 10)

        def _On_lowstate(message: Any) -> None:
            output_ = JointState()
            output_.header.stamp = node_.get_clock().now().to_msg()
            output_.name = list(config_.motor_joint_names)
            try:
                output_.position = Motor_angles(message.motor_state, joint_count_)
            except ValueError as error_:
                node_.get_logger().error(str(error_), throttle_duration_sec=5.0)
                return
            publisher_.publish(output_)

        node_.create_subscription(LowState, config_.lowstate_topic, _On_lowstate, 10)
        print(
            f"[{SCRIPT_NAME}] {config_.lowstate_topic} (unitree_hg/LowState) -> "
            f"{config_.joint_states_topic} (JointState, {joint_count_} joints)"
        )
        rclpy.spin(node_)
    except InvalidTopicNameException as error_:
        return Fail(f"invalid topic in {config_path_}: {error_}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node_.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(Main(sys.argv[1:]))
