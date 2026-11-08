"""Unit tests of the joint states in the live viewer and of the G1 joint-state adapter.

The keep-last joint buffer, the joint configuration and the optional config entry need neither ROS
nor a browser. The placement test runs the viewer scene on a local viser server with a tiny URDF
(base -> waist -> torso, camera fixed to the torso) and needs the conda environment (viser,
yourdfpy); it is skipped otherwise. The adapter is tested without ROS or unitree_hg messages.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from conftest import REPO_ROOT, VIEWER_CONFIG, Subject_at_origin, ViewerScene, Write_viewer_config
from vicon_receiver_tools.config import ConfigError
from vicon_receiver_tools.viewer import (
    Joint_configuration,
    Joint_states_markdown,
    LatestJointStates,
)
from vicon_receiver_tools.viewer_config import Load_viewer_config

ADAPTER_CONFIG: Path = REPO_ROOT / "config" / "g1_joint_states.yaml"

# The adapter is a script: import it as a module
sys.path.insert(0, str(REPO_ROOT / "scripts"))
import g1_joint_states  # noqa: E402

def Joint_state(names: list[str], positions: list[float]) -> SimpleNamespace:
    """Object with the fields of a sensor_msgs/JointState that the viewer reads."""
    return SimpleNamespace(name=names, position=positions)


# -------------------------------------------------------------------------------------------------
# Keep-last joint buffer and configuration
# -------------------------------------------------------------------------------------------------
def test_joint_buffer_keeps_the_last_value_of_every_joint() -> None:
    """A message with some joints updates those and keeps the others (keep-last)."""
    buffer_ = LatestJointStates()
    buffer_.put_message(Joint_state(["a", "b"], [0.1, 0.2]), receive_time_s=10.0)
    assert buffer_.positions() == {"a": 0.1, "b": 0.2}
    buffer_.put_message(Joint_state(["b"], [0.5]), receive_time_s=11.0)
    assert buffer_.positions() == {"a": 0.1, "b": 0.5}
    assert buffer_.age_s(now_s=11.5) == pytest.approx(0.5)


def test_joint_buffer_ignores_missing_and_invalid_positions() -> None:
    """Messages without positions (velocities only) or with NaN change nothing."""
    buffer_ = LatestJointStates()
    buffer_.put_message(Joint_state(["a"], [0.3]))
    assert buffer_.positions() == {"a": 0.3}
    buffer_.put_message(Joint_state(["a", "b"], []))
    assert buffer_.positions() == {"a": 0.3}
    buffer_.put_message(Joint_state(["a", "b"], [float("nan"), 0.4]))
    assert buffer_.positions() == {"a": 0.3, "b": 0.4}


def test_joint_buffer_before_any_message() -> None:
    """No message yet: no angles, no age."""
    buffer_ = LatestJointStates()
    assert buffer_.positions() == {}
    assert buffer_.age_s() is None


def test_joint_buffer_decodes_serialized_messages_once() -> None:
    """Serialized messages are decoded at read time, only the newest one."""
    decoded_: list[bytes] = []

    def _Deserialize(data: bytes) -> SimpleNamespace:
        decoded_.append(data)
        return Joint_state(["a"], [float(data.decode())])

    buffer_ = LatestJointStates(deserialize=_Deserialize)
    buffer_.put_serialized(b"0.1")
    buffer_.put_serialized(b"0.2")
    assert buffer_.positions() == {"a": 0.2}
    assert buffer_.positions() == {"a": 0.2}
    assert decoded_ == [b"0.2"]


def test_joint_configuration_defaults_to_zero() -> None:
    """Angles in the model's joint order; joints never received are 0, unknown names ignored."""
    configuration_ = Joint_configuration(("a", "b", "c"), {"c": 0.3, "a": -0.1, "other": 9.0})
    np.testing.assert_allclose(configuration_, [-0.1, 0.0, 0.3])


def test_joint_states_markdown() -> None:
    """The telemetry line says where the angles come from."""
    names_ = ("a", "b")
    assert "no joint_states_topic" in Joint_states_markdown(names_, {}, None, None)
    assert "nothing on /joint_states yet" in Joint_states_markdown(names_, {}, None, "/joint_states")
    line_ = Joint_states_markdown(names_, {"a": 0.1}, 0.25, "/joint_states")
    assert "1/2 from /joint_states" in line_
    assert "0.2 s ago" in line_ or "0.3 s ago" in line_


# -------------------------------------------------------------------------------------------------
# Viewer config: optional topic
# -------------------------------------------------------------------------------------------------
def test_repository_viewer_config_reads_joint_states() -> None:
    """The repository config takes the joint angles from /joint_states."""
    assert Load_viewer_config(VIEWER_CONFIG).robot.joint_states_topic == "/joint_states"


def test_joint_states_topic_is_optional(tmp_path: Path) -> None:
    """Without the entry (or with null) the robot is drawn at zero angles."""
    path_ = Write_viewer_config(tmp_path, lambda document_: document_["robot"].pop("joint_states_topic"))
    assert Load_viewer_config(path_).robot.joint_states_topic is None
    path_ = Write_viewer_config(tmp_path, lambda document_: document_["robot"].update(joint_states_topic=None))
    assert Load_viewer_config(path_).robot.joint_states_topic is None


def test_empty_joint_states_topic_is_rejected(tmp_path: Path) -> None:
    """An empty topic is a mistake, not a way to disable it."""
    path_ = Write_viewer_config(tmp_path, lambda document_: document_["robot"].update(joint_states_topic=" "))
    with pytest.raises(ConfigError, match=r"robot\.joint_states_topic must not be empty"):
        Load_viewer_config(path_)


# -------------------------------------------------------------------------------------------------
# Viewer scene: the root and the cameras follow the joints around the tracked link
# -------------------------------------------------------------------------------------------------
def test_root_and_cameras_follow_the_joints(jointed_robot_scene: ViewerScene) -> None:
    """With the waist turned, the root turns around the tracked torso; the camera stays on it."""
    viewer_ = jointed_robot_scene.viewer
    # Subject at the origin, identity mount (no mount file): the torso is the world frame
    viewer_._update_robot(Subject_at_origin(jointed_robot_scene.config.robot.subject, "robot"))
    np.testing.assert_allclose(viewer_._robot_frame.position, [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(viewer_._robot_frame.wxyz, [1.0, 0.0, 0.0, 0.0], atol=1e-9)

    joints_ = LatestJointStates()
    joints_.put_message(Joint_state(["waist"], [0.5]))
    viewer_.update_joints(joints_)

    # Torso turned by +0.5 rad about z relative to the base: the base is at -0.5 rad in the
    # (tracked) torso frame, still 1 m below it
    expected_wxyz_ = Rotation.from_euler("z", -0.5).as_quat(scalar_first=True)
    np.testing.assert_allclose(viewer_._robot_frame.position, [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(np.abs(viewer_._robot_frame.wxyz), np.abs(expected_wxyz_), atol=1e-9)
    # The camera is fixed to the tracked torso: it does not move
    np.testing.assert_allclose(viewer_._camera_handles["cam"].position, [0.1, 0.0, 0.0], atol=1e-9)
    assert "1/1 from /joint_states" in viewer_._joints_line


# -------------------------------------------------------------------------------------------------
# G1 adapter (scripts/g1_joint_states.py)
# -------------------------------------------------------------------------------------------------
def test_adapter_config_names_the_29_motors() -> None:
    """The repository config maps the 29 G1 motors, with the waist at motors 12-14."""
    config_ = g1_joint_states.Load_config(ADAPTER_CONFIG)
    assert config_.lowstate_topic == "/lowstate"
    assert config_.joint_states_topic == Load_viewer_config(VIEWER_CONFIG).robot.joint_states_topic
    assert len(config_.motor_joint_names) == 29
    assert config_.motor_joint_names[12:15] == ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")


def test_adapter_config_names_joints_of_the_model() -> None:
    """Every configured joint is an actuated joint of the fetched G1 URDF (if fetched)."""
    yourdfpy_ = pytest.importorskip("yourdfpy")
    viewer_config_ = Load_viewer_config(VIEWER_CONFIG)
    urdf_path_ = REPO_ROOT / viewer_config_.robot.directory / viewer_config_.robot.urdf
    if not urdf_path_.is_file():
        pytest.skip(f"G1 model not fetched ({urdf_path_}): scripts/fetch_g1_model.sh")
    urdf_ = yourdfpy_.URDF.load(urdf_path_, load_meshes=False, build_scene_graph=False)
    names_ = g1_joint_states.Load_config(ADAPTER_CONFIG).motor_joint_names
    assert set(names_) <= set(urdf_.actuated_joint_names)


def test_adapter_config_rejects_a_repeated_joint(tmp_path: Path) -> None:
    """A joint named twice would hide a motor."""
    document_ = yaml.safe_load(ADAPTER_CONFIG.read_text(encoding="utf-8"))
    document_["g1_joint_states"]["motor_joint_names"][1] = document_["g1_joint_states"]["motor_joint_names"][0]
    path_ = tmp_path / "g1_joint_states.yaml"
    path_.write_text(yaml.safe_dump(document_), encoding="utf-8")
    with pytest.raises(ConfigError, match="lists a joint twice"):
        g1_joint_states.Load_config(path_)


def test_adapter_takes_the_motor_angles_in_index_order() -> None:
    """MotorState.q of the first configured motors, in index order; extra slots are dropped."""
    motors_ = [SimpleNamespace(q=0.01 * index_) for index_ in range(35)]
    assert g1_joint_states.Motor_angles(motors_, 3) == pytest.approx([0.0, 0.01, 0.02])
    with pytest.raises(ValueError, match="has 2 motors, the config names 3"):
        g1_joint_states.Motor_angles(motors_[:2], 3)
