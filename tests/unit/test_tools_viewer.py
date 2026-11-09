"""Unit tests of the live viewer parts that need neither a browser nor ROS 2.

Covers the conversion of ViconFrame messages into view data, marker colours, pose helpers, trails,
the telemetry text and the thread-safe frame buffer, plus the viewer config and mount files. One
test runs the scene on a local viser server (conda environment; skipped otherwise).
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from conftest import VIEWER_CONFIG, Subject_at_origin, ViewerScene, Write_viewer_config
from vicon_receiver_tools.config import ConfigError
from vicon_receiver_tools.viewer import (
    CAMERA_OFFSET_M,
    COLOR_FEW,
    COLOR_GOOD,
    Camera_position,
    LatestFrameBuffer,
    Marker_colors,
    Matrix_to_wxyz_position,
    Pose_matrix,
    Telemetry_markdown,
    Trail_segments,
    Viewer_frame_from_message,
    ViewerFrame,
    ViewerMarker,
)
from vicon_receiver_tools.viewer_config import (
    IDENTITY_MOUNT,
    LinkPoseInBodyFrame,
    Load_mount,
    Load_viewer_config,
    Save_mount,
)


def Make_message(occluded: bool = False) -> SimpleNamespace:
    """A ViconFrame-like object with one subject, one segment and two markers."""
    segment_ = SimpleNamespace(
        name="seg",
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        translation_occluded=occluded,
        rotation_occluded=False,
    )
    markers_ = [
        SimpleNamespace(
            name="m1",
            position=SimpleNamespace(x=1.1, y=2.0, z=3.0),
            occluded=False,
            camera_ids=[1, 2, 3, 4, 5],
        ),
        SimpleNamespace(
            name="m2", position=SimpleNamespace(x=0.0, y=0.0, z=0.0), occluded=True, camera_ids=[]
        ),
    ]
    subject_ = SimpleNamespace(
        name="g1-unitree-torso-cam",
        ros_name="g1_unitree_torso_cam",
        quality_valid=True,
        quality=1.32,
        segments=[segment_],
        markers=markers_,
    )
    return SimpleNamespace(
        frame_number=42,
        frames_dropped=0,
        latency_total_s=0.031,
        any_occluded=occluded,
        subjects=[subject_],
    )


# -------------------------------------------------------------------------------------------------
# Message conversion
# -------------------------------------------------------------------------------------------------
def test_frame_view_from_message() -> None:
    """Positions, quaternion order (w first for viser), occlusion and camera counts are kept."""
    view_ = Viewer_frame_from_message(Make_message())
    subject_ = view_.subjects[0]
    segment_ = subject_.segments[0]
    assert view_.frame_number == 42
    assert subject_.quality == pytest.approx(1.32)
    np.testing.assert_allclose(segment_.position, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(segment_.wxyz, [1.0, 0.0, 0.0, 0.0])
    assert not segment_.occluded
    assert [marker_.camera_count for marker_ in subject_.markers] == [5, 0]


def test_occlusion_of_either_part_marks_the_segment() -> None:
    """Translation or rotation occluded: the segment is occluded."""
    assert Viewer_frame_from_message(Make_message(occluded=True)).subjects[0].segments[0].occluded


def test_zero_quaternion_without_flags_counts_as_occluded() -> None:
    """An all-zero rotation (Vicon's occluded pose) is never drawn, even without the flags."""
    message_ = Make_message()
    message_.subjects[0].segments[0].orientation.w = 0.0
    assert Viewer_frame_from_message(message_).subjects[0].segments[0].occluded


# -------------------------------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------------------------------
def test_camera_looks_at_the_robot_from_its_front_left() -> None:
    """The camera offset turns with the robot heading (x forward, y left)."""
    robot_ = np.array([8.5, 6.3, 0.9])
    np.testing.assert_allclose(Camera_position(robot_, 0.0), robot_ + np.asarray(CAMERA_OFFSET_M))
    # Robot facing +y: its front-left is at -x, +y
    offset_ = Camera_position(robot_, np.pi / 2) - robot_
    np.testing.assert_allclose(
        offset_, [-CAMERA_OFFSET_M[1], CAMERA_OFFSET_M[0], CAMERA_OFFSET_M[2]], atol=1e-12
    )


def test_marker_colors_by_camera_count() -> None:
    """Green with enough cameras, yellow with fewer (occluded markers are not drawn)."""
    markers_ = [
        ViewerMarker("a", np.zeros(3), camera_count=6, occluded=False),
        ViewerMarker("b", np.zeros(3), camera_count=2, occluded=False),
        ViewerMarker("c", np.zeros(3), camera_count=4, occluded=False),
    ]
    colors_ = Marker_colors(markers_, good_camera_count=4)
    assert [tuple(color_) for color_ in colors_] == [COLOR_GOOD, COLOR_FEW, COLOR_GOOD]


def test_pose_matrix_round_trip() -> None:
    """wxyz + position -> matrix -> wxyz + position."""
    wxyz_ = np.array([np.cos(0.3), 0.0, 0.0, np.sin(0.3)])
    position_ = np.array([0.5, -1.0, 2.0])
    back_wxyz_, back_position_ = Matrix_to_wxyz_position(Pose_matrix(position_, wxyz_))
    # A quaternion and its opposite are the same rotation
    assert np.allclose(back_wxyz_, wxyz_) or np.allclose(back_wxyz_, -wxyz_)
    np.testing.assert_allclose(back_position_, position_)


def test_trail_segments() -> None:
    """N points give N-1 segments joining consecutive points."""
    points_ = np.array([[0.0, 0, 0], [1.0, 0, 0], [1.0, 1, 0]])
    segments_ = Trail_segments(points_)
    assert segments_.shape == (2, 2, 3)
    np.testing.assert_allclose(segments_[1], [[1.0, 0, 0], [1.0, 1, 0]])
    assert Trail_segments(points_[:1]).shape == (0, 2, 3)


def test_telemetry_markdown() -> None:
    """The panel shows frame, rate, latency and per-subject state."""
    text_ = Telemetry_markdown(Viewer_frame_from_message(Make_message()), rate_hz=99.5)
    assert "**Frame** 42" in text_
    assert "99.5 Hz" in text_
    assert "31.0 ms" in text_
    assert "**g1-unitree-torso-cam**: tracked, quality 1.32, markers 1/2, cameras 5-5" in text_


# -------------------------------------------------------------------------------------------------
# Frame buffer
# -------------------------------------------------------------------------------------------------
def test_frame_buffer_keeps_latest_frame_and_bounded_trail() -> None:
    """The trail keeps the last trail_length visible positions; occluded ones are skipped."""
    buffer_ = LatestFrameBuffer(trail_length=3)
    for index_ in range(5):
        message_ = Make_message(occluded=(index_ == 2))
        message_.subjects[0].segments[0].position.x = float(index_)
        message_.frame_number = index_
        buffer_.put_message(message_, receive_time_s=float(index_) * 0.01)
        buffer_.latest()  # one scene update per frame: every frame reaches the trail

    latest_ = buffer_.latest()
    assert isinstance(latest_, ViewerFrame)
    assert latest_.frame_number == 4
    trail_ = buffer_.trail("g1_unitree_torso_cam/seg")
    np.testing.assert_allclose(trail_[:, 0], [1.0, 3.0, 4.0])
    assert buffer_.rate_hz(now_s=0.045) == pytest.approx(100.0)


def test_frame_buffer_rate_falls_to_zero_when_the_stream_stops() -> None:
    """No frame for longer than the rate window: the displayed rate is 0, not the last value."""
    buffer_ = LatestFrameBuffer(trail_length=3)
    for index_ in range(50):
        buffer_.put_message(Make_message(), receive_time_s=float(index_) * 0.01)
    assert buffer_.rate_hz(now_s=0.5) == pytest.approx(100.0)
    assert buffer_.rate_hz(now_s=5.0) == 0.0


def test_frame_buffer_without_frames() -> None:
    """An empty buffer has no frame, no trail and no rate."""
    buffer_ = LatestFrameBuffer(trail_length=10)
    assert buffer_.latest() is None
    assert buffer_.trail("x/y").shape == (0, 3)
    assert buffer_.rate_hz() == 0.0


# -------------------------------------------------------------------------------------------------
# Viewer config and mount file
# -------------------------------------------------------------------------------------------------
def test_repository_viewer_config_is_valid() -> None:
    """config/viewer.yaml loads, with the cameras of the robot recording."""
    config_ = Load_viewer_config(VIEWER_CONFIG)
    assert config_.robot.subject == "g1-unitree-torso-cam"
    assert config_.robot.mount_link == "torso_link"
    color_ = next(camera_ for camera_ in config_.cameras if camera_.name == "color")
    # 1280 x 720 with fy 912.35: about 43 degrees vertical field of view
    assert np.degrees(color_.vertical_fov_rad()) == pytest.approx(43.08, abs=0.05)
    assert color_.aspect() == pytest.approx(16.0 / 9.0, rel=1e-3)


def test_viewer_config_missing_entry(tmp_path: Path) -> None:
    """A missing entry is reported by its path, not replaced by a default."""
    config_ = Write_viewer_config(tmp_path, lambda document_: document_["viewer"].pop("port"))
    with pytest.raises(ConfigError, match=r"viewer\.port not found"):
        Load_viewer_config(config_)


def test_viewer_config_rejects_non_positive_rate(tmp_path: Path) -> None:
    """Rates and sizes must be positive."""
    config_ = Write_viewer_config(
        tmp_path, lambda document_: document_["viewer"].update(update_rate_hz=0.0)
    )
    with pytest.raises(ConfigError, match=r"viewer\.update_rate_hz must be > 0"):
        Load_viewer_config(config_)


def test_viewer_config_rejects_negative_surface_offset(tmp_path: Path) -> None:
    """Markers cannot sit below the surface of the meshes."""
    config_ = Write_viewer_config(
        tmp_path, lambda document_: document_["mount_estimate"].update(surface_offset_m=-0.01)
    )
    with pytest.raises(ConfigError, match=r"surface_offset_m must be >= 0"):
        Load_viewer_config(config_)


def test_mount_file_round_trip(tmp_path: Path) -> None:
    """A saved mount loads back with the same values."""
    path_ = tmp_path / "mount.yaml"
    mount_ = LinkPoseInBodyFrame(xyz_m=(0.1, -0.2, 0.3), rpy_rad=(0.01, 0.02, 1.5))
    Save_mount(path_, mount_, "torso_link", "subject", "test")
    loaded_ = Load_mount(path_)
    assert loaded_ is not None
    np.testing.assert_allclose(loaded_.xyz_m, mount_.xyz_m)
    np.testing.assert_allclose(loaded_.rpy_rad, mount_.rpy_rad)
    assert "# test" in path_.read_text(encoding="utf-8")


def test_missing_mount_file_gives_none(tmp_path: Path) -> None:
    """No mount file yet: the viewer falls back to the identity mount."""
    assert Load_mount(tmp_path / "missing.yaml") is None
    np.testing.assert_allclose(IDENTITY_MOUNT.matrix(), np.eye(4))


def test_frame_buffer_decodes_only_the_latest_serialized_message() -> None:
    """Serialized messages are decoded lazily: frames between two scene updates are skipped."""
    decoded_: list[bytes] = []

    def _Deserialize(data_: bytes) -> SimpleNamespace:
        decoded_.append(data_)
        message_ = Make_message()
        message_.frame_number = int(data_.decode())
        return message_

    buffer_ = LatestFrameBuffer(trail_length=10, deserialize=_Deserialize)
    for frame_number_ in (1, 2, 3):
        buffer_.put_serialized(str(frame_number_).encode())

    latest_ = buffer_.latest()
    assert latest_ is not None
    assert latest_.frame_number == 3
    assert decoded_ == [b"3"]
    # No new message: nothing decoded again
    buffer_.latest()
    assert decoded_ == [b"3"]


def test_frame_buffer_needs_a_deserializer_for_serialized_data() -> None:
    """Serialized data without a deserializer is a programming error."""
    buffer_ = LatestFrameBuffer(trail_length=10)
    buffer_.put_serialized(b"data")
    with pytest.raises(ValueError, match="without deserialize"):
        buffer_.latest()


def test_viewer_config_has_the_set_up_marker_pairs() -> None:
    """The set-up facts of the mount estimate are configuration, read by name."""
    settings_ = Load_viewer_config(VIEWER_CONFIG).mount_estimate
    assert len(settings_.lateral_marker_pairs) >= 1
    for pair_ in settings_.lateral_marker_pairs:
        assert pair_.left
        assert pair_.right
        assert pair_.left != pair_.right
    assert all(isinstance(link_, str) for link_ in settings_.marker_links.values())


def test_viewer_config_rejects_a_pair_of_one_marker(tmp_path: Path) -> None:
    """Left and right must be two different markers."""

    def _Same_marker_twice(document_: dict) -> None:
        pair_ = document_["mount_estimate"]["lateral_marker_pairs"][0]
        pair_["right"] = pair_["left"]

    config_ = Write_viewer_config(tmp_path, _Same_marker_twice)
    with pytest.raises(ConfigError, match="left and right are the same marker"):
        Load_viewer_config(config_)


# -------------------------------------------------------------------------------------------------
# Scene
# -------------------------------------------------------------------------------------------------
def Scene_frame(number: int, subjects: list) -> ViewerFrame:
    """ViewerFrame with the given subjects."""
    return ViewerFrame(
        frame_number=number, frames_dropped=0, latency_s=0.03, any_occluded=False, subjects=subjects
    )


def test_objects_of_a_removed_subject_are_hidden(jointed_robot_scene: ViewerScene) -> None:
    """A subject missing from the latest frame (removed from the Tracker) leaves no frozen axes,
    markers or robot behind; it shows again when it comes back."""
    viewer_ = jointed_robot_scene.viewer
    robot_ = Subject_at_origin(jointed_robot_scene.config.robot.subject, "robot")
    other_ = Subject_at_origin("wand", "wand")
    buffer_ = LatestFrameBuffer(trail_length=10)

    viewer_.update(Scene_frame(1, [robot_, other_]), buffer_)
    assert viewer_._segment_handles["wand/root"].visible
    assert viewer_._marker_handles["wand"].visible
    assert viewer_._robot_frame.visible

    viewer_.update(Scene_frame(2, []), buffer_)
    assert not viewer_._segment_handles["wand/root"].visible
    assert not viewer_._segment_handles["robot/root"].visible
    assert not viewer_._marker_handles["wand"].visible
    assert not viewer_._robot_frame.visible
    assert not viewer_._camera_handles["cam"].visible

    viewer_.update(Scene_frame(3, [robot_, other_]), buffer_)
    assert viewer_._segment_handles["wand/root"].visible
    assert viewer_._robot_frame.visible


def test_robot_without_segments_is_hidden(jointed_robot_scene: ViewerScene) -> None:
    """A robot subject still in the stream but with no segment left does not stay frozen."""
    viewer_ = jointed_robot_scene.viewer
    robot_ = Subject_at_origin(jointed_robot_scene.config.robot.subject, "robot")
    buffer_ = LatestFrameBuffer(trail_length=10)

    viewer_.update(Scene_frame(1, [robot_]), buffer_)
    assert viewer_._robot_frame.visible

    viewer_.update(Scene_frame(2, [replace(robot_, segments=[])]), buffer_)
    assert not viewer_._robot_frame.visible
    assert not viewer_._camera_handles["cam"].visible
