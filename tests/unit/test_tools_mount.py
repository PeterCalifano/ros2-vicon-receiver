"""Unit tests of the robot model access and of the marker mount estimate.

Need trimesh and yourdfpy (conda environment of environment.yml); skipped otherwise.
The fit is checked on a synthetic "robot" (a torso box and a head box) with a known mount.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

trimesh = pytest.importorskip("trimesh")
pytest.importorskip("yourdfpy")

from vicon_receiver_tools.mount_estimate import (  # noqa: E402
    Fit_mount,
    Marker_constellation,
    MountEstimateError,
    Top_marker_indices,
)
from vicon_receiver_tools.robot_model import (  # noqa: E402
    Link_pose,
    Load_urdf,
    RobotModelError,
    Resolve_mesh_path,
    Rigid_subtree,
)

# Tiny URDF: base -> torso (revolute), torso -> head and torso -> camera (fixed)
TEST_URDF: str = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="torso"/>
  <link name="head"/>
  <link name="camera"/>
  <joint name="waist" type="revolute">
    <parent link="base"/><child link="torso"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="neck" type="fixed">
    <parent link="torso"/><child link="head"/><origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>
  <joint name="camera_mount" type="fixed">
    <parent link="torso"/><child link="camera"/><origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
"""


# -------------------------------------------------------------------------------------------------
# Robot model
# -------------------------------------------------------------------------------------------------
def test_resolve_mesh_path_matches_the_tail(tmp_path: Path) -> None:
    """package:// paths of another workspace are found by their tail in the local copy."""
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "pelvis.obj").write_text("", encoding="utf-8")
    resolved_ = Resolve_mesh_path(
        fname="package://legged_nav_system/profiles/g1/meshes/pelvis.obj", model_dir=tmp_path
    )
    assert resolved_ == str(tmp_path / "meshes" / "pelvis.obj")
    # Unknown files are returned unchanged (yourdfpy reports them)
    assert Resolve_mesh_path(fname="package://x/missing.obj", model_dir=tmp_path) == "package://x/missing.obj"


def test_links_and_poses_of_a_urdf(tmp_path: Path) -> None:
    """Rigid subtree through fixed joints, and link poses in both tree directions."""
    (tmp_path / "test.urdf").write_text(TEST_URDF, encoding="utf-8")
    urdf_ = Load_urdf(tmp_path, "test.urdf", load_meshes=False)

    assert sorted(Rigid_subtree(urdf_, "torso")) == ["camera", "head", "torso"]
    np.testing.assert_allclose(Link_pose(urdf_, "camera", "torso")[:3, 3], [0.1, 0.0, 0.2])
    # Parent in the frame of the child: the inverse direction of the tree
    np.testing.assert_allclose(Link_pose(urdf_, "base", "torso")[:3, 3], [0.0, 0.0, -1.0])
    with pytest.raises(RobotModelError, match="not in the URDF"):
        Link_pose(urdf_, "arm", "torso")
    # Siblings (two fixed children of the torso)
    np.testing.assert_allclose(Link_pose(urdf_, "head", "camera")[:3, 3], [-0.1, 0.0, 0.2])


def test_unconnected_links_raise_a_model_error() -> None:
    """No transform in either direction: RobotModelError (handled by the viewer), not ValueError."""

    def _No_transform(frame_to: str, frame_from: str) -> None:
        raise ValueError(f"no path from {frame_from} to {frame_to}")

    urdf_ = SimpleNamespace(
        robot=SimpleNamespace(links=[SimpleNamespace(name="a"), SimpleNamespace(name="b")]),
        get_transform=_No_transform,
    )
    with pytest.raises(RobotModelError, match="no transform between 'a' and 'b'"):
        Link_pose(urdf_, "a", "b")  # type: ignore[arg-type]


def test_missing_urdf_is_reported(tmp_path: Path) -> None:
    """A model directory without URDF points to the fetch script."""
    with pytest.raises(RobotModelError, match="fetch_g1_model"):
        Load_urdf(tmp_path, "model.urdf", load_meshes=False)


# -------------------------------------------------------------------------------------------------
# Marker constellation from a probe recording
# -------------------------------------------------------------------------------------------------
def test_marker_constellation_in_body_frame() -> None:
    """Markers are expressed in the body frame; occluded samples and invalid rotations are skipped."""
    rotation_ = Rotation.from_euler("z", 90, degrees=True)
    body_points_ = {"m1": np.array([0.1, 0.0, 0.0]), "m2": np.array([0.0, 0.2, 0.0]), "m3": np.array([0.0, 0.0, 0.3])}
    body_position_mm_ = np.array([1000.0, 2000.0, 500.0])

    def _Marker(name_: str, occluded_: bool) -> dict:
        world_ = rotation_.apply(body_points_[name_]) * 1000.0 + body_position_mm_
        return {"name": name_, "position_mm": list(world_), "occluded": occluded_}

    frame_ = {
        "subjects": [
            {
                "name": "body",
                "segments": [
                    {
                        "translation_mm": list(body_position_mm_),
                        "rotation_xyzw": list(rotation_.as_quat()),
                        "translation_occluded": False,
                        "rotation_occluded": False,
                    }
                ],
                "markers": [_Marker("m1", False), _Marker("m2", False), _Marker("m3", False)],
            }
        ]
    }
    occluded_frame_ = {"subjects": [dict(frame_["subjects"][0], markers=[_Marker("m1", True)])]}
    # Occluded pose that lost its flags: all-zero rotation, markers far away; must be ignored
    zero_segment_ = dict(frame_["subjects"][0]["segments"][0], rotation_xyzw=[0.0, 0.0, 0.0, 0.0])
    far_marker_ = {"name": "m1", "position_mm": [9000.0, 9000.0, 9000.0], "occluded": False}
    zero_rotation_frame_ = {
        "subjects": [dict(frame_["subjects"][0], segments=[zero_segment_], markers=[far_marker_])]
    }
    names_, points_ = Marker_constellation({"frames": [frame_, occluded_frame_, zero_rotation_frame_]}, "body")

    assert names_ == ["m1", "m2", "m3"]
    for index_, name_ in enumerate(names_):
        np.testing.assert_allclose(points_[index_], body_points_[name_], atol=1e-9)
    with pytest.raises(MountEstimateError, match="fewer than 3"):
        Marker_constellation({"frames": [frame_]}, "other")


# -------------------------------------------------------------------------------------------------
# Fit
# -------------------------------------------------------------------------------------------------
def Synthetic_robot() -> tuple[object, object]:
    """Torso box (0.30 x 0.20 x 0.50) and head box on top of it, in the torso link frame."""
    torso_ = trimesh.creation.box(extents=(0.30, 0.20, 0.50))
    head_ = trimesh.creation.box(extents=(0.16, 0.16, 0.16))
    head_.apply_translation((0.0, 0.0, 0.35))
    return trimesh.util.concatenate([torso_, head_]), head_


def test_fit_mount_recovers_markers_on_the_surface() -> None:
    """Markers placed 1 cm off a known surface are fitted back onto it, top ones on the head."""
    mesh_, head_ = Synthetic_robot()
    offset_ = 0.010
    # Marker positions in the link frame: on torso faces and on the head sides
    link_points_ = np.array(
        [
            [0.15 + offset_, 0.05, -0.10],   # torso front
            [0.15 + offset_, -0.06, 0.12],   # torso front
            [-0.15 - offset_, 0.00, 0.05],   # torso back
            [0.05, 0.10 + offset_, -0.20],   # torso left
            [0.00, 0.08 + offset_, 0.38],    # head left (top marker)
            [0.00, -0.08 - offset_, 0.36],   # head right (top marker)
        ]
    )
    true_rotation_ = Rotation.from_euler("xyz", [0.0, 0.0, 70.0], degrees=True).as_matrix()
    true_translation_ = np.array([0.02, -0.03, -0.05])
    body_points_ = (link_points_ - true_translation_) @ true_rotation_   # inverse of p_link = R p_body + t

    top_ = Top_marker_indices(body_points_, 2)
    estimate_ = Fit_mount(body_points_, mesh_, surface_offset_m=offset_, constrained_indices=top_, constraint_mesh=head_)

    assert estimate_.rms_m < 0.002
    fitted_ = body_points_ @ estimate_.rotation.T + estimate_.translation
    # The top markers end up next to the head
    _, head_distances_, _ = trimesh.proximity.closest_point(head_, fitted_[top_])
    np.testing.assert_allclose(head_distances_, offset_, atol=0.003)


def test_fit_mount_needs_three_markers() -> None:
    """Two markers do not define a rigid fit."""
    mesh_, _ = Synthetic_robot()
    with pytest.raises(MountEstimateError, match="at least 3"):
        Fit_mount(np.zeros((2, 3)), mesh_)
