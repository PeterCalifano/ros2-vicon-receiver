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
    Estimate_upright_mount,
    Lateral_direction_deg,
    Marker_constellation,
    MountEstimateError,
    SampledMeshSurface,
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


# Marker centres in the torso link frame: head markers on the head sides (one higher), shoulder
# markers left/right at the same height, three more on the torso faces
OFFSET_M: float = 0.010
LINK_MARKERS: np.ndarray = np.array(
    [
        [0.15 + OFFSET_M, 0.05, -0.10],  # torso front
        [-0.15 - OFFSET_M, 0.00, 0.05],  # torso back
        [0.05, 0.10 + OFFSET_M, -0.20],  # torso left, low
        [0.00, 0.04, 0.43 + OFFSET_M],  # head top, left of centre (highest)
        [0.00, -0.08 - OFFSET_M, 0.33],  # head right (lower)
        [-0.15 - OFFSET_M, 0.06, 0.20],  # left shoulder, on the back face
        [-0.15 - OFFSET_M, -0.06, 0.20],  # right shoulder, on the back face
    ]
)


def Body_points(heading_deg: float, origin_in_body: np.ndarray) -> np.ndarray:
    """Markers in a Vicon body frame where the upright link is at heading_deg and origin_in_body,
    centred like Vicon Tracker does (origin at the marker centroid)."""
    link_in_body_ = Rotation.from_euler("z", heading_deg, degrees=True).as_matrix()
    points_ = LINK_MARKERS @ link_in_body_.T + origin_in_body
    return points_ - points_.mean(axis=0)


# Left/right pairs of the synthetic markers (left index, right index): head, back of the shoulders
LATERAL_PAIRS: list[tuple[int, int]] = [(3, 4), (5, 6)]

# Markers on the head: compared with the head mesh only
HEAD_MARKERS: tuple[int, ...] = (3, 4)


def Synthetic_surfaces() -> list[SampledMeshSurface]:
    """Surface of each synthetic marker: the head for the head markers, the whole robot otherwise."""
    mesh_, head_ = Synthetic_robot()
    robot_surface_ = SampledMeshSurface.From_mesh(mesh_)
    head_surface_ = SampledMeshSurface.From_mesh(head_)
    return [head_surface_ if index_ in HEAD_MARKERS else robot_surface_ for index_ in range(len(LINK_MARKERS))]


def test_lateral_direction_from_left_right_pairs() -> None:
    """Each pair points from its right to its left marker; skewed pairs show as disagreement."""
    body_points_ = Body_points(0.0, np.zeros(3))
    mean_deg_, directions_deg_, disagreement_deg_ = Lateral_direction_deg(body_points_, LATERAL_PAIRS)
    # Link at heading 0: its left (y) axis is at +90 deg
    assert mean_deg_ == pytest.approx(90.0, abs=1e-9)
    assert directions_deg_ == pytest.approx([90.0, 90.0], abs=1e-9)
    assert disagreement_deg_ == pytest.approx(0.0, abs=1e-9)

    skewed_ = body_points_.copy()
    skewed_[3, 0] += 0.05  # left head marker 5 cm forward
    _, directions_deg_, disagreement_deg_ = Lateral_direction_deg(skewed_, LATERAL_PAIRS)
    assert directions_deg_[0] < 90.0
    assert disagreement_deg_ > 5.0


def test_estimate_recovers_heading_and_position() -> None:
    """Known upright mount: heading from the pairs, position from the surfaces."""
    surfaces_ = Synthetic_surfaces()
    heading_deg_ = -61.0
    body_points_ = Body_points(heading_deg_, np.array([0.01, -0.02, 0.03]))

    estimate_ = Estimate_upright_mount(body_points_, LATERAL_PAIRS, surfaces_, OFFSET_M)

    assert estimate_.heading_deg == pytest.approx(heading_deg_, abs=0.5)
    assert estimate_.pair_disagreement_deg == pytest.approx(0.0, abs=0.5)
    assert estimate_.rms_m < 0.002
    # The link frame maps the markers back onto their designed positions
    link_points_ = body_points_ @ estimate_.rotation.T + estimate_.translation
    np.testing.assert_allclose(link_points_, LINK_MARKERS, atol=0.004)


def test_swapped_left_and_right_turn_the_robot_around() -> None:
    """Left/right is a stated fact: swapping it gives the opposite heading (and a worse fit)."""
    surfaces_ = Synthetic_surfaces()
    body_points_ = Body_points(-61.0, np.zeros(3))
    swapped_ = [(right_, left_) for left_, right_ in LATERAL_PAIRS]
    estimate_ = Estimate_upright_mount(body_points_, swapped_, surfaces_, OFFSET_M)
    assert estimate_.heading_deg == pytest.approx(119.0, abs=0.5)


def test_estimate_needs_pairs_and_markers() -> None:
    """No pair: no heading; fewer than 4 markers: no position."""
    surfaces_ = Synthetic_surfaces()
    body_points_ = Body_points(0.0, np.zeros(3))
    with pytest.raises(MountEstimateError, match="at least one left/right marker pair"):
        Estimate_upright_mount(body_points_, [], surfaces_, OFFSET_M)
    with pytest.raises(MountEstimateError, match="at least 4 markers"):
        Estimate_upright_mount(body_points_[:3], [(0, 1)], surfaces_[:3], OFFSET_M)
