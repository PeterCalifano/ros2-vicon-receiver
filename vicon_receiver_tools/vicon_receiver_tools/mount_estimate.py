"""Estimate of the pose of a robot link in the body frame of a Vicon rigid body (the "mount").

A viewer draws the robot model at the tracked pose of a Vicon rigid body; it needs the pose of a
robot link in the body frame of that rigid body. This module computes it from the marker
positions of a recording, for an upright link, with marker pairs known to sit left and right on
the robot. It knows nothing about a particular robot or marker placement: which markers form the
pairs and which mesh each marker sits on are inputs (for this repository: config/viewer.yaml,
read and passed on by scripts/estimate_mount.py).

Method, with what each step relies on:

1. Body frame. Vicon Tracker creates an object with its origin at the centroid of its markers and
   its axes along the room axes at that moment.
2. Roll and pitch. The robot stood upright when the object was created, so the link z axis is the
   body z axis. Error: the tilt of the link at that moment.
3. Heading. For each left/right marker pair, the horizontal direction from the right to the left
   marker is the link's y axis; the pairs are averaged and their disagreement is reported (it
   shows how far the pairs are from purely lateral).
4. Position. Every marker centre sits surface_offset_m (marker radius + base) above the surface it
   is on. With the rotation fixed, the link position (3 unknowns) is the least-squares fit of the
   marker-to-surface distances, with standard deviations from the residuals. Markers far off the
   surface (e.g. on hardware that is not in the model) show up in the per-marker errors.

Distances to a surface are computed against a dense sample of it (SURFACE_SPACING_M, KD-tree):
the nearest sample overestimates the distance by about spacing^2 / (2 offset), 0.2 mm here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TypedDict

import numpy as np
import trimesh
import yaml
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from vicon_receiver_tools.viewer import Is_unit_quaternion

# Spacing of the surface samples used for the distances [m]
SURFACE_SPACING_M: float = 0.002

# Smallest horizontal span of a left/right marker pair that defines a direction [m]
MIN_PAIR_SPAN_M: float = 0.01

# Starting points of the position fit around the first guess (the fit is local) [m]
POSITION_START_OFFSETS_M: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 0.0),
    (0.04, 0.0, 0.0),
    (-0.04, 0.0, 0.0),
    (0.0, 0.04, 0.0),
    (0.0, -0.04, 0.0),
    (0.0, 0.0, 0.04),
    (0.0, 0.0, -0.04),
)


class MountEstimateError(Exception):
    """The mount cannot be estimated from the given data."""


@dataclass
class SampledMeshSurface:
    """Dense samples of a mesh surface, for fast point-to-surface distances.

    Attributes:
        points: Samples on the surface (N x 3) [m].
        tree: KD-tree of the samples.
    """

    points: np.ndarray
    tree: cKDTree

    @classmethod
    def From_mesh(
        cls, mesh: trimesh.Trimesh, spacing_m: float = SURFACE_SPACING_M, seed: int = 0
    ) -> SampledMeshSurface:
        """Sample a mesh with about one point per spacing_m^2 of surface (reproducible)."""
        count_ = max(int(mesh.area / spacing_m**2), 1000)
        points_, _ = trimesh.sample.sample_surface(mesh, count_, seed=seed)
        points_ = np.asarray(points_, dtype=float)
        return cls(points=points_, tree=cKDTree(points_))

    def distances(self, points: np.ndarray) -> np.ndarray:
        """Distance of each point to the sampled surface [m]."""
        distances_, _ = self.tree.query(points)
        return np.asarray(distances_, dtype=float)


@dataclass(frozen=True)
class MountEstimateResult:
    """Pose of the robot link in the Vicon body frame, with how well it is supported.

    Attributes:
        rotation: 3x3 rotation, body frame -> link frame (p_link = rotation @ p_body + translation).
        translation: Translation [m], body frame -> link frame.
        heading_deg: Direction of the link x axis in the body x-y plane [deg].
        rms_m: RMS distance error of the markers to the expected surface offset [m].
        marker_errors_m: Per-marker distance error [m], in the order of the markers.
        translation_std_m: Standard deviation of the translation components [m].
        pair_directions_deg: Horizontal direction from the right to the left marker of each pair
            [deg], in the order of the pairs.
        pair_disagreement_deg: Largest angle between a pair direction and their mean [deg].
    """

    rotation: np.ndarray
    translation: np.ndarray
    heading_deg: float
    rms_m: float
    marker_errors_m: np.ndarray = field(repr=False)
    translation_std_m: np.ndarray = field(repr=False)
    pair_directions_deg: list[float]
    pair_disagreement_deg: float

    def link_in_body(self) -> tuple[np.ndarray, np.ndarray]:
        """Pose of the link in the body frame (rotation, translation): the mount of the viewer."""
        rotation_ = self.rotation.T
        return rotation_, -rotation_ @ self.translation


class ProbeMarker(TypedDict, total=False):
    """The fields of a marker of a probe recording that the estimate reads."""

    name: str
    occluded: bool
    position_mm: list[float]


class ProbeSegment(TypedDict, total=False):
    """The fields of a segment of a probe recording that the estimate reads."""

    translation_occluded: bool
    rotation_occluded: bool
    translation_mm: list[float]
    rotation_xyzw: list[float]


class ProbeSubject(TypedDict, total=False):
    """The fields of a subject of a probe recording that the estimate reads."""

    name: str
    segments: list[ProbeSegment]
    markers: list[ProbeMarker]


class ProbeFrame(TypedDict, total=False):
    """The fields of a frame of a probe recording that the estimate reads."""

    subjects: list[ProbeSubject]


class ProbeRecording(TypedDict, total=False):
    """The fields of a probe recording (scripts/probe.sh, JSON) that the estimate reads."""

    frames: list[ProbeFrame]


def Marker_constellation(recording: ProbeRecording, subject: str) -> tuple[list[str], np.ndarray]:
    """Average marker positions of a subject in its Vicon body frame, from a probe recording.

    Frames where the segment is occluded or has no valid rotation are skipped, and occluded
    markers.

    Args:
        recording: Parsed probe JSON (scripts/probe.sh).
        subject: Vicon subject name.

    Returns:
        The marker names and their positions (N x 3) [m] in the body frame of the first segment.

    Raises:
        MountEstimateError: If the subject or its markers are not in the recording.
    """
    sums_: dict[str, np.ndarray] = {}
    counts_: dict[str, int] = {}
    names_: list[str] = []

    for frame_ in recording.get("frames", []):
        for subject_ in frame_.get("subjects", []):
            if subject_.get("name") != subject or not subject_.get("segments"):
                continue
            segment_ = subject_["segments"][0]
            if segment_.get("translation_occluded", True) or segment_.get(
                "rotation_occluded", True
            ):
                continue
            # Vicon sends [0, 0, 0, 0] for an occluded rotation: never use it, flagged or not
            if not Is_unit_quaternion(np.asarray(segment_["rotation_xyzw"], dtype=float)):
                continue

            position_ = np.asarray(segment_["translation_mm"], dtype=float) / 1000.0
            rotation_ = Rotation.from_quat(segment_["rotation_xyzw"]).as_matrix()
            for marker_ in subject_.get("markers", []):
                if marker_.get("occluded", True) or "position_mm" not in marker_:
                    continue
                world_ = np.asarray(marker_["position_mm"], dtype=float) / 1000.0
                body_ = rotation_.T @ (world_ - position_)
                name_ = marker_["name"]
                if name_ not in sums_:
                    names_.append(name_)
                    sums_[name_] = np.zeros(3)
                    counts_[name_] = 0
                sums_[name_] += body_
                counts_[name_] += 1

    if len(names_) < 3:
        raise MountEstimateError(f"fewer than 3 visible markers of '{subject}' in the recording")
    points_ = np.array([sums_[name_] / counts_[name_] for name_ in names_])
    return names_, points_


def Angle_difference_deg(first: float, second: float) -> float:
    """Smallest absolute difference between two angles [deg]."""
    return abs((first - second + 180.0) % 360.0 - 180.0)


def Lateral_direction_deg(
    body_points: np.ndarray, pairs: list[tuple[int, int]]
) -> tuple[float, list[float], float]:
    """Direction of the link's left (y) axis in the body x-y plane, from left/right marker pairs.

    Args:
        body_points: Marker positions in the body frame (N x 3) [m].
        pairs: (left marker index, right marker index) of each pair.

    Returns:
        The mean direction [deg], the direction of each pair [deg] and the largest angle between
        a pair and the mean [deg].

    Raises:
        MountEstimateError: Without pairs, or if the markers of a pair are (almost) on top of
            each other.
    """
    if not pairs:
        raise MountEstimateError("at least one left/right marker pair is needed")
    directions_deg_: list[float] = []
    for left_, right_ in pairs:
        difference_ = body_points[left_] - body_points[right_]
        span_ = float(np.hypot(difference_[0], difference_[1]))
        if span_ < MIN_PAIR_SPAN_M:
            raise MountEstimateError(
                f"markers {left_} and {right_} are {span_ * 1000:.0f} mm apart horizontally: no direction"
            )
        directions_deg_.append(float(np.degrees(np.arctan2(difference_[1], difference_[0]))))

    unit_vectors_ = np.array(
        [[np.cos(np.radians(angle_)), np.sin(np.radians(angle_))] for angle_ in directions_deg_]
    )
    mean_ = unit_vectors_.sum(axis=0)
    mean_deg_ = float(np.degrees(np.arctan2(mean_[1], mean_[0])))
    disagreement_deg_ = max(Angle_difference_deg(angle_, mean_deg_) for angle_ in directions_deg_)
    return mean_deg_, directions_deg_, disagreement_deg_


def Upright_rotation(heading_deg: float) -> np.ndarray:
    """Rotation body -> link of an upright link whose x axis points at heading_deg in the body x-y plane."""
    return Rotation.from_euler("z", heading_deg, degrees=True).as_matrix().T


def Fit_position(
    body_points: np.ndarray,
    rotation: np.ndarray,
    surfaces: list[SampledMeshSurface],
    surface_offset_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Least-squares link position for a fixed rotation: every marker at surface_offset_m from
    its surface.

    Args:
        body_points: Marker positions in the body frame (N x 3) [m], N >= 4.
        rotation: Rotation body -> link.
        surfaces: Surface of each marker, in the link frame (same order as the markers).
        surface_offset_m: Distance of a marker centre from its surface [m].

    Returns:
        The translation, the per-marker errors and the standard deviations of the translation.
    """
    if len(body_points) < 4 or len(surfaces) != len(body_points):
        raise MountEstimateError(
            "at least 4 markers, each with a surface, are needed for the position"
        )
    rotated_ = body_points @ rotation.T

    # Markers grouped by surface: one KD-tree query per surface
    groups_: dict[int, tuple[SampledMeshSurface, list[int]]] = {}
    for index_, surface_ in enumerate(surfaces):
        groups_.setdefault(id(surface_), (surface_, []))[1].append(index_)

    def _Residuals(translation_: np.ndarray) -> np.ndarray:
        link_points_ = rotated_ + translation_
        errors_ = np.zeros(len(body_points))
        for surface_, indices_ in groups_.values():
            errors_[indices_] = surface_.distances(link_points_[indices_]) - surface_offset_m
        return errors_

    # First guess: each marker at the centre of its surface, on average; several starts around it
    centres_ = np.array([surface_.points.mean(axis=0) for surface_ in surfaces])
    guess_ = (centres_ - rotated_).mean(axis=0)
    best_ = min(
        (
            least_squares(_Residuals, guess_ + np.asarray(offset_), diff_step=1e-3, max_nfev=300)
            for offset_ in POSITION_START_OFFSETS_M
        ),
        key=lambda result_: result_.cost,
    )

    errors_ = _Residuals(best_.x)
    # Gauss-Newton covariance: residual variance (N - 3 degrees of freedom) times (J^T J)^-1
    jacobian_ = np.asarray(best_.jac)
    variance_ = float(np.sum(errors_**2)) / max(len(errors_) - 3, 1)
    normal_ = jacobian_.T @ jacobian_
    if np.linalg.cond(normal_) < 1e12:
        std_ = np.sqrt(np.clip(np.diag(np.linalg.inv(normal_)) * variance_, 0.0, None))
    else:
        std_ = np.full(3, np.inf)
    return best_.x.copy(), errors_, std_


def Estimate_upright_mount(
    body_points: np.ndarray,
    lateral_pairs: list[tuple[int, int]],
    surfaces: list[SampledMeshSurface],
    surface_offset_m: float,
) -> MountEstimateResult:
    """LinkPoseInBodyFrame of an upright link from left/right marker pairs and a position fit (see module doc).

    Args:
        body_points: Marker positions in the body frame (N x 3) [m], N >= 4.
        lateral_pairs: (left marker index, right marker index) of each left/right pair.
        surfaces: Surface each marker sits on, in the link frame (same order as the markers).
        surface_offset_m: Distance of a marker centre from its surface [m].

    Raises:
        MountEstimateError: If the inputs do not define a heading or a position.
    """
    if body_points.ndim != 2 or body_points.shape[1] != 3:
        raise MountEstimateError("marker positions must be N x 3")
    lateral_deg_, pair_directions_deg_, disagreement_deg_ = Lateral_direction_deg(
        body_points, lateral_pairs
    )

    # The link's y axis points left; x = y cross z is 90 deg clockwise from it
    heading_deg_ = (lateral_deg_ - 90.0 + 180.0) % 360.0 - 180.0
    rotation_ = Upright_rotation(heading_deg_)
    translation_, errors_, std_ = Fit_position(body_points, rotation_, surfaces, surface_offset_m)

    return MountEstimateResult(
        rotation=rotation_,
        translation=translation_,
        heading_deg=heading_deg_,
        rms_m=float(np.sqrt(np.mean(errors_**2))),
        marker_errors_m=errors_,
        translation_std_m=std_,
        pair_directions_deg=pair_directions_deg_,
        pair_disagreement_deg=disagreement_deg_,
    )


def Write_mount_file(
    path: Path, link: str, subject: str, estimate: MountEstimateResult, recording: str
) -> None:
    """Write the mount (pose of the link in the body frame) as YAML for the viewer."""
    rotation_, translation_ = estimate.link_in_body()
    rpy_ = Rotation.from_matrix(rotation_).as_euler("xyz")
    std_mm_ = ", ".join(f"{value_ * 1000:.0f}" for value_ in estimate.translation_std_m)
    document_ = {
        "link": link,
        "subject": subject,
        "xyz_m": [round(float(value_), 5) for value_ in translation_],
        "rpy_rad": [round(float(value_), 5) for value_ in rpy_],
    }
    header_ = (
        f"# Pose of the robot link '{link}' in the Vicon body frame of subject '{subject}'.\n"
        "# ESTIMATED by scripts/estimate_mount.py, not measured: link upright (robot standing when\n"
        "# the Vicon object was created), heading from the left/right marker pairs of the viewer\n"
        f"# config (they agree within {estimate.pair_disagreement_deg:.1f} deg), position fitted to the meshes\n"
        f"# (RMS {estimate.rms_m * 1000:.1f} mm, std x/y/z {std_mm_} mm). Recording: {recording}.\n"
        "# Adjust it in the viewer (scripts/start_visualizer.sh) and save from there if needed.\n"
    )
    path.write_text(header_ + yaml.safe_dump(document_, sort_keys=False), encoding="utf-8")


def Report(
    estimate: MountEstimateResult,
    names: list[str],
    lateral_pairs: list[tuple[int, int]],
    link: str,
    subject: str,
) -> str:
    """Text report of an estimate: pose, agreement of the pairs, per-marker errors."""
    rotation_, translation_ = estimate.link_in_body()
    rpy_deg_ = Rotation.from_matrix(rotation_).as_euler("xyz", degrees=True)
    std_ = estimate.translation_std_m * 1000.0
    lines_ = [
        f"{link} in the body frame of {subject} (upright: roll = pitch = 0):",
        f"  xyz [m]   {np.round(translation_, 4)}  (std {std_[0]:.0f} / {std_[1]:.0f} / {std_[2]:.0f} mm, link axes)",
        f"  rpy [deg] {np.round(rpy_deg_, 1)}",
        f"  heading   link x axis at {estimate.heading_deg:.1f} deg in the body x-y plane",
    ]
    for (left_, right_), direction_deg_ in zip(lateral_pairs, estimate.pair_directions_deg):
        lines_.append(
            f"  pair      right {names[right_]} -> left {names[left_]}: {direction_deg_:.1f} deg"
        )
    lines_.append(
        f"  pairs agree within {estimate.pair_disagreement_deg:.1f} deg; RMS {estimate.rms_m * 1000:.1f} mm"
    )
    for index_, name_ in enumerate(names):
        lines_.append(f"  {name_:28s} error {estimate.marker_errors_m[index_] * 1000:6.1f} mm")
    return "\n".join(lines_)
