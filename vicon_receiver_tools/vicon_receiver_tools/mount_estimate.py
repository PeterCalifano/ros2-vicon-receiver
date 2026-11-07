"""Rough estimate of where the Vicon markers sit on the robot (the "mount" transform).

The Vicon rigid body (e.g. g1-unitree-torso-cam) has its own frame, defined in Vicon Tracker.
To draw the robot model at the tracked pose, the pose of a robot link (e.g. torso_link) in that
body frame is needed. It was not measured, so it is estimated here:

1. the marker constellation in the body frame is computed from a probe recording;
2. the constellation is fitted onto the surface of the meshes rigidly attached to the link
   (each marker centre sits about one marker radius plus base above the surface);
3. optionally the highest markers are forced onto one link (the user placed the two top markers
   on the head);
4. several starting yaw angles are tried (the body frame was created upright, so roll and pitch
   start at zero) and the best fit is kept.

The result is a rough estimate for visualisation (centimetre level), not a calibration.

Command line::

    python3 -m vicon_receiver_tools.mount_estimate --recording probe.json --viewer-config \\
        config/viewer.yaml [--write]
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

# Distance from the mesh surface to a marker centre [m]: 7 mm marker radius + ~3 mm base
DEFAULT_SURFACE_OFFSET_M: float = 0.010

# Starting yaw angles of the fit [deg]
DEFAULT_YAW_SEEDS_DEG: tuple[float, ...] = tuple(float(angle_) for angle_ in range(0, 360, 30))


class MountEstimateError(Exception):
    """The mount cannot be estimated from the given data."""


@dataclass(frozen=True)
class MountEstimateResult:
    """Pose of a robot link in the Vicon body frame, with the quality of the fit.

    Attributes:
        rotation: 3x3 rotation, body frame -> link frame (p_link = rotation @ p_body + translation).
        translation: Translation [m], body frame -> link frame.
        rms_m: RMS distance error of the markers to the expected surface offset [m].
        marker_errors_m: Per-marker distance error [m], in the order of the markers.
        start_yaw_deg: Starting yaw of the best fit.
    """

    rotation: np.ndarray
    translation: np.ndarray
    rms_m: float
    marker_errors_m: np.ndarray = field(repr=False)
    start_yaw_deg: float

    def link_in_body(self) -> tuple[np.ndarray, np.ndarray]:
        """Pose of the link in the body frame (rotation, translation): the mount of the viewer."""
        rotation_ = self.rotation.T
        return rotation_, -rotation_ @ self.translation


def Marker_constellation(recording: dict[str, Any], subject: str) -> tuple[list[str], np.ndarray]:
    """Average marker positions of a subject in its Vicon body frame, from a probe recording.

    Frames where the segment or a marker is occluded are skipped (per marker).

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
            if segment_.get("translation_occluded", True) or segment_.get("rotation_occluded", True):
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


def Top_marker_indices(body_points: np.ndarray, count: int) -> list[int]:
    """Indices of the markers highest along the body z axis (upright body frame)."""
    order_ = np.argsort(body_points[:, 2])[::-1]
    return [int(index_) for index_ in order_[:count]]


def Fit_mount(
    body_points: np.ndarray,
    mesh: trimesh.Trimesh,
    surface_offset_m: float = DEFAULT_SURFACE_OFFSET_M,
    yaw_seeds_deg: tuple[float, ...] = DEFAULT_YAW_SEEDS_DEG,
    constrained_indices: list[int] | None = None,
    constraint_mesh: trimesh.Trimesh | None = None,
) -> MountEstimateResult:
    """Fit a marker constellation onto a mesh surface.

    Finds the rigid transform p_link = R @ p_body + t that puts every marker at surface_offset_m
    from the mesh; markers in constrained_indices are measured against constraint_mesh instead
    (e.g. the head only).

    Args:
        body_points: Marker positions in the body frame (N x 3) [m], N >= 3.
        mesh: Mesh of the link (and the links fixed to it), in the link frame.
        surface_offset_m: Expected distance of a marker centre from the surface [m].
        yaw_seeds_deg: Starting yaw angles; roll and pitch start at zero.
        constrained_indices: Markers that must lie on constraint_mesh.
        constraint_mesh: Mesh for the constrained markers, in the link frame.

    Returns:
        The best fit over all starts.

    Raises:
        MountEstimateError: If the inputs are inconsistent.
    """
    if body_points.ndim != 2 or body_points.shape[1] != 3 or len(body_points) < 3:
        raise MountEstimateError("at least 3 marker positions (N x 3) are needed")
    constrained_ = set(constrained_indices or [])
    if constrained_ and constraint_mesh is None:
        raise MountEstimateError("constrained markers need a constraint mesh")

    free_indices_ = [index_ for index_ in range(len(body_points)) if index_ not in constrained_]
    constrained_list_ = sorted(constrained_)

    def _Residuals(parameters_: np.ndarray) -> np.ndarray:
        rotation_ = Rotation.from_rotvec(parameters_[:3]).as_matrix()
        link_points_ = body_points @ rotation_.T + parameters_[3:]
        errors_ = np.zeros(len(body_points))
        if free_indices_:
            _, distances_, _ = trimesh.proximity.closest_point(mesh, link_points_[free_indices_])
            errors_[free_indices_] = distances_ - surface_offset_m
        if constrained_list_:
            _, distances_, _ = trimesh.proximity.closest_point(
                constraint_mesh, link_points_[constrained_list_]
            )
            errors_[constrained_list_] = distances_ - surface_offset_m
        return errors_

    # Start with the constellation centred where its markers should be: on the constraint mesh
    # for the constrained ones, on the whole mesh otherwise
    target_mesh_ = constraint_mesh if constrained_list_ else mesh
    source_points_ = body_points[constrained_list_] if constrained_list_ else body_points
    target_centre_ = np.asarray(target_mesh_.bounds).mean(axis=0)

    best_: MountEstimateResult | None = None
    for yaw_deg_ in yaw_seeds_deg:
        start_rotation_ = Rotation.from_euler("z", yaw_deg_, degrees=True)
        start_translation_ = target_centre_ - start_rotation_.apply(source_points_.mean(axis=0))
        start_ = np.concatenate([start_rotation_.as_rotvec(), start_translation_])

        result_ = least_squares(_Residuals, start_, loss="soft_l1", f_scale=0.005, max_nfev=400)
        errors_ = _Residuals(result_.x)
        rms_ = float(np.sqrt(np.mean(errors_**2)))
        if best_ is None or rms_ < best_.rms_m:
            best_ = MountEstimateResult(
                rotation=Rotation.from_rotvec(result_.x[:3]).as_matrix(),
                translation=result_.x[3:].copy(),
                rms_m=rms_,
                marker_errors_m=errors_,
                start_yaw_deg=yaw_deg_,
            )

    assert best_ is not None
    return best_


def Write_mount_file(
    path: Path, link: str, subject: str, estimate: MountEstimateResult, recording: str
) -> None:
    """Write the mount (pose of the link in the body frame) as YAML for the viewer."""
    rotation_, translation_ = estimate.link_in_body()
    rpy_ = Rotation.from_matrix(rotation_).as_euler("xyz")
    document_ = {
        "link": link,
        "subject": subject,
        "xyz_m": [round(float(value_), 5) for value_ in translation_],
        "rpy_rad": [round(float(value_), 5) for value_ in rpy_],
    }
    header_ = (
        f"# Pose of the G1 link '{link}' in the Vicon body frame of subject '{subject}'.\n"
        "# ESTIMATED by vicon_receiver_tools.mount_estimate (markers fitted to the G1 meshes),\n"
        f"# not measured: RMS fit error {estimate.rms_m * 1000:.1f} mm, recording {recording}.\n"
        "# Adjust it in the viewer (scripts/viz.sh) and save from there if needed.\n"
    )
    path.write_text(header_ + yaml.safe_dump(document_, sort_keys=False), encoding="utf-8")


def Main(argv: list[str] | None = None) -> int:
    """Command-line entry point: estimate the mount and optionally write the mount file."""
    from vicon_receiver_tools.robot_model import Link_mesh, Load_urdf, RobotModelError, Rigid_mesh
    from vicon_receiver_tools.viewer_config import Load_viewer_config

    parser_ = argparse.ArgumentParser(
        prog="python3 -m vicon_receiver_tools.mount_estimate",
        description="Estimate the pose of the robot link in the Vicon body frame from a probe "
        "recording (rough, for visualisation).",
    )
    parser_.add_argument("--recording", type=Path, required=True, help="probe JSON (scripts/probe.sh)")
    parser_.add_argument("--viewer-config", type=Path, required=True, help="config/viewer.yaml")
    parser_.add_argument("--repo-root", type=Path, default=Path.cwd(), help="base of relative paths")
    parser_.add_argument("--write", action="store_true", help="write the mount file of the config")
    arguments_ = parser_.parse_args(argv)

    try:
        config_ = Load_viewer_config(arguments_.viewer_config)
        recording_ = json.loads(arguments_.recording.read_text(encoding="utf-8"))
        names_, body_points_ = Marker_constellation(recording_, config_.robot.subject)

        model_dir_ = arguments_.repo_root / config_.robot.directory
        urdf_ = Load_urdf(model_dir_, config_.robot.urdf, load_meshes=False)
        mesh_ = Rigid_mesh(urdf_, config_.robot.mount_link, model_dir_)

        constraint_ = config_.mount_estimate
        constrained_ = Top_marker_indices(body_points_, constraint_.top_markers_count)
        constraint_mesh_ = Link_mesh(urdf_, constraint_.top_markers_link, config_.robot.mount_link, model_dir_)

        estimate_ = Fit_mount(
            body_points_,
            mesh_,
            surface_offset_m=constraint_.surface_offset_m,
            constrained_indices=constrained_,
            constraint_mesh=constraint_mesh_,
        )
    except (OSError, json.JSONDecodeError, MountEstimateError, RobotModelError, ValueError) as error_:
        print(f"mount estimate failed: {error_}", file=sys.stderr)
        return 1

    rotation_, translation_ = estimate_.link_in_body()
    print(f"{config_.robot.mount_link} in the body frame of {config_.robot.subject}:")
    print(f"  xyz [m]   {np.round(translation_, 4)}")
    print(f"  rpy [deg] {np.round(Rotation.from_matrix(rotation_).as_euler('xyz', degrees=True), 1)}")
    print(f"  RMS error {estimate_.rms_m * 1000:.1f} mm (start yaw {estimate_.start_yaw_deg:.0f} deg)")
    for index_, name_ in enumerate(names_):
        on_link_ = " (on " + constraint_.top_markers_link + ")" if index_ in constrained_ else ""
        print(f"  {name_:28s} error {estimate_.marker_errors_m[index_] * 1000:6.1f} mm{on_link_}")

    if arguments_.write:
        mount_path_ = arguments_.repo_root / config_.robot.mount_file
        Write_mount_file(mount_path_, config_.robot.mount_link, config_.robot.subject, estimate_,
                         str(arguments_.recording))
        print(f"written: {mount_path_}")
    return 0


if __name__ == "__main__":
    sys.exit(Main())
