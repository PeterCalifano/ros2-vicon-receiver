"""Configuration of the live viewer (config/viewer.yaml) and of the robot mount file.

Every entry of the viewer config is required, as for the node parameters: a missing or
malformed entry is reported by name (ConfigError) instead of being replaced by a default. The one
exception is robot.joint_states_topic: without it the robot is drawn at zero joint angles.

Example::

    from pathlib import Path
    from vicon_receiver_tools.viewer_config import Load_viewer_config

    config = Load_viewer_config(Path("config/viewer.yaml"))
    print(config.viewer.port, [camera.name for camera in config.cameras])
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from vicon_receiver_tools.config import ConfigError, Load_yaml


@dataclass(frozen=True)
class ViewerSettings:
    """Web server, data source and drawing settings of the viewer."""

    host: str
    port: int
    topic: str
    update_rate_hz: float
    grid_size_m: float
    trail_length: int
    trail_update_rate_hz: float
    marker_radius_m: float
    good_camera_count: int


@dataclass(frozen=True)
class RobotSettings:
    """Robot model and where it is attached."""

    subject: str
    source: str
    directory: str
    urdf: str
    mount_link: str
    mount_file: str
    # sensor_msgs/JointState topic giving the joint angles of the model; None: zero angles
    joint_states_topic: str | None = None


@dataclass(frozen=True)
class PinholeCameraSettings:
    """Pinhole camera drawn as a frustum at an optical frame of the URDF."""

    name: str
    frame: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def vertical_fov_rad(self) -> float:
        """Vertical field of view [rad]."""
        return 2.0 * math.atan(0.5 * self.height / self.fy)

    def aspect(self) -> float:
        """Aspect ratio (width / height) of the image, in metric units of the image plane."""
        return (self.width / self.fx) / (self.height / self.fy)


@dataclass(frozen=True)
class LeftRightMarkerPairSettings:
    """Two markers of the subject placed on the left and on the right of the robot (set-up specific)."""

    left: str
    right: str


@dataclass(frozen=True)
class MountEstimateSettings:
    """Set-up facts used by the mount estimate (scripts/estimate_mount.py).

    Attributes:
        surface_offset_m: Distance of a marker centre from the mesh surface [m].
        lateral_marker_pairs: Marker pairs placed left and right on the robot.
        marker_links: Marker name -> link whose mesh the marker sits on; markers not listed sit on
            the mount link or a link fixed to it.
    """

    surface_offset_m: float
    lateral_marker_pairs: list[LeftRightMarkerPairSettings]
    marker_links: dict[str, str]


@dataclass(frozen=True)
class ViewerConfig:
    """Whole content of config/viewer.yaml."""

    viewer: ViewerSettings
    robot: RobotSettings
    cameras: list[PinholeCameraSettings]
    frustum_scale_m: float
    mount_estimate: MountEstimateSettings


@dataclass(frozen=True)
class LinkPoseInBodyFrame:
    """Pose of a robot link in the Vicon body frame (xyz [m], roll/pitch/yaw [rad], xyz order)."""

    xyz_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]

    def matrix(self) -> np.ndarray:
        """Homogeneous 4x4 transform, link frame -> body frame."""
        transform_ = np.eye(4)
        transform_[:3, :3] = Rotation.from_euler("xyz", self.rpy_rad).as_matrix()
        transform_[:3, 3] = self.xyz_m
        return transform_


IDENTITY_MOUNT: LinkPoseInBodyFrame = LinkPoseInBodyFrame(xyz_m=(0.0, 0.0, 0.0), rpy_rad=(0.0, 0.0, 0.0))


def _Require(section: dict[str, Any], key: str, kind: type, where: str) -> Any:
    """Return section[key], checking that it exists and has the given type."""
    if not isinstance(section, dict) or key not in section:
        raise ConfigError(f"{where}.{key} not found")
    value_ = section[key]
    # YAML integers are valid floats, but booleans are not numbers here
    if kind is float and isinstance(value_, int) and not isinstance(value_, bool):
        value_ = float(value_)
    if not isinstance(value_, kind) or (kind is not bool and isinstance(value_, bool)):
        raise ConfigError(f"{where}.{key} must be a {kind.__name__}, got {value_!r}")
    return value_


def _Optional_text(section: dict[str, Any], key: str, where: str) -> str | None:
    """Return section[key] if given (a non-empty string), None if absent or null."""
    if section.get(key) is None:
        return None
    value_ = _Require(section, key, str, where)
    if not value_.strip():
        raise ConfigError(f"{where}.{key} must not be empty (leave it out to disable it)")
    return value_


def _Require_positive(value: float, where: str) -> float:
    """Return value if it is > 0."""
    if value <= 0:
        raise ConfigError(f"{where} must be > 0, got {value}")
    return value


def Load_viewer_config(path: Path) -> ViewerConfig:
    """Load and validate config/viewer.yaml.

    Raises:
        ConfigError: If the file is invalid or an entry is missing or malformed.
    """
    document_ = Load_yaml(path)
    if not isinstance(document_, dict):
        raise ConfigError(f"{path} is not a mapping")

    viewer_section_ = _Require(document_, "viewer", dict, str(path))
    viewer_ = ViewerSettings(
        host=_Require(viewer_section_, "host", str, "viewer"),
        port=_Require(viewer_section_, "port", int, "viewer"),
        topic=_Require(viewer_section_, "topic", str, "viewer"),
        update_rate_hz=_Require_positive(
            _Require(viewer_section_, "update_rate_hz", float, "viewer"), "viewer.update_rate_hz"
        ),
        grid_size_m=_Require_positive(
            _Require(viewer_section_, "grid_size_m", float, "viewer"), "viewer.grid_size_m"
        ),
        trail_length=int(
            _Require_positive(_Require(viewer_section_, "trail_length", int, "viewer"), "viewer.trail_length")
        ),
        trail_update_rate_hz=_Require_positive(
            _Require(viewer_section_, "trail_update_rate_hz", float, "viewer"),
            "viewer.trail_update_rate_hz",
        ),
        marker_radius_m=_Require_positive(
            _Require(viewer_section_, "marker_radius_m", float, "viewer"), "viewer.marker_radius_m"
        ),
        good_camera_count=int(
            _Require_positive(
                _Require(viewer_section_, "good_camera_count", int, "viewer"), "viewer.good_camera_count"
            )
        ),
    )

    robot_section_ = _Require(document_, "robot", dict, str(path))
    robot_ = RobotSettings(
        subject=_Require(robot_section_, "subject", str, "robot"),
        source=_Require(robot_section_, "source", str, "robot"),
        directory=_Require(robot_section_, "directory", str, "robot"),
        urdf=_Require(robot_section_, "urdf", str, "robot"),
        mount_link=_Require(robot_section_, "mount_link", str, "robot"),
        mount_file=_Require(robot_section_, "mount_file", str, "robot"),
        joint_states_topic=_Optional_text(robot_section_, "joint_states_topic", "robot"),
    )

    cameras_: list[PinholeCameraSettings] = []
    for index_, camera_section_ in enumerate(_Require(document_, "cameras", list, str(path))):
        where_ = f"cameras[{index_}]"
        cameras_.append(
            PinholeCameraSettings(
                name=_Require(camera_section_, "name", str, where_),
                frame=_Require(camera_section_, "frame", str, where_),
                width=int(_Require_positive(_Require(camera_section_, "width", int, where_), f"{where_}.width")),
                height=int(_Require_positive(_Require(camera_section_, "height", int, where_), f"{where_}.height")),
                fx=_Require_positive(_Require(camera_section_, "fx", float, where_), f"{where_}.fx"),
                fy=_Require_positive(_Require(camera_section_, "fy", float, where_), f"{where_}.fy"),
                cx=_Require(camera_section_, "cx", float, where_),
                cy=_Require(camera_section_, "cy", float, where_),
            )
        )

    estimate_section_ = _Require(document_, "mount_estimate", dict, str(path))
    surface_offset_m_ = _Require(estimate_section_, "surface_offset_m", float, "mount_estimate")
    if surface_offset_m_ < 0.0:
        raise ConfigError(f"mount_estimate.surface_offset_m must be >= 0, got {surface_offset_m_}")
    pairs_: list[LeftRightMarkerPairSettings] = []
    for index_, pair_section_ in enumerate(_Require(estimate_section_, "lateral_marker_pairs", list, "mount_estimate")):
        where_ = f"mount_estimate.lateral_marker_pairs[{index_}]"
        pair_ = LeftRightMarkerPairSettings(
            left=_Require(pair_section_, "left", str, where_), right=_Require(pair_section_, "right", str, where_)
        )
        if pair_.left == pair_.right:
            raise ConfigError(f"{where_}: left and right are the same marker '{pair_.left}'")
        pairs_.append(pair_)
    if not pairs_:
        raise ConfigError("mount_estimate.lateral_marker_pairs needs at least one pair")
    marker_links_ = _Require(estimate_section_, "marker_links", dict, "mount_estimate")
    for marker_, link_ in marker_links_.items():
        if not isinstance(marker_, str) or not isinstance(link_, str):
            raise ConfigError(f"mount_estimate.marker_links: '{marker_}: {link_}' must map a marker name to a link name")
    mount_estimate_ = MountEstimateSettings(
        surface_offset_m=surface_offset_m_, lateral_marker_pairs=pairs_, marker_links=dict(marker_links_)
    )

    return ViewerConfig(
        viewer=viewer_,
        robot=robot_,
        cameras=cameras_,
        frustum_scale_m=_Require_positive(
            _Require(document_, "frustum_scale_m", float, str(path)), "frustum_scale_m"
        ),
        mount_estimate=mount_estimate_,
    )


def Load_mount(path: Path) -> LinkPoseInBodyFrame | None:
    """Load the mount file, or return None if it does not exist yet.

    Raises:
        ConfigError: If the file exists but is malformed.
    """
    if not path.is_file():
        return None
    document_ = Load_yaml(path)
    xyz_ = _Require(document_, "xyz_m", list, str(path))
    rpy_ = _Require(document_, "rpy_rad", list, str(path))
    if len(xyz_) != 3 or len(rpy_) != 3:
        raise ConfigError(f"{path}: xyz_m and rpy_rad need 3 values each")
    return LinkPoseInBodyFrame(
        xyz_m=tuple(float(value_) for value_ in xyz_),  # type: ignore[arg-type]
        rpy_rad=tuple(float(value_) for value_ in rpy_),  # type: ignore[arg-type]
    )


def Save_mount(path: Path, mount: LinkPoseInBodyFrame, link: str, subject: str, note: str) -> None:
    """Write the mount file, with a comment saying where the values come from."""
    document_ = {
        "link": link,
        "subject": subject,
        "xyz_m": [round(value_, 5) for value_ in mount.xyz_m],
        "rpy_rad": [round(value_, 5) for value_ in mount.rpy_rad],
    }
    header_ = (
        f"# Pose of the G1 link '{link}' in the Vicon body frame of subject '{subject}'.\n"
        f"# {note}\n"
    )
    path.write_text(header_ + yaml.safe_dump(document_, sort_keys=False), encoding="utf-8")
