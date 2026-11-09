"""Live viewer of the Vicon stream in a web browser (viser).

Subscribes to the ViconFrame topic of the vicon_client node and shows, in the Vicon frame:

- the axes of every subject segment, hidden when occluded, and a trail of recent positions;
- the labeled markers, coloured by the number of cameras that saw them (green: enough cameras,
  yellow: fewer); occluded markers are not drawn (Vicon sends no position for them);
- the G1 Unitree model attached to its Vicon subject through the mount transform, and the camera
  frustums at the optical frames of the model;
- a telemetry panel (frame number, rate, latency, dropped frames, quality, occlusion);
- sliders to adjust the mount, and a button to save it to the mount file.

Rendering runs in the browser, so the machine running the node only sends small updates
(update_rate_hz of the viewer config). Start it with scripts/start_visualizer.sh.

Needs viser, yourdfpy and trimesh (in the conda environment of environment.yml) and the
ROS 2 environment of the workspace (vicon_receiver_msgs, rclpy).
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from vicon_receiver_tools.viewer_config import (
    IDENTITY_MOUNT,
    Is_unit_quaternion,
    LinkPoseInBodyFrame,
    Load_mount,
    Load_viewer_config,
    Save_mount,
    ViewerConfig,
)

if TYPE_CHECKING:
    import viser
    import yourdfpy
    from sensor_msgs.msg import JointState
    from viser.extras import ViserUrdf

    from vicon_receiver_msgs.msg import SegmentState, ViconFrame

# Marker colours (RGB): enough cameras, few cameras
COLOR_GOOD: tuple[int, int, int] = (40, 180, 60)
COLOR_FEW: tuple[int, int, int] = (235, 180, 30)

# Colour of the trails
COLOR_TRAIL: tuple[int, int, int] = (60, 120, 230)

# Where a browser camera is put to look at the robot, in the robot's heading frame (x forward,
# y left, z up) [m]: front-left, so that the camera frustums show which way they point
CAMERA_OFFSET_M: tuple[float, float, float] = (1.6, 1.6, 0.8)

# Line width of the camera frustums [pixels]
FRUSTUM_LINE_WIDTH_PX: float = 2.0

# Colours of the camera frustums, in the order of the cameras in the config (cycled)
FRUSTUM_COLORS: tuple[tuple[int, int, int], ...] = ((30, 90, 220), (230, 130, 20), (40, 160, 70))

# Period of the telemetry panel updates [s] (text does not need the scene rate)
TELEMETRY_PERIOD_S: float = 0.5

# Receive times used for the displayed rate [s]: the rate falls to 0 this long after the last frame
RATE_WINDOW_S: float = 1.0

_LOGGER: logging.Logger = logging.getLogger(__name__)


# -------------------------------------------------------------------------------------------------
# Plain data views of a ViconFrame (independent of ROS, for the scene and the tests)
# -------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class ViewerSegment:
    """One segment: pose in the Vicon frame and occlusion."""

    name: str
    position: np.ndarray
    wxyz: np.ndarray
    occluded: bool


@dataclass(frozen=True)
class ViewerMarker:
    """One labeled marker: position in the Vicon frame and visibility."""

    name: str
    position: np.ndarray
    camera_count: int
    occluded: bool


@dataclass(frozen=True)
class ViewerSubject:
    """One subject of a frame."""

    name: str
    ros_name: str
    quality: float | None
    segments: list[ViewerSegment]
    markers: list[ViewerMarker]


@dataclass(frozen=True)
class ViewerFrame:
    """What the viewer needs from one ViconFrame."""

    frame_number: int
    frames_dropped: int
    latency_s: float
    any_occluded: bool
    subjects: list[ViewerSubject]


def Segment_key(subject_ros_name: str, segment_name: str) -> str:
    """Key of a segment in the scene and in the trails."""
    return f"{subject_ros_name}/{segment_name}"


def Viewer_segment_from_message(segment: SegmentState) -> ViewerSegment:
    """Convert a SegmentState; a segment without a valid rotation counts as occluded."""
    # Vicon/ROS order x, y, z, w -> viser order w, x, y, z
    wxyz_ = np.array(
        [segment.orientation.w, segment.orientation.x, segment.orientation.y, segment.orientation.z]
    )
    occluded_ = bool(
        segment.translation_occluded or segment.rotation_occluded
    ) or not Is_unit_quaternion(wxyz_)
    return ViewerSegment(
        name=segment.name,
        position=np.array([segment.position.x, segment.position.y, segment.position.z]),
        wxyz=wxyz_,
        occluded=occluded_,
    )


def Viewer_frame_from_message(message: ViconFrame) -> ViewerFrame:
    """Convert a vicon_receiver_msgs/ViconFrame (or an object with the same fields)."""
    subjects_: list[ViewerSubject] = []
    for subject_ in message.subjects:
        segments_ = [Viewer_segment_from_message(segment_) for segment_ in subject_.segments]
        markers_ = [
            ViewerMarker(
                name=marker_.name,
                position=np.array([marker_.position.x, marker_.position.y, marker_.position.z]),
                camera_count=len(marker_.camera_ids),
                occluded=bool(marker_.occluded),
            )
            for marker_ in subject_.markers
        ]
        subjects_.append(
            ViewerSubject(
                name=subject_.name,
                ros_name=subject_.ros_name,
                quality=float(subject_.quality) if subject_.quality_valid else None,
                segments=segments_,
                markers=markers_,
            )
        )

    return ViewerFrame(
        frame_number=int(message.frame_number),
        frames_dropped=int(message.frames_dropped),
        latency_s=float(message.latency_total_s),
        any_occluded=bool(message.any_occluded),
        subjects=subjects_,
    )


# -------------------------------------------------------------------------------------------------
# Pure helpers
# -------------------------------------------------------------------------------------------------
def Marker_colors(markers: list[ViewerMarker], good_camera_count: int) -> np.ndarray:
    """RGB colour (N x 3, uint8) of each visible marker from the number of cameras that saw it."""
    colors_ = np.zeros((len(markers), 3), dtype=np.uint8)
    for index_, marker_ in enumerate(markers):
        colors_[index_] = COLOR_GOOD if marker_.camera_count >= good_camera_count else COLOR_FEW
    return colors_


def Pose_matrix(position: np.ndarray, wxyz: np.ndarray) -> np.ndarray:
    """Homogeneous 4x4 transform from a position and a w, x, y, z quaternion."""
    transform_ = np.eye(4)
    xyzw_ = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]])
    transform_[:3, :3] = Rotation.from_quat(xyzw_).as_matrix()
    transform_[:3, 3] = position
    return transform_


def Matrix_to_wxyz_position(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Quaternion (w, x, y, z) and position of a homogeneous 4x4 transform."""
    xyzw_ = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return np.array([xyzw_[3], xyzw_[0], xyzw_[1], xyzw_[2]]), transform[:3, 3].copy()


def Camera_position(robot_position: np.ndarray, heading_rad: float) -> np.ndarray:
    """Where a browser camera looks from: CAMERA_OFFSET_M turned by the robot heading."""
    cos_, sin_ = np.cos(heading_rad), np.sin(heading_rad)
    offset_x_, offset_y_, offset_z_ = CAMERA_OFFSET_M
    offset_ = np.array(
        [cos_ * offset_x_ - sin_ * offset_y_, sin_ * offset_x_ + cos_ * offset_y_, offset_z_]
    )
    return np.asarray(robot_position, dtype=float) + offset_


def Trail_segments(points: np.ndarray) -> np.ndarray:
    """Line segments (N-1 x 2 x 3) joining consecutive trail points (N x 3)."""
    if len(points) < 2:
        return np.zeros((0, 2, 3))
    return np.stack([points[:-1], points[1:]], axis=1)


def Telemetry_markdown(frame: ViewerFrame, rate_hz: float) -> str:
    """Text of the telemetry panel for the latest frame."""
    lines_ = [
        f"**Frame** {frame.frame_number}  ",
        f"**Rate** {rate_hz:.1f} Hz (local clock)  ",
        f"**Latency** {frame.latency_s * 1000.0:.1f} ms  ",
        f"**Dropped** {frame.frames_dropped}  ",
    ]
    for subject_ in frame.subjects:
        quality_ = "n/a" if subject_.quality is None else f"{subject_.quality:.2f}"
        occluded_ = any(segment_.occluded for segment_ in subject_.segments)
        visible_ = [marker_ for marker_ in subject_.markers if not marker_.occluded]
        cameras_ = [marker_.camera_count for marker_ in visible_ if marker_.camera_count > 0]
        camera_range_ = f"{min(cameras_)}-{max(cameras_)}" if cameras_ else "n/a"
        state_ = "**OCCLUDED**" if occluded_ else "tracked"
        lines_.append(
            f"\n**{subject_.name}**: {state_}, quality {quality_}, markers "
            f"{len(visible_)}/{len(subject_.markers)}, cameras {camera_range_}  "
        )
    return "\n".join(lines_)


def Joint_configuration(joint_names: tuple[str, ...], positions: dict[str, float]) -> np.ndarray:
    """Joint angles in the order of joint_names; joints without a received value are 0."""
    return np.array([positions.get(name_, 0.0) for name_ in joint_names], dtype=float)


def Joint_states_markdown(
    joint_names: tuple[str, ...],
    positions: dict[str, float],
    age_s: float | None,
    topic: str | None,
) -> str:
    """Telemetry line about the joint angles used for the robot model."""
    if topic is None:
        return "**Joints** zero angles (no joint_states_topic)  "
    if age_s is None:
        return f"**Joints** zero angles (nothing on {topic} yet)  "
    matched_ = sum(1 for name_ in joint_names if name_ in positions)
    return f"**Joints** {matched_}/{len(joint_names)} from {topic}, last {age_s:.1f} s ago  "


# -------------------------------------------------------------------------------------------------
# Frame buffer (written by the ROS callback, read by the scene loop)
# -------------------------------------------------------------------------------------------------
@dataclass
class LatestFrameBuffer:
    """Latest frame, trails of the subjects and receive rate, shared between two threads.

    The ROS callback (100 Hz) only stores the serialized message and its receive time; the
    message is decoded, converted into a ViewerFrame and added to the trails once per scene update
    (latest()). Decoding every message in Python would cost several percent of a core.

    Attributes:
        trail_length: Number of positions kept per subject (sampled at the scene update rate).
        deserialize: Turns a serialized message into a message object (rclpy deserialize).
    """

    trail_length: int
    deserialize: Callable[[bytes], ViconFrame] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _pending: bytes | ViconFrame | None = field(default=None, repr=False)
    _latest_view: ViewerFrame | None = field(default=None, repr=False)
    _trails: dict[str, deque] = field(default_factory=dict, repr=False)
    _receive_times: deque = field(default_factory=lambda: deque(maxlen=200), repr=False)

    def put_serialized(self, data: bytes, receive_time_s: float | None = None) -> None:
        """Store a serialized ViconFrame (ROS callback with raw=True): no decoding here."""
        self._store(data, receive_time_s)

    def put_message(self, message: ViconFrame, receive_time_s: float | None = None) -> None:
        """Store an already decoded ViconFrame (or an object with the same fields)."""
        self._store(message, receive_time_s)

    def latest(self) -> ViewerFrame | None:
        """Most recent frame; a new message is decoded and added to the trails here."""
        with self._lock:
            if self._pending is not None:
                message_ = self._pending
                if isinstance(message_, bytes):
                    if self.deserialize is None:
                        raise ValueError(
                            "LatestFrameBuffer: serialized message without deserialize"
                        )
                    message_ = self.deserialize(message_)
                self._pending = None
                self._latest_view = Viewer_frame_from_message(message_)
                self._extend_trails(self._latest_view)
            return self._latest_view

    def trail(self, key: str) -> np.ndarray:
        """Trail positions (N x 3) of a "<subject ros name>/<segment>" key."""
        with self._lock:
            points_ = list(self._trails.get(key, ()))
        return np.array(points_, dtype=float) if points_ else np.zeros((0, 3))

    def rate_hz(self, now_s: float | None = None) -> float:
        """Frames per second over the last RATE_WINDOW_S (local clock); 0 once the stream stops.

        Args:
            now_s: Current time on the clock of the receive times (default: time.monotonic()).
        """
        now_ = time.monotonic() if now_s is None else now_s
        with self._lock:
            times_ = [time_ for time_ in self._receive_times if now_ - time_ <= RATE_WINDOW_S]
        if len(times_) < 2 or times_[-1] <= times_[0]:
            return 0.0
        return (len(times_) - 1) / (times_[-1] - times_[0])

    def _store(self, item: bytes | ViconFrame, receive_time_s: float | None) -> None:
        """Keep only the newest item; frames in between are not drawn."""
        now_ = time.monotonic() if receive_time_s is None else receive_time_s
        with self._lock:
            self._pending = item
            self._receive_times.append(now_)

    def _extend_trails(self, frame: ViewerFrame) -> None:
        """Append the visible segment positions of a frame to the trails (lock held)."""
        for subject_ in frame.subjects:
            for segment_ in subject_.segments:
                if segment_.occluded:
                    continue
                key_ = Segment_key(subject_.ros_name, segment_.name)
                trail_ = self._trails.setdefault(key_, deque(maxlen=self.trail_length))
                trail_.append(segment_.position.copy())


@dataclass
class LatestJointStates:
    """Latest angle of every joint (keep-last), shared between the ROS callback and the scene loop.

    The callback only stores the newest serialized sensor_msgs/JointState; it is decoded once per
    scene update (positions()) and merged into the angles kept so far, so a joint keeps its last
    value until a message brings a new one. Messages arriving between two scene updates are not
    decoded: a publisher sending only some joints may be missed for one update.

    Attributes:
        deserialize: Turns a serialized message into a message object (rclpy deserialize).
    """

    deserialize: Callable[[bytes], JointState] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _pending: bytes | JointState | None = field(default=None, repr=False)
    _positions: dict[str, float] = field(default_factory=dict, repr=False)
    _last_receive_s: float | None = field(default=None, repr=False)

    def put_serialized(self, data: bytes, receive_time_s: float | None = None) -> None:
        """Store a serialized JointState (ROS callback with raw=True): no decoding here."""
        self._store(data, receive_time_s)

    def put_message(self, message: JointState, receive_time_s: float | None = None) -> None:
        """Store an already decoded JointState (or an object with name and position)."""
        self._store(message, receive_time_s)

    def positions(self) -> dict[str, float]:
        """Angles kept so far [rad] by joint name; a new message is decoded and merged here."""
        with self._lock:
            if self._pending is not None:
                message_ = self._pending
                if isinstance(message_, bytes):
                    if self.deserialize is None:
                        raise ValueError(
                            "LatestJointStates: serialized message without deserialize"
                        )
                    message_ = self.deserialize(message_)
                self._pending = None
                # strict=False: JointState allows an empty position array
                for name_, position_ in zip(message_.name, message_.position, strict=False):
                    if np.isfinite(position_):
                        self._positions[name_] = float(position_)
            return dict(self._positions)

    def age_s(self, now_s: float | None = None) -> float | None:
        """Time since the last message [s] (local clock), None before the first one."""
        with self._lock:
            last_ = self._last_receive_s
        if last_ is None:
            return None
        return (time.monotonic() if now_s is None else now_s) - last_

    def _store(self, item: bytes | JointState, receive_time_s: float | None) -> None:
        """Keep only the newest message until the next scene update."""
        with self._lock:
            self._pending = item
            self._last_receive_s = time.monotonic() if receive_time_s is None else receive_time_s


# -------------------------------------------------------------------------------------------------
# Scene
# -------------------------------------------------------------------------------------------------
class ViserLiveViewer:
    """Scene and GUI of the viewer on a viser server."""

    def __init__(self, config: ViewerConfig, repo_root: Path, server: viser.ViserServer) -> None:
        """Build the static scene, load the robot model (if fetched) and the GUI."""
        self._config = config
        self._repo_root = repo_root
        self._server = server
        self._segment_handles: dict[str, viser.FrameHandle] = {}
        self._trail_handles: dict[str, viser.LineSegmentsHandle] = {}
        self._marker_handles: dict[str, viser.PointCloudHandle] = {}
        self._last_telemetry_s = 0.0
        self._last_trail_s = 0.0

        grid_size_ = config.viewer.grid_size_m
        server.scene.add_grid(
            "/grid", width=grid_size_, height=grid_size_, cell_size=0.5, section_size=1.0
        )
        server.scene.add_frame("/vicon", axes_length=0.3, axes_radius=0.01)

        mount_path_ = repo_root / config.robot.mount_file
        loaded_mount_ = Load_mount(mount_path_)
        self._mount: LinkPoseInBodyFrame = (
            loaded_mount_ if loaded_mount_ is not None else IDENTITY_MOUNT
        )
        self._mount_note = (
            f"mount from {config.robot.mount_file}"
            if loaded_mount_ is not None
            else f"no {config.robot.mount_file}: identity mount (run scripts/estimate_mount.py)"
        )

        self._robot_frame: viser.FrameHandle | None = None
        # Last robot position: browser cameras are pointed at it (the robot may be far from the
        # Vicon origin, where a browser camera starts)
        self._robot_position: np.ndarray | None = None
        self._robot_heading_rad = 0.0
        server.on_client_connect(self._point_camera_at_robot)
        self._camera_handles: dict[str, viser.CameraFrustumHandle] = {}
        self._root_in_mount = np.eye(4)
        self._cameras_in_mount: dict[str, np.ndarray] = {}
        # Model and its joint angles (zero until joint states arrive)
        self._urdf: yourdfpy.URDF | None = None
        self._viser_urdf: ViserUrdf | None = None
        self._joint_names: tuple[str, ...] = ()
        self._configuration = np.zeros(0)
        self._joints_line = Joint_states_markdown((), {}, None, config.robot.joint_states_topic)
        # Pose of the mount link in the world at the last valid subject pose
        self._mount_in_world: np.ndarray | None = None
        self._robot_note = self._load_robot()

        self._build_gui()

    # --- construction ------------------------------------------------------------------------
    def _load_robot(self) -> str:
        """Load the robot model and the camera frustums; return a status text."""
        from viser.extras import ViserUrdf

        from vicon_receiver_tools.robot_model import Link_pose, Load_urdf, RobotModelError

        robot_ = self._config.robot
        model_dir_ = self._repo_root / robot_.directory
        try:
            urdf_ = Load_urdf(model_dir_, robot_.urdf, load_meshes=True)
            # Check the links once (zero angles); _apply_configuration() recomputes the poses
            Link_pose(urdf_, urdf_.base_link, robot_.mount_link)
            for camera_ in self._config.cameras:
                Link_pose(urdf_, camera_.frame, robot_.mount_link)
        except RobotModelError as error_:
            return f"robot model not shown: {error_}"

        # The URDF nodes hang below /robot, whose pose is set at every update
        self._robot_frame = self._server.scene.add_frame("/robot", show_axes=False)
        self._urdf = urdf_
        self._viser_urdf = ViserUrdf(self._server, urdf_, root_node_name="/robot")
        self._joint_names = tuple(self._viser_urdf.get_actuated_joint_names())
        # ViserUrdf creates the link frames at identity and places them only in update_cfg():
        # without this call every link is drawn at the root. Zero angles until joint states arrive.
        self._apply_configuration(np.zeros(len(self._joint_names)))

        for index_, camera_ in enumerate(self._config.cameras):
            self._camera_handles[camera_.name] = self._server.scene.add_camera_frustum(
                f"/cameras/{camera_.name}",
                fov=camera_.vertical_fov_rad(),
                aspect=camera_.aspect(),
                scale=self._config.frustum_scale_m,
                color=FRUSTUM_COLORS[index_ % len(FRUSTUM_COLORS)],
                # viser's default is 2 cm thick bars in world units: hides the robot
                thickness=FRUSTUM_LINE_WIDTH_PX,
                thickness_units="screen",
            )

        # Nothing is shown before the first valid pose of the subject
        self._set_robot_visible(False)
        return f"robot model {robot_.urdf} attached to {robot_.subject} at {robot_.mount_link}"

    def _build_gui(self) -> None:
        """Telemetry panel, mount sliders and save button."""
        gui_ = self._server.gui
        self._telemetry = gui_.add_markdown("Waiting for ViconFrame ...")
        gui_.add_markdown(f"_{self._robot_note}_  \n_{self._mount_note}_")

        with gui_.add_folder(
            f"Mount: {self._config.robot.mount_link} in Vicon body", expand_by_default=False
        ):
            xyz_ = self._mount.xyz_m
            rpy_deg_ = np.degrees(self._mount.rpy_rad)
            self._sliders = {
                "x": gui_.add_slider("x [m]", -0.6, 0.6, 0.001, float(xyz_[0])),
                "y": gui_.add_slider("y [m]", -0.6, 0.6, 0.001, float(xyz_[1])),
                "z": gui_.add_slider("z [m]", -0.6, 0.6, 0.001, float(xyz_[2])),
                "roll": gui_.add_slider("roll [deg]", -180.0, 180.0, 0.5, float(rpy_deg_[0])),
                "pitch": gui_.add_slider("pitch [deg]", -180.0, 180.0, 0.5, float(rpy_deg_[1])),
                "yaw": gui_.add_slider("yaw [deg]", -180.0, 180.0, 0.5, float(rpy_deg_[2])),
            }
            for slider_ in self._sliders.values():
                slider_.on_update(lambda _event: self._read_mount_sliders())
            save_button_ = gui_.add_button("Save mount")
            save_button_.on_click(lambda _event: self._save_mount())

    # --- mount -------------------------------------------------------------------------------
    def _read_mount_sliders(self) -> None:
        """Take the mount from the sliders."""
        values_ = {name_: float(slider_.value) for name_, slider_ in self._sliders.items()}
        self._mount = LinkPoseInBodyFrame(
            xyz_m=(values_["x"], values_["y"], values_["z"]),
            rpy_rad=tuple(np.radians([values_["roll"], values_["pitch"], values_["yaw"]])),  # type: ignore[arg-type]
        )

    def _save_mount(self) -> None:
        """Write the current mount to the mount file."""
        path_ = self._repo_root / self._config.robot.mount_file
        Save_mount(
            path_,
            self._mount,
            self._config.robot.mount_link,
            self._config.robot.subject,
            "Adjusted in the live viewer (scripts/start_visualizer.sh).",
        )
        _LOGGER.info("mount saved to %s", path_)

    # --- updates -----------------------------------------------------------------------------
    def update(self, frame: ViewerFrame, buffer: LatestFrameBuffer) -> None:
        """Move everything to the latest frame."""
        scene_ = self._server.scene
        now_ = time.monotonic()
        redraw_trails_ = now_ - self._last_trail_s >= 1.0 / self._config.viewer.trail_update_rate_hz
        if redraw_trails_:
            self._last_trail_s = now_

        for subject_ in frame.subjects:
            for segment_ in subject_.segments:
                key_ = Segment_key(subject_.ros_name, segment_.name)
                handle_ = self._segment_handles.get(key_)
                if handle_ is None:
                    handle_ = scene_.add_frame(
                        f"/subjects/{key_}", axes_length=0.15, axes_radius=0.006
                    )
                    self._segment_handles[key_] = handle_
                handle_.visible = not segment_.occluded
                if not segment_.occluded:
                    handle_.position = segment_.position
                    handle_.wxyz = segment_.wxyz

                segments_ = (
                    Trail_segments(buffer.trail(key_)) if redraw_trails_ else np.zeros((0, 2, 3))
                )
                if len(segments_) > 0:
                    self._trail_handles[key_] = scene_.add_line_segments(
                        f"/trails/{key_}", points=segments_, colors=COLOR_TRAIL, thickness=0.004
                    )

            self._update_markers(subject_)
            if subject_.name == self._config.robot.subject:
                self._update_robot(subject_)

        self._hide_absent(frame)
        self.update_telemetry(frame, buffer)

    def _hide_absent(self, frame: ViewerFrame) -> None:
        """Hide what belongs to subjects or segments missing from the frame.

        They were removed from the Tracker: nothing stays frozen at its last pose.
        """
        subjects_ = {subject_.ros_name for subject_ in frame.subjects}
        segments_ = {
            Segment_key(subject_.ros_name, segment_.name)
            for subject_ in frame.subjects
            for segment_ in subject_.segments
        }
        for handles_ in (self._segment_handles, self._trail_handles):
            for key_, handle_ in handles_.items():
                if key_ not in segments_:
                    handle_.visible = False
        for ros_name_, handle_ in self._marker_handles.items():
            if ros_name_ not in subjects_:
                handle_.visible = False
        robot_present_ = any(
            subject_.name == self._config.robot.subject and subject_.segments
            for subject_ in frame.subjects
        )
        if not robot_present_ and self._robot_frame is not None:
            self._set_robot_visible(False)

    def _set_robot_visible(self, visible: bool) -> None:
        """Show or hide the robot model and the camera frustums together."""
        if self._robot_frame is None:
            return
        self._robot_frame.visible = visible
        for handle_ in self._camera_handles.values():
            handle_.visible = visible

    def update_telemetry(self, frame: ViewerFrame, buffer: LatestFrameBuffer) -> None:
        """Refresh the telemetry panel (at most every TELEMETRY_PERIOD_S), new frames or not."""
        now_ = time.monotonic()
        if now_ - self._last_telemetry_s >= TELEMETRY_PERIOD_S:
            self._telemetry.content = (
                Telemetry_markdown(frame, buffer.rate_hz()) + "\n\n" + self._joints_line
            )
            self._last_telemetry_s = now_

    def update_joints(self, joint_states: LatestJointStates) -> None:
        """Take the latest joint angles: the model, its root and the frustums follow them."""
        if self._viser_urdf is None:
            return
        positions_ = joint_states.positions()
        self._joints_line = Joint_states_markdown(
            self._joint_names,
            positions_,
            joint_states.age_s(),
            self._config.robot.joint_states_topic,
        )
        configuration_ = Joint_configuration(self._joint_names, positions_)
        if np.array_equal(configuration_, self._configuration):
            return
        self._apply_configuration(configuration_)
        # The subject tracks the mount link: with new angles the root and the cameras move around it
        self._place_robot()

    def _apply_configuration(self, configuration: np.ndarray) -> None:
        """Set the joint angles of the model and the root and camera poses in the mount frame."""
        if self._viser_urdf is None or self._urdf is None:
            return
        from vicon_receiver_tools.robot_model import Link_pose

        robot_ = self._config.robot
        # Also updates self._urdf, which ViserUrdf shares: Link_pose() then uses these angles
        self._viser_urdf.update_cfg(configuration)
        self._configuration = configuration.copy()
        self._root_in_mount = Link_pose(self._urdf, self._urdf.base_link, robot_.mount_link)
        for camera_ in self._config.cameras:
            self._cameras_in_mount[camera_.name] = Link_pose(
                self._urdf, camera_.frame, robot_.mount_link
            )

    def _update_markers(self, subject: ViewerSubject) -> None:
        """Markers of one subject as a point cloud, without the occluded ones (no position)."""
        visible_ = [marker_ for marker_ in subject.markers if not marker_.occluded]
        handle_ = self._marker_handles.get(subject.ros_name)
        if not visible_:
            # Hide the markers of the previous frame instead of leaving them frozen in the scene
            if handle_ is not None:
                handle_.visible = False
            return
        points_ = np.array([marker_.position for marker_ in visible_], dtype=np.float32)
        colors_ = Marker_colors(visible_, self._config.viewer.good_camera_count)
        self._marker_handles[subject.ros_name] = self._server.scene.add_point_cloud(
            f"/markers/{subject.ros_name}",
            points=points_,
            colors=colors_,
            point_size=2.0 * self._config.viewer.marker_radius_m,
            point_shape="circle",
        )

    def _point_camera_at_robot(self, client: viser.ClientHandle) -> None:
        """Point a browser camera at the robot (once it has been seen), from CAMERA_OFFSET_M."""
        if self._robot_position is None:
            return
        client.camera.position = Camera_position(self._robot_position, self._robot_heading_rad)
        client.camera.look_at = self._robot_position
        client.camera.up_direction = (0.0, 0.0, 1.0)

    def _update_robot(self, subject: ViewerSubject) -> None:
        """Place the robot model and the frustums at the pose of the subject."""
        if self._robot_frame is None or not subject.segments:
            return
        segment_ = subject.segments[0]
        self._set_robot_visible(not segment_.occluded)
        if segment_.occluded:
            return

        first_pose_ = self._mount_in_world is None
        self._mount_in_world = Pose_matrix(segment_.position, segment_.wxyz) @ self._mount.matrix()
        self._place_robot()
        if first_pose_:
            for client_ in self._server.get_clients().values():
                self._point_camera_at_robot(client_)

    def _place_robot(self) -> None:
        """Pose of the robot root and frustums from the mount link pose and the joint angles."""
        if self._mount_in_world is None or self._robot_frame is None:
            return
        mount_in_world_ = self._mount_in_world
        wxyz_, position_ = Matrix_to_wxyz_position(mount_in_world_ @ self._root_in_mount)
        self._robot_frame.wxyz = wxyz_
        self._robot_frame.position = position_
        self._robot_position = np.asarray(position_, dtype=float)
        self._robot_heading_rad = float(np.arctan2(mount_in_world_[1, 0], mount_in_world_[0, 0]))
        for name_, handle_ in self._camera_handles.items():
            wxyz_, position_ = Matrix_to_wxyz_position(
                mount_in_world_ @ self._cameras_in_mount[name_]
            )
            handle_.wxyz = wxyz_
            handle_.position = position_


# -------------------------------------------------------------------------------------------------
# Command line
# -------------------------------------------------------------------------------------------------
def Main(argv: list[str] | None = None) -> int:
    """Run the viewer until Ctrl+C."""
    parser_ = argparse.ArgumentParser(
        prog="python3 -m vicon_receiver_tools.viewer",
        description="Live viewer of the ViconFrame topic in a web browser (viser).",
    )
    parser_.add_argument("--config", type=Path, required=True, help="config/viewer.yaml")
    parser_.add_argument(
        "--repo-root", type=Path, default=Path.cwd(), help="base of relative paths"
    )
    arguments_ = parser_.parse_args(argv)

    # Only this module reports at INFO; the libraries it uses stay at WARNING
    logging.basicConfig(level=logging.WARNING, format="[viewer] %(message)s", stream=sys.stdout)
    _LOGGER.setLevel(logging.INFO)

    from vicon_receiver_tools.config import ConfigError

    try:
        config_ = Load_viewer_config(arguments_.config)
    except ConfigError as error_:
        print(f"[viewer] invalid config: {error_}", file=sys.stderr)
        return 1

    import rclpy
    import viser
    from rclpy.executors import ExternalShutdownException
    from rclpy.serialization import deserialize_message

    from vicon_receiver_msgs.msg import ViconFrame

    server_ = viser.ViserServer(host=config_.viewer.host, port=config_.viewer.port, verbose=False)
    viewer_ = ViserLiveViewer(config_, arguments_.repo_root, server_)
    buffer_ = LatestFrameBuffer(
        trail_length=config_.viewer.trail_length,
        deserialize=lambda data_: deserialize_message(data_, ViconFrame),
    )

    rclpy.init()
    node_ = rclpy.create_node("vicon_viewer")
    node_.create_subscription(
        ViconFrame, config_.viewer.topic, buffer_.put_serialized, 10, raw=True
    )
    joint_states_ = LatestJointStates()
    if config_.robot.joint_states_topic is not None:
        from sensor_msgs.msg import JointState

        joint_states_.deserialize = lambda data_: deserialize_message(data_, JointState)
        node_.create_subscription(
            JointState, config_.robot.joint_states_topic, joint_states_.put_serialized, 10, raw=True
        )

    def _Spin() -> None:
        # Ctrl+C shuts rclpy down from the main thread: end the spin quietly
        with contextlib.suppress(ExternalShutdownException):
            rclpy.spin(node_)

    spinner_ = threading.Thread(target=_Spin, daemon=True)
    spinner_.start()
    joints_note_ = config_.robot.joint_states_topic or "none: zero angles"
    _LOGGER.info(
        "open http://%s:%s (topic %s, joint states %s)",
        config_.viewer.host,
        config_.viewer.port,
        config_.viewer.topic,
        joints_note_,
    )

    period_s_ = 1.0 / config_.viewer.update_rate_hz
    last_frame_: ViewerFrame | None = None
    try:
        while rclpy.ok():
            viewer_.update_joints(joint_states_)
            frame_ = buffer_.latest()
            if frame_ is not None and frame_ is not last_frame_:
                if last_frame_ is None:
                    subjects_ = ", ".join(subject_.name for subject_ in frame_.subjects) or "none"
                    _LOGGER.info(
                        "receiving frames (frame %d, subjects: %s)", frame_.frame_number, subjects_
                    )
                viewer_.update(frame_, buffer_)
                last_frame_ = frame_
            elif frame_ is not None:
                # No new frame: the scene stays, only the telemetry (rate falling to 0) changes
                viewer_.update_telemetry(frame_, buffer_)
            time.sleep(period_s_)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        server_.stop()
    return 0


if __name__ == "__main__":
    sys.exit(Main())
