"""Live viewer of the Vicon stream in a web browser (viser).

Subscribes to the ViconFrame topic of the vicon_client node and shows, in the Vicon frame:

- the axes of every subject segment, hidden when occluded, and a trail of recent positions;
- the labeled markers, coloured by the number of cameras that saw them (green: enough cameras,
  yellow: fewer, red: occluded);
- the G1 Unitree model attached to its Vicon subject through the mount transform, and the camera
  frustums at the optical frames of the model;
- a telemetry panel (frame number, rate, latency, dropped frames, quality, occlusion);
- sliders to adjust the mount, and a button to save it to the mount file.

Rendering runs in the browser, so the machine running the node only sends small updates
(update_rate_hz of the viewer config). Start it with scripts/viz.sh.

Needs viser, yourdfpy and trimesh (in the conda environment of environment.yml) and the
ROS 2 environment of the workspace (vicon_receiver_msgs, rclpy).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from vicon_receiver_tools.viewer_config import (
    IDENTITY_MOUNT,
    Load_mount,
    Load_viewer_config,
    LinkPoseInBodyFrame,
    Save_mount,
    ViewerConfig,
)

# Marker colours (RGB): enough cameras, few cameras, occluded
COLOR_GOOD: tuple[int, int, int] = (40, 180, 60)
COLOR_FEW: tuple[int, int, int] = (235, 180, 30)
COLOR_OCCLUDED: tuple[int, int, int] = (220, 40, 40)

# Colour of the trails
COLOR_TRAIL: tuple[int, int, int] = (60, 120, 230)

# Period of the telemetry panel updates [s] (text does not need the scene rate)
TELEMETRY_PERIOD_S: float = 0.5


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


def Viewer_frame_from_message(message: Any) -> ViewerFrame:
    """Convert a vicon_receiver_msgs/ViconFrame (or an object with the same fields)."""
    subjects_: list[ViewerSubject] = []
    for subject_ in message.subjects:
        segments_ = [
            ViewerSegment(
                name=segment_.name,
                position=np.array([segment_.position.x, segment_.position.y, segment_.position.z]),
                # Vicon/ROS order x, y, z, w -> viser order w, x, y, z
                wxyz=np.array(
                    [
                        segment_.orientation.w,
                        segment_.orientation.x,
                        segment_.orientation.y,
                        segment_.orientation.z,
                    ]
                ),
                occluded=bool(segment_.translation_occluded or segment_.rotation_occluded),
            )
            for segment_ in subject_.segments
        ]
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
    """RGB colour (N x 3, uint8) of each marker from its visibility."""
    colors_ = np.zeros((len(markers), 3), dtype=np.uint8)
    for index_, marker_ in enumerate(markers):
        if marker_.occluded:
            colors_[index_] = COLOR_OCCLUDED
        elif marker_.camera_count >= good_camera_count:
            colors_[index_] = COLOR_GOOD
        else:
            colors_[index_] = COLOR_FEW
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
    deserialize: Callable[[bytes], Any] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _pending: Any = field(default=None, repr=False)
    _pending_is_serialized: bool = field(default=False, repr=False)
    _latest_view: ViewerFrame | None = field(default=None, repr=False)
    _trails: dict[str, deque] = field(default_factory=dict, repr=False)
    _receive_times: deque = field(default_factory=lambda: deque(maxlen=200), repr=False)

    def put_serialized(self, data: bytes, receive_time_s: float | None = None) -> None:
        """Store a serialized ViconFrame (ROS callback with raw=True): no decoding here."""
        self._store(data, True, receive_time_s)

    def put_message(self, message: Any, receive_time_s: float | None = None) -> None:
        """Store an already decoded ViconFrame (or an object with the same fields)."""
        self._store(message, False, receive_time_s)

    def latest(self) -> ViewerFrame | None:
        """Most recent frame; a new message is decoded and added to the trails here."""
        with self._lock:
            if self._pending is not None:
                message_ = self._pending
                if self._pending_is_serialized:
                    if self.deserialize is None:
                        raise ValueError("LatestFrameBuffer: serialized message without deserialize")
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

    def rate_hz(self) -> float:
        """Frames per second over the stored receive times (local clock)."""
        with self._lock:
            times_ = list(self._receive_times)
        if len(times_) < 2 or times_[-1] <= times_[0]:
            return 0.0
        return (len(times_) - 1) / (times_[-1] - times_[0])

    def _store(self, item: Any, serialized: bool, receive_time_s: float | None) -> None:
        """Keep only the newest item; frames in between are not drawn."""
        now_ = time.monotonic() if receive_time_s is None else receive_time_s
        with self._lock:
            self._pending = item
            self._pending_is_serialized = serialized
            self._receive_times.append(now_)

    def _extend_trails(self, frame: ViewerFrame) -> None:
        """Append the visible segment positions of a frame to the trails (lock held)."""
        for subject_ in frame.subjects:
            for segment_ in subject_.segments:
                if segment_.occluded:
                    continue
                key_ = f"{subject_.ros_name}/{segment_.name}"
                trail_ = self._trails.setdefault(key_, deque(maxlen=self.trail_length))
                trail_.append(segment_.position.copy())


# -------------------------------------------------------------------------------------------------
# Scene
# -------------------------------------------------------------------------------------------------
class ViserLiveViewer:
    """Scene and GUI of the viewer on a viser server."""

    def __init__(self, config: ViewerConfig, repo_root: Path, server: Any) -> None:
        """Build the static scene, load the robot model (if fetched) and the GUI."""
        self._config = config
        self._repo_root = repo_root
        self._server = server
        self._segment_handles: dict[str, Any] = {}
        self._marker_handles: dict[str, Any] = {}
        self._last_telemetry_s = 0.0
        self._last_trail_s = 0.0

        server.scene.add_grid("/grid", width=10.0, height=10.0, cell_size=0.5, section_size=1.0)
        server.scene.add_frame("/vicon", axes_length=0.3, axes_radius=0.01)

        mount_path_ = repo_root / config.robot.mount_file
        loaded_mount_ = Load_mount(mount_path_)
        self._mount: LinkPoseInBodyFrame = loaded_mount_ if loaded_mount_ is not None else IDENTITY_MOUNT
        self._mount_note = (
            f"mount from {config.robot.mount_file}"
            if loaded_mount_ is not None
            else f"no {config.robot.mount_file}: identity mount (run scripts/estimate_mount.sh)"
        )

        self._robot_frame: Any = None
        self._camera_handles: dict[str, Any] = {}
        self._root_in_mount = np.eye(4)
        self._cameras_in_mount: dict[str, np.ndarray] = {}
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
            self._root_in_mount = Link_pose(urdf_, urdf_.base_link, robot_.mount_link)
            for camera_ in self._config.cameras:
                self._cameras_in_mount[camera_.name] = Link_pose(urdf_, camera_.frame, robot_.mount_link)
        except RobotModelError as error_:
            return f"robot model not shown: {error_}"

        # The URDF nodes hang below /robot, whose pose is set at every update
        self._robot_frame = self._server.scene.add_frame("/robot", show_axes=False, visible=False)
        ViserUrdf(self._server, urdf_, root_node_name="/robot")

        for camera_ in self._config.cameras:
            self._camera_handles[camera_.name] = self._server.scene.add_camera_frustum(
                f"/cameras/{camera_.name}",
                fov=camera_.vertical_fov_rad(),
                aspect=camera_.aspect(),
                scale=self._config.frustum_scale_m,
                visible=False,
            )
        return f"robot model {robot_.urdf} attached to {robot_.subject} at {robot_.mount_link}"

    def _build_gui(self) -> None:
        """Telemetry panel, mount sliders and save button."""
        gui_ = self._server.gui
        self._telemetry = gui_.add_markdown("Waiting for ViconFrame ...")
        gui_.add_markdown(f"_{self._robot_note}_  \n_{self._mount_note}_")

        with gui_.add_folder(f"Mount: {self._config.robot.mount_link} in Vicon body", expand_by_default=False):
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
            "Adjusted in the live viewer (scripts/viz.sh).",
        )
        print(f"[viewer] mount saved to {path_}")

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
                key_ = f"{subject_.ros_name}/{segment_.name}"
                handle_ = self._segment_handles.get(key_)
                if handle_ is None:
                    handle_ = scene_.add_frame(f"/subjects/{key_}", axes_length=0.15, axes_radius=0.006)
                    self._segment_handles[key_] = handle_
                handle_.visible = not segment_.occluded
                if not segment_.occluded:
                    handle_.position = segment_.position
                    handle_.wxyz = segment_.wxyz

                segments_ = Trail_segments(buffer.trail(key_)) if redraw_trails_ else np.zeros((0, 2, 3))
                if len(segments_) > 0:
                    scene_.add_line_segments(
                        f"/trails/{key_}", points=segments_, colors=COLOR_TRAIL, thickness=0.004
                    )

            self._update_markers(subject_)
            if subject_.name == self._config.robot.subject:
                self._update_robot(subject_)

        if now_ - self._last_telemetry_s >= TELEMETRY_PERIOD_S:
            self._telemetry.content = Telemetry_markdown(frame, buffer.rate_hz())
            self._last_telemetry_s = now_

    def _update_markers(self, subject: ViewerSubject) -> None:
        """Markers of one subject as a point cloud, occluded ones left out."""
        visible_ = [marker_ for marker_ in subject.markers if not marker_.occluded]
        if not visible_:
            return
        points_ = np.array([marker_.position for marker_ in visible_], dtype=np.float32)
        colors_ = Marker_colors(visible_, self._config.viewer.good_camera_count)
        self._server.scene.add_point_cloud(
            f"/markers/{subject.ros_name}",
            points=points_,
            colors=colors_,
            point_size=2.0 * self._config.viewer.marker_radius_m,
            point_shape="circle",
        )

    def _update_robot(self, subject: ViewerSubject) -> None:
        """Place the robot model and the frustums at the pose of the subject."""
        if self._robot_frame is None or not subject.segments:
            return
        segment_ = subject.segments[0]
        visible_ = not segment_.occluded
        self._robot_frame.visible = visible_
        for handle_ in self._camera_handles.values():
            handle_.visible = visible_
        if not visible_:
            return

        mount_in_world_ = Pose_matrix(segment_.position, segment_.wxyz) @ self._mount.matrix()
        wxyz_, position_ = Matrix_to_wxyz_position(mount_in_world_ @ self._root_in_mount)
        self._robot_frame.wxyz = wxyz_
        self._robot_frame.position = position_
        for name_, handle_ in self._camera_handles.items():
            wxyz_, position_ = Matrix_to_wxyz_position(mount_in_world_ @ self._cameras_in_mount[name_])
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
    parser_.add_argument("--repo-root", type=Path, default=Path.cwd(), help="base of relative paths")
    arguments_ = parser_.parse_args(argv)

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
    def _Spin() -> None:
        # Ctrl+C shuts rclpy down from the main thread: end the spin quietly
        try:
            rclpy.spin(node_)
        except ExternalShutdownException:
            pass

    spinner_ = threading.Thread(target=_Spin, daemon=True)
    spinner_.start()
    print(f"[viewer] open http://{config_.viewer.host}:{config_.viewer.port} (topic {config_.viewer.topic})")

    period_s_ = 1.0 / config_.viewer.update_rate_hz
    first_frame_reported_ = False
    try:
        while rclpy.ok():
            frame_ = buffer_.latest()
            if frame_ is not None:
                if not first_frame_reported_:
                    subjects_ = ", ".join(subject_.name for subject_ in frame_.subjects) or "none"
                    print(f"[viewer] receiving frames (frame {frame_.frame_number}, subjects: {subjects_})")
                    first_frame_reported_ = True
                viewer_.update(frame_, buffer_)
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
