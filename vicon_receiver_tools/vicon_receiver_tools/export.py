"""Optional export of a recorded session (MCAP bag) to CSV and/or HDF5.

A session recorded by scripts/record.sh is a directory with an MCAP bag. The bag is the stored
data; this module converts its ViconFrame messages into files for analysis tools, only when asked
(scripts/export.sh SESSION --format csv,hdf5). Every format is one exporter class fed with the same
stream of frames, so formats can be added without touching the reader.

Outputs, in <session>/exports/:

csv
    frames.csv                  one row per frame: numbers, stamps, latency, occlusion
    poses_<frame_id>.csv        one file per segment: pose and occlusion mask per frame
    markers_<subject>.csv       one file per subject: one row per marker and frame
hdf5
    session.h5                  arrays aligned on the frame index (missing values: NaN,
                                occluded: true), session metadata and configuration as attributes

Stamps are in seconds: "stamp_s" is the estimated camera acquisition time, "receive_stamp_s" the
local receive time. Positions are in metres in the Vicon frame, quaternions x, y, z, w.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

import numpy as np

from vicon_receiver_tools.config import ConfigError, Read_parameter


class ExportError(Exception):
    """The session cannot be exported."""


# -------------------------------------------------------------------------------------------------
# Reading the bag
# -------------------------------------------------------------------------------------------------
def Session_topic(session_dir: Path) -> str:
    """ViconFrame topic of a session, from the namespace of its copied node configuration."""
    config_ = session_dir / "config" / "vicon_client.yaml"
    try:
        namespace_ = str(Read_parameter(config_, "namespace")).strip("/")
    except ConfigError as error_:
        raise ExportError(f"cannot read the session configuration: {error_}") from error_
    return f"/{namespace_}/frame"


def Read_vicon_frames(bag_dir: Path, topic: str) -> Iterator[Any]:
    """Yield the ViconFrame messages of an MCAP bag, in recording order.

    Raises:
        ExportError: If the bag cannot be opened or has no such topic.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from vicon_receiver_msgs.msg import ViconFrame

    reader_ = rosbag2_py.SequentialReader()
    try:
        reader_.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except RuntimeError as error_:
        raise ExportError(f"cannot open the bag {bag_dir}: {error_}") from error_

    topics_ = {metadata_.name for metadata_ in reader_.get_all_topics_and_types()}
    if topic not in topics_:
        raise ExportError(f"topic {topic} not in the bag (topics: {', '.join(sorted(topics_))})")

    reader_.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader_.has_next():
        _, data_, _ = reader_.read_next()
        yield deserialize_message(data_, ViconFrame)


def Stamp_seconds(stamp: Any) -> float:
    """Seconds of a builtin_interfaces/Time."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def Safe_name(name: str) -> str:
    """Name usable in a file name or an HDF5 path (no '/' or spaces)."""
    return "".join(character_ if character_.isalnum() or character_ in "-_." else "_" for character_ in name)


# -------------------------------------------------------------------------------------------------
# CSV
# -------------------------------------------------------------------------------------------------
FRAME_COLUMNS: list[str] = [
    "frame_number",
    "stamp_s",
    "receive_stamp_s",
    "frames_dropped",
    "frame_rate_hz",
    "latency_total_s",
    "any_occluded",
]
POSE_COLUMNS: list[str] = [
    "frame_number",
    "stamp_s",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "translation_occluded",
    "rotation_occluded",
    "quality",
]
MARKER_COLUMNS: list[str] = ["frame_number", "stamp_s", "marker", "x", "y", "z", "occluded", "camera_count"]


@dataclass
class CsvExporter:
    """Writes the frames, poses and markers as CSV files (streaming, one row per sample)."""

    output_dir: Path
    _files: dict[str, TextIO] = field(default_factory=dict, repr=False)
    _writers: dict[str, Any] = field(default_factory=dict, repr=False)

    name: str = "csv"

    def _writer(self, file_name: str, columns: list[str]) -> Any:
        """CSV writer of a file, created with its header on first use."""
        if file_name not in self._writers:
            handle_ = (self.output_dir / file_name).open("w", newline="", encoding="utf-8")
            writer_ = csv.writer(handle_)
            writer_.writerow(columns)
            self._files[file_name] = handle_
            self._writers[file_name] = writer_
        return self._writers[file_name]

    def add(self, frame: Any) -> None:
        """Write the rows of one frame."""
        stamp_ = Stamp_seconds(frame.header.stamp)
        self._writer("frames.csv", FRAME_COLUMNS).writerow(
            [
                frame.frame_number,
                f"{stamp_:.9f}",
                f"{Stamp_seconds(frame.receive_stamp):.9f}",
                frame.frames_dropped,
                frame.frame_rate_hz,
                frame.latency_total_s,
                int(frame.any_occluded),
            ]
        )

        for subject_ in frame.subjects:
            quality_ = subject_.quality if subject_.quality_valid else ""
            for segment_ in subject_.segments:
                self._writer(f"poses_{Safe_name(segment_.frame_id)}.csv", POSE_COLUMNS).writerow(
                    [
                        frame.frame_number,
                        f"{stamp_:.9f}",
                        segment_.position.x,
                        segment_.position.y,
                        segment_.position.z,
                        segment_.orientation.x,
                        segment_.orientation.y,
                        segment_.orientation.z,
                        segment_.orientation.w,
                        int(segment_.translation_occluded),
                        int(segment_.rotation_occluded),
                        quality_,
                    ]
                )

            if subject_.markers:
                writer_ = self._writer(f"markers_{Safe_name(subject_.ros_name)}.csv", MARKER_COLUMNS)
                for marker_ in subject_.markers:
                    writer_.writerow(
                        [
                            frame.frame_number,
                            f"{stamp_:.9f}",
                            marker_.name,
                            marker_.position.x,
                            marker_.position.y,
                            marker_.position.z,
                            int(marker_.occluded),
                            len(marker_.camera_ids),
                        ]
                    )

    def finish(self, metadata: dict[str, str]) -> list[Path]:
        """Close the files and return their paths."""
        del metadata  # CSV files carry no metadata: it stays in the session directory
        paths_ = [Path(handle_.name) for handle_ in self._files.values()]
        for handle_ in self._files.values():
            handle_.close()
        return paths_


# -------------------------------------------------------------------------------------------------
# HDF5
# -------------------------------------------------------------------------------------------------
@dataclass
class Hdf5Exporter:
    """Collects every frame and writes one HDF5 file with arrays aligned on the frame index."""

    output_dir: Path
    name: str = "hdf5"
    _frames: dict[str, list[Any]] = field(default_factory=lambda: {column_: [] for column_ in FRAME_COLUMNS})
    # key -> {frame index: values}; turned into dense arrays at the end
    _segments: dict[tuple[str, str], dict[int, tuple[Any, ...]]] = field(default_factory=dict)
    _markers: dict[tuple[str, str], dict[int, tuple[Any, ...]]] = field(default_factory=dict)
    _quality: dict[str, dict[int, float]] = field(default_factory=dict)

    def add(self, frame: Any) -> None:
        """Store one frame."""
        index_ = len(self._frames["frame_number"])
        self._frames["frame_number"].append(frame.frame_number)
        self._frames["stamp_s"].append(Stamp_seconds(frame.header.stamp))
        self._frames["receive_stamp_s"].append(Stamp_seconds(frame.receive_stamp))
        self._frames["frames_dropped"].append(frame.frames_dropped)
        self._frames["frame_rate_hz"].append(frame.frame_rate_hz)
        self._frames["latency_total_s"].append(frame.latency_total_s)
        self._frames["any_occluded"].append(bool(frame.any_occluded))

        for subject_ in frame.subjects:
            subject_key_ = Safe_name(subject_.name)
            self._quality.setdefault(subject_key_, {})[index_] = (
                subject_.quality if subject_.quality_valid else math.nan
            )
            for segment_ in subject_.segments:
                self._segments.setdefault((subject_key_, Safe_name(segment_.name)), {})[index_] = (
                    (segment_.position.x, segment_.position.y, segment_.position.z),
                    (segment_.orientation.x, segment_.orientation.y, segment_.orientation.z, segment_.orientation.w),
                    bool(segment_.translation_occluded),
                    bool(segment_.rotation_occluded),
                )
            for marker_ in subject_.markers:
                self._markers.setdefault((subject_key_, Safe_name(marker_.name)), {})[index_] = (
                    (marker_.position.x, marker_.position.y, marker_.position.z),
                    bool(marker_.occluded),
                    len(marker_.camera_ids),
                )

    def finish(self, metadata: dict[str, str]) -> list[Path]:
        """Write session.h5 and return its path."""
        try:
            import h5py
        except ImportError as error_:
            raise ExportError("h5py is not installed (it is in the conda environment of environment.yml)") from error_

        count_ = len(self._frames["frame_number"])
        path_ = self.output_dir / "session.h5"
        with h5py.File(path_, "w") as file_:
            for key_, value_ in metadata.items():
                file_.attrs[key_] = value_

            frames_ = file_.create_group("frames")
            frames_.create_dataset("frame_number", data=np.array(self._frames["frame_number"], dtype=np.uint64))
            frames_.create_dataset("frames_dropped", data=np.array(self._frames["frames_dropped"], dtype=np.uint32))
            frames_.create_dataset("any_occluded", data=np.array(self._frames["any_occluded"], dtype=bool))
            for column_ in ("stamp_s", "receive_stamp_s", "frame_rate_hz", "latency_total_s"):
                frames_.create_dataset(column_, data=np.array(self._frames[column_], dtype=np.float64))

            for subject_key_, values_ in self._quality.items():
                quality_ = np.full(count_, math.nan)
                for index_, value_ in values_.items():
                    quality_[index_] = value_
                file_.create_dataset(f"subjects/{subject_key_}/quality", data=quality_)

            for (subject_key_, segment_key_), samples_ in self._segments.items():
                position_ = np.full((count_, 3), math.nan)
                orientation_ = np.full((count_, 4), math.nan)
                translation_occluded_ = np.ones(count_, dtype=bool)
                rotation_occluded_ = np.ones(count_, dtype=bool)
                for index_, (point_, quaternion_, t_occluded_, r_occluded_) in samples_.items():
                    position_[index_] = point_
                    orientation_[index_] = quaternion_
                    translation_occluded_[index_] = t_occluded_
                    rotation_occluded_[index_] = r_occluded_
                group_ = file_.create_group(f"subjects/{subject_key_}/segments/{segment_key_}")
                group_.create_dataset("position", data=position_)
                group_.create_dataset("orientation_xyzw", data=orientation_)
                group_.create_dataset("translation_occluded", data=translation_occluded_)
                group_.create_dataset("rotation_occluded", data=rotation_occluded_)

            for (subject_key_, marker_key_), samples_ in self._markers.items():
                position_ = np.full((count_, 3), math.nan)
                occluded_ = np.ones(count_, dtype=bool)
                camera_count_ = np.zeros(count_, dtype=np.uint32)
                for index_, (point_, marker_occluded_, cameras_) in samples_.items():
                    position_[index_] = point_
                    occluded_[index_] = marker_occluded_
                    camera_count_[index_] = cameras_
                group_ = file_.create_group(f"subjects/{subject_key_}/markers/{marker_key_}")
                group_.create_dataset("position", data=position_)
                group_.create_dataset("occluded", data=occluded_)
                group_.create_dataset("camera_count", data=camera_count_)
        return [path_]


# Registered formats: name -> exporter class
EXPORTERS: dict[str, type] = {"csv": CsvExporter, "hdf5": Hdf5Exporter}


def Session_metadata(session_dir: Path) -> dict[str, str]:
    """Text of the session metadata and configuration files, for the outputs that keep them."""
    metadata_: dict[str, str] = {}
    meta_path_ = session_dir / "meta.yaml"
    if meta_path_.is_file():
        metadata_["meta_yaml"] = meta_path_.read_text(encoding="utf-8")
    for config_path_ in sorted((session_dir / "config").glob("*.yaml")):
        metadata_[f"config_{config_path_.stem}_yaml"] = config_path_.read_text(encoding="utf-8")
    return metadata_


def Export(session_dir: Path, formats: list[str], topic: str | None = None) -> tuple[int, list[Path]]:
    """Export a session in the given formats.

    Args:
        session_dir: Session directory (with bag/ and config/).
        formats: Names of EXPORTERS to run.
        topic: ViconFrame topic; by default from the session configuration.

    Returns:
        The number of frames and the written files.

    Raises:
        ExportError: If a format is unknown, the bag cannot be read or has no frames.
    """
    unknown_ = [format_ for format_ in formats if format_ not in EXPORTERS]
    if unknown_ or not formats:
        raise ExportError(f"unknown or missing format {unknown_} (available: {', '.join(EXPORTERS)})")

    bag_dir_ = session_dir / "bag"
    if not bag_dir_.is_dir():
        raise ExportError(f"no bag in {session_dir}")
    topic_ = topic if topic is not None else Session_topic(session_dir)

    output_dir_ = session_dir / "exports"
    output_dir_.mkdir(exist_ok=True)
    exporters_ = [EXPORTERS[format_](output_dir=output_dir_) for format_ in formats]

    count_ = 0
    for frame_ in Read_vicon_frames(bag_dir_, topic_):
        for exporter_ in exporters_:
            exporter_.add(frame_)
        count_ += 1
    if count_ == 0:
        raise ExportError(f"no {topic_} messages in {bag_dir_}")

    metadata_ = Session_metadata(session_dir)
    paths_: list[Path] = []
    for exporter_ in exporters_:
        paths_.extend(exporter_.finish(metadata_))
    return count_, paths_


def Main(argv: list[str] | None = None) -> int:
    """Command-line entry point."""
    parser_ = argparse.ArgumentParser(
        prog="python3 -m vicon_receiver_tools.export",
        description="Export the ViconFrame messages of a recorded session (MCAP) to CSV and/or HDF5.",
    )
    parser_.add_argument("session_dir", type=Path, help="session directory (scripts/record.sh)")
    parser_.add_argument(
        "--format",
        required=True,
        help=f"comma-separated formats: {', '.join(EXPORTERS)} (no default: export is opt-in)",
    )
    arguments_ = parser_.parse_args(argv)

    formats_ = [format_.strip() for format_ in arguments_.format.split(",") if format_.strip()]
    try:
        count_, paths_ = Export(arguments_.session_dir, formats_)
    except ExportError as error_:
        print(f"export failed: {error_}", file=sys.stderr)
        return 1

    print(f"exported {count_} frames:")
    for path_ in paths_:
        print(f"  {path_}")
    return 0


if __name__ == "__main__":
    sys.exit(Main())
