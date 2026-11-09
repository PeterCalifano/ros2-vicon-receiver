"""Unit tests of the session export (vicon_receiver_tools.export) without ROS 2.

The bag reader is replaced by a generator of ViconFrame-like objects, so the CSV exporter and the
error handling of Export() run without rosbag2. The end-to-end path (record + export of a real MCAP
bag) is covered by tests/integration/test_record_export.py.
"""

from __future__ import annotations

import csv
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace

import pytest
from vicon_receiver_tools.export import Export, ExportError

from vicon_receiver_tools import export

TOPIC: str = "/vicon/frame"


def Make_frame(frame_number: int) -> SimpleNamespace:
    """A ViconFrame-like object: one subject, one segment, one marker."""
    segment_ = SimpleNamespace(
        name="body",
        frame_id="body_body",
        position=SimpleNamespace(x=0.001 * frame_number, y=0.0, z=1.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        translation_occluded=False,
        rotation_occluded=False,
    )
    marker_ = SimpleNamespace(
        name="body1",
        position=SimpleNamespace(x=0.0, y=0.0, z=1.0),
        occluded=False,
        camera_ids=[1, 2, 3],
    )
    subject_ = SimpleNamespace(
        name="body",
        ros_name="body",
        quality_valid=True,
        quality=1.3,
        segments=[segment_],
        markers=[marker_],
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=frame_number * 10_000_000)),
        receive_stamp=SimpleNamespace(sec=100, nanosec=frame_number * 10_000_000 + 31_000_000),
        frame_number=frame_number,
        frames_dropped=0,
        frame_rate_hz=100.0,
        latency_total_s=0.031,
        any_occluded=False,
        subjects=[subject_],
    )


@pytest.fixture
def session_dir(tmp_path: Path) -> Path:
    """Session directory with an (empty) bag directory; the reader is replaced in the tests."""
    (tmp_path / "bag").mkdir()
    return tmp_path


def Use_frames(monkeypatch: pytest.MonkeyPatch, frames: Iterator[SimpleNamespace]) -> None:
    """Make Export() read the given frames instead of an MCAP bag."""
    monkeypatch.setattr(export, "Read_vicon_frames", lambda bag_dir, topic: frames)


def test_csv_export_writes_one_row_per_frame(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Frames, poses and markers files with one row per frame (and per marker)."""
    Use_frames(monkeypatch, iter([Make_frame(1), Make_frame(2)]))
    count_, paths_ = Export(session_dir, ["csv"], topic=TOPIC)

    assert count_ == 2
    assert sorted(path_.name for path_ in paths_) == [
        "frames.csv",
        "markers_body.csv",
        "poses_body_body.csv",
    ]
    with (session_dir / "exports" / "frames.csv").open(newline="", encoding="utf-8") as handle_:
        rows_ = list(csv.DictReader(handle_))
    assert [row_["frame_number"] for row_ in rows_] == ["1", "2"]
    assert rows_[0]["stamp_s"] == "100.010000000"


def test_failed_read_leaves_no_partial_files(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A bag that breaks half-way: the error is reported and no partial CSV is left behind."""

    def _Broken_bag() -> Iterator[SimpleNamespace]:
        yield Make_frame(1)
        raise ExportError("truncated bag")

    Use_frames(monkeypatch, _Broken_bag())
    with pytest.raises(ExportError, match="truncated bag"):
        Export(session_dir, ["csv", "hdf5"], topic=TOPIC)
    assert list((session_dir / "exports").iterdir()) == []


def test_empty_bag_is_an_error(session_dir: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """No ViconFrame on the topic: nothing to export."""
    Use_frames(monkeypatch, iter([]))
    with pytest.raises(ExportError, match="no /vicon/frame messages"):
        Export(session_dir, ["csv"], topic=TOPIC)


def test_unknown_format_is_rejected(session_dir: Path) -> None:
    """Only registered formats; nothing implicit."""
    with pytest.raises(ExportError, match="unknown or missing format"):
        Export(session_dir, ["parquet"], topic=TOPIC)


def test_a_format_given_twice_is_exported_once(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two CSV exporters would write the same files: a repeated format counts once."""
    Use_frames(monkeypatch, iter([Make_frame(1), Make_frame(2)]))
    count_, paths_ = Export(session_dir, ["csv", "csv"], topic=TOPIC)
    assert count_ == 2
    assert len(paths_) == len(set(paths_))
    with (session_dir / "exports" / "frames.csv").open(encoding="utf-8") as stream_:
        assert len(list(csv.DictReader(stream_))) == 2


def Two_subject_frame(names: tuple[str, str], frame_ids: tuple[str, str]) -> SimpleNamespace:
    """A frame with two subjects, with the given subject names and segment frame ids."""
    frame_ = Make_frame(1)
    first_ = frame_.subjects[0]
    second_ = SimpleNamespace(**vars(first_))
    second_.segments = [SimpleNamespace(**vars(first_.segments[0]))]
    first_.name, second_.name = names
    first_.ros_name, second_.ros_name = names
    first_.segments[0].frame_id, second_.segments[0].frame_id = frame_ids
    frame_.subjects.append(second_)
    return frame_


def test_csv_export_refuses_segments_sharing_a_file(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """a_b/c and a/b_c have the same frame id: their rows must not be mixed in one file."""
    Use_frames(monkeypatch, iter([Two_subject_frame(("a_b", "a"), ("a_b_c", "a_b_c"))]))
    with pytest.raises(ExportError, match=r"would be mixed in poses_a_b_c\.csv"):
        Export(session_dir, ["csv"], topic=TOPIC)
    assert list((session_dir / "exports").iterdir()) == []


def test_hdf5_export_refuses_subjects_sharing_a_group(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Names that convert to the same HDF5 group are refused too."""
    Use_frames(monkeypatch, iter([Two_subject_frame(("a b", "a_b"), ("first", "second"))]))
    with pytest.raises(ExportError, match="would be mixed in subjects/a_b"):
        Export(session_dir, ["hdf5"], topic=TOPIC)


def test_hdf5_export_refuses_segments_sharing_a_key(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two segments of one subject that convert to one name would overwrite each other."""
    frame_ = Make_frame(1)
    first_ = frame_.subjects[0].segments[0]
    second_ = SimpleNamespace(**vars(first_))
    first_.name, second_.name = "a b", "a_b"
    frame_.subjects[0].segments.append(second_)
    Use_frames(monkeypatch, iter([frame_]))
    with pytest.raises(ExportError, match="would be mixed in subjects/body/segments/a_b"):
        Export(session_dir, ["hdf5"], topic=TOPIC)


def test_export_keeps_distinct_segments_apart(
    session_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two subjects with their own frame ids get their own files, on every frame."""
    frames_ = [Two_subject_frame(("a", "b"), ("a_body", "b_body")) for _ in range(2)]
    Use_frames(monkeypatch, iter(frames_))
    count_, _ = Export(session_dir, ["csv"], topic=TOPIC)
    assert count_ == 2
    assert (session_dir / "exports" / "poses_a_body.csv").is_file()
    assert (session_dir / "exports" / "poses_b_body.csv").is_file()
