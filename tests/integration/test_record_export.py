"""Integration test of recording (scripts/record.sh) and export (scripts/export.sh).

A synthetic publisher (tests/fixtures/fake_vicon_frames.py) stands in for the node. The test
records a few seconds to MCAP, stops the recorder with SIGINT (like Ctrl+C), exports the session
to CSV and HDF5, and checks the injected pattern (occlusions, dropped frames, markers) in the files.
"""

from __future__ import annotations

import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

from conftest import (
    CONFIG_FILE,
    REPO_ROOT,
    SCRIPTS_DIR,
    ConfigFactory,
    Tools_env_python,
    Wait_or_kill,
)

pytestmark = pytest.mark.integration

FIXTURE: Path = REPO_ROOT / "tests" / "fixtures" / "fake_vicon_frames.py"

# Separate ROS domain, so that other ROS 2 processes on the machine are not recorded
TEST_DOMAIN_ID: str = "89"

# Pattern of the fixture
OCCLUSION_PERIOD: int = 50
DROPPED_FRAMES: int = 3

# Start-up of record.sh (node check, ros2 bag record) can take several seconds on a loaded machine
RECORDING_START_TIMEOUT_S: float = 30.0

# Recording time once the bag file exists [s]
RECORDING_DURATION_S: float = 4.0


def Wait_for_recording(output_dir: Path, timeout_s: float) -> bool:
    """Wait until a session under output_dir has an MCAP file, i.e. the recorder is recording."""
    deadline_ = time.monotonic() + timeout_s
    while time.monotonic() < deadline_:
        if list(output_dir.glob("*/bag/*.mcap")):
            return True
        time.sleep(0.1)
    return False


def Read_csv(path: Path) -> list[dict[str, str]]:
    """Rows of a CSV file as dictionaries."""
    with path.open(newline="", encoding="utf-8") as handle_:
        return list(csv.DictReader(handle_))


def Record_session(ros_env: dict[str, str], tmp_path: Path, stop_signal: signal.Signals) -> Path:
    """Record ~4 s of the synthetic stream, stop with a signal and return the session directory."""
    environment_ = dict(ros_env)
    environment_["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID

    publisher_ = subprocess.Popen(
        [sys.executable, str(FIXTURE), "--duration", "40"],
        env=environment_,
        stdout=subprocess.DEVNULL,
        start_new_session=True,
    )
    recorder_: subprocess.Popen[str] | None = None
    try:
        time.sleep(1.5)
        recorder_ = subprocess.Popen(
            [str(SCRIPTS_DIR / "record.sh"), "-c", str(CONFIG_FILE), "-t", "test", "-o", str(tmp_path)],
            env=environment_,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        # Stop only once the recorder is recording: a stop during its start-up leaves no bag
        assert Wait_for_recording(tmp_path, RECORDING_START_TIMEOUT_S), "the recorder did not start"
        time.sleep(RECORDING_DURATION_S)
        # The signal reaches the whole group, as Ctrl+C or a closed terminal do
        os.killpg(recorder_.pid, stop_signal)
        output_, _ = recorder_.communicate(timeout=60)
        assert recorder_.returncode == 0, output_
    finally:
        # A failed step must not leave the recorder or the publisher running
        if recorder_ is not None and recorder_.poll() is None:
            os.killpg(recorder_.pid, signal.SIGTERM)
            Wait_or_kill(recorder_, 30)
        # The publisher runs longer than any start-up: stop it once the recording is done
        if publisher_.poll() is None:
            os.killpg(publisher_.pid, signal.SIGTERM)
        Wait_or_kill(publisher_, 30)

    sessions_ = [path_ for path_ in tmp_path.iterdir() if path_.is_dir()]
    assert len(sessions_) == 1, output_
    return sessions_[0]


@pytest.fixture
def recorded_session(ros_env: dict[str, str], tmp_path: Path) -> Path:
    """Session stopped with Ctrl+C."""
    return Record_session(ros_env, tmp_path, signal.SIGINT)


def test_record_and_export(recorded_session: Path, ros_env: dict[str, str]) -> None:
    """MCAP session with metadata, then CSV and HDF5 exports with the injected pattern."""
    session_ = recorded_session
    assert session_.name.endswith("_test")
    assert list((session_ / "bag").glob("*.mcap")), "no MCAP file"
    assert (session_ / "config" / "vicon_client.yaml").is_file()
    # The mount file named by the viewer config
    assert (session_ / "config" / "g1_mount.yaml").is_file()
    assert "Topic: /vicon/frame" in (session_ / "bag_info.txt").read_text(encoding="utf-8")

    metadata_ = yaml.safe_load((session_ / "meta.yaml").read_text(encoding="utf-8"))
    assert metadata_["storage"] == "mcap"
    assert metadata_["tag"] == "test"
    assert "stop_utc" in metadata_ and "clock_source" in metadata_

    # Nothing is exported unless asked: no exports/ yet
    assert not (session_ / "exports").exists()

    # export.sh runs in the conda environment of environment.yml (h5py, rosbag2 through ROS)
    if not Tools_env_python():
        pytest.skip("conda environment of environment.yml missing (scripts/build.sh): export untested")

    export_ = subprocess.run(
        [str(SCRIPTS_DIR / "export.sh"), str(session_), "--format", "csv,hdf5"],
        env=ros_env,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert export_.returncode == 0, export_.stdout + export_.stderr
    exports_ = session_ / "exports"

    # Frames: increasing numbers, the injected gaps (every 3 s) reported with their size
    frames_ = Read_csv(exports_ / "frames.csv")
    assert len(frames_) > 300
    numbers_ = [int(row_["frame_number"]) for row_ in frames_]
    assert numbers_ == sorted(numbers_)
    drops_ = [int(row_["frames_dropped"]) for row_ in frames_ if row_["frames_dropped"] != "0"]
    assert drops_ and set(drops_) == {DROPPED_FRAMES}

    # Poses: occlusion mask exactly on the multiples of the occlusion period, pose kept as sent
    poses_ = Read_csv(exports_ / "poses_test_body_test_body.csv")
    assert len(poses_) == len(frames_)
    for row_ in poses_:
        expected_occluded_ = int(row_["frame_number"]) % OCCLUSION_PERIOD == 0
        assert bool(int(row_["translation_occluded"])) == expected_occluded_
        if expected_occluded_:
            assert float(row_["qw"]) == 0.0
        else:
            assert float(row_["x"]) == pytest.approx(0.001 * int(row_["frame_number"]))

    # Markers: three per frame, camera counts 3, 4, 5 when visible
    markers_ = Read_csv(exports_ / "markers_test_body.csv")
    assert len(markers_) == 3 * len(frames_)
    visible_counts_ = {int(row_["camera_count"]) for row_ in markers_ if row_["occluded"] == "0"}
    assert visible_counts_ == {3, 4, 5}

    # HDF5: the same frames, arrays aligned on the frame index
    h5py_ = pytest.importorskip("h5py")
    with h5py_.File(exports_ / "session.h5", "r") as file_:
        count_ = len(file_["frames/frame_number"])
        assert count_ == len(frames_)
        segment_ = file_["subjects/test-body/segments/test-body"]
        assert segment_["position"].shape == (count_, 3)
        assert segment_["orientation_xyzw"].shape == (count_, 4)
        assert int(segment_["translation_occluded"][()].sum()) == sum(
            1 for row_ in poses_ if row_["translation_occluded"] == "1"
        )
        assert file_["subjects/test-body/markers/test-body3/camera_count"][()].max() == 5
        assert "meta_yaml" in file_.attrs


def test_record_stops_before_creating_a_session_on_a_missing_entry(
    ros_env: dict[str, str], tmp_path: Path, make_config: ConfigFactory
) -> None:
    """A config without stream_mode: record.sh fails by name and creates no session directory."""
    environment_ = dict(ros_env)
    environment_["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
    config_ = make_config(remove=["stream_mode"])
    output_dir_ = tmp_path / "sessions"

    publisher_ = subprocess.Popen(
        [sys.executable, str(FIXTURE), "--duration", "6"],
        env=environment_,
        stdout=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        time.sleep(1.5)
        result_ = subprocess.run(
            [str(SCRIPTS_DIR / "record.sh"), "-c", str(config_), "-o", str(output_dir_)],
            env=environment_,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
    finally:
        Wait_or_kill(publisher_, 30)

    assert result_.returncode != 0, result_.stdout
    assert "stream_mode" in result_.stderr
    assert not output_dir_.exists() or not any(output_dir_.iterdir())


def test_closing_the_terminal_closes_the_bag(ros_env: dict[str, str], tmp_path: Path) -> None:
    """SIGHUP to the whole group: the recorder survives it and closes the bag on our SIGTERM."""
    session_ = Record_session(ros_env, tmp_path, signal.SIGHUP)
    assert (session_ / "bag" / "metadata.yaml").is_file()
    assert "stop_utc" in yaml.safe_load((session_ / "meta.yaml").read_text(encoding="utf-8"))


def test_record_refuses_an_empty_namespace(
    ros_env: dict[str, str], tmp_path: Path, make_config: ConfigFactory
) -> None:
    """With an empty namespace the topic filter would match nothing but TF."""
    output_dir_ = tmp_path / "sessions"
    result_ = subprocess.run(
        [str(SCRIPTS_DIR / "record.sh"), "-c", str(make_config(overrides={"namespace": ""})), "-o", str(output_dir_)],
        env=ros_env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result_.returncode == 1
    assert "namespace" in result_.stderr
    assert not output_dir_.exists()


def test_export_requires_a_format(tmp_path: Path, ros_env: dict[str, str]) -> None:
    """No implicit export: --format is required."""
    result_ = subprocess.run(
        [str(SCRIPTS_DIR / "export.sh"), str(tmp_path)], env=ros_env, capture_output=True, text=True, check=False
    )
    assert result_.returncode == 1
    assert "--format are required" in result_.stderr
