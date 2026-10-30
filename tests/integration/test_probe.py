"""Integration tests of the vicon_probe executable and of scripts/probe.sh.

Without a Vicon system, the tests cover the command line and every failure path: a local TCP
listener stands in for a server that accepts connections but does not speak the DataStream
protocol, and 192.0.2.1 (RFC 5737) for an unreachable one.

The test marked ``vicon_live`` records a few frames from a real server; it runs only when
VICON_LIVE_HOST is set, e.g. ``VICON_LIVE_HOST=10.250.223.40 python3 -m pytest tests -m vicon_live``.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any

import pytest

from conftest import DEFAULT_TIMEOUT_S, PROBE_EXECUTABLE, ShellRunner, TcpListener

pytestmark = pytest.mark.integration

# RFC 5737 documentation address: guaranteed not to host a Vicon server
UNREACHABLE_ADDRESS: str = "192.0.2.1"

# Exit codes of vicon_probe (see vicon_receiver/include/vicon_receiver/probe/vicon_probe.hpp)
EXIT_INVALID_ARGUMENTS: int = 1
EXIT_CONNECTION_FAILED: int = 2
EXIT_OUTPUT_ERROR: int = 3


def Run_probe(args: list[str], env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    """Run vicon_probe with the given arguments and capture its output."""
    return subprocess.run(
        [str(PROBE_EXECUTABLE), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )


def Load_recording(path: Path) -> dict[str, Any]:
    """Parse a probe recording; fails the test if it is not valid JSON."""
    return json.loads(path.read_text(encoding="utf-8"))


# -------------------------------------------------------------------------------------------------
# vicon_probe command line
# -------------------------------------------------------------------------------------------------
def test_probe_help(ros_env: dict[str, str]) -> None:
    """--help prints the usage, including the exit codes, and succeeds."""
    result_ = Run_probe(["--help"], ros_env)
    assert result_.returncode == 0
    assert result_.stdout.startswith("Usage: vicon_probe --host HOST[:PORT] --frames N")
    assert "Exit codes:" in result_.stdout


def test_probe_requires_host(ros_env: dict[str, str]) -> None:
    """Missing required options are reported with the usage, before connecting."""
    result_ = Run_probe(["--frames", "10"], ros_env)
    assert result_.returncode == EXIT_INVALID_ARGUMENTS
    assert "vicon_probe: --host is required" in result_.stderr
    assert "Usage:" in result_.stderr


def test_probe_rejects_unwritable_output(ros_env: dict[str, str], tmp_path: Path) -> None:
    """An output file that cannot be created is reported before connecting."""
    output_ = tmp_path / "missing_dir" / "recording.json"
    result_ = Run_probe(["--host", "127.0.0.1", "--frames", "1", "--json", str(output_)], ros_env)
    assert result_.returncode == EXIT_OUTPUT_ERROR
    assert f"cannot open '{output_}'" in result_.stderr
    assert "Connecting" not in result_.stdout


# -------------------------------------------------------------------------------------------------
# Connection failures
# -------------------------------------------------------------------------------------------------
def test_probe_non_vicon_server_fails_with_sdk_result(
    ros_env: dict[str, str], tcp_listener: TcpListener, tmp_path: Path
) -> None:
    """A server that accepts TCP but is not DataStream: the SDK result is reported and recorded."""
    output_ = tmp_path / "recording.json"
    result_ = Run_probe(
        [
            "--host",
            tcp_listener.host_spec,
            "--frames",
            "3",
            "--timeout-ms",
            "500",
            "--json",
            str(output_),
        ],
        ros_env,
    )
    assert result_.returncode == EXIT_CONNECTION_FAILED
    assert f"Connection to {tcp_listener.host_spec} failed: ClientConnectionFailed" in result_.stdout

    # The recording documents the failed attempt and is still valid JSON
    recording_ = Load_recording(output_)
    assert recording_["probe"]["host"] == tcp_listener.host_spec
    assert recording_["probe"]["requested_frames"] == 3
    assert recording_["connection"]["result"] == "ClientConnectionFailed"
    assert recording_["connection"]["timeout_ms"] == 500
    assert "frames" not in recording_


def test_probe_sh_stops_when_server_unreachable(shell: ShellRunner, tmp_path: Path) -> None:
    """probe.sh applies the same reachability check as launch.sh and does not run the probe."""
    output_ = tmp_path / "recording.json"
    result_ = shell.run_script(
        "probe.sh",
        ["--host", UNREACHABLE_ADDRESS, "-o", str(output_)],
        extra_env={"VICON_CHECK_TIMEOUT_S": "1"},
    )
    assert result_.returncode == 1
    assert "Vicon Tracker PC" in result_.stderr
    assert UNREACHABLE_ADDRESS in result_.stderr
    assert "Recording" not in result_.stdout
    assert not output_.exists()


def test_probe_sh_passes_options_to_probe(
    shell: ShellRunner, tcp_listener: TcpListener, tmp_path: Path
) -> None:
    """probe.sh forwards host, frames, timeout and output to vicon_probe."""
    output_ = tmp_path / "recording.json"
    result_ = shell.run_script(
        "probe.sh",
        ["--host", tcp_listener.host_spec, "-n", "7", "--timeout-ms", "300", "-o", str(output_)],
    )
    assert result_.returncode == EXIT_CONNECTION_FAILED
    assert f"Recording 7 frame(s) from {tcp_listener.host_spec}" in result_.stdout

    recording_ = Load_recording(output_)
    assert recording_["probe"]["requested_frames"] == 7
    assert recording_["connection"]["timeout_ms"] == 300


def test_probe_sh_reads_host_from_config(
    shell: ShellRunner, make_config: Any, tcp_listener: TcpListener, tmp_path: Path
) -> None:
    """Without --host, the server address comes from the parameters file."""
    config_ = make_config(overrides={"hostname": tcp_listener.host_spec})
    output_ = tmp_path / "recording.json"
    result_ = shell.run_script(
        "probe.sh", ["-c", str(config_), "-n", "1", "--timeout-ms", "300", "-o", str(output_)]
    )
    assert result_.returncode == EXIT_CONNECTION_FAILED
    assert Load_recording(output_)["probe"]["host"] == tcp_listener.host_spec


# -------------------------------------------------------------------------------------------------
# Real Vicon server (opt-in)
# -------------------------------------------------------------------------------------------------
@pytest.mark.vicon_live
def test_probe_records_live_frames(shell: ShellRunner, tmp_path: Path) -> None:
    """Record a few frames from a real server and check the structure of the recording."""
    live_host_ = os.environ.get("VICON_LIVE_HOST")
    if not live_host_:
        pytest.skip("set VICON_LIVE_HOST=host[:port] to test against a real Vicon server")

    frame_count_ = 20
    output_ = tmp_path / "live.json"
    result_ = shell.run_script(
        "probe.sh", ["--host", live_host_, "-n", str(frame_count_), "-o", str(output_)]
    )
    assert result_.returncode == 0, result_.stdout + result_.stderr

    recording_ = Load_recording(output_)
    assert recording_["connection"]["result"] == "Success"
    assert len(recording_["frames"]) == frame_count_
    assert recording_["summary"]["frames_recorded"] == frame_count_

    for frame_ in recording_["frames"]:
        assert "frame_number" in frame_
        assert "latency_total_s" in frame_
        for subject_ in frame_["subjects"]:
            for segment_ in subject_["segments"]:
                assert len(segment_["translation_mm"]) == 3
                assert len(segment_["rotation_xyzw"]) == 4
