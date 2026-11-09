"""Live test of the vicon_client node against a real Vicon server (opt-in).

Runs only when VICON_LIVE_HOST is set, e.g.
    VICON_LIVE_HOST=192.0.2.10 python3 -m pytest tests -m vicon_live

The node runs through scripts/launch.sh with the repository config (hostname replaced); a
subscriber in another process receives ViconFrame messages and checks their content.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest
from conftest import SCRIPTS_DIR, ConfigFactory, Stop_process_group

pytestmark = [pytest.mark.integration, pytest.mark.vicon_live]

# Receives ViconFrame for a few seconds and prints a JSON summary
SUBSCRIBER_SOURCE: str = textwrap.dedent(
    """
    import json, time
    import rclpy
    from vicon_receiver_msgs.msg import ViconFrame

    rclpy.init()
    node = rclpy.create_node("live_test_subscriber")
    frames = []
    node.create_subscription(ViconFrame, "/vicon/frame", frames.append, 1000)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    last = frames[-1] if frames else None
    print(json.dumps({
        "count": len(frames),
        "numbers_increasing": all(
            b.frame_number > a.frame_number for a, b in zip(frames, frames[1:])
        ),
        "stamps_before_receive": all(
            seconds(f.header.stamp) < seconds(f.receive_stamp) for f in frames
        ),
        "frame_id": last.header.frame_id if last else None,
        "subjects": [s.name for s in last.subjects] if last else [],
        "segment_frame_ids": (
            [g.frame_id for s in last.subjects for g in s.segments] if last else []
        ),
        "markers": sum(len(s.markers) for s in last.subjects) if last else 0,
        "rate_hz": last.frame_rate_hz if last else 0.0,
    }))
    rclpy.shutdown()
    """
)


def test_node_publishes_vicon_frames(
    ros_env: dict[str, str], make_config: ConfigFactory, tmp_path: Path
) -> None:
    """The node streams ViconFrame with subjects, segments and markers, and prints telemetry."""
    live_host_ = os.environ.get("VICON_LIVE_HOST")
    if not live_host_:
        pytest.skip("set VICON_LIVE_HOST=host[:port] to test against a real Vicon server")

    config_ = make_config(overrides={"hostname": live_host_})
    log_path_ = tmp_path / "node.log"
    with log_path_.open("w", encoding="utf-8") as log_:
        node_ = subprocess.Popen(
            [str(SCRIPTS_DIR / "launch.sh"), "-c", str(config_)],
            env=ros_env,
            stdout=log_,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        time.sleep(3.0)
        subscriber_ = subprocess.run(
            [sys.executable, "-c", SUBSCRIBER_SOURCE],
            env=ros_env,
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
    finally:
        Stop_process_group(node_)

    summary_ = json.loads(subscriber_.stdout.strip().splitlines()[-1])
    assert summary_["count"] > 100, summary_
    assert summary_["numbers_increasing"], summary_
    assert summary_["stamps_before_receive"], summary_
    assert summary_["frame_id"] == "vicon"
    assert summary_["subjects"], summary_
    assert summary_["segment_frame_ids"], summary_
    assert summary_["markers"] > 0, summary_
    assert summary_["rate_hz"] > 0.0

    # Console telemetry: at least one status line
    node_output_ = log_path_.read_text(encoding="utf-8")
    assert " Hz | dropped " in node_output_, node_output_[-2000:]
