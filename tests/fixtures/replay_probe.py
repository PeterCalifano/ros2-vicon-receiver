"""Test fixture: publish a probe recording (scripts/probe.sh JSON) as ViconFrame messages.

Replays the recorded frames on /vicon/frame, in a loop, at the recorded rate, with current stamps,
so that the live viewer (scripts/start_visualizer.sh) can be checked without a streaming Tracker.
Units are converted like the node does (mm -> m); ROS names with the node's rule.

Usage (ROS 2 + workspace sourced, e.g. source scripts/source_ros_env.sh):
    python3 tests/fixtures/replay_probe.py probe_output/<recording>.json [--seconds S]
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time
from vicon_receiver_msgs.msg import MarkerState, SegmentState, SubjectState, ViconFrame


def Ros_name(name: str) -> str:
    """ROS name token of a Vicon name (as vicon_receiver::to_ros_name_token)."""
    token_ = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return token_ if token_[:1].isalpha() else "vicon" + token_


def To_time(seconds: float) -> Time:
    """builtin_interfaces/Time from seconds."""
    whole_ = int(seconds)
    return Time(sec=whole_, nanosec=min(int(round((seconds - whole_) * 1e9)), 999_999_999))


def Make_frame(frame: dict[str, Any]) -> ViconFrame:
    """ViconFrame of one recorded frame, stamped now."""
    message_ = ViconFrame()
    now_ = time.time()
    message_.header.frame_id = "vicon"
    message_.header.stamp = To_time(now_ - frame.get("latency_total_s", 0.0))
    message_.receive_stamp = To_time(now_)
    message_.frame_number = int(frame["frame_number"])
    message_.frame_rate_hz = float(frame.get("frame_rate_hz", 100.0))
    message_.latency_total_s = float(frame.get("latency_total_s", 0.0))
    for subject_ in frame["subjects"]:
        state_ = SubjectState(name=subject_["name"], ros_name=Ros_name(subject_["name"]))
        if subject_.get("quality") is not None:
            state_.quality_valid = True
            state_.quality = float(subject_["quality"])
        for segment_ in subject_["segments"]:
            seg_ = SegmentState(name=segment_["name"], frame_id=f"{state_.ros_name}_{Ros_name(segment_['name'])}")
            seg_.position.x, seg_.position.y, seg_.position.z = [float(v) / 1000.0 for v in segment_["translation_mm"]]
            seg_.orientation.x, seg_.orientation.y, seg_.orientation.z, seg_.orientation.w = [float(v) for v in segment_["rotation_xyzw"]]
            seg_.translation_occluded = bool(segment_["translation_occluded"])
            seg_.rotation_occluded = bool(segment_["rotation_occluded"])
            state_.segments.append(seg_)
            message_.any_occluded |= seg_.translation_occluded or seg_.rotation_occluded
        for marker_ in subject_.get("markers", []):
            mark_ = MarkerState(name=marker_["name"], parent_segment=subject_["segments"][0]["name"])
            if "position_mm" in marker_:
                mark_.position.x, mark_.position.y, mark_.position.z = [float(v) / 1000.0 for v in marker_["position_mm"]]
            mark_.occluded = bool(marker_.get("occluded", True))
            mark_.camera_ids = [int(c) for c in marker_.get("camera_ids", [])]
            state_.markers.append(mark_)
        message_.subjects.append(state_)
    return message_


def Main(argv: list[str]) -> int:
    """Replay until --seconds have passed or Ctrl+C."""
    parser_ = argparse.ArgumentParser(description="Replay a probe recording as ViconFrame messages.")
    parser_.add_argument("recording", type=Path, help="probe JSON (scripts/probe.sh)")
    parser_.add_argument("--seconds", type=float, default=3600.0, help="how long to replay [s]")
    arguments_ = parser_.parse_args(argv)
    document_ = json.loads(arguments_.recording.read_text(encoding="utf-8"))
    frames_ = [frame_ for frame_ in document_["frames"] if frame_.get("subjects")]
    if not frames_:
        print(f"no frames with subjects in {arguments_.recording}", file=sys.stderr)
        return 1
    rclpy.init()
    node_ = rclpy.create_node("vicon_probe_replay")
    publisher_ = node_.create_publisher(ViconFrame, "/vicon/frame", 100)
    period_ = 1.0 / float(frames_[0].get("frame_rate_hz", 100.0))
    start_ = time.monotonic()
    index_ = 0
    while rclpy.ok() and time.monotonic() - start_ < arguments_.seconds:
        publisher_.publish(Make_frame(frames_[index_ % len(frames_)]))
        index_ += 1
        delay_ = start_ + index_ * period_ - time.monotonic()
        if delay_ > 0:
            time.sleep(delay_)
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(Main(sys.argv[1:]))
