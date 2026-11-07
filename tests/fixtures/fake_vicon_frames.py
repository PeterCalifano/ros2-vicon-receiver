"""Test fixture: publish synthetic ViconFrame messages, like the vicon_client node.

Used by tests/integration/test_record_export.py to test recording and export without a Vicon
system. The node is called vicon_client (scripts/record.sh checks that it runs) and publishes on
/vicon/frame a known pattern:

- frame numbers from FIRST_FRAME_NUMBER, consecutive, except a jump of DROPPED_FRAMES + 1 at
  frame index DROP_AT_INDEX (frames_dropped = DROPPED_FRAMES);
- one subject "test-body" with one segment moving along x by 1 mm per frame number;
- the segment is occluded (all-zero pose, flags set) when the frame number is a multiple of
  OCCLUSION_PERIOD;
- three markers seen by 3, 4 and 5 cameras, occluded with the segment.

Usage: python3 fake_vicon_frames.py --duration SECONDS [--rate HZ]
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from vicon_receiver_msgs.msg import MarkerState, SegmentState, SubjectState, ViconFrame

FIRST_FRAME_NUMBER: int = 1000
DROP_AT_INDEX: int = 400
DROPPED_FRAMES: int = 3
OCCLUSION_PERIOD: int = 50
LATENCY_S: float = 0.030


def To_time(seconds: float) -> Time:
    """builtin_interfaces/Time from seconds."""
    whole_ = int(seconds)
    return Time(sec=whole_, nanosec=int(round((seconds - whole_) * 1e9)) % 1_000_000_000)


def Frame_number(index: int) -> int:
    """Frame number of the index-th published frame (with the injected gap)."""
    gap_ = DROPPED_FRAMES if index >= DROP_AT_INDEX else 0
    return FIRST_FRAME_NUMBER + index + gap_


def Make_frame(index: int, now_s: float) -> ViconFrame:
    """Synthetic ViconFrame number index."""
    frame_number_ = Frame_number(index)
    occluded_ = frame_number_ % OCCLUSION_PERIOD == 0

    segment_ = SegmentState(name="test-body", frame_id="test_body_test_body")
    if not occluded_:
        segment_.position.x = 0.001 * frame_number_
        segment_.position.z = 1.0
        segment_.orientation.w = 1.0
    else:
        # What Vicon sends for an occluded segment: all zeros, flagged
        segment_.orientation.w = 0.0
    segment_.translation_occluded = occluded_
    segment_.rotation_occluded = occluded_

    markers_ = []
    for marker_index_, cameras_ in enumerate((3, 4, 5)):
        marker_ = MarkerState(name=f"test-body{marker_index_ + 1}", parent_segment="test-body")
        marker_.occluded = occluded_
        if not occluded_:
            marker_.position.x = segment_.position.x + 0.05 * marker_index_
            marker_.position.z = 1.0
            marker_.camera_ids = list(range(100, 100 + cameras_))
        markers_.append(marker_)

    subject_ = SubjectState(
        name="test-body", ros_name="test_body", quality_valid=True, quality=1.25, segments=[segment_], markers=markers_
    )

    frame_ = ViconFrame(subjects=[subject_])
    frame_.header.frame_id = "vicon"
    frame_.header.stamp = To_time(now_s - LATENCY_S)
    frame_.receive_stamp = To_time(now_s)
    frame_.frame_number = frame_number_
    frame_.frames_dropped = DROPPED_FRAMES if index == DROP_AT_INDEX else 0
    frame_.frame_rate_hz = 100.0
    frame_.latency_total_s = LATENCY_S
    frame_.latency_stage_names = ["Data Delivered"]
    frame_.latency_stage_values_s = [LATENCY_S]
    frame_.any_occluded = occluded_
    return frame_


def Main(argv: list[str]) -> int:
    """Publish for the given duration and print the number of frames."""
    parser_ = argparse.ArgumentParser(description="Publish synthetic ViconFrame messages.")
    parser_.add_argument("--duration", type=float, required=True, help="seconds to publish")
    parser_.add_argument("--rate", type=float, default=100.0, help="frames per second")
    arguments_ = parser_.parse_args(argv)

    rclpy.init()
    node_ = rclpy.create_node("vicon_client")
    publisher_ = node_.create_publisher(ViconFrame, "/vicon/frame", 100)

    period_s_ = 1.0 / arguments_.rate
    start_ = time.monotonic()
    index_ = 0
    while time.monotonic() - start_ < arguments_.duration:
        publisher_.publish(Make_frame(index_, time.time()))
        index_ += 1
        # Keep the rate on average, whatever the time spent publishing
        sleep_s_ = start_ + index_ * period_s_ - time.monotonic()
        if sleep_s_ > 0:
            time.sleep(sleep_s_)

    print(f"published {index_} frames")
    node_.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(Main(sys.argv[1:]))
