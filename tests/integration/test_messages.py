"""Integration test of the vicon_receiver_msgs interfaces (generated Python types).

The message layout is an interface of recordings and of the estimator: this test pins the field
names and types, so a change to them is a deliberate one.
"""

from __future__ import annotations

import json
import subprocess
import sys
import textwrap

import pytest
from conftest import DEFAULT_TIMEOUT_S

pytestmark = pytest.mark.integration

# Expected fields (name -> ROS type) of each message
EXPECTED_FIELDS: dict[str, dict[str, str]] = {
    "ViconFrame": {
        "header": "std_msgs/Header",
        "receive_stamp": "builtin_interfaces/Time",
        "frame_number": "uint64",
        "frames_dropped": "uint32",
        "frame_rate_hz": "double",
        "latency_total_s": "double",
        "latency_stage_names": "sequence<string>",
        "latency_stage_values_s": "sequence<double>",
        "any_occluded": "boolean",
        "subjects": "sequence<vicon_receiver_msgs/SubjectState>",
    },
    "SubjectState": {
        "name": "string",
        "ros_name": "string",
        "quality_valid": "boolean",
        "quality": "double",
        "segments": "sequence<vicon_receiver_msgs/SegmentState>",
        "markers": "sequence<vicon_receiver_msgs/MarkerState>",
    },
    "SegmentState": {
        "name": "string",
        "frame_id": "string",
        "position": "geometry_msgs/Point",
        "orientation": "geometry_msgs/Quaternion",
        "translation_occluded": "boolean",
        "rotation_occluded": "boolean",
    },
    "MarkerState": {
        "name": "string",
        "parent_segment": "string",
        "position": "geometry_msgs/Point",
        "occluded": "boolean",
        "camera_ids": "sequence<uint32>",
    },
}

# Prints the fields of every message as JSON, in the ROS environment
INTROSPECTION_SOURCE: str = textwrap.dedent(
    """
    import json
    import vicon_receiver_msgs.msg as messages

    names = ["ViconFrame", "SubjectState", "SegmentState", "MarkerState"]
    fields = {name: getattr(messages, name).get_fields_and_field_types() for name in names}
    print(json.dumps(fields))
    """
)


def test_message_fields(ros_env: dict[str, str]) -> None:
    """Every message has exactly the expected fields and types."""
    result_ = subprocess.run(
        [sys.executable, "-c", INTROSPECTION_SOURCE],
        env=ros_env,
        capture_output=True,
        text=True,
        timeout=DEFAULT_TIMEOUT_S,
        check=False,
    )
    assert result_.returncode == 0, result_.stderr
    fields_ = json.loads(result_.stdout)

    for message_name_, expected_ in EXPECTED_FIELDS.items():
        assert fields_[message_name_] == expected_, message_name_
