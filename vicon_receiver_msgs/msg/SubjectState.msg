# One Vicon subject (tracked object) in one frame.

# Subject name as streamed by Vicon (may contain characters such as '-')
string name

# Name converted for ROS topics and frames (vicon_receiver::to_ros_name_token)
string ros_name

# Object quality reported by Vicon Tracker (GetObjectQuality). It is a Tracker metric whose
# range is NOT 0..1 (about 1.3 on the Tracker used for testing): compare values, do not threshold blindly.
# quality_valid is false if the server did not provide it for this frame.
bool quality_valid
float64 quality

# Segments of the subject (one for a rigid body), with their occlusion mask
SegmentState[] segments

# Labeled markers of the subject (empty if marker data is disabled)
MarkerState[] markers
