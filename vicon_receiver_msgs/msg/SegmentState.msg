# Global pose of one segment of a subject in one frame, as streamed by Vicon.
# When occluded, Vicon sends an all-zero pose (orientation [0,0,0,0], not a valid rotation):
# it is kept as is, flagged by the occlusion mask below.

# Segment name as streamed by Vicon
string name

# TF child frame of the segment (vicon_receiver::make_segment_frame_id), "<subject>_<segment>"
string frame_id

# Position in the Vicon world frame [m] (GetSegmentGlobalTranslation, converted from mm)
geometry_msgs/Point position

# Orientation in the Vicon world frame, quaternion x, y, z, w as streamed
# (GetSegmentGlobalRotationQuaternion)
geometry_msgs/Quaternion orientation

# Occlusion mask: true if Vicon flagged the translation / the rotation as occluded
bool translation_occluded
bool rotation_occluded
