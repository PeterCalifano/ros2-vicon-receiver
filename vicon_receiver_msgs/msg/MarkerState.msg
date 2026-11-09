# One labeled marker of a subject in one frame.

# Marker name as streamed by Vicon
string name

# Segment the marker is attached to (GetMarkerParentName)
string parent_segment

# Reconstructed position in the Vicon world frame [m] (GetMarkerGlobalTranslation, from mm)
geometry_msgs/Point position

# True if Vicon flagged the marker as occluded (position not reconstructed in this frame)
bool occluded

# IDs of the cameras whose rays reconstructed the marker (GetMarkerRayContribution); empty if
# ray data is disabled. The count is a direct visibility measure (3..7 cameras observed live).
uint32[] camera_ids
