# One frame of the Vicon DataStream server, with everything the receiver reads from it.
# Published once per received frame, as soon as it arrives; nothing is filtered out:
# occluded segments are included with their occlusion flags (consumers decide).

# stamp: estimated camera ACQUISITION time of the frame on the local clock
#   = t_ref + (frame_number - frame_ref) / frame_rate_hz, with t_ref estimated from
#   receive_stamp - latency_total_s over a sliding window (see vicon_receiver::AcquisitionClock).
# frame_id: the Vicon world frame (vicon_frame parameter); positions are expressed in it.
std_msgs/Header header

# Local time at which the frame was received (raw, so the stamp can be re-estimated offline)
builtin_interfaces/Time receive_stamp

# Frame number reported by the server (GetFrameNumber): consecutive at frame_rate_hz
uint64 frame_number

# Frames missing since the previous message (gap in frame_number; 0 if consecutive)
uint32 frames_dropped

# Frame rate reported by the server (GetFrameRate) [Hz]
float64 frame_rate_hz

# Latency of the server pipeline, camera to DataStream (GetLatencyTotal) [s]
float64 latency_total_s

# Latency of each pipeline stage (GetLatencySampleName/Value), e.g. "Data Delivered" [s]
string[] latency_stage_names
float64[] latency_stage_values_s

# True if at least one segment of one subject is occluded in this frame
bool any_occluded

# Every subject of the frame
SubjectState[] subjects
