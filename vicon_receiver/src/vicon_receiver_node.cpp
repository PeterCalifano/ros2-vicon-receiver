/**
 * @file vicon_receiver_node.cpp
 * @author OPT4SMART group (original), PeterC (petercalifano.gs@gmail.com) (modifications)
 * @brief ViconReceiverNode: receives Vicon frames and publishes one ViconFrame per frame, the
 *        valid segment poses as PoseStamped and TF, and a console status line. The executable
 *        entry point is in main.cpp.
 * @version 1.2.0
 * @date 2026-10-30
 */

#include "vicon_receiver/vicon_receiver_node.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "vicon_receiver/ros_conversions.hpp"
#include "vicon_receiver/utilities.hpp"

using namespace ViconDataStreamSDK::CPP;

namespace
{
    /// Depth of the ViconFrame publisher queue (1 s at 100 Hz): a slow subscriber, e.g. a
    /// recorder, gets every frame instead of only the latest.
    constexpr std::size_t kFrameQueueDepth = 100;

    /// Disagreement between the local clock and the Vicon frame clock that triggers a warning
    /// (healthy clocks differ by a few ppm; 1 % means a broken local clock).
    constexpr double kClockRateErrorLimit = 0.01;

    /// Tolerance when deciding whether a frame-number jump can be made of dropped frames [s].
    constexpr double kStaleFrameMarginS = 1.0;

    /**
     * @brief Check that a vector-valued parameter has the expected number of elements.
     *
     * @param name           Parameter name, used in the error message.
     * @param values         Parameter value read from the parameters file.
     * @param expected_size  Required number of elements.
     * @throws std::invalid_argument if the size does not match.
     */
    void require_vector_size(const std::string &name,
                             const std::vector<double> &values,
                             const std::size_t expected_size)
    {
        if (values.size() == expected_size)
        {
            return;
        }

        const std::string message = "parameter '" + name + "' must have " +
                                    std::to_string(expected_size) + " elements, got " +
                                    std::to_string(values.size());
        throw std::invalid_argument(message);
    }

    /// Whether the translation or the rotation of a segment is flagged as occluded.
    bool is_occluded(const vicon_receiver_msgs::msg::SegmentState &segment)
    {
        return segment.translation_occluded || segment.rotation_occluded;
    }
} // namespace

// Constructor for the ViconReceiverNode class
ViconReceiverNode::ViconReceiverNode(const rclcpp::NodeOptions &options) : Node(kNodeName, options)
{
    // Parameters are declared WITHOUT default values on purpose: the parameters file
    // (config/vicon_client.yaml) is the single source of truth. If an entry is missing or
    // has the wrong type, rclcpp throws rclcpp::exceptions::InvalidParameterTypeException,
    // reported in main().
    this->declare_parameter<std::string>("hostname");
    this->declare_parameter<int>("buffer_size");
    this->declare_parameter<std::string>("namespace");
    this->declare_parameter<std::string>("world_frame");
    this->declare_parameter<std::string>("vicon_frame");
    this->declare_parameter<std::vector<double>>("map_xyz");
    this->declare_parameter<std::vector<double>>("map_rpy");
    this->declare_parameter<bool>("map_rpy_in_degrees");

    // Retrieve parameters values
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);

    this->get_parameter("world_frame", world_frame);
    this->get_parameter("vicon_frame", vicon_frame);
    this->get_parameter("map_xyz", map_xyz);
    this->get_parameter("map_rpy", map_rpy);
    this->get_parameter("map_rpy_in_degrees", map_rpy_in_degrees);

    // The static map -> vicon transform needs exactly one value per axis
    require_vector_size("map_xyz", map_xyz, 3);
    require_vector_size("map_rpy", map_rpy, 3);

    // Streaming, stamping and telemetry parameters
    this->read_frame_parameters();

    // Publish static transform from map to vicon origin
    if (map_rpy_in_degrees)
    {
        for (unsigned int i = 0; i < map_rpy.size(); i++)
        {
            map_rpy[i] = map_rpy[i] * M_PI / 180.0;
        }
    }
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_static_transform();

    // Initialize the tf2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // One ViconFrame per received frame, reliable, so that recorders do not lose frames.
    // Qualified: ViconReceiverNode::create_publisher(subject, segment) hides the Node template.
    frame_publisher_ = rclcpp::Node::create_publisher<vicon_receiver_msgs::msg::ViconFrame>(
        ns_name + "/frame", rclcpp::QoS(rclcpp::KeepLast(kFrameQueueDepth)).reliable());

    status_period_start = std::chrono::steady_clock::now();
}

// Read the parameters added for ViconFrame, acquisition stamps and telemetry
void ViconReceiverNode::read_frame_parameters()
{
    // Required like every other parameter (no defaults)
    this->declare_parameter<std::string>("stream_mode");
    this->declare_parameter<bool>("enable_marker_data");
    this->declare_parameter<bool>("enable_marker_ray_data");
    this->declare_parameter<int>("acquisition_stamp_window_frames");
    this->declare_parameter<double>("status_period_s");

    // Stream mode, by name as in the parameters file
    const std::string stream_mode_text = this->get_parameter("stream_mode").as_string();
    try
    {
        stream_mode = vicon_receiver::stream_mode_from_name(stream_mode_text);
    }
    catch (const std::invalid_argument &error)
    {
        throw std::invalid_argument("parameter 'stream_mode': " + std::string(error.what()));
    }

    // Marker data; the cameras of a marker can only be read with the marker itself
    enable_marker_data = this->get_parameter("enable_marker_data").as_bool();
    enable_marker_ray_data = this->get_parameter("enable_marker_ray_data").as_bool();
    if (enable_marker_ray_data && !enable_marker_data)
    {
        throw std::invalid_argument(
            "parameter 'enable_marker_ray_data' is true but 'enable_marker_data' is false: "
            "the cameras of a marker are read with the marker");
    }

    // Window of the acquisition-time estimate, in frames
    const std::int64_t window_frames =
        this->get_parameter("acquisition_stamp_window_frames").as_int();
    if (window_frames < 1)
    {
        throw std::invalid_argument(
            "parameter 'acquisition_stamp_window_frames' must be >= 1, got " +
            std::to_string(window_frames));
    }
    acquisition_clock =
        std::make_unique<vicon_receiver::AcquisitionClock>(static_cast<std::size_t>(window_frames));

    // Console status line
    status_period_s = this->get_parameter("status_period_s").as_double();
    if (!std::isfinite(status_period_s) || status_period_s < 0.0)
    {
        throw std::invalid_argument("parameter 'status_period_s' must be >= 0 (0 disables the "
                                    "status line), got " +
                                    std::to_string(status_period_s));
    }
}

// Publish the static transform from map to vicon origin
void ViconReceiverNode::publish_static_transform()
{
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = world_frame;
    static_tf.child_frame_id = vicon_frame;

    static_tf.transform.translation.x = map_xyz[0];
    static_tf.transform.translation.y = map_xyz[1];
    static_tf.transform.translation.z = map_xyz[2];
    tf2::Quaternion q;
    q.setRPY(map_rpy[0], map_rpy[1], map_rpy[2]);
    static_tf.transform.rotation.x = q.x();
    static_tf.transform.rotation.y = q.y();
    static_tf.transform.rotation.z = q.z();
    static_tf.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(static_tf);

    string msg = "Published static transform from " + world_frame + " to " + vicon_frame;
    cout << msg << endl;
}

// Connect to the Vicon server
bool ViconReceiverNode::connect()
{
    // Log connection attempt
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;

    int counter = 0;
    // Retry connection until successful
    while (!vicon_client.IsConnected().Connected && rclcpp::ok())
    {
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
        }
    }
    if (!rclcpp::ok())
    {
        std::cout << "Shutdown requested before connection established." << std::endl;
        return false;
    }

    // Log successful connection
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    // Enable the data the node uses: segments always, markers and their cameras on request
    vicon_client.EnableSegmentData();
    if (enable_marker_data)
    {
        vicon_client.EnableMarkerData();
    }
    if (enable_marker_ray_data)
    {
        vicon_client.EnableMarkerRayData();
    }

    // Stream mode from the parameters file (ServerPush: every frame is delivered) and buffer size
    vicon_client.SetStreamMode(stream_mode);
    vicon_client.SetBufferSize(buffer_size);

    // Log initialization completion
    msg = "Initialization complete (stream mode " + vicon_receiver::stream_mode_name(stream_mode) +
          ")";
    cout << msg << endl;

    return true;
}

// Disconnect from the Vicon server
bool ViconReceiverNode::disconnect()
{
    // If already disconnected, return true
    if (!vicon_client.IsConnected().Connected)
        return true;

    sleep(1); // Wait before disconnecting

    // Disable the data streams enabled in connect()
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableMarkerRayData();

    // Log disconnection attempt
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;

    // Disconnect from the server
    vicon_client.Disconnect();

    // Log successful disconnection
    msg = "Successfully disconnected";
    cout << msg << endl;

    // Verify disconnection
    return !vicon_client.IsConnected().Connected;
}

// Receive one frame and publish it
void ViconReceiverNode::get_frame()
{
    // Waits for the next frame (ServerPush) or requests the latest one (ClientPull)
    const Output_GetFrame frame_result = vicon_client.GetFrame();

    // Taken right after the frame arrived: input of the acquisition-time estimate
    const rclcpp::Time receive_time = this->get_clock()->now();

    if (frame_result.Result != Result::Success)
    {
        if (frame_result.Result == Result::NotConnected)
        {
            RCLCPP_WARN(this->get_logger(), "Connection to the Vicon server lost: reconnecting");
            this->connect();
        }
        else if (frame_result.Result != Result::NoFrame)
        {
            // NoFrame only means that no frame is available yet (right after connecting)
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "GetFrame failed: %s",
                                 vicon_receiver::result_name(frame_result.Result).c_str());
        }
        return;
    }

    vicon_receiver_msgs::msg::ViconFrame frame_msg;
    this->read_frame_header(receive_time, frame_msg);
    this->read_subjects(frame_msg);
    frame_publisher_->publish(frame_msg);
    this->update_status(frame_msg);
}

// Frame numbers, rate, latency and stamps of the current frame
void ViconReceiverNode::read_frame_header(const rclcpp::Time &receive_time,
                                          vicon_receiver_msgs::msg::ViconFrame &frame_msg)
{
    frame_msg.header.frame_id = vicon_frame;
    frame_msg.receive_stamp = receive_time;

    const Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();
    const bool frame_number_ok = frame_number.Result == Result::Success;

    const Output_GetFrameRate frame_rate = vicon_client.GetFrameRate();
    const bool frame_rate_ok = frame_rate.Result == Result::Success && frame_rate.FrameRateHz > 0.0;

    // A jump larger than what the server can have produced since the previous frame means that
    // the previous frame was stale (a Vicon server first sends the frame left over from a
    // previous connection): not dropped frames, but a new timeline
    const std::optional<std::uint64_t> previous_frame = dropped_frames.last_frame_number();
    if (frame_number_ok && frame_rate_ok && previous_frame.has_value() &&
        frame_number.FrameNumber > previous_frame.value() && previous_receive_s.has_value())
    {
        const std::uint64_t frames_missing = frame_number.FrameNumber - previous_frame.value() - 1;
        const double receive_interval_s = receive_time.seconds() - previous_receive_s.value();
        if (vicon_receiver::is_implausible_frame_jump(
                frames_missing, receive_interval_s, frame_rate.FrameRateHz, kStaleFrameMarginS))
        {
            RCLCPP_INFO(this->get_logger(),
                        "Frame number jumped by %lu frames in %.3f s: the previous frame was "
                        "stale (left over from a previous connection); restarting the frame "
                        "timeline",
                        static_cast<unsigned long>(frames_missing),
                        receive_interval_s);
            dropped_frames.restart_sequence();
            acquisition_clock->reset();
        }
    }
    previous_receive_s = receive_time.seconds();

    if (frame_number_ok)
    {
        frame_msg.frame_number = frame_number.FrameNumber;
        const std::uint64_t missed = dropped_frames.record(frame_number.FrameNumber);
        const std::uint64_t max_reported = std::numeric_limits<std::uint32_t>::max();
        frame_msg.frames_dropped = static_cast<std::uint32_t>(std::min(missed, max_reported));
    }

    if (frame_rate_ok)
    {
        frame_msg.frame_rate_hz = frame_rate.FrameRateHz;
    }

    // Latency of the server pipeline (0 if not reported), and of each of its stages
    const Output_GetLatencyTotal latency = vicon_client.GetLatencyTotal();
    if (latency.Result == Result::Success)
    {
        frame_msg.latency_total_s = latency.Total;
    }
    const Output_GetLatencySampleCount sample_count = vicon_client.GetLatencySampleCount();
    if (sample_count.Result == Result::Success)
    {
        for (unsigned int index = 0; index < sample_count.Count; ++index)
        {
            const Output_GetLatencySampleName name = vicon_client.GetLatencySampleName(index);
            if (name.Result != Result::Success)
            {
                continue;
            }
            const Output_GetLatencySampleValue value =
                vicon_client.GetLatencySampleValue(name.Name);
            if (value.Result == Result::Success)
            {
                frame_msg.latency_stage_names.push_back(name.Name);
                frame_msg.latency_stage_values_s.push_back(value.Value);
            }
        }
    }

    // Stamp: estimated camera acquisition time (frame-number timeline, see AcquisitionClock)
    if (frame_number_ok && frame_rate_ok)
    {
        const double acquisition_s = acquisition_clock->update(frame_number.FrameNumber,
                                                               frame_rate.FrameRateHz,
                                                               receive_time.seconds(),
                                                               frame_msg.latency_total_s);
        frame_msg.header.stamp = vicon_receiver::to_time_msg(acquisition_s);
    }
    else
    {
        // Without frame number and rate there is no camera timeline: fall back to receive time
        if (!missing_rate_warned)
        {
            RCLCPP_WARN(this->get_logger(),
                        "No frame number or frame rate from the server: frames are stamped with "
                        "their receive time instead of the acquisition time");
            missing_rate_warned = true;
        }
        frame_msg.header.stamp = receive_time;
    }
}

// Every subject of the current frame; valid poses also go to PoseStamped and TF
void ViconReceiverNode::read_subjects(vicon_receiver_msgs::msg::ViconFrame &frame_msg)
{
    const Output_GetSubjectCount subject_count = vicon_client.GetSubjectCount();
    if (subject_count.Result != Result::Success)
    {
        return;
    }

    for (unsigned int subject_index = 0; subject_index < subject_count.SubjectCount;
         ++subject_index)
    {
        const std::string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        vicon_receiver_msgs::msg::SubjectState subject_msg;
        subject_msg.name = subject_name;
        subject_msg.ros_name = vicon_receiver::to_ros_name_token(subject_name);

        // Tracker quality metric (not normalised; see SubjectState.msg)
        const Output_GetObjectQuality quality = vicon_client.GetObjectQuality(subject_name);
        subject_msg.quality_valid = quality.Result == Result::Success;
        if (subject_msg.quality_valid)
        {
            subject_msg.quality = quality.Quality;
        }

        const Output_GetSegmentCount segment_count = vicon_client.GetSegmentCount(subject_name);
        const unsigned int segments =
            segment_count.Result == Result::Success ? segment_count.SegmentCount : 0;
        for (unsigned int segment_index = 0; segment_index < segments; ++segment_index)
        {
            const std::string segment_name =
                vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // Every segment goes into ViconFrame, occluded or not, with its occlusion mask
            vicon_receiver_msgs::msg::SegmentState segment_msg = vicon_receiver::make_segment_state(
                segment_name,
                vicon_receiver::make_segment_frame_id(subject_name, segment_name),
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name),
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name));

            if (is_occluded(segment_msg))
            {
                frame_msg.any_occluded = true;
            }

            // Only valid poses go to the live-monitoring outputs (no occlusion field there)
            if (vicon_receiver::is_valid_pose(segment_msg))
            {
                this->publish_segment_pose(subject_name, segment_msg, frame_msg.header.stamp);
            }
            subject_msg.segments.push_back(std::move(segment_msg));
        }

        if (enable_marker_data)
        {
            this->read_markers(subject_name, subject_msg);
        }
        frame_msg.subjects.push_back(std::move(subject_msg));
    }
}

// Labeled markers of one subject, with the cameras that saw them
void ViconReceiverNode::read_markers(const std::string &subject_name,
                                     vicon_receiver_msgs::msg::SubjectState &subject_msg)
{
    const Output_GetMarkerCount marker_count = vicon_client.GetMarkerCount(subject_name);
    if (marker_count.Result != Result::Success)
    {
        return;
    }

    for (unsigned int marker_index = 0; marker_index < marker_count.MarkerCount; ++marker_index)
    {
        const Output_GetMarkerName marker = vicon_client.GetMarkerName(subject_name, marker_index);
        if (marker.Result != Result::Success)
        {
            continue;
        }
        const std::string marker_name = marker.MarkerName;

        const Output_GetMarkerParentName parent =
            vicon_client.GetMarkerParentName(subject_name, marker_name);
        const std::string parent_name =
            parent.Result == Result::Success ? std::string(parent.SegmentName) : std::string();

        // Each ray contribution is one camera that saw the marker in this frame
        std::vector<std::uint32_t> camera_ids;
        if (enable_marker_ray_data)
        {
            const Output_GetMarkerRayContributionCount ray_count =
                vicon_client.GetMarkerRayContributionCount(subject_name, marker_name);
            const unsigned int rays =
                ray_count.Result == Result::Success ? ray_count.RayContributionsCount : 0;
            for (unsigned int ray_index = 0; ray_index < rays; ++ray_index)
            {
                const Output_GetMarkerRayContribution ray =
                    vicon_client.GetMarkerRayContribution(subject_name, marker_name, ray_index);
                if (ray.Result == Result::Success)
                {
                    camera_ids.push_back(ray.CameraID);
                }
            }
        }

        subject_msg.markers.push_back(vicon_receiver::make_marker_state(
            marker_name,
            parent_name,
            vicon_client.GetMarkerGlobalTranslation(subject_name, marker_name),
            camera_ids));
    }
}

// One valid segment pose on its PoseStamped topic (world frame) and on TF (Vicon frame)
void ViconReceiverNode::publish_segment_pose(const std::string &subject_name,
                                             const vicon_receiver_msgs::msg::SegmentState &segment,
                                             const builtin_interfaces::msg::Time &stamp)
{
    const std::string &segment_name = segment.name;

    // TF from the Vicon frame to the segment frame, stamped with the acquisition time
    const geometry_msgs::msg::TransformStamped tf_msg =
        vicon_receiver::make_segment_transform(segment, vicon_frame, stamp);

    // Publish the position data
    boost::mutex::scoped_try_lock lock(mutex);
    if (lock.owns_lock())
    {
        // Check if a publisher exists for the segment
        map<string, SegmentPosePublisher>::iterator pub_it =
            pub_map.find(subject_name + "/" + segment_name);
        if (pub_it != pub_map.end())
        {
            SegmentPosePublisher &pub = pub_it->second;

            if (pub.is_ready)
            {
                // Build a PoseStamped in the Vicon frame from the computed TransformStamped.
                geometry_msgs::msg::PoseStamped vicon_pose_msg;

                // Header: copy timestamp and frame_id ("vicon") from the transform header.
                vicon_pose_msg.header = tf_msg.header;

                // Position: copy the already meter-converted translation components.
                vicon_pose_msg.pose.position.x = tf_msg.transform.translation.x;
                vicon_pose_msg.pose.position.y = tf_msg.transform.translation.y;
                vicon_pose_msg.pose.position.z = tf_msg.transform.translation.z;

                // Orientation: copy the quaternion (x, y, z, w) directly from the transform.
                vicon_pose_msg.pose.orientation = tf_msg.transform.rotation;

                // Update timestamp of static transform
                static_tf.header.stamp = tf_msg.header.stamp;

                // Transform the pose to the global frame
                geometry_msgs::msg::PoseStamped global_pose_msg;
                tf2::doTransform(vicon_pose_msg, global_pose_msg, static_tf);

                // Publish the transformed pose
                pub.publish(global_pose_msg);
            }
        }
        else
        {
            // Create a publisher if it doesn't exist, de-duplicating concurrent attempts.
            // Segments whose publisher failed once are not retried (error already logged).
            std::string key = subject_name + "/" + segment_name;
            if (failed_publishers.find(key) != failed_publishers.end())
            {
                lock.unlock();
            }
            else if (pending_publishers.find(key) == pending_publishers.end())
            {
                pending_publishers.insert(key);
                lock.unlock();
                create_publisher(subject_name, segment_name);
            }
            else
            {
                // Another thread is already creating this publisher
                lock.unlock();
            }
        }
    }

    // Broadcast the transform
    tf_broadcaster_->sendTransform(tf_msg);
}

// Console telemetry: one line per status period
void ViconReceiverNode::update_status(const vicon_receiver_msgs::msg::ViconFrame &frame_msg)
{
    if (status_period_s <= 0.0)
    {
        return;
    }

    // The latency is 0 when the server did not report it
    std::optional<double> latency_s;
    if (frame_msg.latency_total_s > 0.0)
    {
        latency_s = frame_msg.latency_total_s;
    }
    std::optional<double> frame_rate_hz;
    if (frame_msg.frame_rate_hz > 0.0)
    {
        frame_rate_hz = frame_msg.frame_rate_hz;
    }
    console_status_line.add_frame(frame_msg.frames_dropped, latency_s, frame_rate_hz);

    for (const vicon_receiver_msgs::msg::SubjectState &subject : frame_msg.subjects)
    {
        bool occluded = false;
        for (const vicon_receiver_msgs::msg::SegmentState &segment : subject.segments)
        {
            occluded = occluded || is_occluded(segment);
        }

        // Cameras per visible marker (none when ray data is disabled)
        std::vector<std::size_t> camera_counts;
        for (const vicon_receiver_msgs::msg::MarkerState &marker : subject.markers)
        {
            if (!marker.occluded && !marker.camera_ids.empty())
            {
                camera_counts.push_back(marker.camera_ids.size());
            }
        }

        std::optional<double> quality;
        if (subject.quality_valid)
        {
            quality = subject.quality;
        }
        console_status_line.add_subject(subject.name, quality, occluded, camera_counts);
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - status_period_start).count();
    if (elapsed_s >= status_period_s)
    {
        // Vicon frame clock vs local clock: they must agree, or every local stamp is off
        // Not in the first period: frames queued while connecting arrive in a burst
        const std::optional<double> clock_error =
            console_status_line.local_clock_rate_error(elapsed_s);
        if (clock_check_armed && clock_error.has_value() &&
            std::abs(clock_error.value()) > kClockRateErrorLimit)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "The local clock runs %+.1f %% fast against the Vicon frame clock (frames received "
                "+ "
                "dropped vs reported rate). Local stamps are unreliable: check the system "
                "clock (on WSL, a miscalibrated 'tsc' clocksource; see CLAUDE.md).",
                -100.0 * clock_error.value());
        }

        RCLCPP_INFO(this->get_logger(), "%s", console_status_line.take_line(elapsed_s).c_str());
        status_period_start = now;
        clock_check_armed = true;
    }
}

// Create a publisher for a specific subject and segment
void ViconReceiverNode::create_publisher(const string subject_name, const string segment_name)
{
    // Launch a thread to create the publisher
    boost::thread(&ViconReceiverNode::create_publisher_thread, this, subject_name, segment_name);
}

// Thread function to create a publisher
void ViconReceiverNode::create_publisher_thread(const string subject_name,
                                                const string segment_name)
{
    // Construct the topic name (valid ROS names) and the key (original Vicon names)
    std::string topic_name =
        vicon_receiver::make_segment_topic_name(ns_name, subject_name, segment_name);
    std::string key = subject_name + "/" + segment_name;

    // Log publisher creation
    string msg = "Creating publisher for segment " + segment_name + " from subject " +
                 subject_name + " on topic " + topic_name;
    cout << msg << endl;

    boost::mutex::scoped_lock lock(mutex);

    // Two segments whose names convert to the same tokens would share a topic: refuse the second
    const auto owner_it = topic_owners.find(topic_name);
    if (owner_it != topic_owners.end() && owner_it->second != key)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Not publishing segment '%s': topic '%s' is already used by segment '%s'. "
                     "Rename one of them in Vicon Tracker.",
                     key.c_str(),
                     topic_name.c_str(),
                     owner_it->second.c_str());
        failed_publishers.insert(key);
        pending_publishers.erase(key);
        return;
    }

    // Create and store the publisher. This runs in a detached thread: an exception escaping it
    // would terminate the whole node, so failures are logged and the segment is skipped.
    try
    {
        pub_map.insert(std::map<std::string, SegmentPosePublisher>::value_type(
            key, SegmentPosePublisher(topic_name, this)));
        topic_owners[topic_name] = key;
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Not publishing segment '%s': cannot create publisher on '%s': %s",
                     key.c_str(),
                     topic_name.c_str(),
                     error.what());
        failed_publishers.insert(key);
    }

    // Clear the pending flag
    pending_publishers.erase(key);
    lock.unlock();
}

// DEVNOTE: code location may be reevaluated in the future based on further developments.
// SegmentPosePublisher: one PoseStamped publisher of a segment
SegmentPosePublisher::SegmentPosePublisher(std::string topic_name, rclcpp::Node *node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
    is_ready = true;
}

void SegmentPosePublisher::publish(geometry_msgs::msg::PoseStamped pose_msg)
{
    position_publisher_->publish(pose_msg);
}
