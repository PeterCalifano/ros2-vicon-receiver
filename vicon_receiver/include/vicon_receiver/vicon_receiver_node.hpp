/**
 * @file vicon_receiver_node.hpp
 * @author OPT4SMART group (original), PeterC (petercalifano.gs@gmail.com) (modifications)
 * @brief ViconReceiverNode: receives Vicon frames and publishes ViconFrame, PoseStamped and TF.
 * @version 1.2.0
 * @date 2026-10-30
 */

#if !defined(VICON_RECEIVER_NODE_HPP)
#define VICON_RECEIVER_NODE_HPP

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "vicon_receiver/ros_conversions.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unistd.h>
#include "boost/thread.hpp"
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "vicon_receiver/frame_timing.hpp"
#include "vicon_receiver/console_status_line.hpp"
#include "vicon_receiver_msgs/msg/vicon_frame.hpp"

using namespace std;

// DEVNOTE: code location may be reevaluated in the future based on further developments.
// Class that allows segment data to be published in a ROS2 topic.
class SegmentPosePublisher
{
  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;

  public:
    bool is_ready = false;

    SegmentPosePublisher(std::string topic_name, rclcpp::Node *node);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(geometry_msgs::msg::PoseStamped p);
};

// Main Node class
class ViconReceiverNode : public rclcpp::Node
{
  private:
    ViconDataStreamSDK::CPP::Client vicon_client;

    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, SegmentPosePublisher> pub_map;
    boost::mutex mutex;
    std::set<std::string> pending_publishers;
    /// Segments ("subject/segment") whose publisher could not be created; never retried.
    std::set<std::string> failed_publishers;
    /// Owner ("subject/segment") of each topic, to refuse two segments on the same topic.
    std::map<std::string, std::string> topic_owners;

    geometry_msgs::msg::TransformStamped static_tf;
    string world_frame;
    string vicon_frame;
    vector<double> map_xyz;
    vector<double> map_rpy;
    bool map_rpy_in_degrees;

    // --- Streaming, stamping and telemetry parameters ---
    /// SDK stream mode (ServerPush keeps every frame; ClientPull asks for the latest one)
    ViconDataStreamSDK::CPP::StreamMode::Enum stream_mode;
    /// Read the labeled markers of every subject into ViconFrame
    bool enable_marker_data;
    /// Read the cameras that saw each marker (needs marker data)
    bool enable_marker_ray_data;
    /// Period of the console status line [s]; 0 disables it
    double status_period_s;

    // --- Frame processing state ---
    /// Consecutive frame numbers that confirm the live stream after connecting: more than the
    /// leftovers of the server's previous session seen back to back (up to 2)
    static constexpr std::uint64_t kLiveStreamConsecutiveFrames = 3;
    /// Holds back the frames left over from the server's previous session after connecting
    vicon_receiver::StartupLeftoverFrameFilter leftover_frame_filter{kLiveStreamConsecutiveFrames};
    /// Estimated camera acquisition time of each frame
    std::unique_ptr<vicon_receiver::AcquisitionClock> acquisition_clock;
    /// Local clock rate compared with the Vicon frame clock
    std::unique_ptr<vicon_receiver::LocalClockRateMonitor> local_clock_rate_monitor;
    /// Frames missing from the frame-number sequence
    vicon_receiver::DroppedFrameCounter dropped_frames;
    /// Console telemetry of the current status period
    vicon_receiver::ConsoleStatusLine console_status_line;
    /// Start of the current status period
    std::chrono::steady_clock::time_point status_period_start;
    /// Local receive time of the previous frame [s] (detects stale frames)
    std::optional<double> previous_receive_s;
    /// Whether the "no frame rate: receive time used as stamp" warning was already logged
    bool missing_rate_warned = false;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /// One ViconFrame per received frame
    rclcpp::Publisher<vicon_receiver_msgs::msg::ViconFrame>::SharedPtr frame_publisher_;

    void publish_static_transform();

    /// Read and validate the streaming, stamping and telemetry parameters.
    void read_frame_parameters();

    /// Forget everything about the previous frame timeline (called after each connection).
    void reset_stream_state();

    /**
     * @brief Whether the current frame belongs to the live stream (see StartupLeftoverFrameFilter).
     *
     * After connecting, the frames left over from the server's previous session and the first
     * live frames (until the stream is confirmed) are held back: neither published nor counted.
     *
     * @param receive_steady_s  Monotonic local time at which the frame was received [s].
     */
    bool accept_live_frame(double receive_steady_s);

    /**
     * @brief Fill the frame-level fields of a ViconFrame (numbers, rate, latency, stamps) and
     *        check the local clock rate.
     *
     * @param receive_time      Local time at which the frame was received.
     * @param receive_steady_s  Monotonic local time at which the frame was received [s].
     * @param frame_msg         Message to fill; header.stamp gets the estimated acquisition time.
     */
    void read_frame_header(const rclcpp::Time &receive_time,
                           double receive_steady_s,
                           vicon_receiver_msgs::msg::ViconFrame &frame_msg);

    /**
     * @brief Read every subject of the current frame into the message, and publish the valid
     *        segment poses on their PoseStamped topics and on TF.
     */
    void read_subjects(vicon_receiver_msgs::msg::ViconFrame &frame_msg);

    /// Read the labeled markers of one subject (with the cameras that saw them, if enabled).
    void read_markers(const std::string &subject_name,
                      vicon_receiver_msgs::msg::SubjectState &subject_msg);

    /**
     * @brief Publish one valid segment pose: PoseStamped in world_frame and TF from vicon_frame.
     *
     * Creates the PoseStamped publisher of the segment on first use (in a separate thread).
     */
    void publish_segment_pose(const std::string &subject_name,
                              const vicon_receiver_msgs::msg::SegmentState &segment,
                              const builtin_interfaces::msg::Time &stamp);

    /// Add the frame to the telemetry and print the status line when the period is over.
    void update_status(const vicon_receiver_msgs::msg::ViconFrame &frame_msg);

    /// Print the status line if the period is over (also while no frame arrives).
    void print_status_if_due();

  public:
    /// Name of the node; also the top-level key of the parameters file.
    static constexpr const char *kNodeName = "vicon_client";

    /**
     * @brief Create the node, read and validate its parameters and publish the static
     *        world_frame -> vicon_frame transform. Does not connect to the Vicon server.
     *
     * @param options  Node options; tests use them to inject parameter overrides.
     * @throws rclcpp::exceptions::UninitializedStaticallyTypedParameterException if a parameter
     *         is missing.
     * @throws rclcpp::exceptions::InvalidParameterTypeException if a parameter has the wrong type.
     * @throws std::invalid_argument if a parameter value is invalid (vector size, stream mode,
     *         window, status period, marker options).
     */
    explicit ViconReceiverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    /**
     * @brief Receive one frame and publish it.
     *
     * Waits for the next frame (ServerPush) or requests it (ClientPull), publishes one
     * ViconFrame with every subject (occlusion mask included), the valid segment poses on
     * PoseStamped and TF (stamped with the estimated acquisition time), and updates the console
     * telemetry. Reconnects if the connection was lost.
     */
    void get_frame();

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name);

    /**
     * @brief Create the PoseStamped publisher of one segment (run in a separate thread).
     *
     * The topic is "<namespace>/<subject>/<segment>" with the Vicon names converted by
     * vicon_receiver::to_ros_name_token(). If the publisher cannot be created (invalid name, or
     * topic already used by another segment whose names convert to the same tokens), the error is
     * logged and the segment is never retried; TF is still broadcast for it.
     */
    void create_publisher_thread(const string subject_name, const string segment_name);
};

#endif // VICON_RECEIVER_NODE_HPP
