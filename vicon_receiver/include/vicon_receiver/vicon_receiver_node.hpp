/**
 * @file vicon_receiver_node.hpp
 * @author OPT4SMART group (original), PeterC (petercalifano.gs@gmail.com) (modifications)
 * @brief ViconReceiverNode: receives Vicon frames and publishes ViconFrame, PoseStamped and TF.
 * @version 1.2.0
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_NODE_HPP
#define VICON_RECEIVER_NODE_HPP

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "vicon_receiver/ros_conversions.hpp"
#include <chrono>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "vicon_receiver/frame_timing.hpp"
#include "vicon_receiver/console_status_line.hpp"
#include "vicon_receiver/utilities.hpp"
#include "vicon_receiver_msgs/msg/vicon_frame.hpp"

// DEVNOTE: code location may be reevaluated in the future based on further developments.
// Class that allows segment data to be published in a ROS2 topic.
class SegmentPosePublisher
{
  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;

  public:
    SegmentPosePublisher(const std::string &topic_name, rclcpp::Node *node);

    // Publishes the given position in the ROS2 topic whose name is indicated in
    // the constructor.
    void publish(const geometry_msgs::msg::PoseStamped &pose_msg);
};

// Test access to the private members of the node; defined only in tests/.
class ViconReceiverNodeTestAccess;

// Main Node class
class ViconReceiverNode : public rclcpp::Node
{
    friend class ViconReceiverNodeTestAccess;

  private:
    ViconDataStreamSDK::CPP::Client vicon_client;

    // Parameters are set by the constructor from the parameters file (no defaults there); the
    // initialisers below only rule out indeterminate values.
    std::string hostname;
    unsigned int buffer_size = 0;
    std::string ns_name;
    std::map<std::string, SegmentPosePublisher> pub_map;
    /// Segments ("subject/segment") whose publisher could not be created; never retried.
    std::set<std::string> failed_publishers;
    /// Owner ("subject/segment") of each topic, to refuse two segments on the same topic.
    vicon_receiver::NameOwnership topic_owners;
    /// Owner ("subject/segment") of each TF child frame.
    vicon_receiver::NameOwnership tf_frame_owners;
    /// Segments refused a TF frame (logged once each).
    std::set<std::string> refused_tf_segments;

    geometry_msgs::msg::TransformStamped static_tf;
    std::string world_frame;
    std::string vicon_frame;
    std::vector<double> map_xyz;
    std::vector<double> map_rpy;
    bool map_rpy_in_degrees = false;

    // --- Streaming, stamping and telemetry parameters ---
    /// SDK stream mode (ServerPush keeps every frame; ClientPull asks for the latest one)
    ViconDataStreamSDK::CPP::StreamMode::Enum stream_mode =
        ViconDataStreamSDK::CPP::StreamMode::ServerPush;
    /// Read the labeled markers of every subject into ViconFrame
    bool enable_marker_data = false;
    /// Read the cameras that saw each marker (needs marker data)
    bool enable_marker_ray_data = false;
    /// Period of the console status line [s]; 0 disables it
    double status_period_s = 0.0;
    /// Depth of the ViconFrame publisher queue (frames kept for a slow subscriber)
    std::size_t frame_queue_depth = 0;
    /// Duration of one local-clock-rate measurement [s]
    double clock_check_measurement_s = 0.0;
    /// Relative clock rate error that triggers a warning (0.01 = 1 %)
    double clock_rate_warning_limit = 0.0;
    /// Seconds without answer from the server before the connection counts as lost
    int connection_timeout_s = 0;
    /// Pause between two connection attempts [s]
    double connect_retry_period_s = 0.0;
    /// Pause before asking again when the server has no frame [s]
    double no_frame_pause_s = 0.0;

    // --- Frame processing state ---
    /// Holds back the frames left over from the server's previous session after connecting
    std::unique_ptr<vicon_receiver::StartupLeftoverFrameFilter> leftover_frame_filter;
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
    /// Monotonic local receive time of the previous frame [s] (detects stale frames)
    std::optional<double> previous_receive_steady_s;
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
     * @brief Make a dead connection fail after connection_timeout_s (TCP keepalive on the SDK
     *        socket), so that GetFrame reports NotConnected instead of waiting forever.
     */
    void watch_connection();

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
     * Creates the PoseStamped publisher of the segment on first use, in the frame loop.
     */
    void publish_segment_pose(const std::string &subject_name,
                              const vicon_receiver_msgs::msg::SegmentState &segment,
                              const builtin_interfaces::msg::Time &stamp);

    /// Add the frame to the telemetry and print the status line when the period is over.
    void update_status(const vicon_receiver_msgs::msg::ViconFrame &frame_msg);

    /// Print the status line if the period is over (also while no frame arrives).
    void print_status_if_due();

    /**
     * @brief Create the PoseStamped publisher of one segment, the first time the segment is seen.
     *
     * Runs in the frame loop; it costs a few milliseconds once per segment. The topic is
     * "<namespace>/<subject>/<segment>" with the Vicon names converted by
     * vicon_receiver::to_ros_name_token(). If the publisher cannot be created (invalid name, or
     * topic already used by another segment whose names convert to the same tokens), the error is
     * logged and the segment is never retried; TF is still broadcast for it, unless another
     * segment already owns its TF frame.
     */
    void create_segment_publisher(const std::string &subject_name, const std::string &segment_name);

  public:
    /// Name of the node; also the top-level key of the parameters file.
    static constexpr const char *kNodeName = "vicon_client";

    /// Exit status of the node when the connection to the Vicon server is lost.
    static constexpr int kConnectionLostExitStatus = 2;

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
    [[nodiscard]] bool connect();

    // Stops the current connection to a DataStream server (if any).
    [[nodiscard]] bool disconnect();

    /**
     * @brief Receive one frame and publish it.
     *
     * Waits for the next frame (ServerPush) or requests it (ClientPull), publishes one
     * ViconFrame with every subject (occlusion mask included), the valid segment poses on
     * PoseStamped and TF (stamped with the estimated acquisition time), and updates the console
     * telemetry.
     *
     * @return false if the connection to the server is lost (no reconnection: the caller stops),
     *         true otherwise (frame published, or no frame available yet).
     */
    [[nodiscard]] bool get_frame();

    /**
     * @brief Receive and publish frames (get_frame()) until ROS 2 shuts down or the connection
     *        to the Vicon server is lost.
     *
     * @return 0 when ROS 2 was shut down (Ctrl+C); kConnectionLostExitStatus when the connection
     *         was lost (logged as FATAL; no reconnection, e.g. for a supervisor to restart the node).
     */
    [[nodiscard]] int run_frame_loop();
};

#endif // VICON_RECEIVER_NODE_HPP
