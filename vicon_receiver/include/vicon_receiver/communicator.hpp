#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include "name_conversion.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>
#include <unistd.h>
#include "boost/thread.hpp"
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Transform.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

// Main Node class
class Communicator : public rclcpp::Node
{
  private:
    ViconDataStreamSDK::CPP::Client vicon_client;

    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, Publisher> pub_map;
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

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void publish_static_transform();

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
     * @throws std::invalid_argument if map_xyz or map_rpy does not have 3 elements.
     */
    explicit Communicator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the
    // received segment data to the Publisher class.
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

#endif // COMMUNICATOR_HPP
