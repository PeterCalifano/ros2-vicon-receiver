/**
 * @file test_vicon_receiver_node_publishers.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the creation of segment publishers with Vicon names: names that are not
 *        valid in ROS (e.g. "g1-unitree-torso-cam") are converted instead of crashing the node,
 *        two segments never share a topic or a TF frame, and a failing publisher is skipped. The
 *        node is built with parameter overrides and never connects to a Vicon server.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/vicon_receiver_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "node_test_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"

/// Friend of ViconReceiverNode: reaches the private steps of the frame loop.
class ViconReceiverNodeTestAccess
{
  public:
    static void create_segment_publisher(ViconReceiverNode &node,
                                         const std::string &subject_name,
                                         const std::string &segment_name)
    {
        node.create_segment_publisher(subject_name, segment_name);
    }

    static void publish_segment_pose(ViconReceiverNode &node,
                                     const std::string &subject_name,
                                     const vicon_receiver_msgs::msg::SegmentState &segment)
    {
        node.publish_segment_pose(subject_name, segment, builtin_interfaces::msg::Time());
    }
};

namespace
{
    /// Time allowed for the ROS graph to report a newly created publisher.
    constexpr std::chrono::seconds kGraphTimeout{3};

    /**
     * @brief Node options with a valid parameter set (as in config/vicon_client.yaml) and the
     *        given topic namespace, isolated from any global --ros-args.
     */
    rclcpp::NodeOptions make_options(const std::string &topic_namespace)
    {
        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(node_test_parameters::make_valid_parameters(topic_namespace));
        return options;
    }

    /**
     * @brief Number of publishers on a topic, waiting up to kGraphTimeout for at least
     *        @p expected_count to appear (the ROS graph is updated asynchronously).
     */
    size_t wait_for_publishers(const rclcpp::Node &node,
                               const std::string &topic_name,
                               const size_t expected_count)
    {
        const auto deadline = std::chrono::steady_clock::now() + kGraphTimeout;
        size_t count = node.count_publishers(topic_name);
        while (count < expected_count && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            count = node.count_publishers(topic_name);
        }
        return count;
    }
} // namespace

TEST(ViconReceiverNodePublishers, ViconNameWithDashesGetsValidTopic)
{
    // Regression: the node aborted with InvalidTopicNameError on the lab subject name
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_dashes"));

    ASSERT_NO_THROW(ViconReceiverNodeTestAccess::create_segment_publisher(
        *node, "g1-unitree-torso-cam", "g1-unitree-torso-cam"));

    const std::string topic_name = "/test_dashes/g1_unitree_torso_cam/g1_unitree_torso_cam";
    EXPECT_EQ(wait_for_publishers(*node, topic_name, 1), 1U) << topic_name;
}

TEST(ViconReceiverNodePublishers, CollidingNamesDoNotShareATopic)
{
    // "a-b" and "a_b" convert to the same token: only the first segment gets the topic
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_collision"));

    ViconReceiverNodeTestAccess::create_segment_publisher(*node, "a-b", "x");
    ViconReceiverNodeTestAccess::create_segment_publisher(*node, "a_b", "x");

    const std::string topic_name = "/test_collision/a_b/x";
    EXPECT_EQ(wait_for_publishers(*node, topic_name, 1), 1U);

    // Give the graph time to show a second publisher, if one had been (wrongly) created
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_EQ(node->count_publishers(topic_name), 1U);
}

TEST(ViconReceiverNodePublishers, InvalidNamespaceIsRejectedAtStartup)
{
    // The namespace parameter is used as given. The constructor creates the ViconFrame
    // publisher on "<namespace>/frame", so an invalid namespace is a configuration error
    // reported at startup (std::invalid_argument, FATAL in main()), instead of a failure in
    // the publisher thread later on.
    EXPECT_THROW(ViconReceiverNode node(make_options("bad namespace")), std::invalid_argument);
}

TEST(ViconReceiverNodePublishers, FirstPoseOfASegmentIsPublished)
{
    // The publisher is created in the frame loop, so the first pose is not lost
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_first_pose"));

    vicon_receiver_msgs::msg::SegmentState segment;
    segment.name = "body";
    segment.frame_id = "first_pose_body";
    segment.orientation.w = 1.0;
    ViconReceiverNodeTestAccess::publish_segment_pose(*node, "first_pose", segment);

    EXPECT_EQ(wait_for_publishers(*node, "/test_first_pose/first_pose/body", 1), 1U);
}

TEST(ViconReceiverNodePublishers, CollidingTfFrameKeepsItsFirstOwner)
{
    // "tf_a_b"/"c" and "tf_a"/"b_c" have different topics but the same TF child frame
    const std::string frame = "tf_a_b_c";
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_tf_collision"));
    auto listener = rclcpp::Node::make_shared("tf_collision_listener");

    std::vector<double> received_x;
    const auto subscription = listener->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        tf2_ros::DynamicListenerQoS(),
        [&received_x, &frame](const tf2_msgs::msg::TFMessage &message)
        {
            for (const auto &transform : message.transforms)
            {
                if (transform.child_frame_id == frame)
                {
                    received_x.push_back(transform.transform.translation.x);
                }
            }
        });
    const auto deadline = std::chrono::steady_clock::now() + kGraphTimeout;
    while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ASSERT_GE(subscription->get_publisher_count(), 1U);

    vicon_receiver_msgs::msg::SegmentState first;
    first.name = "c";
    first.frame_id = frame;
    first.position.x = 1.0;
    first.orientation.w = 1.0;
    vicon_receiver_msgs::msg::SegmentState second = first;
    second.name = "b_c";
    second.position.x = 2.0;

    ViconReceiverNodeTestAccess::publish_segment_pose(*node, "tf_a_b", first);
    ViconReceiverNodeTestAccess::publish_segment_pose(*node, "tf_a", second);
    ViconReceiverNodeTestAccess::publish_segment_pose(*node, "tf_a_b", first);

    // The refused segment could arrive at any point: keep listening a little after the last one
    const auto listen_until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < listen_until)
    {
        rclcpp::spin_some(listener);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(received_x, (std::vector<double>{1.0, 1.0}));
}

TEST(ViconReceiverNodeConnection, LostConnectionStopsTheFrameLoop)
{
    // A node that never connected gets NotConnected from the SDK, as after a lost connection:
    // the frame loop stops at once with the connection-lost exit status (no reconnection)
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_connection"));

    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(node->run_frame_loop(), ViconReceiverNode::kConnectionLostExitStatus);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(5));
}

// -------------------------------------------------------------------------------------------------
// Test entry point: rclcpp must be initialised once for the whole test binary
// -------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    const int result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}
