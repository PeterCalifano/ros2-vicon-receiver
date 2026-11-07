/**
 * @file test_vicon_receiver_node_publishers.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the creation of segment publishers with Vicon names: names that are not
 *        valid in ROS (e.g. "g1-unitree-torso-cam") are converted instead of crashing the node,
 *        two segments never share a topic, and a failing publisher is skipped. The node is built
 *        with parameter overrides and never connects to a Vicon server.
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

#include "rclcpp/rclcpp.hpp"

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
        std::vector<rclcpp::Parameter> parameters;
        parameters.emplace_back("hostname", "192.168.10.1");
        parameters.emplace_back("buffer_size", 200);
        parameters.emplace_back("namespace", topic_namespace);
        parameters.emplace_back("world_frame", "map");
        parameters.emplace_back("vicon_frame", "vicon");
        parameters.emplace_back("map_xyz", std::vector<double>{0.0, 0.0, 0.0});
        parameters.emplace_back("map_rpy", std::vector<double>{0.0, 0.0, 0.0});
        parameters.emplace_back("map_rpy_in_degrees", false);
        parameters.emplace_back("stream_mode", "ServerPush");
        parameters.emplace_back("enable_marker_data", true);
        parameters.emplace_back("enable_marker_ray_data", true);
        parameters.emplace_back("acquisition_stamp_window_frames", 1000);
        parameters.emplace_back("status_period_s", 1.0);
        parameters.emplace_back("frame_queue_depth", 100);
        parameters.emplace_back("live_stream_confirmation_frames", 3);
        parameters.emplace_back("clock_check_measurement_s", 10.0);
        parameters.emplace_back("clock_check_block_s", 1.0);
        parameters.emplace_back("clock_rate_warning_limit", 0.01);
        parameters.emplace_back("connection_timeout_s", 5);

        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(parameters);
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

    ASSERT_NO_THROW(node->create_publisher_thread("g1-unitree-torso-cam", "g1-unitree-torso-cam"));

    const std::string topic_name = "/test_dashes/g1_unitree_torso_cam/g1_unitree_torso_cam";
    EXPECT_EQ(wait_for_publishers(*node, topic_name, 1), 1U) << topic_name;
}

TEST(ViconReceiverNodePublishers, CollidingNamesDoNotShareATopic)
{
    // "a-b" and "a_b" convert to the same token: only the first segment gets the topic
    auto node = std::make_shared<ViconReceiverNode>(make_options("test_collision"));

    node->create_publisher_thread("a-b", "x");
    node->create_publisher_thread("a_b", "x");

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
