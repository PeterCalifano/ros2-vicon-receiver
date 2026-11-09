/**
 * @file node_test_parameters.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Parameter set accepted by ViconReceiverNode, shared by the node unit tests.
 * @version 0.1
 * @date 2026-11-09
 */

#ifndef VICON_RECEIVER_TESTS_NODE_TEST_PARAMETERS_HPP
#define VICON_RECEIVER_TESTS_NODE_TEST_PARAMETERS_HPP

#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"

namespace node_test_parameters
{
    /**
     * @brief Parameter set equivalent to config/vicon_client.yaml, accepted by the node.
     *
     * @param topic_namespace  Value of the "namespace" parameter.
     */
    inline std::vector<rclcpp::Parameter> make_valid_parameters(const std::string &topic_namespace)
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
        parameters.emplace_back("connect_retry_period_s", 1.0);
        parameters.emplace_back("no_frame_pause_s", 0.005);
        return parameters;
    }
} // namespace node_test_parameters

#endif // VICON_RECEIVER_TESTS_NODE_TEST_PARAMETERS_HPP
