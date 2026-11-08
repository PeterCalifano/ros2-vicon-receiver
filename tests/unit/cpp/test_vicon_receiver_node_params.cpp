/**
 * @file test_vicon_receiver_node_params.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the ViconReceiverNode parameter contract: every parameter is required, types
 *        and vector sizes are validated, and the static world_frame -> vicon_frame transform is
 *        built from map_xyz / map_rpy (radians or degrees). No Vicon server is needed: the node
 *        is constructed with parameter overrides and never connects.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/vicon_receiver_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"

namespace
{
    /// Absolute tolerance for comparing quaternion and translation components.
    constexpr double kTolerance = 1e-9;

    /// Maximum time to wait for the static transform published by the node under test.
    constexpr std::chrono::seconds kStaticTfTimeout{5};

    /**
     * @brief Parameter set equivalent to config/vicon_client.yaml, accepted by the node.
     */
    std::vector<rclcpp::Parameter> make_valid_parameters()
    {
        std::vector<rclcpp::Parameter> parameters;
        parameters.emplace_back("hostname", "192.168.10.1");
        parameters.emplace_back("buffer_size", 200);
        parameters.emplace_back("namespace", "vicon");
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

    /**
     * @brief Copy of a parameter set without the parameter called @p name.
     */
    std::vector<rclcpp::Parameter>
    without_parameter(const std::vector<rclcpp::Parameter> &parameters, const std::string &name)
    {
        std::vector<rclcpp::Parameter> result;
        for (const rclcpp::Parameter &parameter : parameters)
        {
            if (parameter.get_name() != name)
            {
                result.push_back(parameter);
            }
        }
        return result;
    }

    /**
     * @brief Copy of a parameter set where the parameter with the same name as @p replacement is
     *        replaced by it (value and type).
     */
    std::vector<rclcpp::Parameter> with_parameter(const std::vector<rclcpp::Parameter> &parameters,
                                                  const rclcpp::Parameter &replacement)
    {
        std::vector<rclcpp::Parameter> result =
            without_parameter(parameters, replacement.get_name());
        result.push_back(replacement);
        return result;
    }

    /**
     * @brief Node options that pass @p parameters as overrides, isolated from any global
     *        --ros-args of the test process.
     */
    rclcpp::NodeOptions make_options(const std::vector<rclcpp::Parameter> &parameters)
    {
        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        options.parameter_overrides(parameters);
        return options;
    }

    /**
     * @brief Wait for the static transform whose child frame is @p child_frame on /tf_static.
     *
     * The subscription uses the transient-local QoS of static TF, so a transform published before
     * the subscription was created is still received.
     *
     * @return The transform, or std::nullopt if it did not arrive within kStaticTfTimeout.
     */
    std::optional<geometry_msgs::msg::TransformStamped>
    wait_for_static_transform(const std::string &child_frame)
    {
        auto listener_node = std::make_shared<rclcpp::Node>("static_tf_listener_test");
        std::optional<geometry_msgs::msg::TransformStamped> received;

        auto on_message = [&received, &child_frame](const tf2_msgs::msg::TFMessage &message)
        {
            for (const geometry_msgs::msg::TransformStamped &transform : message.transforms)
            {
                if (transform.child_frame_id == child_frame)
                {
                    received = transform;
                }
            }
        };
        auto subscription = listener_node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", tf2_ros::StaticListenerQoS(), on_message);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(listener_node);

        const auto deadline = std::chrono::steady_clock::now() + kStaticTfTimeout;
        while (!received.has_value() && std::chrono::steady_clock::now() < deadline)
        {
            executor.spin_some(std::chrono::milliseconds(50));
        }
        return received;
    }
} // namespace

// -------------------------------------------------------------------------------------------------
// Valid configuration
// -------------------------------------------------------------------------------------------------

TEST(ViconReceiverNodeParams, ValidParametersAreAccepted)
{
    const rclcpp::NodeOptions options = make_options(make_valid_parameters());

    std::shared_ptr<ViconReceiverNode> node;
    ASSERT_NO_THROW(node = std::make_shared<ViconReceiverNode>(options));

    // The values from the overrides are the ones the node reads
    EXPECT_EQ(node->get_parameter("hostname").as_string(), "192.168.10.1");
    EXPECT_EQ(node->get_parameter("buffer_size").as_int(), 200);
    EXPECT_EQ(node->get_parameter("namespace").as_string(), "vicon");
}

// -------------------------------------------------------------------------------------------------
// Missing parameters: there are no defaults, every parameter is required
// -------------------------------------------------------------------------------------------------

/// Parameterised over the name of the parameter left out of an otherwise valid set.
class MissingParameterTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(MissingParameterTest, MissingParameterIsRejected)
{
    const std::string missing_name = GetParam();
    const rclcpp::NodeOptions options =
        make_options(without_parameter(make_valid_parameters(), missing_name));

    EXPECT_THROW(ViconReceiverNode node(options),
                 rclcpp::exceptions::UninitializedStaticallyTypedParameterException);
}

INSTANTIATE_TEST_SUITE_P(EveryParameter,
                         MissingParameterTest,
                         ::testing::Values("hostname",
                                           "buffer_size",
                                           "namespace",
                                           "world_frame",
                                           "vicon_frame",
                                           "map_xyz",
                                           "map_rpy",
                                           "map_rpy_in_degrees",
                                           "stream_mode",
                                           "enable_marker_data",
                                           "enable_marker_ray_data",
                                           "acquisition_stamp_window_frames",
                                           "status_period_s",
                                           "frame_queue_depth",
                                           "live_stream_confirmation_frames",
                                           "clock_check_measurement_s",
                                           "clock_check_block_s",
                                           "clock_rate_warning_limit",
                                           "connection_timeout_s",
                                           "connect_retry_period_s",
                                           "no_frame_pause_s"));

// -------------------------------------------------------------------------------------------------
// Invalid values of the streaming, stamping and telemetry parameters
// -------------------------------------------------------------------------------------------------

/// One invalid parameter value and a fragment of the expected error message.
struct InvalidValueCase
{
    std::string name;         ///< Test case name
    rclcpp::Parameter value;  ///< Parameter replacing the valid one
    std::string message_part; ///< Expected in the std::invalid_argument message
};

/// Parameterised over invalid values; each must be rejected with a message naming it.
class InvalidValueTest : public ::testing::TestWithParam<InvalidValueCase>
{
};

TEST_P(InvalidValueTest, InvalidValueIsRejectedWithMessage)
{
    const InvalidValueCase &test_case = GetParam();
    const rclcpp::NodeOptions options =
        make_options(with_parameter(make_valid_parameters(), test_case.value));

    try
    {
        ViconReceiverNode node(options);
        FAIL() << "Expected std::invalid_argument";
    }
    catch (const std::invalid_argument &error)
    {
        const std::string message = error.what();
        EXPECT_NE(message.find(test_case.message_part), std::string::npos) << message;
    }
}

INSTANTIATE_TEST_SUITE_P(
    FrameParameters,
    InvalidValueTest,
    ::testing::Values(
        InvalidValueCase{"unknown_stream_mode",
                         rclcpp::Parameter("stream_mode", "Push"),
                         "unknown stream mode 'Push'"},
        InvalidValueCase{"lowercase_stream_mode",
                         rclcpp::Parameter("stream_mode", "serverpush"),
                         "valid: ServerPush, ClientPull, ClientPullPreFetch"},
        InvalidValueCase{"empty_window",
                         rclcpp::Parameter("acquisition_stamp_window_frames", 0),
                         "'acquisition_stamp_window_frames' must be >= 1"},
        InvalidValueCase{"negative_status_period",
                         rclcpp::Parameter("status_period_s", -1.0),
                         "'status_period_s' must be >= 0"},
        InvalidValueCase{"empty_queue",
                         rclcpp::Parameter("frame_queue_depth", 0),
                         "'frame_queue_depth' must be >= 1"},
        InvalidValueCase{"single_confirmation_frame",
                         rclcpp::Parameter("live_stream_confirmation_frames", 1),
                         "'live_stream_confirmation_frames' must be >= 2"},
        InvalidValueCase{"zero_clock_block",
                         rclcpp::Parameter("clock_check_block_s", 0.0),
                         "'clock_check_block_s' must be > 0"},
        InvalidValueCase{"short_clock_measurement",
                         rclcpp::Parameter("clock_check_measurement_s", 2.0),
                         "'clock_check_measurement_s' must be > 2 x clock_check_block_s"},
        InvalidValueCase{"short_connection_timeout",
                         rclcpp::Parameter("connection_timeout_s", 1),
                         "'connection_timeout_s' must be in 2..3600"},
        InvalidValueCase{"zero_buffer_size",
                         rclcpp::Parameter("buffer_size", 0),
                         "'buffer_size' must be in 1..100000"},
        InvalidValueCase{"negative_buffer_size",
                         rclcpp::Parameter("buffer_size", -1),
                         "'buffer_size' must be in 1..100000"},
        InvalidValueCase{"huge_buffer_size",
                         rclcpp::Parameter("buffer_size", 100001),
                         "'buffer_size' must be in 1..100000"},
        InvalidValueCase{"zero_connect_retry_period",
                         rclcpp::Parameter("connect_retry_period_s", 0.0),
                         "'connect_retry_period_s' must be > 0"},
        InvalidValueCase{"zero_no_frame_pause",
                         rclcpp::Parameter("no_frame_pause_s", 0.0),
                         "'no_frame_pause_s' must be > 0"},
        InvalidValueCase{"zero_warning_limit",
                         rclcpp::Parameter("clock_rate_warning_limit", 0.0),
                         "'clock_rate_warning_limit' must be > 0"},
        InvalidValueCase{"rays_without_markers",
                         rclcpp::Parameter("enable_marker_data", false),
                         "'enable_marker_ray_data' is true but 'enable_marker_data' is false"}),
    [](const ::testing::TestParamInfo<InvalidValueCase> &info) { return info.param.name; });

TEST(ViconReceiverNodeParams, EveryStreamModeIsAccepted)
{
    for (const char *mode : {"ServerPush", "ClientPull", "ClientPullPreFetch"})
    {
        const rclcpp::Parameter stream_mode("stream_mode", mode);
        EXPECT_NO_THROW(ViconReceiverNode node(
            make_options(with_parameter(make_valid_parameters(), stream_mode))))
            << mode;
    }
}

TEST(ViconReceiverNodeParams, StatusLineCanBeDisabled)
{
    const rclcpp::Parameter disabled("status_period_s", 0.0);
    EXPECT_NO_THROW(
        ViconReceiverNode node(make_options(with_parameter(make_valid_parameters(), disabled))));
}

// -------------------------------------------------------------------------------------------------
// Wrong types
// -------------------------------------------------------------------------------------------------

TEST(ViconReceiverNodeParams, IntegerArrayForDoubleArrayIsRejected)
{
    // In YAML, "[0, 0, 0]" is an integer array, while the node expects doubles ("[0.0, ...]")
    const rclcpp::Parameter integer_array("map_xyz", std::vector<int64_t>{0, 0, 0});
    const rclcpp::NodeOptions options =
        make_options(with_parameter(make_valid_parameters(), integer_array));

    EXPECT_THROW(ViconReceiverNode node(options),
                 rclcpp::exceptions::InvalidParameterTypeException);
}

TEST(ViconReceiverNodeParams, StringForIntegerIsRejected)
{
    const rclcpp::Parameter string_value("buffer_size", "200");
    const rclcpp::NodeOptions options =
        make_options(with_parameter(make_valid_parameters(), string_value));

    EXPECT_THROW(ViconReceiverNode node(options),
                 rclcpp::exceptions::InvalidParameterTypeException);
}

// -------------------------------------------------------------------------------------------------
// Vector sizes of the static transform parameters
// -------------------------------------------------------------------------------------------------

TEST(ViconReceiverNodeParams, TranslationWithTwoElementsIsRejected)
{
    const rclcpp::Parameter short_xyz("map_xyz", std::vector<double>{1.0, 2.0});
    const rclcpp::NodeOptions options =
        make_options(with_parameter(make_valid_parameters(), short_xyz));

    try
    {
        ViconReceiverNode node(options);
        FAIL() << "Expected std::invalid_argument for a 2-element map_xyz";
    }
    catch (const std::invalid_argument &error)
    {
        // The message names the parameter and the sizes, so the user can fix the YAML
        const std::string message = error.what();
        EXPECT_NE(message.find("map_xyz"), std::string::npos) << message;
        EXPECT_NE(message.find("got 2"), std::string::npos) << message;
    }
}

TEST(ViconReceiverNodeParams, RotationWithFourElementsIsRejected)
{
    const rclcpp::Parameter long_rpy("map_rpy", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    const rclcpp::NodeOptions options =
        make_options(with_parameter(make_valid_parameters(), long_rpy));

    EXPECT_THROW(ViconReceiverNode node(options), std::invalid_argument);
}

// -------------------------------------------------------------------------------------------------
// Static world_frame -> vicon_frame transform
// -------------------------------------------------------------------------------------------------

/// Expected static transform for a given map_rpy / map_rpy_in_degrees pair.
struct StaticTransformCase
{
    std::string name;        ///< Test case name (also used to make the frame names unique)
    std::vector<double> rpy; ///< map_rpy parameter
    bool rpy_in_degrees;     ///< map_rpy_in_degrees parameter
    double expected_qz;      ///< Expected quaternion z component
    double expected_qw;      ///< Expected quaternion w component
};

/// Parameterised over equivalent yaw rotations given in radians and in degrees.
class StaticTransformTest : public ::testing::TestWithParam<StaticTransformCase>
{
};

TEST_P(StaticTransformTest, StaticTransformMatchesParameters)
{
    const StaticTransformCase &test_case = GetParam();

    // Unique frame names, so transforms published by other test cases cannot match
    const std::string world_frame = "test_world_" + test_case.name;
    const std::string vicon_frame = "test_vicon_" + test_case.name;
    const std::vector<double> translation = {1.0, -2.0, 0.5};

    std::vector<rclcpp::Parameter> parameters = make_valid_parameters();
    parameters = with_parameter(parameters, rclcpp::Parameter("world_frame", world_frame));
    parameters = with_parameter(parameters, rclcpp::Parameter("vicon_frame", vicon_frame));
    parameters = with_parameter(parameters, rclcpp::Parameter("map_xyz", translation));
    parameters = with_parameter(parameters, rclcpp::Parameter("map_rpy", test_case.rpy));
    parameters = with_parameter(parameters,
                                rclcpp::Parameter("map_rpy_in_degrees", test_case.rpy_in_degrees));

    // The constructor publishes the static transform
    ViconReceiverNode node(make_options(parameters));

    const std::optional<geometry_msgs::msg::TransformStamped> transform =
        wait_for_static_transform(vicon_frame);
    ASSERT_TRUE(transform.has_value()) << "No static transform received for " << vicon_frame;

    EXPECT_EQ(transform->header.frame_id, world_frame);
    EXPECT_NEAR(transform->transform.translation.x, translation[0], kTolerance);
    EXPECT_NEAR(transform->transform.translation.y, translation[1], kTolerance);
    EXPECT_NEAR(transform->transform.translation.z, translation[2], kTolerance);

    EXPECT_NEAR(transform->transform.rotation.x, 0.0, kTolerance);
    EXPECT_NEAR(transform->transform.rotation.y, 0.0, kTolerance);
    EXPECT_NEAR(transform->transform.rotation.z, test_case.expected_qz, kTolerance);
    EXPECT_NEAR(transform->transform.rotation.w, test_case.expected_qw, kTolerance);
}

// A yaw of 90 deg is the quaternion (0, 0, sin(45 deg), cos(45 deg))
INSTANTIATE_TEST_SUITE_P(
    YawRotation,
    StaticTransformTest,
    ::testing::Values(
        StaticTransformCase{"identity", {0.0, 0.0, 0.0}, false, 0.0, 1.0},
        StaticTransformCase{"yaw90_radians", {0.0, 0.0, M_PI / 2.0}, false, M_SQRT1_2, M_SQRT1_2},
        StaticTransformCase{"yaw90_degrees", {0.0, 0.0, 90.0}, true, M_SQRT1_2, M_SQRT1_2}),
    [](const ::testing::TestParamInfo<StaticTransformCase> &info) { return info.param.name; });

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
