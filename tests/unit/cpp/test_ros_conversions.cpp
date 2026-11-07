/**
 * @file test_ros_conversions.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of ros_conversions.hpp: Vicon names -> ROS names, SDK results -> messages, TF,
 *        times.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/ros_conversions.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rmw/validate_full_topic_name.h"

// =================================================================================================
// Vicon names -> ROS names
// =================================================================================================
namespace name_conversion_tests
{
    namespace
    {
        /**
     * @brief Validate a full (absolute) topic name with the ROS middleware rules.
     *
     * @return An empty string if the name is valid, otherwise the reason it is invalid.
     */
        std::string topic_name_problem(const std::string &topic_name)
        {
            int validation_result = RMW_TOPIC_VALID;
            size_t invalid_index = 0;
            const rmw_ret_t status = rmw_validate_full_topic_name(
                topic_name.c_str(), &validation_result, &invalid_index);
            if (status != RMW_RET_OK)
            {
                return "validator failed";
            }
            if (validation_result == RMW_TOPIC_VALID)
            {
                return "";
            }
            return rmw_full_topic_name_validation_result_string(validation_result);
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // Conversion rules
    // ---------------------------------------------------------------------------------------------

    /// A Vicon name and the token it must convert to.
    struct NameCase
    {
        std::string vicon_name;     ///< Name as reported by Vicon Tracker
        std::string expected_token; ///< Expected ROS name token
    };

    /// Parameterised over Vicon names covering every conversion rule.
    class RosNameTokenTest : public ::testing::TestWithParam<NameCase>
    {
    };

    TEST_P(RosNameTokenTest, ConvertsToExpectedToken)
    {
        const NameCase &name_case = GetParam();
        EXPECT_EQ(vicon_receiver::to_ros_name_token(name_case.vicon_name),
                  name_case.expected_token);
    }

    TEST_P(RosNameTokenTest, TokenIsValidInTopicNames)
    {
        const std::string token = vicon_receiver::to_ros_name_token(GetParam().vicon_name);
        const std::string topic_name = "/vicon/" + token + "/" + token;
        EXPECT_EQ(topic_name_problem(topic_name), "") << topic_name;
    }

    TEST_P(RosNameTokenTest, TokenIsNotHidden)
    {
        // ROS tools hide topics whose name tokens start with '_'
        const std::string token = vicon_receiver::to_ros_name_token(GetParam().vicon_name);
        ASSERT_FALSE(token.empty());
        EXPECT_NE(token.front(), '_') << token;
    }

    INSTANTIATE_TEST_SUITE_P(ViconNames,
                             RosNameTokenTest,
                             ::testing::Values(
                                 // Valid names are kept as they are
                                 NameCase{"qcar", "qcar"},
                                 NameCase{"cf0", "cf0"},
                                 NameCase{"Robot_A", "Robot_A"},
                                 // The subject streamed by the lab system
                                 NameCase{"g1-unitree-torso-cam", "g1_unitree_torso_cam"},
                                 // Other characters Tracker accepts
                                 NameCase{"my robot.v2", "my_robot_v2"},
                                 NameCase{"arm/left", "arm_left"},
                                 // UTF-8 'Ä' is two bytes, each replaced
                                 NameCase{"Roboter-\xC3\x84", "Roboter___"},
                                 // Tokens must start with a letter
                                 NameCase{"1robot", "vicon_1robot"},
                                 NameCase{"-robot", "vicon_robot"},
                                 NameCase{"_hidden", "vicon_hidden"},
                                 NameCase{"", "vicon"}));

    // ---------------------------------------------------------------------------------------------
    // Frame ids and topic names of a segment
    // ---------------------------------------------------------------------------------------------

    TEST(RosNames, SegmentFrameIdUsesConvertedNames)
    {
        EXPECT_EQ(
            vicon_receiver::make_segment_frame_id("g1-unitree-torso-cam", "g1-unitree-torso-cam"),
            "g1_unitree_torso_cam_g1_unitree_torso_cam");
        EXPECT_EQ(vicon_receiver::make_segment_frame_id("qcar", "qcar"), "qcar_qcar");
    }

    TEST(RosNames, SegmentTopicNameUsesConvertedNames)
    {
        const std::string topic_name = vicon_receiver::make_segment_topic_name(
            "vicon", "g1-unitree-torso-cam", "g1-unitree-torso-cam");
        EXPECT_EQ(topic_name, "vicon/g1_unitree_torso_cam/g1_unitree_torso_cam");
        EXPECT_EQ(topic_name_problem("/" + topic_name), "");
    }

    TEST(RosNames, SlashInViconNameDoesNotAddNamespaceLevels)
    {
        const std::string topic_name = vicon_receiver::make_segment_topic_name("vicon", "a/b", "c");
        EXPECT_EQ(topic_name, "vicon/a_b/c");
    }
} // namespace name_conversion_tests

// =================================================================================================
// SDK results -> messages, transforms and times
// =================================================================================================
namespace frame_conversion_tests
{
    namespace sdk = ViconDataStreamSDK::CPP;
    using vicon_receiver::is_valid_pose;
    using vicon_receiver::make_marker_state;
    using vicon_receiver::make_segment_state;

    namespace
    {
        /// Translation result of the SDK [mm].
        sdk::Output_GetSegmentGlobalTranslation make_translation(
            double x_mm, double y_mm, double z_mm, bool occluded, sdk::Result::Enum result)
        {
            sdk::Output_GetSegmentGlobalTranslation output{};
            output.Result = result;
            output.Translation[0] = x_mm;
            output.Translation[1] = y_mm;
            output.Translation[2] = z_mm;
            output.Occluded = occluded;
            return output;
        }

        /// Rotation result of the SDK (quaternion x, y, z, w).
        sdk::Output_GetSegmentGlobalRotationQuaternion make_rotation(
            double x, double y, double z, double w, bool occluded, sdk::Result::Enum result)
        {
            sdk::Output_GetSegmentGlobalRotationQuaternion output{};
            output.Result = result;
            output.Rotation[0] = x;
            output.Rotation[1] = y;
            output.Rotation[2] = z;
            output.Rotation[3] = w;
            output.Occluded = occluded;
            return output;
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // Segments
    // ---------------------------------------------------------------------------------------------

    TEST(FrameConversion, VisibleSegmentInMetres)
    {
        // Values of the live recording: 8.5 m from the Vicon origin, nearly identity rotation
        const auto segment = make_segment_state(
            "g1-unitree-torso-cam",
            "g1_unitree_torso_cam_g1_unitree_torso_cam",
            make_translation(8525.63, 6349.81, 1154.34, false, sdk::Result::Success),
            make_rotation(0.0047, -0.0007, -0.0006, 0.99998857, false, sdk::Result::Success));

        EXPECT_EQ(segment.name, "g1-unitree-torso-cam");
        EXPECT_EQ(segment.frame_id, "g1_unitree_torso_cam_g1_unitree_torso_cam");
        EXPECT_DOUBLE_EQ(segment.position.x, 8.52563);
        EXPECT_DOUBLE_EQ(segment.position.y, 6.34981);
        EXPECT_DOUBLE_EQ(segment.position.z, 1.15434);
        EXPECT_DOUBLE_EQ(segment.orientation.x, 0.0047);
        EXPECT_DOUBLE_EQ(segment.orientation.w, 0.99998857);
        EXPECT_FALSE(segment.translation_occluded);
        EXPECT_FALSE(segment.rotation_occluded);
        EXPECT_TRUE(is_valid_pose(segment));
    }

    TEST(FrameConversion, OccludedSegmentIsKeptButNotValid)
    {
        // What Vicon sends for an occluded segment (first live frame): all zeros, flagged
        const auto segment =
            make_segment_state("seg",
                               "subj_seg",
                               make_translation(0.0, 0.0, 0.0, true, sdk::Result::Success),
                               make_rotation(0.0, 0.0, 0.0, 0.0, true, sdk::Result::Success));

        EXPECT_TRUE(segment.translation_occluded);
        EXPECT_TRUE(segment.rotation_occluded);
        EXPECT_DOUBLE_EQ(segment.orientation.w, 0.0);
        EXPECT_FALSE(is_valid_pose(segment));
    }

    TEST(FrameConversion, OnlyRotationOccludedIsNotValid)
    {
        const auto segment =
            make_segment_state("seg",
                               "subj_seg",
                               make_translation(1000.0, 0.0, 0.0, false, sdk::Result::Success),
                               make_rotation(0.0, 0.0, 0.0, 1.0, true, sdk::Result::Success));
        EXPECT_FALSE(segment.translation_occluded);
        EXPECT_TRUE(segment.rotation_occluded);
        EXPECT_FALSE(is_valid_pose(segment));
    }

    TEST(FrameConversion, FailedCallIsReportedAsOccluded)
    {
        // E.g. InvalidSegmentName: the values are not a measurement
        const auto segment = make_segment_state(
            "seg",
            "subj_seg",
            make_translation(123.0, 0.0, 0.0, false, sdk::Result::InvalidSegmentName),
            make_rotation(0.0, 0.0, 0.0, 1.0, false, sdk::Result::NoFrame));

        EXPECT_TRUE(segment.translation_occluded);
        EXPECT_TRUE(segment.rotation_occluded);
        EXPECT_DOUBLE_EQ(segment.position.x, 0.0);
        EXPECT_DOUBLE_EQ(segment.orientation.w, 0.0);
        EXPECT_FALSE(is_valid_pose(segment));
    }

    TEST(FrameConversion, NonUnitQuaternionIsNotValidEvenIfNotFlagged)
    {
        const auto segment =
            make_segment_state("seg",
                               "subj_seg",
                               make_translation(0.0, 0.0, 0.0, false, sdk::Result::Success),
                               make_rotation(0.0, 0.0, 0.0, 0.0, false, sdk::Result::Success));
        EXPECT_FALSE(is_valid_pose(segment));
    }

    TEST(FrameConversion, SegmentTransformForTf)
    {
        const auto segment = make_segment_state(
            "seg",
            "subj_seg",
            make_translation(1500.0, -250.0, 10.0, false, sdk::Result::Success),
            make_rotation(0.0, 0.0, 0.70710678, 0.70710678, false, sdk::Result::Success));
        const builtin_interfaces::msg::Time stamp = vicon_receiver::to_time_msg(1793364136.811);

        const auto transform = vicon_receiver::make_segment_transform(segment, "vicon", stamp);
        EXPECT_EQ(transform.header.frame_id, "vicon");
        EXPECT_EQ(transform.child_frame_id, "subj_seg");
        EXPECT_EQ(transform.header.stamp, stamp);
        EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.5);
        EXPECT_DOUBLE_EQ(transform.transform.translation.y, -0.25);
        EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.01);
        EXPECT_DOUBLE_EQ(transform.transform.rotation.z, 0.70710678);
    }

    // ---------------------------------------------------------------------------------------------
    // Markers
    // ---------------------------------------------------------------------------------------------

    TEST(FrameConversion, MarkerWithCameras)
    {
        sdk::Output_GetMarkerGlobalTranslation translation{};
        translation.Result = sdk::Result::Success;
        translation.Translation[0] = 8568.16;
        translation.Translation[1] = 6228.37;
        translation.Translation[2] = 1011.22;
        translation.Occluded = false;
        const std::vector<std::uint32_t> cameras = {2144752, 2144760, 2144765};

        const auto marker = make_marker_state(
            "g1-unitree-torso-cam1", "g1-unitree-torso-cam", translation, cameras);
        EXPECT_EQ(marker.name, "g1-unitree-torso-cam1");
        EXPECT_EQ(marker.parent_segment, "g1-unitree-torso-cam");
        EXPECT_DOUBLE_EQ(marker.position.x, 8.56816);
        EXPECT_DOUBLE_EQ(marker.position.z, 1.01122);
        EXPECT_FALSE(marker.occluded);
        EXPECT_EQ(marker.camera_ids, cameras);
    }

    TEST(FrameConversion, FailedMarkerCallIsOccluded)
    {
        sdk::Output_GetMarkerGlobalTranslation translation{};
        translation.Result = sdk::Result::InvalidMarkerName;
        translation.Translation[0] = 99.0;
        const auto marker = make_marker_state("m", "s", translation, {});
        EXPECT_TRUE(marker.occluded);
        EXPECT_DOUBLE_EQ(marker.position.x, 0.0);
    }

    // ---------------------------------------------------------------------------------------------
    // Time stamps
    // ---------------------------------------------------------------------------------------------

    TEST(FrameConversion, TimeRoundTripKeepsMicroseconds)
    {
        const double seconds = 1793364136.811012;
        const builtin_interfaces::msg::Time time = vicon_receiver::to_time_msg(seconds);
        EXPECT_EQ(time.sec, 1793364136);
        EXPECT_NEAR(static_cast<double>(time.nanosec), 811012000.0, 1000.0);
        EXPECT_NEAR(vicon_receiver::to_seconds(time), seconds, 1e-6);
    }

    TEST(FrameConversion, NanosecondsNeverReachOneSecond)
    {
        // Rounding up must carry into the seconds
        const builtin_interfaces::msg::Time time = vicon_receiver::to_time_msg(10.9999999999);
        EXPECT_EQ(time.sec, 11);
        EXPECT_EQ(time.nanosec, 0u);
    }

    TEST(FrameConversion, InvalidTimesAreRejected)
    {
        EXPECT_THROW(vicon_receiver::to_time_msg(-1.0), std::invalid_argument);
        EXPECT_THROW(vicon_receiver::to_time_msg(std::numeric_limits<double>::quiet_NaN()),
                     std::invalid_argument);
    }
} // namespace frame_conversion_tests
