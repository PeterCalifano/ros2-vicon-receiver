/**
 * @file test_name_conversion.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the conversion of Vicon subject/segment names into ROS 2 names. Every
 *        converted name is checked with the ROS middleware validator, so "valid" means exactly
 *        what rclcpp accepts when creating a publisher.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/name_conversion.hpp"

#include <gtest/gtest.h>

#include <string>

#include "rmw/validate_full_topic_name.h"

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
        const rmw_ret_t status =
            rmw_validate_full_topic_name(topic_name.c_str(), &validation_result, &invalid_index);
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

// -------------------------------------------------------------------------------------------------
// Conversion rules
// -------------------------------------------------------------------------------------------------

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
    EXPECT_EQ(vicon_receiver::to_ros_name_token(name_case.vicon_name), name_case.expected_token);
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

// -------------------------------------------------------------------------------------------------
// Frame ids and topic names of a segment
// -------------------------------------------------------------------------------------------------

TEST(RosNames, SegmentFrameIdUsesConvertedNames)
{
    EXPECT_EQ(vicon_receiver::make_segment_frame_id("g1-unitree-torso-cam", "g1-unitree-torso-cam"),
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
