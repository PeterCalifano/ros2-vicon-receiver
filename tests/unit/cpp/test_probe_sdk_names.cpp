/**
 * @file test_probe_sdk_names.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the readable names of the Vicon SDK enumerations.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/sdk_names.hpp"

#include <gtest/gtest.h>

namespace sdk = ViconDataStreamSDK::CPP;

TEST(SdkNames, ResultNamesMatchEnumerators)
{
    EXPECT_EQ(vicon_probe::result_name(sdk::Result::Success), "Success");
    EXPECT_EQ(vicon_probe::result_name(sdk::Result::ClientConnectionFailed),
              "ClientConnectionFailed");
    EXPECT_EQ(vicon_probe::result_name(sdk::Result::NoFrame), "NoFrame");
    EXPECT_EQ(vicon_probe::result_name(sdk::Result::InvalidSubjectName), "InvalidSubjectName");
    EXPECT_EQ(vicon_probe::result_name(sdk::Result::ArgumentOutOfRange), "ArgumentOutOfRange");
}

TEST(SdkNames, EveryResultHasAName)
{
    // From the first to the last enumerator of SDK 1.12: none falls back to "Unknown(<n>)"
    for (int value = sdk::Result::Unknown; value <= sdk::Result::ArgumentOutOfRange; ++value)
    {
        const std::string name = vicon_probe::result_name(static_cast<sdk::Result::Enum>(value));
        EXPECT_EQ(name.find("Unknown("), std::string::npos) << value;
    }
}

TEST(SdkNames, ValuesOutsideTheEnumerationAreNumbered)
{
    // E.g. a result added by a newer SDK
    const int future_value = sdk::Result::ArgumentOutOfRange + 1;
    EXPECT_EQ(vicon_probe::result_name(static_cast<sdk::Result::Enum>(future_value)),
              "Unknown(" + std::to_string(future_value) + ")");
}

TEST(SdkNames, OtherEnumerations)
{
    EXPECT_EQ(vicon_probe::direction_name(sdk::Direction::Forward), "Forward");
    EXPECT_EQ(vicon_probe::direction_name(sdk::Direction::Up), "Up");
    EXPECT_EQ(vicon_probe::server_orientation_name(sdk::ServerOrientation::ZUp), "ZUp");
    EXPECT_EQ(vicon_probe::timecode_standard_name(sdk::TimecodeStandard::PAL), "PAL");
    EXPECT_EQ(vicon_probe::timecode_standard_name(sdk::TimecodeStandard::None), "None");
    EXPECT_EQ(vicon_probe::device_type_name(sdk::DeviceType::ForcePlate), "ForcePlate");
}
