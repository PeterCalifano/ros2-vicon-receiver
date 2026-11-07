/**
 * @file test_utilities.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of utilities.hpp: SDK names.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/utilities.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

// =================================================================================================
// Names of the Vicon SDK enumerations
// =================================================================================================
namespace sdk_names_tests
{
    namespace sdk = ViconDataStreamSDK::CPP;

    TEST(SdkNames, ResultNamesMatchEnumerators)
    {
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::Success), "Success");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::ClientConnectionFailed),
                  "ClientConnectionFailed");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::NoFrame), "NoFrame");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::InvalidSubjectName),
                  "InvalidSubjectName");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::ArgumentOutOfRange),
                  "ArgumentOutOfRange");
    }

    TEST(SdkNames, EveryResultHasAName)
    {
        // From the first to the last enumerator of SDK 1.12: none falls back to "Unknown(<n>)"
        for (int value = sdk::Result::Unknown; value <= sdk::Result::ArgumentOutOfRange; ++value)
        {
            const std::string name =
                vicon_receiver::result_name(static_cast<sdk::Result::Enum>(value));
            EXPECT_EQ(name.find("Unknown("), std::string::npos) << value;
        }
    }

    TEST(SdkNames, ValuesOutsideTheEnumerationAreNumbered)
    {
        // E.g. a result added by a newer SDK
        const int future_value = sdk::Result::ArgumentOutOfRange + 1;
        EXPECT_EQ(vicon_receiver::result_name(static_cast<sdk::Result::Enum>(future_value)),
                  "Unknown(" + std::to_string(future_value) + ")");
    }

    TEST(SdkNames, OtherEnumerations)
    {
        EXPECT_EQ(vicon_receiver::direction_name(sdk::Direction::Forward), "Forward");
        EXPECT_EQ(vicon_receiver::direction_name(sdk::Direction::Up), "Up");
        EXPECT_EQ(vicon_receiver::server_orientation_name(sdk::ServerOrientation::ZUp), "ZUp");
        EXPECT_EQ(vicon_receiver::timecode_standard_name(sdk::TimecodeStandard::PAL), "PAL");
        EXPECT_EQ(vicon_receiver::timecode_standard_name(sdk::TimecodeStandard::None), "None");
        EXPECT_EQ(vicon_receiver::device_type_name(sdk::DeviceType::ForcePlate), "ForcePlate");
    }

    TEST(SdkNames, StreamModeNamesRoundTrip)
    {
        for (const sdk::StreamMode::Enum mode : {sdk::StreamMode::ServerPush,
                                                 sdk::StreamMode::ClientPull,
                                                 sdk::StreamMode::ClientPullPreFetch})
        {
            const std::string name = vicon_receiver::stream_mode_name(mode);
            EXPECT_EQ(vicon_receiver::stream_mode_from_name(name), mode) << name;
        }
        EXPECT_EQ(vicon_receiver::stream_mode_name(sdk::StreamMode::ServerPush), "ServerPush");
    }

    TEST(SdkNames, UnknownStreamModeNameIsRejected)
    {
        // Names are case-sensitive, as written in the parameters file
        EXPECT_THROW(vicon_receiver::stream_mode_from_name("serverpush"), std::invalid_argument);
        EXPECT_THROW(vicon_receiver::stream_mode_from_name(""), std::invalid_argument);
    }
} // namespace sdk_names_tests

// =================================================================================================
// Running statistics
// =================================================================================================
namespace running_min_mean_max_tests
{
    using vicon_receiver::RunningMinMeanMax;

    TEST(RunningMinMeanMax, EmptyStatisticsAreNaN)
    {
        const RunningMinMeanMax statistics;
        EXPECT_EQ(statistics.count(), 0u);
        EXPECT_TRUE(std::isnan(statistics.minimum()));
        EXPECT_TRUE(std::isnan(statistics.mean()));
        EXPECT_TRUE(std::isnan(statistics.maximum()));
    }

    TEST(RunningMinMeanMax, MinimumMeanMaximum)
    {
        RunningMinMeanMax statistics;
        for (const double sample : {3.0, 1.0, 2.0})
        {
            statistics.add(sample);
        }
        EXPECT_EQ(statistics.count(), 3u);
        EXPECT_DOUBLE_EQ(statistics.minimum(), 1.0);
        EXPECT_DOUBLE_EQ(statistics.mean(), 2.0);
        EXPECT_DOUBLE_EQ(statistics.maximum(), 3.0);

        statistics.reset();
        EXPECT_EQ(statistics.count(), 0u);
    }
} // namespace running_min_mean_max_tests
