/**
 * @file test_probe_options.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the vicon_probe command-line parsing.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/probe_options.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

using vicon_probe::parse_probe_options;
using vicon_probe::ProbeOptions;

namespace
{
    /**
     * @brief Parse arguments that must be invalid and return the error message.
     *
     * Fails the test if the arguments are accepted.
     */
    std::string parse_error(const std::vector<std::string> &arguments)
    {
        try
        {
            parse_probe_options(arguments);
        }
        catch (const std::invalid_argument &error)
        {
            return error.what();
        }
        ADD_FAILURE() << "arguments were accepted";
        return "";
    }
} // namespace

// -------------------------------------------------------------------------------------------------
// Valid arguments
// -------------------------------------------------------------------------------------------------

TEST(ProbeOptions, RequiredOptionsOnly)
{
    const ProbeOptions options = parse_probe_options({"--host", "192.168.10.1", "--frames", "100"});

    EXPECT_EQ(options.host, "192.168.10.1");
    EXPECT_EQ(options.frame_count, 100u);
    EXPECT_TRUE(options.json_path.empty());
    EXPECT_FALSE(options.connection_timeout_ms.has_value());
    EXPECT_FALSE(options.include_centroids);
    EXPECT_FALSE(options.show_help);
}

TEST(ProbeOptions, AllOptionsInAnyOrder)
{
    const ProbeOptions options = parse_probe_options({"--centroids",
                                                      "--json",
                                                      "out.json",
                                                      "--frames",
                                                      "5",
                                                      "--timeout-ms",
                                                      "500",
                                                      "--host",
                                                      "10.250.223.40:801"});

    EXPECT_EQ(options.host, "10.250.223.40:801");
    EXPECT_EQ(options.frame_count, 5u);
    EXPECT_EQ(options.json_path, "out.json");
    ASSERT_TRUE(options.connection_timeout_ms.has_value());
    EXPECT_EQ(options.connection_timeout_ms.value(), 500u);
    EXPECT_TRUE(options.include_centroids);
}

TEST(ProbeOptions, HelpNeedsNoOtherOption)
{
    EXPECT_TRUE(parse_probe_options({"--help"}).show_help);
    EXPECT_TRUE(parse_probe_options({"-h"}).show_help);
}

TEST(ProbeOptions, MinimumTimeoutIsAccepted)
{
    const ProbeOptions options =
        parse_probe_options({"--host", "h", "--frames", "1", "--timeout-ms", "10"});
    EXPECT_EQ(options.connection_timeout_ms.value(), vicon_probe::kMinConnectionTimeoutMs);
}

// -------------------------------------------------------------------------------------------------
// Invalid arguments
// -------------------------------------------------------------------------------------------------

TEST(ProbeOptions, HostIsRequired)
{
    EXPECT_EQ(parse_error({"--frames", "10"}), "--host is required");
}

TEST(ProbeOptions, FramesAreRequired)
{
    // No hidden default number of frames
    EXPECT_EQ(parse_error({"--host", "192.168.10.1"}), "--frames is required");
}

TEST(ProbeOptions, OptionWithoutValue)
{
    EXPECT_EQ(parse_error({"--frames", "10", "--host"}), "--host requires a value");
}

TEST(ProbeOptions, UnknownOption)
{
    EXPECT_EQ(parse_error({"--host", "h", "--frames", "1", "--verbose"}),
              "unknown option '--verbose'");
}

class InvalidFramesTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(InvalidFramesTest, InvalidFrameCountIsRejected)
{
    const std::string message = parse_error({"--host", "h", "--frames", GetParam()});
    EXPECT_NE(message.find("--frames expects an integer >= 1"), std::string::npos) << message;
}

// Zero, signs, spaces, decimals, text and values above UINT_MAX
INSTANTIATE_TEST_SUITE_P(
    BadValues,
    InvalidFramesTest,
    ::testing::Values(
        "0", "-5", "+5", " 5", "5.0", "ten", "", "4294967296", "99999999999999999999"));

TEST(ProbeOptions, TimeoutBelowSdkMinimumIsRejected)
{
    const std::string message = parse_error({"--host", "h", "--frames", "1", "--timeout-ms", "9"});
    EXPECT_NE(message.find("--timeout-ms expects an integer >= 10"), std::string::npos) << message;
}

TEST(ProbeOptions, UsageListsEveryOption)
{
    const std::string usage = vicon_probe::probe_usage("vicon_probe");
    for (const char *option :
         {"--host", "--frames", "--json", "--timeout-ms", "--centroids", "--help"})
    {
        EXPECT_NE(usage.find(option), std::string::npos) << option;
    }
}
