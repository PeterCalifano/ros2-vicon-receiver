/**
 * @file test_console_status_line.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of the console telemetry line (vicon_receiver::ConsoleStatusLine).
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/utilities.hpp"
#include "vicon_receiver/console_status_line.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

using vicon_receiver::ConsoleStatusLine;

// -------------------------------------------------------------------------------------------------
// ConsoleStatusLine
// -------------------------------------------------------------------------------------------------

TEST(ConsoleStatusLine, OneSubjectLine)
{
    // 100 frames in 1 s, 2 dropped, latency 30-32 ms, subject occluded in 10 frames,
    // 3 to 7 cameras per marker
    ConsoleStatusLine reporter;
    for (int frame = 0; frame < 100; ++frame)
    {
        const std::uint64_t dropped = (frame == 50) ? 2 : 0;
        const double latency_s = 0.030 + 0.002 * static_cast<double>(frame % 2);
        reporter.add_frame(dropped, latency_s);

        const bool occluded = frame < 10;
        const std::vector<std::size_t> cameras =
            occluded ? std::vector<std::size_t>{} : std::vector<std::size_t>{3, 5, 7};
        reporter.add_subject("g1-unitree-torso-cam", 1.32, occluded, cameras);
    }

    EXPECT_EQ(reporter.take_line(1.0),
              "100.0 Hz | dropped 2 | latency 31.0 ms (30.0-32.0) | "
              "g1-unitree-torso-cam q=1.32 occl 10% cams 3-7");
}

TEST(ConsoleStatusLine, MissingValuesAreShownAsNotAvailable)
{
    ConsoleStatusLine reporter;
    reporter.add_frame(0, std::nullopt);
    reporter.add_subject("cf0", std::nullopt, false, {});

    EXPECT_EQ(reporter.take_line(0.5),
              "2.0 Hz | dropped 0 | latency n/a | cf0 q=n/a occl 0% cams n/a");
}

TEST(ConsoleStatusLine, SubjectsAreListedByName)
{
    ConsoleStatusLine reporter;
    reporter.add_frame(0, 0.03);
    reporter.add_subject("qcar", 1.0, false, {4});
    reporter.add_subject("cf0", 2.0, true, {});

    const std::string line = reporter.take_line(1.0);
    EXPECT_LT(line.find("| cf0 q=2.00 occl 100% cams n/a"),
              line.find("| qcar q=1.00 occl 0% cams 4-4"));
}

TEST(ConsoleStatusLine, FrameWithoutSubjects)
{
    ConsoleStatusLine reporter;
    reporter.add_frame(0, 0.03);
    EXPECT_EQ(reporter.take_line(1.0),
              "1.0 Hz | dropped 0 | latency 30.0 ms (30.0-30.0) | no subjects");
}

TEST(ConsoleStatusLine, PeriodWithoutFrames)
{
    ConsoleStatusLine reporter;
    EXPECT_EQ(reporter.take_line(2.0), "no frames in the last 2.0 s");
}

TEST(ConsoleStatusLine, TakingTheLineStartsANewPeriod)
{
    ConsoleStatusLine reporter;
    reporter.add_frame(5, 0.03);
    reporter.add_subject("cf0", 1.0, true, {3});
    reporter.take_line(1.0);

    // Nothing from the previous period is carried over
    EXPECT_EQ(reporter.take_line(1.0), "no frames in the last 1.0 s");
}

// -------------------------------------------------------------------------------------------------
// Local clock vs Vicon frame clock
// -------------------------------------------------------------------------------------------------

TEST(ConsoleStatusLine, HealthyClockAgreesWithFrameRate)
{
    ConsoleStatusLine reporter;
    for (int frame = 0; frame < 100; ++frame)
    {
        reporter.add_frame(0, 0.03, 100.0);
    }
    const std::optional<double> error = reporter.local_clock_rate_error(1.0);
    ASSERT_TRUE(error.has_value());
    EXPECT_NEAR(error.value(), 0.0, 1e-12);
}

TEST(ConsoleStatusLine, FastLocalClockIsDetected)
{
    // Measured on WSL: 100 Hz frames, none dropped, but 1.042 s of local time per 100 frames
    ConsoleStatusLine reporter;
    for (int frame = 0; frame < 100; ++frame)
    {
        reporter.add_frame(0, 0.03, 100.0);
    }
    const std::optional<double> error = reporter.local_clock_rate_error(1.042);
    ASSERT_TRUE(error.has_value());
    EXPECT_NEAR(error.value(), 1.0 / 1.042 - 1.0, 1e-12);
}

TEST(ConsoleStatusLine, DroppedFramesCountAsProduced)
{
    // 90 received + 10 dropped in 1 s at 100 Hz: the clocks agree
    ConsoleStatusLine reporter;
    for (int frame = 0; frame < 90; ++frame)
    {
        reporter.add_frame(frame == 0 ? 10 : 0, 0.03, 100.0);
    }
    EXPECT_NEAR(reporter.local_clock_rate_error(1.0).value(), 0.0, 1e-12);
}

TEST(ConsoleStatusLine, ClockCheckNeedsEnoughFramesAndARate)
{
    ConsoleStatusLine few_frames;
    few_frames.add_frame(0, 0.03, 100.0);
    EXPECT_FALSE(few_frames.local_clock_rate_error(0.01).has_value());

    ConsoleStatusLine no_rate;
    for (int frame = 0; frame < 100; ++frame)
    {
        no_rate.add_frame(0, 0.03);
    }
    EXPECT_FALSE(no_rate.local_clock_rate_error(1.0).has_value());
}
