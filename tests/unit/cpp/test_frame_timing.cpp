/**
 * @file test_frame_timing.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of frame_timing.hpp: acquisition time, dropped/stale frames.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/frame_timing.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

// =================================================================================================
// Acquisition time of the frames
// =================================================================================================
namespace acquisition_clock_tests
{
    using vicon_receiver::AcquisitionClock;

    namespace
    {
        /// Frame rate of the synthetic camera timeline [Hz] (as in the live recording).
        constexpr double kRateHz = 100.0;

        /// Frame period [s].
        constexpr double kPeriodS = 1.0 / kRateHz;

        /// Acquisition time of frame 0 on the local clock [s] (Unix-like magnitude).
        constexpr double kStartS = 1.79e9;

        /// Constant pipeline latency reported by the server [s] (as in the live recording).
        constexpr double kLatencyS = 0.031;

        /// Tolerance on times [s]: doubles around 1.8e9 s resolve ~0.2 us.
        constexpr double kTimeToleranceS = 1e-6;

        /// True acquisition time of a frame on the synthetic timeline.
        double true_acquisition_time(std::uint64_t frame_number)
        {
            return kStartS + static_cast<double>(frame_number) * kPeriodS;
        }

        /// Receive time of a frame: acquisition + server latency + transport delay.
        double receive_time(std::uint64_t frame_number, double transport_delay_s)
        {
            return true_acquisition_time(frame_number) + kLatencyS + transport_delay_s;
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // Arguments
    // ---------------------------------------------------------------------------------------------

    TEST(AcquisitionClock, EmptyWindowIsRejected)
    {
        EXPECT_THROW(AcquisitionClock clock(0), std::invalid_argument);
    }

    TEST(AcquisitionClock, InvalidFrameRateIsRejected)
    {
        AcquisitionClock clock(10);
        EXPECT_THROW(clock.update(1, 0.0, kStartS, kLatencyS), std::invalid_argument);
        EXPECT_THROW(clock.update(1, -100.0, kStartS, kLatencyS), std::invalid_argument);
        EXPECT_THROW(clock.update(1, std::nan(""), kStartS, kLatencyS), std::invalid_argument);
    }

    // ---------------------------------------------------------------------------------------------
    // Timeline recovery
    // ---------------------------------------------------------------------------------------------

    TEST(AcquisitionClock, NoTransportDelayGivesExactAcquisitionTimes)
    {
        AcquisitionClock clock(1000);
        for (std::uint64_t frame = 0; frame < 300; ++frame)
        {
            const double stamp = clock.update(frame, kRateHz, receive_time(frame, 0.0), kLatencyS);
            EXPECT_NEAR(stamp, true_acquisition_time(frame), kTimeToleranceS) << frame;
        }
        EXPECT_EQ(clock.restarts(), 1u);
    }

    TEST(AcquisitionClock, PositiveDelaysAreRemovedDownToTheSmallestOne)
    {
        // Random delays of 2..40 ms, bursts of +80 ms every 10th frame (frames arriving together),
        // and one frame (50) with the smallest delay, 0.5 ms
        std::mt19937 generator(42);
        std::uniform_real_distribution<double> delay_distribution(0.002, 0.040);
        constexpr std::uint64_t kFastestFrame = 50;
        constexpr double kSmallestDelayS = 0.0005;

        AcquisitionClock clock(1000);
        double previous_stamp = 0.0;
        for (std::uint64_t frame = 0; frame < 600; ++frame)
        {
            double delay_s = delay_distribution(generator);
            if (frame % 10 == 0)
            {
                delay_s += 0.080;
            }
            if (frame == kFastestFrame)
            {
                delay_s = kSmallestDelayS;
            }

            const double stamp =
                clock.update(frame, kRateHz, receive_time(frame, delay_s), kLatencyS);
            const double error_s = stamp - true_acquisition_time(frame);

            // Never earlier than the truth, never later than this frame's own delay
            EXPECT_GE(error_s, -kTimeToleranceS) << frame;
            EXPECT_LE(error_s, delay_s + kTimeToleranceS) << frame;

            if (frame >= kFastestFrame)
            {
                // From the fastest frame on, only its delay remains, and the spacing is exact
                EXPECT_NEAR(error_s, kSmallestDelayS, kTimeToleranceS) << frame;
                if (frame > kFastestFrame)
                {
                    EXPECT_NEAR(stamp - previous_stamp, kPeriodS, kTimeToleranceS) << frame;
                }
            }
            previous_stamp = stamp;
        }
    }

    TEST(AcquisitionClock, StampsDoNotFollowArrivalJitter)
    {
        // Frames arrive in bursts (receive times jitter by tens of ms), the stamps stay on the grid
        AcquisitionClock clock(1000);
        std::vector<double> stamps;
        for (std::uint64_t frame = 0; frame < 100; ++frame)
        {
            const double burst_delay_s = 0.001 + 0.010 * static_cast<double>(4 - frame % 5);
            stamps.push_back(
                clock.update(frame, kRateHz, receive_time(frame, burst_delay_s), kLatencyS));
        }

        // After the first burst the smallest delay (1 ms) has been seen: exact spacing
        for (std::size_t index = 5; index < stamps.size(); ++index)
        {
            EXPECT_NEAR(stamps[index] - stamps[index - 1], kPeriodS, kTimeToleranceS) << index;
        }
    }

    TEST(AcquisitionClock, DroppedFramesKeepTheTimeline)
    {
        AcquisitionClock clock(1000);
        double stamp_before_gap = 0.0;
        for (std::uint64_t frame = 0; frame < 10; ++frame)
        {
            stamp_before_gap = clock.update(frame, kRateHz, receive_time(frame, 0.001), kLatencyS);
        }

        // Frames 10..14 never arrive
        const double stamp_after_gap =
            clock.update(15, kRateHz, receive_time(15, 0.001), kLatencyS);

        EXPECT_NEAR(stamp_after_gap - stamp_before_gap, 6.0 * kPeriodS, kTimeToleranceS);
        EXPECT_EQ(clock.restarts(), 1u);
    }

    // ---------------------------------------------------------------------------------------------
    // Restarts
    // ---------------------------------------------------------------------------------------------

    TEST(AcquisitionClock, NonIncreasingFrameNumberRestartsTheTimeline)
    {
        AcquisitionClock clock(1000);
        clock.update(100, kRateHz, receive_time(100, 0.001), kLatencyS);
        clock.update(101, kRateHz, receive_time(101, 0.001), kLatencyS);
        EXPECT_EQ(clock.restarts(), 1u);

        // Server restarted its numbering
        const double stamp = clock.update(3, kRateHz, receive_time(200, 0.001), kLatencyS);
        EXPECT_EQ(clock.restarts(), 2u);
        EXPECT_EQ(clock.reference_frame(), 3u);
        EXPECT_NEAR(stamp, true_acquisition_time(200) + 0.001, kTimeToleranceS);

        // A repeated frame number also restarts
        clock.update(3, kRateHz, receive_time(201, 0.001), kLatencyS);
        EXPECT_EQ(clock.restarts(), 3u);
    }

    TEST(AcquisitionClock, FrameRateChangeRestartsTheTimeline)
    {
        AcquisitionClock clock(1000);
        clock.update(10, kRateHz, receive_time(10, 0.001), kLatencyS);
        clock.update(11, 200.0, receive_time(11, 0.001), kLatencyS);
        EXPECT_EQ(clock.restarts(), 2u);
        EXPECT_EQ(clock.reference_frame(), 11u);
    }

    // ---------------------------------------------------------------------------------------------
    // Sliding window
    // ---------------------------------------------------------------------------------------------

    TEST(AcquisitionClock, SmallestDelayLeavesTheWindow)
    {
        // Window of 100 frames: the 0.1 ms delay of frame 10 is forgotten after frame 109
        AcquisitionClock clock(100);
        for (std::uint64_t frame = 0; frame < 200; ++frame)
        {
            const double delay_s = (frame == 10) ? 0.0001 : 0.002;
            const double stamp =
                clock.update(frame, kRateHz, receive_time(frame, delay_s), kLatencyS);
            const double error_s = stamp - true_acquisition_time(frame);

            if (frame >= 10 && frame < 110)
            {
                EXPECT_NEAR(error_s, 0.0001, kTimeToleranceS) << frame;
            }
            if (frame >= 110)
            {
                EXPECT_NEAR(error_s, 0.002, kTimeToleranceS) << frame;
            }
        }
    }

    TEST(AcquisitionClock, SlowClockDriftIsFollowed)
    {
        // The local clock runs 100 ppm faster than the camera clock: over 60 s the receive times
        // drift by 6 ms. A 1 s window keeps the error at the transport delay plus 0.1 ms of drift.
        constexpr double kDrift = 1e-4;
        constexpr double kDelayS = 0.001;
        AcquisitionClock clock(100);

        for (std::uint64_t frame = 0; frame < 6000; ++frame)
        {
            const double local_acquisition_s =
                kStartS + static_cast<double>(frame) * kPeriodS * (1.0 + kDrift);
            const double received_s = local_acquisition_s + kLatencyS + kDelayS;
            const double stamp = clock.update(frame, kRateHz, received_s, kLatencyS);

            const double error_s = stamp - local_acquisition_s;
            EXPECT_GE(error_s, -kTimeToleranceS) << frame;
            EXPECT_LE(error_s, kDelayS + 100.0 * kPeriodS * kDrift + kTimeToleranceS) << frame;
        }
    }

    TEST(AcquisitionClock, ResetStartsANewTimeline)
    {
        AcquisitionClock clock(1000);
        clock.update(100, kRateHz, receive_time(100, 0.001), kLatencyS);
        clock.reset();

        // After a reset, the next frame is the reference of a new timeline
        const double stamp = clock.update(200, kRateHz, receive_time(200, 0.002), kLatencyS);
        EXPECT_EQ(clock.restarts(), 2u);
        EXPECT_EQ(clock.reference_frame(), 200u);
        EXPECT_NEAR(stamp, true_acquisition_time(200) + 0.002, kTimeToleranceS);
    }
} // namespace acquisition_clock_tests

// =================================================================================================
// Dropped and stale frames
// =================================================================================================
namespace dropped_frame_counter_tests
{
    using vicon_receiver::DroppedFrameCounter;

    TEST(DroppedFrameCounter, StartsEmpty)
    {
        const DroppedFrameCounter tracker;
        EXPECT_EQ(tracker.received(), 0u);
        EXPECT_EQ(tracker.missed(), 0u);
        EXPECT_EQ(tracker.discontinuities(), 0u);
    }

    TEST(DroppedFrameCounter, FirstFrameHasNoGap)
    {
        DroppedFrameCounter tracker;
        EXPECT_EQ(tracker.record(123456), 0u);
        EXPECT_EQ(tracker.received(), 1u);
    }

    TEST(DroppedFrameCounter, ConsecutiveFramesHaveNoGap)
    {
        DroppedFrameCounter tracker;
        tracker.record(10);
        EXPECT_EQ(tracker.record(11), 0u);
        EXPECT_EQ(tracker.record(12), 0u);
        EXPECT_EQ(tracker.missed(), 0u);
    }

    TEST(DroppedFrameCounter, JumpsCountMissedFrames)
    {
        DroppedFrameCounter tracker;
        tracker.record(10);
        EXPECT_EQ(tracker.record(13), 2u); // 11 and 12 missed
        EXPECT_EQ(tracker.record(14), 0u);
        EXPECT_EQ(tracker.record(20), 5u); // 15..19 missed
        EXPECT_EQ(tracker.missed(), 7u);
        EXPECT_EQ(tracker.received(), 4u);
    }

    TEST(DroppedFrameCounter, RepeatedFrameIsADiscontinuity)
    {
        DroppedFrameCounter tracker;
        tracker.record(10);
        EXPECT_EQ(tracker.record(10), 0u);
        EXPECT_EQ(tracker.discontinuities(), 1u);
        EXPECT_EQ(tracker.missed(), 0u);
    }

    TEST(DroppedFrameCounter, DecreasingFrameRestartsTheSequence)
    {
        // E.g. the server restarted its numbering
        DroppedFrameCounter tracker;
        tracker.record(5000);
        EXPECT_EQ(tracker.record(3), 0u);
        EXPECT_EQ(tracker.discontinuities(), 1u);

        // Counting continues from the new number
        EXPECT_EQ(tracker.record(5), 1u);
        EXPECT_EQ(tracker.missed(), 1u);
    }

    TEST(DroppedFrameCounter, RestartSequenceForgetsThePreviousFrame)
    {
        DroppedFrameCounter tracker;
        tracker.record(10);
        EXPECT_EQ(tracker.last_frame_number().value(), 10u);

        tracker.restart_sequence();
        EXPECT_FALSE(tracker.last_frame_number().has_value());

        // The next frame is a first frame: no gap, whatever its number
        EXPECT_EQ(tracker.record(5000), 0u);
        EXPECT_EQ(tracker.missed(), 0u);
    }

    TEST(FrameJump, StaleFirstFrameIsImplausible)
    {
        // Live case: jump of 18469 frames between two frames received 10 ms apart at 100 Hz
        EXPECT_TRUE(vicon_receiver::is_implausible_frame_jump(18469, 0.010, 100.0, 1.0));
    }

    TEST(FrameJump, RealOutageIsPlausible)
    {
        // 5 s without frames at 100 Hz: up to 600 frames can be missing (with a 1 s margin)
        EXPECT_FALSE(vicon_receiver::is_implausible_frame_jump(499, 5.0, 100.0, 1.0));
        EXPECT_FALSE(vicon_receiver::is_implausible_frame_jump(600, 5.0, 100.0, 1.0));
        EXPECT_TRUE(vicon_receiver::is_implausible_frame_jump(601, 5.0, 100.0, 1.0));
    }

    TEST(FrameJump, NegativeIntervalIsTreatedAsZero)
    {
        // A local clock stepped backwards must not make every gap implausible
        EXPECT_FALSE(vicon_receiver::is_implausible_frame_jump(50, -0.5, 100.0, 1.0));
    }
} // namespace dropped_frame_counter_tests
