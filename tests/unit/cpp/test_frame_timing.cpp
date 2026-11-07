/**
 * @file test_frame_timing.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of frame_timing.hpp: acquisition time, dropped/stale frames, leftover frames,
 *        local clock rate.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/frame_timing.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
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

    TEST(FrameJump, AnyJumpAtConnectionStartMeansStaleFrames)
    {
        // Live case: previous session ended 0.45 s earlier, jump of 46 frames between the first two
        // frames of the new connection, received back to back
        EXPECT_TRUE(vicon_receiver::is_stale_previous_frame(45, 0.0, 100.0, 1, 1.0));
        // Consecutive first frames: nothing stale
        EXPECT_FALSE(vicon_receiver::is_stale_previous_frame(0, 0.0, 100.0, 1, 1.0));
        // Live case: two left-over frames, the jump comes with the third frame
        EXPECT_TRUE(vicon_receiver::is_stale_previous_frame(42, 0.002, 100.0, 2, 1.0));
        // Last frame of the start-up window, then the normal rule
        EXPECT_TRUE(vicon_receiver::is_stale_previous_frame(
            42, 0.002, 100.0, vicon_receiver::kConnectionStartupFrames, 1.0));
        EXPECT_FALSE(vicon_receiver::is_stale_previous_frame(
            42, 0.002, 100.0, vicon_receiver::kConnectionStartupFrames + 1, 1.0));
    }

    TEST(FrameJump, LaterJumpsUseThePlausibilityRule)
    {
        // The same 45-frame jump later in the session is plausible: counted as dropped frames
        EXPECT_FALSE(vicon_receiver::is_stale_previous_frame(45, 0.0, 100.0, 500, 1.0));
        EXPECT_TRUE(vicon_receiver::is_stale_previous_frame(18469, 0.01, 100.0, 500, 1.0));
    }
} // namespace dropped_frame_counter_tests

// =================================================================================================
// Frames left over from the server's previous session
// =================================================================================================
namespace startup_leftover_frame_filter_tests
{
    // ---------------------------------------------------------------------------------------------
    // StartupLeftoverFrameFilter: frames left over from the previous session are held back
    // ---------------------------------------------------------------------------------------------

    using vicon_receiver::StartupLeftoverFrameFilter;

    namespace
    {
        /// Frame rate of the live recordings [Hz].
        constexpr double kFilterRateHz = 100.0;

        /// Run length used by the node.
        constexpr std::uint64_t kConfirmationFrames = 3;
    } // namespace

    TEST(StartupLeftoverFrameFilter, NeedsARunOfAtLeastTwo)
    {
        EXPECT_THROW(StartupLeftoverFrameFilter(1), std::invalid_argument);
        EXPECT_NO_THROW(StartupLeftoverFrameFilter(2));
    }

    TEST(StartupLeftoverFrameFilter, OneLeftoverFrame)
    {
        // Live case (b2b.json, receive times relative to the first frame): leftover 1985661, then
        // the live stream from 1985710
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(1985661, 0.0, kFilterRateHz));
        EXPECT_FALSE(filter.accept(1985710, 0.0001, kFilterRateHz));
        EXPECT_FALSE(filter.accept(1985711, 0.0035, kFilterRateHz));
        EXPECT_TRUE(filter.accept(1985712, 0.0134, kFilterRateHz));
        EXPECT_TRUE(filter.confirmed());
        EXPECT_EQ(filter.rejected(), 3u);
    }

    TEST(StartupLeftoverFrameFilter, TwoLeftoverFrames)
    {
        // Live case (b2b_ServerPush.json): leftovers 1976227 and 1976234, live from 1976277
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(1976227, 0.0, kFilterRateHz));
        EXPECT_FALSE(filter.accept(1976234, 0.0001, kFilterRateHz));
        EXPECT_FALSE(filter.accept(1976277, 0.0024, kFilterRateHz));
        EXPECT_FALSE(filter.accept(1976278, 0.0123, kFilterRateHz));
        EXPECT_TRUE(filter.accept(1976279, 0.0230, kFilterRateHz));
        EXPECT_EQ(filter.rejected(), 4u);
    }

    TEST(StartupLeftoverFrameFilter, ConsecutiveLeftoversOfAStoppedTrackerAreNeverConfirmed)
    {
        // Live case (2026-10-30, Tracker not streaming): leftovers 2175488 and 2175489 back to back,
        // then nothing. Even a third consecutive leftover arriving back to back is not live.
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(2175488, 0.0, kFilterRateHz));
        EXPECT_FALSE(filter.accept(2175489, 0.0001, kFilterRateHz));
        EXPECT_FALSE(filter.accept(2175490, 0.0002, kFilterRateHz));
        EXPECT_FALSE(filter.confirmed());
    }

    TEST(StartupLeftoverFrameFilter, BurstAtStartIsConfirmedOnceFramesFollowAtTheRate)
    {
        // Three frames queued while connecting arrive together, then live frames every 10 ms
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(500, 0.0, kFilterRateHz));
        EXPECT_FALSE(filter.accept(501, 0.0001, kFilterRateHz));
        EXPECT_FALSE(filter.accept(502, 0.0002, kFilterRateHz));
        EXPECT_FALSE(filter.accept(503, 0.0100, kFilterRateHz));
        EXPECT_TRUE(filter.accept(504, 0.0210, kFilterRateHz));
    }

    TEST(StartupLeftoverFrameFilter, UnknownRateChecksFrameNumbersOnly)
    {
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(10, 0.0, 0.0));
        EXPECT_FALSE(filter.accept(11, 0.0, 0.0));
        EXPECT_TRUE(filter.accept(12, 0.0, 0.0));
    }

    TEST(StartupLeftoverFrameFilter, ConfirmedStreamAcceptsGapsAndRestarts)
    {
        // Once live, dropped frames and server restarts are handled by the gap tracker, not here
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        filter.accept(100, 0.00, kFilterRateHz);
        filter.accept(101, 0.01, kFilterRateHz);
        filter.accept(102, 0.02, kFilterRateHz);
        EXPECT_TRUE(filter.accept(150, 0.03, kFilterRateHz));
        EXPECT_TRUE(filter.accept(3, 0.04, kFilterRateHz));
    }

    TEST(StartupLeftoverFrameFilter, RestartHoldsBackFramesAgain)
    {
        // After a reconnection the server may send leftovers again
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        filter.accept(100, 0.00, kFilterRateHz);
        filter.accept(101, 0.01, kFilterRateHz);
        filter.accept(102, 0.02, kFilterRateHz);
        filter.restart();
        EXPECT_FALSE(filter.confirmed());
        EXPECT_EQ(filter.rejected(), 0u);
        EXPECT_FALSE(filter.accept(102, 5.00, kFilterRateHz));
        EXPECT_FALSE(filter.accept(103, 5.01, kFilterRateHz));
        EXPECT_TRUE(filter.accept(104, 5.02, kFilterRateHz));
    }

    TEST(StartupLeftoverFrameFilter, ThreeConsecutiveLeftoversBackToBackAreNotLive)
    {
        // Live case (2026-10-30, reconnecting to a Tracker that was not streaming): 2175488, then
        // 2175495, 2175496, 2175497 within 1.7 ms. Three consecutive numbers, but far too fast for
        // live frames at 100 Hz (20 ms): nothing is accepted.
        StartupLeftoverFrameFilter filter(kConfirmationFrames);
        EXPECT_FALSE(filter.accept(2175488, 0.0, kFilterRateHz));
        EXPECT_FALSE(filter.accept(2175495, 0.00016, kFilterRateHz));
        EXPECT_FALSE(filter.accept(2175496, 0.00157, kFilterRateHz));
        EXPECT_FALSE(filter.accept(2175497, 0.00171, kFilterRateHz));
        EXPECT_FALSE(filter.confirmed());
    }
} // namespace startup_leftover_frame_filter_tests

// =================================================================================================
// Rate of the local clock
// =================================================================================================
namespace local_clock_rate_monitor_tests
{
    using vicon_receiver::LocalClockRateMonitor;

    namespace
    {
        /// Frame rate of the synthetic stream [Hz] (as in the live recording).
        constexpr double kRateHz = 100.0;

        /// Measurement and block durations used by the node [s].
        constexpr double kMeasurementS = 10.0;
        constexpr double kBlockS = 1.0;

        /// First frame number (magnitude seen on a live Tracker).
        constexpr std::uint64_t kFirstFrame = 1958072;

        /// Monotonic local time of the first frame [s].
        constexpr double kStartS = 5000.0;

        /**
     * @brief Feed frames to the check and collect the rate errors it reports.
     *
     * @param check        Check under test.
     * @param frame_count  Number of consecutive frames.
     * @param rate_error   Rate error of the local clock (0.042: 4.2 % fast).
     * @param delay        Transport delay of each frame [s], as a function of its index.
     */
        template <typename DelayFunction>
        std::vector<double> run_stream(LocalClockRateMonitor &check,
                                       std::uint64_t frame_count,
                                       double rate_error,
                                       DelayFunction delay)
        {
            std::vector<double> errors;
            for (std::uint64_t index = 0; index < frame_count; ++index)
            {
                const double vicon_time_s = static_cast<double>(index) / kRateHz;
                const double local_time_s =
                    kStartS + (1.0 + rate_error) * vicon_time_s + delay(index);
                const std::optional<double> error =
                    check.add_frame(kFirstFrame + index, kRateHz, local_time_s);
                if (error.has_value())
                {
                    errors.push_back(error.value());
                }
            }
            return errors;
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------------

    TEST(LocalClockRateMonitor, RejectsInvalidDurations)
    {
        EXPECT_THROW(LocalClockRateMonitor(10.0, 0.0), std::invalid_argument);
        EXPECT_THROW(LocalClockRateMonitor(2.0, 1.0), std::invalid_argument);
        EXPECT_NO_THROW(LocalClockRateMonitor(kMeasurementS, kBlockS));
    }

    TEST(LocalClockRateMonitor, RejectsInvalidRate)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        EXPECT_THROW(check.add_frame(kFirstFrame, 0.0, kStartS), std::invalid_argument);
    }

    // ---------------------------------------------------------------------------------------------
    // Measurements
    // ---------------------------------------------------------------------------------------------

    TEST(LocalClockRateMonitor, NothingBeforeTheEndOfAMeasurement)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        const std::vector<double> errors =
            run_stream(check, 900, 0.0, [](std::uint64_t) { return 0.005; });
        EXPECT_TRUE(errors.empty());
    }

    TEST(LocalClockRateMonitor, HealthyClockWithArrivalJitterAgrees)
    {
        // Jitter up to 10 ms on every frame (as measured live): far below the 1 % warning level
        std::mt19937 generator(7);
        std::uniform_real_distribution<double> jitter(0.0, 0.010);
        std::vector<double> delays(2500);
        for (double &delay : delays)
        {
            delay = 0.002 + jitter(generator);
        }

        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        const std::vector<double> errors = run_stream(
            check, delays.size(), 0.0, [&](std::uint64_t index) { return delays[index]; });
        ASSERT_EQ(errors.size(), 2u);
        for (const double error : errors)
        {
            EXPECT_NEAR(error, 0.0, 0.001);
        }
    }

    TEST(LocalClockRateMonitor, FastLocalClockIsMeasured)
    {
        // Measured on WSL with the 'tsc' clocksource: the local clock runs 4.2 % fast
        std::mt19937 generator(11);
        std::uniform_real_distribution<double> jitter(0.0, 0.010);
        std::vector<double> delays(1100);
        for (double &delay : delays)
        {
            delay = jitter(generator);
        }

        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        const std::vector<double> errors = run_stream(
            check, delays.size(), 0.042, [&](std::uint64_t index) { return delays[index]; });
        ASSERT_EQ(errors.size(), 1u);
        EXPECT_NEAR(errors.front(), 0.042, 0.001);
    }

    TEST(LocalClockRateMonitor, SlowLocalClockIsNegative)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        const std::vector<double> errors =
            run_stream(check, 1100, -0.02, [](std::uint64_t) { return 0.003; });
        ASSERT_EQ(errors.size(), 1u);
        EXPECT_NEAR(errors.front(), -0.02, 1e-6);
    }

    TEST(LocalClockRateMonitor, NetworkStallInTheMiddleDoesNotBias)
    {
        // 0.5 s stall: frames 400-449 queue up and arrive in a burst when the network recovers
        auto delay = [](std::uint64_t index)
        {
            if (index >= 400 && index < 450)
            {
                return 0.003 + 0.5 - static_cast<double>(index - 400) / kRateHz;
            }
            return 0.003;
        };

        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        const std::vector<double> errors = run_stream(check, 1100, 0.0, delay);
        ASSERT_EQ(errors.size(), 1u);
        EXPECT_NEAR(errors.front(), 0.0, 1e-6);
        EXPECT_EQ(check.restarts(), 0u);
    }

    TEST(LocalClockRateMonitor, DroppedFramesDoNotMatter)
    {
        // Only every third frame received: the frame numbers still give the Vicon time
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        std::vector<double> errors;
        for (std::uint64_t index = 0; index < 1200; index += 3)
        {
            const double local_time_s =
                kStartS + 1.01 * static_cast<double>(index) / kRateHz + 0.004;
            const std::optional<double> error =
                check.add_frame(kFirstFrame + index, kRateHz, local_time_s);
            if (error.has_value())
            {
                errors.push_back(error.value());
            }
        }
        ASSERT_EQ(errors.size(), 1u);
        EXPECT_NEAR(errors.front(), 0.01, 1e-6);
    }

    // ---------------------------------------------------------------------------------------------
    // Broken timelines
    // ---------------------------------------------------------------------------------------------

    TEST(LocalClockRateMonitor, NonIncreasingFrameNumberRestartsTheMeasurement)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        run_stream(check, 900, 0.0, [](std::uint64_t) { return 0.003; });

        // Server restarted: frame numbers start again lower, 9 s of measurement are dropped
        EXPECT_FALSE(check.add_frame(10, kRateHz, kStartS + 9.01).has_value());
        EXPECT_EQ(check.restarts(), 1u);
        EXPECT_FALSE(check.add_frame(11, kRateHz, kStartS + 9.02).has_value());
    }

    TEST(LocalClockRateMonitor, PauseLongerThanABlockRestartsTheMeasurement)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        run_stream(check, 500, 0.0, [](std::uint64_t) { return 0.003; });

        // Tracker paused for 3 s
        EXPECT_FALSE(check.add_frame(kFirstFrame + 500, kRateHz, kStartS + 8.0).has_value());
        EXPECT_EQ(check.restarts(), 1u);
    }

    TEST(LocalClockRateMonitor, RateChangeRestartsTheMeasurement)
    {
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        run_stream(check, 500, 0.0, [](std::uint64_t) { return 0.003; });
        EXPECT_FALSE(check.add_frame(kFirstFrame + 500, 200.0, kStartS + 5.0).has_value());
        EXPECT_EQ(check.restarts(), 1u);
    }

    TEST(LocalClockRateMonitor, RestartDropsTheMeasurement)
    {
        // As after a reconnection: nothing is reported until a full new measurement
        LocalClockRateMonitor check(kMeasurementS, kBlockS);
        run_stream(check, 900, 0.0, [](std::uint64_t) { return 0.003; });
        check.restart();
        for (std::uint64_t index = 900; index < 1100; ++index)
        {
            const double local_time_s = kStartS + static_cast<double>(index) / kRateHz;
            EXPECT_FALSE(check.add_frame(kFirstFrame + index, kRateHz, local_time_s).has_value());
        }
    }
} // namespace local_clock_rate_monitor_tests
