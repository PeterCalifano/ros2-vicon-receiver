/**
 * @file test_probe_frame_gap.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of vicon_probe::FrameGapTracker (missed-frame detection).
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/frame_gap.hpp"

#include <gtest/gtest.h>

using vicon_probe::FrameGapTracker;

TEST(FrameGapTracker, StartsEmpty)
{
    const FrameGapTracker tracker;
    EXPECT_EQ(tracker.received(), 0u);
    EXPECT_EQ(tracker.missed(), 0u);
    EXPECT_EQ(tracker.discontinuities(), 0u);
}

TEST(FrameGapTracker, FirstFrameHasNoGap)
{
    FrameGapTracker tracker;
    EXPECT_EQ(tracker.record(123456), 0u);
    EXPECT_EQ(tracker.received(), 1u);
}

TEST(FrameGapTracker, ConsecutiveFramesHaveNoGap)
{
    FrameGapTracker tracker;
    tracker.record(10);
    EXPECT_EQ(tracker.record(11), 0u);
    EXPECT_EQ(tracker.record(12), 0u);
    EXPECT_EQ(tracker.missed(), 0u);
}

TEST(FrameGapTracker, JumpsCountMissedFrames)
{
    FrameGapTracker tracker;
    tracker.record(10);
    EXPECT_EQ(tracker.record(13), 2u); // 11 and 12 missed
    EXPECT_EQ(tracker.record(14), 0u);
    EXPECT_EQ(tracker.record(20), 5u); // 15..19 missed
    EXPECT_EQ(tracker.missed(), 7u);
    EXPECT_EQ(tracker.received(), 4u);
}

TEST(FrameGapTracker, RepeatedFrameIsADiscontinuity)
{
    FrameGapTracker tracker;
    tracker.record(10);
    EXPECT_EQ(tracker.record(10), 0u);
    EXPECT_EQ(tracker.discontinuities(), 1u);
    EXPECT_EQ(tracker.missed(), 0u);
}

TEST(FrameGapTracker, DecreasingFrameRestartsTheSequence)
{
    // E.g. the server restarted its numbering
    FrameGapTracker tracker;
    tracker.record(5000);
    EXPECT_EQ(tracker.record(3), 0u);
    EXPECT_EQ(tracker.discontinuities(), 1u);

    // Counting continues from the new number
    EXPECT_EQ(tracker.record(5), 1u);
    EXPECT_EQ(tracker.missed(), 1u);
}
