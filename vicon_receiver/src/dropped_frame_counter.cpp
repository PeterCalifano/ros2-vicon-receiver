/**
 * @file dropped_frame_counter.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Detection of frames missed between consecutive Vicon frame numbers.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/dropped_frame_counter.hpp"

namespace vicon_receiver
{
    std::uint64_t DroppedFrameCounter::record(std::uint64_t frame_number)
    {
        received_ += 1;

        std::uint64_t gap = 0;
        if (previous_frame_number_.has_value())
        {
            const std::uint64_t previous = previous_frame_number_.value();
            if (frame_number > previous)
            {
                gap = frame_number - previous - 1;
            }
            else
            {
                // Server restart or repeated frame: start counting again from here
                discontinuities_ += 1;
            }
        }

        missed_ += gap;
        previous_frame_number_ = frame_number;
        return gap;
    }

    std::uint64_t DroppedFrameCounter::received() const
    {
        return received_;
    }

    std::uint64_t DroppedFrameCounter::missed() const
    {
        return missed_;
    }

    std::uint64_t DroppedFrameCounter::discontinuities() const
    {
        return discontinuities_;
    }
} // namespace vicon_receiver
