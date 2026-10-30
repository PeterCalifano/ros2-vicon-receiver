/**
 * @file frame_gap.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Detection of frames missed between consecutive Vicon frame numbers.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_PROBE_FRAME_GAP_HPP
#define VICON_RECEIVER_PROBE_FRAME_GAP_HPP

#include <cstdint>
#include <optional>

namespace vicon_probe
{
    /**
     * @brief Counts the frames missed by a client from the sequence of received frame numbers.
     *
     * The Vicon server numbers its frames consecutively. A jump from N to N + k (k > 1) means that
     * k - 1 frames were not received. A frame number that does not increase (server restarted, or
     * the same frame delivered twice) is counted as a discontinuity and restarts the sequence.
     */
    class FrameGapTracker
    {
      public:
        /**
         * @brief Record the number of a received frame.
         *
         * @param frame_number  Frame number reported by the server.
         * @return Number of frames missed since the previous received frame (0 for the first
         *         frame, a consecutive frame or a discontinuity).
         */
        std::uint64_t record(std::uint64_t frame_number);

        /// Number of frames recorded so far.
        std::uint64_t received() const;

        /// Total number of missed frames.
        std::uint64_t missed() const;

        /// Number of non-increasing frame numbers seen.
        std::uint64_t discontinuities() const;

      private:
        std::optional<std::uint64_t> previous_frame_number_;
        std::uint64_t received_ = 0;
        std::uint64_t missed_ = 0;
        std::uint64_t discontinuities_ = 0;
    };
} // namespace vicon_probe

#endif // VICON_RECEIVER_PROBE_FRAME_GAP_HPP
