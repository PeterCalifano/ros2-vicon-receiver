/**
 * @file acquisition_clock.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Estimation of the camera acquisition time of Vicon frames on the local clock.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_ACQUISITION_CLOCK_HPP
#define VICON_RECEIVER_ACQUISITION_CLOCK_HPP

#include <cstddef>
#include <cstdint>
#include <deque>

namespace vicon_receiver
{
    /**
     * @brief Estimates when the cameras acquired each Vicon frame, on the local (receive) clock.
     *
     * The Vicon server gives no absolute capture time, but it numbers its frames at a fixed rate
     * and reports the latency of its own pipeline. The camera timeline is therefore
     *
     *     t_acquisition(k) = t_ref + (k - k_ref) / rate
     *
     * where k is the frame number and t_ref the acquisition time of the reference frame k_ref.
     * Every received frame gives a candidate for t_ref:
     *
     *     receive_time - latency_total - (k - k_ref) / rate  =  t_ref + transport delay
     *
     * The transport delay (network, buffering, scheduling) is always positive, so the smallest
     * candidate is the best estimate. The minimum is taken over a sliding window of frames, so
     * that a slow drift between the Vicon and the local clocks is followed.
     *
     * The timeline restarts (new reference frame) when the frame number does not increase
     * (server restart, repeated frame) or the frame rate changes.
     *
     * Properties: stamps are spaced exactly by the frame period while the minimum does not change;
     * the remaining error is the smallest transport delay in the window (not observable here).
     */
    class AcquisitionClock
    {
      public:
        /**
         * @param window_frames  Number of consecutive frame numbers over which the minimum is
         *                       taken (e.g. 1000 = 10 s at 100 Hz).
         * @throws std::invalid_argument if window_frames is 0.
         */
        explicit AcquisitionClock(std::size_t window_frames);

        /**
         * @brief Add a received frame and return its estimated acquisition time.
         *
         * @param frame_number    Frame number reported by the server.
         * @param frame_rate_hz   Frame rate reported by the server (> 0).
         * @param receive_time_s  Local time at which the frame was received [s].
         * @param latency_s       Latency reported by the server for this frame [s] (0 if unknown).
         * @return Estimated acquisition time of the frame on the local clock [s].
         * @throws std::invalid_argument if frame_rate_hz is not positive and finite.
         */
        double update(std::uint64_t frame_number,
                      double frame_rate_hz,
                      double receive_time_s,
                      double latency_s);

        /// Current estimate of the acquisition time of the reference frame [s].
        double reference_time_s() const;

        /// Frame number of the current reference frame.
        std::uint64_t reference_frame() const;

        /// Number of times the timeline was (re)started, including the first frame.
        std::uint64_t restarts() const;

      private:
        /// One candidate of the reference time, from one received frame.
        struct FrameReferenceTime
        {
            std::uint64_t frame_number; ///< Frame that produced the candidate
            double reference_time_s;    ///< receive - latency - nominal offset of the frame
        };

        /// Start a new timeline with the given frame as reference.
        void restart(std::uint64_t frame_number, double frame_rate_hz);

        std::size_t window_frames_;
        bool started_ = false;
        std::uint64_t reference_frame_ = 0;
        std::uint64_t last_frame_ = 0;
        double frame_rate_hz_ = 0.0;
        std::uint64_t restarts_ = 0;

        /// Candidates in the window, with increasing reference times (monotonic queue): the front
        /// is the minimum of the window.
        std::deque<FrameReferenceTime> window_;
    };
} // namespace vicon_receiver

#endif // VICON_RECEIVER_ACQUISITION_CLOCK_HPP
