/**
 * @file frame_timing.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Timing of the Vicon frame stream: acquisition time of the frames, dropped, stale and
 *        leftover frames, rate of the local clock.
 * @version 0.1
 * @date 2026-11-07
 */

#ifndef VICON_RECEIVER_FRAME_TIMING_HPP
#define VICON_RECEIVER_FRAME_TIMING_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace vicon_receiver
{
    // --------------------------------------------------------------------------------------------
    // Acquisition time of the frames
    // --------------------------------------------------------------------------------------------

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
     *
     * TODO: improve the acquisition-time estimate. Its accuracy may be bounded by what the Vicon
     * stream provides rather than by the receiver: the latency total covers the Tracker pipeline
     * (cameras to DataStream) but not the network, and the lab Tracker sends no timecode and a
     * hardware frame number of 0, so no absolute capture time is available. Needs investigation:
     * which time information the Tracker can stream (timecode, hardware frame number, the SDK's
     * UDP multicast mode instead of the TCP connection used now), and how much of the remaining
     * error comes from the receive side (network, scheduling, local clock).
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

        /// Forget the timeline: the next frame starts a new one (e.g. after a stale frame).
        void reset();

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

    // --------------------------------------------------------------------------------------------
    // Dropped and stale frames
    // --------------------------------------------------------------------------------------------

    /**
     * @brief Counts the frames missed by a client from the sequence of received frame numbers.
     *
     * The Vicon server numbers its frames consecutively. A jump from N to N + k (k > 1) means that
     * k - 1 frames were not received. A frame number that does not increase (server restarted, or
     * the same frame delivered twice) is counted as a discontinuity and restarts the sequence.
     */
    class DroppedFrameCounter
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

        /// Last recorded frame number, if any.
        std::optional<std::uint64_t> last_frame_number() const;

        /// Start a new sequence: the next frame is counted as the first (counters are kept).
        void restart_sequence();

      private:
        std::optional<std::uint64_t> previous_frame_number_;
        std::uint64_t received_ = 0;
        std::uint64_t missed_ = 0;
        std::uint64_t discontinuities_ = 0;
    };

    /**
     * @brief Whether a forward jump of the frame number cannot be made of dropped frames.
     *
     * A server cannot drop more frames than it produced since the previous frame arrived. A
     * larger jump means that the previous frame was stale, e.g. the frame left over from a
     * previous connection that a Vicon server sends first after connecting.
     *
     * @param frames_missing      Frames missing between the previous and the current frame.
     * @param receive_interval_s  Local time between the two frames [s].
     * @param frame_rate_hz       Frame rate reported by the server [Hz].
     * @param margin_s            Tolerance added to the interval (clock jitter, buffering) [s].
     */
    bool is_implausible_frame_jump(std::uint64_t frames_missing,
                                   double receive_interval_s,
                                   double frame_rate_hz,
                                   double margin_s);

    /// Frames after connecting during which any frame-number jump is taken as stale leftovers.
    constexpr std::uint64_t kConnectionStartupFrames = 10;

    /**
     * @brief Whether the previous frame was stale, i.e. the frame-number jump to the current one
     *        is not made of dropped frames.
     *
     * After connecting, a Vicon server first sends one or two frames left over from its previous
     * session (observed live). So:
     * - within the first kConnectionStartupFrames frames after connecting, any forward jump
     *   means that the frames before it were stale (even if the previous session ended a
     *   fraction of a second ago); a real drop there is not counted;
     * - later, a jump is stale only if it is implausible (is_implausible_frame_jump()).
     *
     * @param frames_missing         Frames missing between the previous and the current frame.
     * @param receive_interval_s     Local time between the two frames [s].
     * @param frame_rate_hz          Frame rate reported by the server [Hz].
     * @param frames_before_current  Frames received since connecting, before the current one.
     * @param margin_s               Tolerance of the plausibility check [s].
     */
    bool is_stale_previous_frame(std::uint64_t frames_missing,
                                 double receive_interval_s,
                                 double frame_rate_hz,
                                 std::uint64_t frames_before_current,
                                 double margin_s);
} // namespace vicon_receiver

#endif // VICON_RECEIVER_FRAME_TIMING_HPP
