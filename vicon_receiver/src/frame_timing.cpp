/**
 * @file frame_timing.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Timing of the Vicon frame stream: acquisition time of the frames, dropped, stale and
 *        leftover frames, rate of the local clock.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/frame_timing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace vicon_receiver
{
    namespace
    {
        /// Relative change of the frame rate considered a new rate (new timeline, new measurement).
        constexpr double kFrameRateTolerance = 1e-9;
    } // namespace

    // --------------------------------------------------------------------------------------------
    // Acquisition time of the frames
    // --------------------------------------------------------------------------------------------

    AcquisitionClock::AcquisitionClock(std::size_t window_frames) : window_frames_(window_frames)
    {
        if (window_frames_ == 0)
        {
            throw std::invalid_argument(
                "AcquisitionClock: the window must contain at least 1 frame");
        }
    }

    double AcquisitionClock::update(std::uint64_t frame_number,
                                    double frame_rate_hz,
                                    double receive_time_s,
                                    double latency_s)
    {
        if (!std::isfinite(frame_rate_hz) || frame_rate_hz <= 0.0)
        {
            throw std::invalid_argument("AcquisitionClock: invalid frame rate " +
                                        std::to_string(frame_rate_hz));
        }

        // A non-increasing frame number or a new rate breaks the timeline: start a new one
        const bool rate_changed =
            std::abs(frame_rate_hz - frame_rate_hz_) > kFrameRateTolerance * frame_rate_hz;
        if (!started_ || frame_number <= last_frame_ || rate_changed)
        {
            restart(frame_number, frame_rate_hz);
        }
        last_frame_ = frame_number;

        // Time of this frame relative to the reference frame, on the camera timeline
        const double frames_since_reference = static_cast<double>(frame_number - reference_frame_);
        const double nominal_offset_s = frames_since_reference / frame_rate_hz_;

        // Reference time from this frame: too late by the (positive) transport delay of this frame
        const double candidate_s = receive_time_s - latency_s - nominal_offset_s;

        // Monotonic queue: candidates that are not smaller than the new one can never be the
        // minimum again while the new one is in the window
        while (!window_.empty() && window_.back().reference_time_s >= candidate_s)
        {
            window_.pop_back();
        }
        window_.push_back(FrameReferenceTime{frame_number, candidate_s});

        // Drop candidates whose frame left the window
        while (frame_number - window_.front().frame_number >= window_frames_)
        {
            window_.pop_front();
        }

        return window_.front().reference_time_s + nominal_offset_s;
    }

    double AcquisitionClock::reference_time_s() const
    {
        if (window_.empty())
        {
            return std::nan("");
        }
        return window_.front().reference_time_s;
    }

    std::uint64_t AcquisitionClock::reference_frame() const
    {
        return reference_frame_;
    }

    void AcquisitionClock::reset()
    {
        started_ = false;
        window_.clear();
    }

    std::uint64_t AcquisitionClock::restarts() const
    {
        return restarts_;
    }

    void AcquisitionClock::restart(std::uint64_t frame_number, double frame_rate_hz)
    {
        started_ = true;
        reference_frame_ = frame_number;
        frame_rate_hz_ = frame_rate_hz;
        window_.clear();
        restarts_ += 1;
    }

    // --------------------------------------------------------------------------------------------
    // Dropped and stale frames
    // --------------------------------------------------------------------------------------------

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
    std::optional<std::uint64_t> DroppedFrameCounter::last_frame_number() const
    {
        return previous_frame_number_;
    }

    void DroppedFrameCounter::restart_sequence()
    {
        previous_frame_number_.reset();
    }

    bool is_implausible_frame_jump(std::uint64_t frames_missing,
                                   double receive_interval_s,
                                   double frame_rate_hz,
                                   double margin_s)
    {
        // Most frames the server can have produced since the previous frame was received
        const double producible_frames =
            frame_rate_hz * (std::max(receive_interval_s, 0.0) + margin_s);
        return static_cast<double>(frames_missing) > producible_frames;
    }
    bool is_stale_previous_frame(std::uint64_t frames_missing,
                                 double receive_interval_s,
                                 double frame_rate_hz,
                                 std::uint64_t frames_before_current,
                                 double margin_s)
    {
        if (frames_missing == 0)
        {
            return false;
        }

        // Start of a connection: the frames before the jump are left-overs of the previous session
        if (frames_before_current <= kConnectionStartupFrames)
        {
            return true;
        }
        return is_implausible_frame_jump(
            frames_missing, receive_interval_s, frame_rate_hz, margin_s);
    }
} // namespace vicon_receiver
