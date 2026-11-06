/**
 * @file acquisition_clock.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Estimation of the camera acquisition time of Vicon frames on the local clock.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/acquisition_clock.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace vicon_receiver
{
    namespace
    {
        /// Relative change of the frame rate considered a new rate (and a new timeline).
        constexpr double kFrameRateTolerance = 1e-9;
    } // namespace

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

        // FrameReferenceTime reference time: too late by the (positive) transport delay of this frame
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
} // namespace vicon_receiver
