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

        /// Fraction of the nominal duration of a run over which its frames must have arrived:
        /// leftovers come back to back, live frames at the rate (a few ms apart while draining).
        constexpr double kMinRunSpanFraction = 0.5;
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

    // --------------------------------------------------------------------------------------------
    // Frames left over from the server's previous session
    // --------------------------------------------------------------------------------------------

    StartupLeftoverFrameFilter::StartupLeftoverFrameFilter(std::uint64_t consecutive_frames)
        : consecutive_frames_(consecutive_frames)
    {
        if (consecutive_frames_ < 2)
        {
            throw std::invalid_argument(
                "StartupLeftoverFrameFilter: need at least 2 consecutive frames, got " +
                std::to_string(consecutive_frames_));
        }
    }

    bool StartupLeftoverFrameFilter::accept(std::uint64_t frame_number,
                                            double receive_time_s,
                                            double frame_rate_hz)
    {
        if (confirmed_)
        {
            return true;
        }

        // A forward step continues the run if the frames it skips can have been produced since
        // the previous frame: a consumer slower than the camera (ClientPull) skips frame numbers
        // on a healthy stream, while the jump after leftovers arrives at once
        bool continues_run = false;
        if (previous_frame_number_.has_value() && frame_number > previous_frame_number_.value())
        {
            const std::uint64_t frames_missing = frame_number - previous_frame_number_.value() - 1;
            continues_run =
                is_valid_frame_rate(frame_rate_hz)
                    ? !is_implausible_frame_jump(frames_missing,
                                                 receive_time_s - previous_receive_time_s_,
                                                 frame_rate_hz,
                                                 0.0)
                    : frames_missing == 0;
        }
        if (continues_run)
        {
            run_length_ += 1;
        }
        else
        {
            run_length_ = 1;
            run_start_time_s_ = receive_time_s;
            run_start_frame_number_ = frame_number;
        }
        previous_frame_number_ = frame_number;
        previous_receive_time_s_ = receive_time_s;

        // Long enough, and not a burst of buffered frames
        if (run_length_ >= consecutive_frames_)
        {
            const double nominal_span_s =
                is_valid_frame_rate(frame_rate_hz)
                    ? static_cast<double>(frame_number - run_start_frame_number_) / frame_rate_hz
                    : 0.0;
            if (receive_time_s - run_start_time_s_ >= kMinRunSpanFraction * nominal_span_s)
            {
                confirmed_ = true;
                return true;
            }
        }

        rejected_ += 1;
        return false;
    }

    bool StartupLeftoverFrameFilter::confirmed() const
    {
        return confirmed_;
    }

    std::uint64_t StartupLeftoverFrameFilter::rejected() const
    {
        return rejected_;
    }

    void StartupLeftoverFrameFilter::restart()
    {
        confirmed_ = false;
        previous_frame_number_.reset();
        run_length_ = 0;
        run_start_time_s_ = 0.0;
        rejected_ = 0;
    }

    // --------------------------------------------------------------------------------------------
    // Rate of the local clock
    // --------------------------------------------------------------------------------------------

    LocalClockRateMonitor::LocalClockRateMonitor(double measurement_s, double block_s)
        : measurement_s_(measurement_s), block_s_(block_s)
    {
        // The first and the last block must not overlap
        if (!std::isfinite(block_s_) || block_s_ <= 0.0 || !std::isfinite(measurement_s_) ||
            measurement_s_ <= 2.0 * block_s_)
        {
            throw std::invalid_argument(
                "LocalClockRateMonitor: need block_s > 0 and measurement_s > "
                "2 block_s, got block_s " +
                std::to_string(block_s_) + ", measurement_s " + std::to_string(measurement_s_));
        }
    }

    std::optional<double> LocalClockRateMonitor::add_frame(std::uint64_t frame_number,
                                                           double frame_rate_hz,
                                                           double local_time_s)
    {
        if (!std::isfinite(frame_rate_hz) || frame_rate_hz <= 0.0)
        {
            throw std::invalid_argument("LocalClockRateMonitor: invalid frame rate " +
                                        std::to_string(frame_rate_hz));
        }

        // A broken timeline cannot be compared: start a new measurement at this frame
        if (started_)
        {
            const bool rate_changed =
                std::abs(frame_rate_hz - frame_rate_hz_) > kFrameRateTolerance * frame_rate_hz_;
            const bool paused = local_time_s - last_time_s_ > block_s_;
            if (frame_number <= last_frame_ || rate_changed || paused)
            {
                restarts_ += 1;
                started_ = false;
            }
        }
        if (!started_)
        {
            start(frame_number, frame_rate_hz, local_time_s);
        }
        last_frame_ = frame_number;
        last_time_s_ = local_time_s;

        // Offset of this frame: local time minus Vicon time since the reference frame
        const double vicon_time_s =
            static_cast<double>(frame_number - reference_frame_) / frame_rate_hz_;
        const double offset_s = local_time_s - vicon_time_s;

        const double elapsed_s = local_time_s - start_time_s_;
        if (elapsed_s < block_s_)
        {
            keep_minimum(first_block_, offset_s, frame_number);
        }
        else if (elapsed_s >= measurement_s_ - block_s_)
        {
            keep_minimum(last_block_, offset_s, frame_number);
        }

        if (elapsed_s < measurement_s_ || !first_block_.has_value() || !last_block_.has_value())
        {
            return std::nullopt;
        }

        // Growth of the least delayed offset per second of Vicon time
        const double vicon_interval_s =
            static_cast<double>(last_block_->frame_number - first_block_->frame_number) /
            frame_rate_hz_;
        const double rate_error =
            (last_block_->offset_s - first_block_->offset_s) / vicon_interval_s;

        // The next measurement starts with this frame
        start(frame_number, frame_rate_hz, local_time_s);
        keep_minimum(first_block_, local_time_s, frame_number);
        return rate_error;
    }

    void LocalClockRateMonitor::restart()
    {
        started_ = false;
        first_block_.reset();
        last_block_.reset();
    }

    std::uint64_t LocalClockRateMonitor::restarts() const
    {
        return restarts_;
    }

    void LocalClockRateMonitor::start(std::uint64_t frame_number,
                                      double frame_rate_hz,
                                      double local_time_s)
    {
        started_ = true;
        reference_frame_ = frame_number;
        frame_rate_hz_ = frame_rate_hz;
        start_time_s_ = local_time_s;
        first_block_.reset();
        last_block_.reset();
    }

    void LocalClockRateMonitor::keep_minimum(std::optional<LeastDelayedFrame> &minimum,
                                             double offset_s,
                                             std::uint64_t frame_number)
    {
        if (!minimum.has_value() || offset_s < minimum->offset_s)
        {
            minimum = LeastDelayedFrame{offset_s, frame_number};
        }
    }
} // namespace vicon_receiver
