/**
 * @file console_status_line.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Live console telemetry of the receiver: one summary line per status period.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_CONSOLE_STATUS_LINE_HPP
#define VICON_RECEIVER_CONSOLE_STATUS_LINE_HPP

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "vicon_receiver/utilities.hpp"

namespace vicon_receiver
{
    /**
     * @brief Accumulates what the receiver sees during a status period and formats it as one line.
     *
     * Example line (one subject, 1 s period):
     * @code
     *   100.0 Hz | dropped 0 | latency 31.1 ms (27.9-34.0) | g1-unitree-torso-cam q=1.32 occl 0% cams 3-7
     * @endcode
     *
     * - rate: frames received / elapsed time;
     * - dropped: frames missing from the frame-number sequence;
     * - latency: mean (min-max) of the server pipeline latency;
     * - per subject: mean quality, percentage of frames with an occluded segment, and the smallest
     *   and largest number of cameras that saw a visible marker.
     */
    class ConsoleStatusLine
    {
      public:
        /**
         * @brief Record one received frame.
         *
         * @param frames_dropped  Frames missing before this one.
         * @param latency_s       Server pipeline latency [s], if the server reported it.
         */
        void add_frame(std::uint64_t frames_dropped, std::optional<double> latency_s);

        /**
         * @brief Record one subject of the current frame.
         *
         * @param name           Subject name.
         * @param quality        Object quality, if the server reported it.
         * @param occluded       Whether at least one segment of the subject is occluded.
         * @param camera_counts  Number of cameras that saw each visible marker of the subject.
         */
        void add_subject(const std::string &name,
                         std::optional<double> quality,
                         bool occluded,
                         const std::vector<std::size_t> &camera_counts);

        /**
         * @brief Format the line for the current period and start a new period.
         *
         * @param elapsed_s  Duration of the period [s] (> 0).
         */
        std::string take_line(double elapsed_s);

        /// Forget the current period (e.g. after reconnecting) and start a new one.
        void reset();

      private:
        /// What one subject did during the period.
        struct SubjectPeriodTelemetry
        {
            std::uint64_t frames = 0;          ///< Frames in which the subject was present
            std::uint64_t occluded_frames = 0; ///< Frames with an occluded segment
            RunningMinMeanMax quality;         ///< Object quality
            RunningMinMeanMax camera_counts;   ///< Cameras per visible marker
        };

        /// Format the part of the line of one subject.
        static std::string format_subject(const std::string &name,
                                          const SubjectPeriodTelemetry &subject);

        std::uint64_t frames_ = 0;
        std::uint64_t frames_dropped_ = 0;
        RunningMinMeanMax latency_s_;
        std::map<std::string, SubjectPeriodTelemetry> subjects_;
    };
} // namespace vicon_receiver

#endif // VICON_RECEIVER_CONSOLE_STATUS_LINE_HPP
