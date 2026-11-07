/**
 * @file console_status_line.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Live console telemetry of the receiver: one summary line per status period.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/console_status_line.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace vicon_receiver
{
    namespace
    {
        /// Format a number with a fixed number of decimals.
        std::string fixed(double number, int decimals)
        {
            std::ostringstream text;
            text << std::fixed << std::setprecision(decimals) << number;
            return text.str();
        }
    } // namespace

    void ConsoleStatusLine::add_frame(std::uint64_t frames_dropped, std::optional<double> latency_s)
    {
        frames_ += 1;
        frames_dropped_ += frames_dropped;
        if (latency_s.has_value())
        {
            latency_s_.add(latency_s.value());
        }
    }

    void ConsoleStatusLine::add_subject(const std::string &name,
                                        std::optional<double> quality,
                                        bool occluded,
                                        const std::vector<std::size_t> &camera_counts)
    {
        SubjectPeriodTelemetry &subject = subjects_[name];
        subject.frames += 1;
        if (occluded)
        {
            subject.occluded_frames += 1;
        }
        if (quality.has_value())
        {
            subject.quality.add(quality.value());
        }
        for (const std::size_t count : camera_counts)
        {
            subject.camera_counts.add(static_cast<double>(count));
        }
    }

    std::string ConsoleStatusLine::take_line(double elapsed_s)
    {
        std::ostringstream line;

        if (frames_ == 0)
        {
            line << "no frames in the last " << fixed(elapsed_s, 1) << " s";
        }
        else
        {
            const double rate_hz = elapsed_s > 0.0 ? static_cast<double>(frames_) / elapsed_s : 0.0;
            line << fixed(rate_hz, 1) << " Hz | dropped " << frames_dropped_ << " | latency ";

            if (latency_s_.count() > 0)
            {
                line << fixed(latency_s_.mean() * 1000.0, 1) << " ms ("
                     << fixed(latency_s_.minimum() * 1000.0, 1) << "-"
                     << fixed(latency_s_.maximum() * 1000.0, 1) << ")";
            }
            else
            {
                line << "n/a";
            }

            if (subjects_.empty())
            {
                line << " | no subjects";
            }
            for (const auto &[name, subject] : subjects_)
            {
                line << " | " << format_subject(name, subject);
            }
        }

        this->reset();
        return line.str();
    }

    void ConsoleStatusLine::reset()
    {
        frames_ = 0;
        frames_dropped_ = 0;
        latency_s_.reset();
        subjects_.clear();
    }

    std::string ConsoleStatusLine::format_subject(const std::string &name,
                                                  const SubjectPeriodTelemetry &subject)
    {
        std::ostringstream text;
        text << name;

        if (subject.quality.count() > 0)
        {
            text << " q=" << fixed(subject.quality.mean(), 2);
        }
        else
        {
            text << " q=n/a";
        }

        const double occluded_percent = 100.0 * static_cast<double>(subject.occluded_frames) /
                                        static_cast<double>(subject.frames);
        text << " occl " << fixed(occluded_percent, 0) << "%";

        if (subject.camera_counts.count() > 0)
        {
            text << " cams " << fixed(subject.camera_counts.minimum(), 0) << "-"
                 << fixed(subject.camera_counts.maximum(), 0);
        }
        else
        {
            text << " cams n/a";
        }
        return text.str();
    }
} // namespace vicon_receiver
