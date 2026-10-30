/**
 * @file vicon_probe.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Probe that records everything a Vicon DataStream server sends, to find out which data
 *        (quality, occlusion, latency, markers, cameras, ...) a given setup actually provides.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_PROBE_VICON_PROBE_HPP
#define VICON_RECEIVER_PROBE_VICON_PROBE_HPP

#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "vicon_receiver/probe/frame_gap.hpp"
#include "vicon_receiver/probe/json_writer.hpp"
#include "vicon_receiver/probe/probe_options.hpp"

namespace vicon_probe
{
    /// Process exit codes of the probe (also listed in probe_usage()).
    enum class ExitCode : int
    {
        Success = 0,
        InvalidArguments = 1,
        ConnectionFailed = 2,
        OutputError = 3,
        StreamLost = 4
    };

    /**
     * @brief Collects the SDK calls of one JSON object that did not return Success.
     *
     * Written as an "errors" member ({"GetObjectQuality": "InvalidSubjectName", ...}) so the
     * recording shows which data the server does not provide, instead of silently omitting it.
     */
    class CallErrors
    {
      public:
        /**
         * @brief Record the result of an SDK call.
         * @return true if the call succeeded.
         */
        bool check(const std::string &call, ViconDataStreamSDK::CPP::Result::Enum result);

        /// Whether no call failed.
        bool empty() const;

        /// Write the failed calls as an "errors" member, if there is any.
        void write(JsonWriter &writer) const;

      private:
        std::vector<std::pair<std::string, std::string>> failures_;
    };

    /// Running minimum, mean and maximum of a quantity.
    class RunningStatistics
    {
      public:
        /// Add one sample.
        void add(double sample);

        /// Number of samples.
        std::uint64_t count() const;

        /// Smallest sample (NaN if there is none).
        double minimum() const;

        /// Mean of the samples (NaN if there is none).
        double mean() const;

        /// Largest sample (NaN if there is none).
        double maximum() const;

      private:
        std::uint64_t count_ = 0;
        double sum_ = 0.0;
        double minimum_ = std::numeric_limits<double>::infinity();
        double maximum_ = -std::numeric_limits<double>::infinity();
    };

    /// Per-subject statistics over the recorded frames, printed in the final summary.
    struct SubjectStatistics
    {
        std::uint64_t frames_seen = 0;           ///< Frames in which the subject was present
        std::uint64_t frames_with_occlusion = 0; ///< Frames with at least one occluded segment
        RunningStatistics quality;               ///< GetObjectQuality values
    };

    /**
     * @brief Connects once to a DataStream server, records a number of frames and reports.
     *
     * Everything is written as JSON to the given stream (the full recording) and summarised on the
     * console (session information, one line per frame, final statistics). Translations are kept
     * in the SDK units (millimetres) and quaternions in the SDK order (x, y, z, w).
     */
    class ViconProbe
    {
      public:
        /**
         * @param options      Probe options (host, number of frames, ...).
         * @param console      Stream for the human-readable output.
         * @param json_stream  Stream for the JSON recording (may discard its output).
         */
        ViconProbe(const ProbeOptions &options, std::ostream &console, std::ostream &json_stream);

        /**
         * @brief Run the probe until the requested frames are recorded or a stop is requested.
         *
         * @param stop_requested  Set (e.g. by a SIGINT handler) to stop after the current frame;
         *                        the JSON document is still closed properly.
         * @return The process exit code.
         */
        ExitCode run(const std::atomic<bool> &stop_requested);

      private:
        /// Write the "probe" member: what was requested and when.
        void write_probe_information();

        /// Connect once and write the "connection" member. Returns true on success.
        bool connect();

        /// Enable the data streams, select the stream mode and write the "stream" member.
        void enable_streams();

        /// Fetch frames until one is available. Returns false if the stream fails or a stop is
        /// requested first.
        bool wait_for_first_frame(const std::atomic<bool> &stop_requested);

        /// Write the "server" member: axis mapping, orientation, frame rates.
        void write_server_information();

        /// Write the "cameras" member: identification and calibration of every camera.
        void write_cameras();

        /// Write the "devices" member: name and type of every device.
        void write_devices();

        /// Write the "subjects" member: segment tree, static transforms and markers of every
        /// subject in the current frame.
        void write_subject_structure();

        /// Write one element of the "frames" array for the current frame.
        void write_frame();

        /// Write the current pose, quality and markers of one subject in the current frame.
        void write_subject_frame(const std::string &subject_name, std::string &console_line);

        /// Write the labeled and unlabeled markers of the current frame.
        void write_frame_markers(CallErrors &frame_errors);

        /// Write the 2D centroids of every camera in the current frame.
        void write_frame_centroids(CallErrors &frame_errors);

        /// Write the "summary" member with the statistics of the recording.
        void write_summary(bool interrupted, bool stream_lost);

        /// Print the session information (from the first frame) on the console.
        void print_session_summary();

        /// Print the final statistics on the console.
        void print_final_summary(bool interrupted, bool stream_lost);

        const ProbeOptions options_;
        std::ostream &console_;
        JsonWriter json_;
        ViconDataStreamSDK::CPP::Client client_;

        /// Frames written so far (counted even if their frame number cannot be read)
        std::uint64_t frames_recorded_ = 0;

        /// Failed calls that list cameras, devices or subjects (written in the summary)
        CallErrors session_errors_;

        FrameGapTracker frame_gaps_;
        RunningStatistics latency_s_;
        std::map<std::string, SubjectStatistics> subject_statistics_;
    };

    /// Version of the SDK library as "major.minor.point.revision".
    std::string sdk_version(const ViconDataStreamSDK::CPP::Client &client);
} // namespace vicon_probe

#endif // VICON_RECEIVER_PROBE_VICON_PROBE_HPP
