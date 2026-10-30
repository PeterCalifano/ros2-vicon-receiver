/**
 * @file probe_options.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Command-line options of the vicon_probe executable.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_PROBE_PROBE_OPTIONS_HPP
#define VICON_RECEIVER_PROBE_PROBE_OPTIONS_HPP

#include <optional>
#include <string>
#include <vector>

namespace vicon_probe
{
    /// Minimum connection timeout accepted by the Vicon SDK [ms].
    constexpr unsigned int kMinConnectionTimeoutMs = 10;

    /// Options of one probe run. There are no hidden defaults: host and frame count are required.
    struct ProbeOptions
    {
        std::string host;               ///< DataStream server as "host[:port]"
        unsigned int frame_count = 0;   ///< Number of frames to record
        std::string json_path;          ///< Output JSON file; empty: no JSON output
        bool include_centroids = false; ///< Also record the 2D camera centroids (heavy)
        bool show_help = false;         ///< Print the usage and exit

        /// Connection timeout passed to the SDK [ms]; unset: SDK default (5000 ms).
        std::optional<unsigned int> connection_timeout_ms;
    };

    /**
     * @brief Parse the command-line arguments (without the program name).
     *
     * @throws std::invalid_argument with a message for the user if the arguments are invalid
     *         (unknown option, missing or malformed value, missing required option).
     */
    ProbeOptions parse_probe_options(const std::vector<std::string> &arguments);

    /// Usage text of the program.
    std::string probe_usage(const std::string &program_name);
} // namespace vicon_probe

#endif // VICON_RECEIVER_PROBE_PROBE_OPTIONS_HPP
