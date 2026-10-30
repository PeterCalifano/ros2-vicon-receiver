/**
 * @file probe_options.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Command-line parsing of the vicon_probe executable.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/probe/probe_options.hpp"

#include <limits>
#include <stdexcept>

namespace vicon_probe
{
    namespace
    {
        /**
         * @brief Parse a decimal unsigned integer in [minimum, UINT_MAX].
         *
         * @param option  Option name, used in the error message.
         * @param text    Text to parse; it must contain only digits.
         * @param minimum Smallest accepted value.
         * @throws std::invalid_argument if the text is not a number in range.
         */
        unsigned int
        parse_unsigned(const std::string &option, const std::string &text, unsigned int minimum)
        {
            const std::string error = option + " expects an integer >= " + std::to_string(minimum) +
                                      ", got '" + text + "'";

            // std::stoul accepts signs and leading spaces: require digits only
            if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
            {
                throw std::invalid_argument(error);
            }

            unsigned long value = 0;
            try
            {
                value = std::stoul(text);
            }
            catch (const std::out_of_range &)
            {
                throw std::invalid_argument(error);
            }

            if (value < minimum || value > std::numeric_limits<unsigned int>::max())
            {
                throw std::invalid_argument(error);
            }
            return static_cast<unsigned int>(value);
        }

        /**
         * @brief Return the value following an option, advancing the index.
         * @throws std::invalid_argument if the option is the last argument.
         */
        const std::string &take_value(const std::vector<std::string> &arguments, std::size_t &index)
        {
            const std::string &option = arguments[index];
            if (index + 1 >= arguments.size())
            {
                throw std::invalid_argument(option + " requires a value");
            }
            index += 1;
            return arguments[index];
        }
    } // namespace

    ProbeOptions parse_probe_options(const std::vector<std::string> &arguments)
    {
        ProbeOptions options;
        bool frames_given = false;

        for (std::size_t index = 0; index < arguments.size(); ++index)
        {
            const std::string &argument = arguments[index];

            if (argument == "-h" || argument == "--help")
            {
                options.show_help = true;
            }
            else if (argument == "--host")
            {
                options.host = take_value(arguments, index);
            }
            else if (argument == "--frames")
            {
                options.frame_count = parse_unsigned(argument, take_value(arguments, index), 1);
                frames_given = true;
            }
            else if (argument == "--json")
            {
                options.json_path = take_value(arguments, index);
            }
            else if (argument == "--timeout-ms")
            {
                options.connection_timeout_ms =
                    parse_unsigned(argument, take_value(arguments, index), kMinConnectionTimeoutMs);
            }
            else if (argument == "--centroids")
            {
                options.include_centroids = true;
            }
            else
            {
                throw std::invalid_argument("unknown option '" + argument + "'");
            }
        }

        // With --help nothing else is required
        if (options.show_help)
        {
            return options;
        }

        if (options.host.empty())
        {
            throw std::invalid_argument("--host is required");
        }
        if (!frames_given)
        {
            throw std::invalid_argument("--frames is required");
        }
        return options;
    }

    std::string probe_usage(const std::string &program_name)
    {
        return "Usage: " + program_name +
               " --host HOST[:PORT] --frames N [options]\n"
               "\n"
               "Connect once to a Vicon DataStream server, record N frames of everything it\n"
               "streams (subjects, segments, markers, quality, occlusion, latency, cameras) and\n"
               "print a summary. With --json, write the full recording to a file.\n"
               "\n"
               "Options:\n"
               "  --host HOST[:PORT]  DataStream server (port 801 if omitted)\n"
               "  --frames N          Number of frames to record (>= 1)\n"
               "  --json FILE         Write the recording as JSON to FILE\n"
               "  --timeout-ms MS     Connection timeout (>= 10; SDK default: 5000)\n"
               "  --centroids         Also record the 2D centroids of every camera (heavy)\n"
               "  -h, --help          Show this help and exit\n"
               "\n"
               "Exit codes: 0 success, 1 invalid arguments, 2 connection failed,\n"
               "            3 output file error, 4 stream lost while recording.\n";
    }
} // namespace vicon_probe
