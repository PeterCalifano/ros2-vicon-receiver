/**
 * @file probe_main.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Entry point of the vicon_probe executable (no ROS needed).
 * @version 0.1
 * @date 2026-10-30
 */

#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vicon_receiver/probe/probe_options.hpp"
#include "vicon_receiver/probe/vicon_probe.hpp"

namespace
{
    /// Name used in messages and in the usage text.
    const char *const kProgramName = "vicon_probe";

    /// Set by SIGINT/SIGTERM: the probe stops after the current frame and closes the JSON file.
    std::atomic<bool> g_stop_requested{false};

    extern "C" void handle_stop_signal(int /*signal_number*/)
    {
        g_stop_requested.store(true);
    }
} // namespace

// Main function
int main(int argc, char **argv)
{
    using vicon_probe::ExitCode;

    const std::vector<std::string> arguments(argv + 1, argv + argc);

    vicon_probe::ProbeOptions options;
    try
    {
        options = vicon_probe::parse_probe_options(arguments);
    }
    catch (const std::invalid_argument &error)
    {
        std::cerr << kProgramName << ": " << error.what() << "\n\n"
                  << vicon_probe::probe_usage(kProgramName);
        return static_cast<int>(ExitCode::InvalidArguments);
    }

    if (options.show_help)
    {
        std::cout << vicon_probe::probe_usage(kProgramName);
        return static_cast<int>(ExitCode::Success);
    }

    // Without --json the recording goes to a stream without buffer, which discards it
    std::ostream discarded_output(nullptr);
    std::ofstream json_file;
    std::ostream *json_stream = &discarded_output;
    if (!options.json_path.empty())
    {
        json_file.open(options.json_path);
        if (!json_file.is_open())
        {
            std::cerr << kProgramName << ": cannot open '" << options.json_path
                      << "' for writing\n";
            return static_cast<int>(ExitCode::OutputError);
        }
        json_stream = &json_file;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    vicon_probe::ViconProbe probe(options, std::cout, *json_stream);
    const ExitCode exit_code = probe.run(g_stop_requested);

    if (json_file.is_open())
    {
        json_file.close();
        if (json_file.fail())
        {
            std::cerr << kProgramName << ": error while writing '" << options.json_path << "'\n";
            return static_cast<int>(ExitCode::OutputError);
        }
        std::cout << "Recording written to " << options.json_path << std::endl;
    }
    return static_cast<int>(exit_code);
}
