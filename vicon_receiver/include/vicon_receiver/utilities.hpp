/**
 * @file utilities.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Small utilities shared by the node and the probe.
 * @version 0.1
 * @date 2026-11-07
 */

#ifndef VICON_RECEIVER_UTILITIES_HPP
#define VICON_RECEIVER_UTILITIES_HPP

#include <cstdint>
#include <limits>
#include <string>

#include "vicon-datastream-sdk/DataStreamClient.h"

namespace vicon_receiver
{
    // --------------------------------------------------------------------------------------------
    // Running statistics
    // --------------------------------------------------------------------------------------------

    /// Running minimum, mean and maximum of a quantity, without storing the samples.
    class RunningMinMeanMax
    {
      public:
        /// Add one sample.
        void add(double sample);

        /// Forget every sample.
        void reset();

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

    // --------------------------------------------------------------------------------------------
    // Names of the Vicon SDK enumerations
    // --------------------------------------------------------------------------------------------

    /// Name of an SDK call result, e.g. "Success", "ClientConnectionFailed".
    std::string result_name(ViconDataStreamSDK::CPP::Result::Enum result);

    /// Name of an axis direction, e.g. "Forward".
    std::string direction_name(ViconDataStreamSDK::CPP::Direction::Enum direction);

    /// Name of the server up-axis orientation, e.g. "ZUp".
    std::string
    server_orientation_name(ViconDataStreamSDK::CPP::ServerOrientation::Enum orientation);

    /// Name of a timecode standard, e.g. "PAL".
    std::string timecode_standard_name(ViconDataStreamSDK::CPP::TimecodeStandard::Enum standard);

    /// Name of a device type, e.g. "ForcePlate".
    std::string device_type_name(ViconDataStreamSDK::CPP::DeviceType::Enum device_type);

    /// Name of a stream mode: "ServerPush", "ClientPull" or "ClientPullPreFetch".
    std::string stream_mode_name(ViconDataStreamSDK::CPP::StreamMode::Enum mode);

    /**
     * @brief Stream mode from its name, as written in the parameters file.
     *
     * @param name  "ServerPush", "ClientPull" or "ClientPullPreFetch" (case-sensitive).
     * @throws std::invalid_argument naming the valid values if the name is unknown.
     */
    ViconDataStreamSDK::CPP::StreamMode::Enum stream_mode_from_name(const std::string &name);
} // namespace vicon_receiver

#endif // VICON_RECEIVER_UTILITIES_HPP
