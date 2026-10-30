/**
 * @file sdk_names.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Readable names of the Vicon DataStream SDK enumerations, for logs and probe output.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_PROBE_SDK_NAMES_HPP
#define VICON_RECEIVER_PROBE_SDK_NAMES_HPP

#include <string>

#include "vicon-datastream-sdk/DataStreamClient.h"

namespace vicon_probe
{
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
} // namespace vicon_probe

#endif // VICON_RECEIVER_PROBE_SDK_NAMES_HPP
