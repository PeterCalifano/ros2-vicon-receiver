/**
 * @file utilities.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Small utilities shared by the node and the probe.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/utilities.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vicon_receiver
{
    namespace sdk = ViconDataStreamSDK::CPP;

    namespace
    {
        /// Name used for values outside the enumeration (e.g. from a newer SDK).
        std::string unknown_value_name(int value)
        {
            return "Unknown(" + std::to_string(value) + ")";
        }
    } // namespace

    // --------------------------------------------------------------------------------------------
    // Running statistics
    // --------------------------------------------------------------------------------------------

    void RunningMinMeanMax::add(double sample)
    {
        count_ += 1;
        sum_ += sample;
        minimum_ = std::min(minimum_, sample);
        maximum_ = std::max(maximum_, sample);
    }

    void RunningMinMeanMax::reset()
    {
        *this = RunningMinMeanMax();
    }

    std::uint64_t RunningMinMeanMax::count() const
    {
        return count_;
    }

    double RunningMinMeanMax::minimum() const
    {
        return count_ == 0 ? std::nan("") : minimum_;
    }

    double RunningMinMeanMax::mean() const
    {
        return count_ == 0 ? std::nan("") : sum_ / static_cast<double>(count_);
    }

    double RunningMinMeanMax::maximum() const
    {
        return count_ == 0 ? std::nan("") : maximum_;
    }

    // --------------------------------------------------------------------------------------------
    // Names of the Vicon SDK enumerations
    // --------------------------------------------------------------------------------------------

    std::string result_name(sdk::Result::Enum result)
    {
        switch (result)
        {
        case sdk::Result::Unknown:
            return "Unknown";
        case sdk::Result::NotImplemented:
            return "NotImplemented";
        case sdk::Result::Success:
            return "Success";
        case sdk::Result::InvalidHostName:
            return "InvalidHostName";
        case sdk::Result::InvalidMulticastIP:
            return "InvalidMulticastIP";
        case sdk::Result::ClientAlreadyConnected:
            return "ClientAlreadyConnected";
        case sdk::Result::ClientConnectionFailed:
            return "ClientConnectionFailed";
        case sdk::Result::ServerAlreadyTransmittingMulticast:
            return "ServerAlreadyTransmittingMulticast";
        case sdk::Result::ServerNotTransmittingMulticast:
            return "ServerNotTransmittingMulticast";
        case sdk::Result::NotConnected:
            return "NotConnected";
        case sdk::Result::NoFrame:
            return "NoFrame";
        case sdk::Result::InvalidIndex:
            return "InvalidIndex";
        case sdk::Result::InvalidCameraName:
            return "InvalidCameraName";
        case sdk::Result::InvalidSubjectName:
            return "InvalidSubjectName";
        case sdk::Result::InvalidSegmentName:
            return "InvalidSegmentName";
        case sdk::Result::InvalidMarkerName:
            return "InvalidMarkerName";
        case sdk::Result::InvalidDeviceName:
            return "InvalidDeviceName";
        case sdk::Result::InvalidDeviceOutputName:
            return "InvalidDeviceOutputName";
        case sdk::Result::InvalidLatencySampleName:
            return "InvalidLatencySampleName";
        case sdk::Result::CoLinearAxes:
            return "CoLinearAxes";
        case sdk::Result::LeftHandedAxes:
            return "LeftHandedAxes";
        case sdk::Result::HapticAlreadySet:
            return "HapticAlreadySet";
        case sdk::Result::EarlyDataRequested:
            return "EarlyDataRequested";
        case sdk::Result::LateDataRequested:
            return "LateDataRequested";
        case sdk::Result::InvalidOperation:
            return "InvalidOperation";
        case sdk::Result::NotSupported:
            return "NotSupported";
        case sdk::Result::ConfigurationFailed:
            return "ConfigurationFailed";
        case sdk::Result::NotPresent:
            return "NotPresent";
        case sdk::Result::ArgumentOutOfRange:
            return "ArgumentOutOfRange";
        }
        return unknown_value_name(static_cast<int>(result));
    }

    std::string direction_name(sdk::Direction::Enum direction)
    {
        switch (direction)
        {
        case sdk::Direction::Up:
            return "Up";
        case sdk::Direction::Down:
            return "Down";
        case sdk::Direction::Left:
            return "Left";
        case sdk::Direction::Right:
            return "Right";
        case sdk::Direction::Forward:
            return "Forward";
        case sdk::Direction::Backward:
            return "Backward";
        }
        return unknown_value_name(static_cast<int>(direction));
    }

    std::string server_orientation_name(sdk::ServerOrientation::Enum orientation)
    {
        switch (orientation)
        {
        case sdk::ServerOrientation::Unknown:
            return "Unknown";
        case sdk::ServerOrientation::YUp:
            return "YUp";
        case sdk::ServerOrientation::ZUp:
            return "ZUp";
        }
        return unknown_value_name(static_cast<int>(orientation));
    }

    std::string timecode_standard_name(sdk::TimecodeStandard::Enum standard)
    {
        switch (standard)
        {
        case sdk::TimecodeStandard::None:
            return "None";
        case sdk::TimecodeStandard::PAL:
            return "PAL";
        case sdk::TimecodeStandard::NTSC:
            return "NTSC";
        case sdk::TimecodeStandard::NTSCDrop:
            return "NTSCDrop";
        case sdk::TimecodeStandard::Film:
            return "Film";
        case sdk::TimecodeStandard::NTSCFilm:
            return "NTSCFilm";
        case sdk::TimecodeStandard::ATSC:
            return "ATSC";
        }
        return unknown_value_name(static_cast<int>(standard));
    }

    std::string device_type_name(sdk::DeviceType::Enum device_type)
    {
        switch (device_type)
        {
        case sdk::DeviceType::Unknown:
            return "Unknown";
        case sdk::DeviceType::ForcePlate:
            return "ForcePlate";
        case sdk::DeviceType::EyeTracker:
            return "EyeTracker";
        }
        return unknown_value_name(static_cast<int>(device_type));
    }

    std::string stream_mode_name(sdk::StreamMode::Enum mode)
    {
        switch (mode)
        {
        case sdk::StreamMode::ClientPull:
            return "ClientPull";
        case sdk::StreamMode::ClientPullPreFetch:
            return "ClientPullPreFetch";
        case sdk::StreamMode::ServerPush:
            return "ServerPush";
        }
        return unknown_value_name(static_cast<int>(mode));
    }

    sdk::StreamMode::Enum stream_mode_from_name(const std::string &name)
    {
        const sdk::StreamMode::Enum modes[] = {sdk::StreamMode::ServerPush,
                                               sdk::StreamMode::ClientPull,
                                               sdk::StreamMode::ClientPullPreFetch};
        for (const sdk::StreamMode::Enum mode : modes)
        {
            if (stream_mode_name(mode) == name)
            {
                return mode;
            }
        }
        throw std::invalid_argument("unknown stream mode '" + name +
                                    "' (valid: ServerPush, ClientPull, ClientPullPreFetch)");
    }
} // namespace vicon_receiver
