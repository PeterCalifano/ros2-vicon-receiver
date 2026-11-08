/**
 * @file utilities.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Small utilities shared by the node and the probe.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/utilities.hpp"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

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

        /// State of an established connection in /proc/net/tcp (hexadecimal).
        constexpr const char *kEstablishedState = "01";

        /// Largest TCP_KEEPCNT accepted by Linux (MAX_TCP_KEEPCNT); 128 gives EINVAL.
        constexpr int kMaxKeepaliveProbes = 127;

        /// Integer division rounded up, for positive operands.
        int divide_rounding_up(int dividend, int divisor)
        {
            return (dividend + divisor - 1) / divisor;
        }

        /// Set one integer socket option, or throw with its name.
        void set_option(int socket_fd, int level, int option, int value, const char *name)
        {
            if (setsockopt(socket_fd, level, option, &value, sizeof(value)) != 0)
            {
                throw std::system_error(errno, std::generic_category(), name);
            }
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

    // --------------------------------------------------------------------------------------------
    // TCP keepalive of the SDK connection
    // --------------------------------------------------------------------------------------------

    std::uint16_t server_port(const std::string &host_and_port)
    {
        // An IPv6 address has several ':'; only "host:port" with one ':' carries a port
        const std::size_t colon = host_and_port.rfind(':');
        if (colon == std::string::npos || host_and_port.find(':') != colon)
        {
            return kDefaultDataStreamPort;
        }

        const std::string port_text = host_and_port.substr(colon + 1);
        std::size_t parsed = 0;
        unsigned long port = 0;
        try
        {
            port = std::stoul(port_text, &parsed);
        }
        catch (const std::exception &)
        {
            parsed = 0;
        }
        if (port_text.empty() || parsed != port_text.size() || port < 1 || port > 65535)
        {
            throw std::invalid_argument("invalid port '" + port_text + "' in '" + host_and_port +
                                        "'");
        }
        return static_cast<std::uint16_t>(port);
    }

    std::set<std::uint64_t> established_socket_inodes(std::istream &table,
                                                      std::uint16_t remote_port)
    {
        std::set<std::uint64_t> inodes;
        std::string line;
        std::getline(table, line); // header

        while (std::getline(table, line))
        {
            // sl local_address rem_address st tx:rx tr:when retrnsmt uid timeout inode ...
            std::istringstream fields(line);
            std::string slot;
            std::string local_address;
            std::string remote_address;
            std::string state;
            std::string queues;
            std::string timer;
            std::string retransmits;
            std::string uid;
            std::string timeout;
            std::uint64_t inode = 0;
            if (!(fields >> slot >> local_address >> remote_address >> state >> queues >> timer >>
                  retransmits >> uid >> timeout >> inode))
            {
                continue;
            }

            const std::size_t colon = remote_address.rfind(':');
            if (colon == std::string::npos || state != kEstablishedState || inode == 0)
            {
                continue;
            }
            const unsigned long port = std::stoul(remote_address.substr(colon + 1), nullptr, 16);
            if (port == remote_port)
            {
                inodes.insert(inode);
            }
        }
        return inodes;
    }

    std::vector<int> find_tcp_sockets_to_port(std::uint16_t remote_port)
    {
        std::set<std::uint64_t> inodes;
        for (const char *table_path : {"/proc/net/tcp", "/proc/net/tcp6"})
        {
            std::ifstream table(table_path);
            if (table)
            {
                const std::set<std::uint64_t> found = established_socket_inodes(table, remote_port);
                inodes.insert(found.begin(), found.end());
            }
        }

        // Every open file descriptor of this process that is one of those sockets
        std::vector<int> sockets;
        std::error_code error;
        for (const auto &entry : std::filesystem::directory_iterator("/proc/self/fd", error))
        {
            std::error_code link_error;
            const std::string target = std::filesystem::read_symlink(entry.path(), link_error);
            const std::string prefix = "socket:[";
            if (link_error || target.rfind(prefix, 0) != 0 || target.back() != ']')
            {
                continue;
            }
            const std::uint64_t inode =
                std::stoull(target.substr(prefix.size(), target.size() - prefix.size() - 1));
            if (inodes.count(inode) > 0)
            {
                sockets.push_back(std::stoi(entry.path().filename().string()));
            }
        }
        return sockets;
    }

    void enable_tcp_keepalive(int socket_fd, int timeout_s)
    {
        if (timeout_s < 2)
        {
            throw std::invalid_argument("TCP keepalive timeout must be >= 2 s, got " +
                                        std::to_string(timeout_s));
        }

        // First probe after one interval without data, then one per interval, so the timeout
        // spans probe_count + 1 intervals. Up to 128 s: one probe per second, timeout_s - 1
        // probes. Longer timeouts space the probes so that their number stays within the limit.
        const int interval_s = divide_rounding_up(timeout_s, kMaxKeepaliveProbes + 1);
        const int probe_count = divide_rounding_up(timeout_s, interval_s) - 1;
        set_option(socket_fd, SOL_SOCKET, SO_KEEPALIVE, 1, "SO_KEEPALIVE");
        set_option(socket_fd, IPPROTO_TCP, TCP_KEEPIDLE, interval_s, "TCP_KEEPIDLE");
        set_option(socket_fd, IPPROTO_TCP, TCP_KEEPINTVL, interval_s, "TCP_KEEPINTVL");
        set_option(socket_fd, IPPROTO_TCP, TCP_KEEPCNT, probe_count, "TCP_KEEPCNT");
        // Also bound the time sent data may stay unacknowledged (requests to the server)
        set_option(socket_fd, IPPROTO_TCP, TCP_USER_TIMEOUT, timeout_s * 1000, "TCP_USER_TIMEOUT");
    }
} // namespace vicon_receiver
