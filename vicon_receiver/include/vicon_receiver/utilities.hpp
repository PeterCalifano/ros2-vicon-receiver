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
#include <istream>
#include <limits>
#include <set>
#include <string>
#include <vector>

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

    // --------------------------------------------------------------------------------------------
    // TCP keepalive of the SDK connection
    // --------------------------------------------------------------------------------------------

    /// DataStream port of a Vicon server when the host name gives none.
    constexpr std::uint16_t kDefaultDataStreamPort = 801;

    /**
     * @brief Port of a "host[:port]" server address (kDefaultDataStreamPort without a port).
     *
     * @throws std::invalid_argument if the port is not a number in 1..65535.
     */
    std::uint16_t server_port(const std::string &host_and_port);

    /**
     * @brief Socket inodes of the established TCP connections to a remote port, from the text of
     *        /proc/net/tcp or /proc/net/tcp6.
     *
     * @param table        Content of the table (header line first).
     * @param remote_port  Port of the remote end.
     */
    std::set<std::uint64_t> established_socket_inodes(std::istream &table,
                                                      std::uint16_t remote_port);

    /**
     * @brief File descriptors of this process for its established TCP connections to a remote
     *        port (IPv4 and IPv6), found through /proc (Linux).
     *
     * The Vicon SDK opens its connection internally; this is how the node finds it.
     */
    std::vector<int> find_tcp_sockets_to_port(std::uint16_t remote_port);

    /**
     * @brief Enable TCP keepalive on a connected socket so that a dead connection fails.
     *
     * While no data arrives, the kernel sends a probe every second (for timeouts above 128 s,
     * every ceil(timeout_s / 128) seconds, as Linux allows at most 127 probes); after about
     * timeout_s seconds without any answer (server down, network or cable gone), the connection
     * is closed with an error, which the Vicon SDK reports as NotConnected. An idle but alive
     * server answers the probes, so the connection stays open.
     *
     * @param socket_fd  Connected TCP socket.
     * @param timeout_s  Seconds without answer before the connection fails (>= 2).
     * @throws std::invalid_argument if timeout_s < 2.
     * @throws std::system_error if a socket option cannot be set.
     */
    void enable_tcp_keepalive(int socket_fd, int timeout_s);
} // namespace vicon_receiver

#endif // VICON_RECEIVER_UTILITIES_HPP
