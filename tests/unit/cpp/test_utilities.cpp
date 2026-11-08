/**
 * @file test_utilities.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Unit tests of utilities.hpp: SDK names, TCP keepalive.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/utilities.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

// =================================================================================================
// Names of the Vicon SDK enumerations
// =================================================================================================
namespace sdk_names_tests
{
    namespace sdk = ViconDataStreamSDK::CPP;

    TEST(SdkNames, ResultNamesMatchEnumerators)
    {
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::Success), "Success");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::ClientConnectionFailed),
                  "ClientConnectionFailed");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::NoFrame), "NoFrame");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::InvalidSubjectName),
                  "InvalidSubjectName");
        EXPECT_EQ(vicon_receiver::result_name(sdk::Result::ArgumentOutOfRange),
                  "ArgumentOutOfRange");
    }

    TEST(SdkNames, EveryResultHasAName)
    {
        // From the first to the last enumerator of SDK 1.12: none falls back to "Unknown(<n>)"
        for (int value = sdk::Result::Unknown; value <= sdk::Result::ArgumentOutOfRange; ++value)
        {
            const std::string name =
                vicon_receiver::result_name(static_cast<sdk::Result::Enum>(value));
            EXPECT_EQ(name.find("Unknown("), std::string::npos) << value;
        }
    }

    TEST(SdkNames, ValuesOutsideTheEnumerationAreNumbered)
    {
        // E.g. a result added by a newer SDK
        const int future_value = sdk::Result::ArgumentOutOfRange + 1;
        EXPECT_EQ(vicon_receiver::result_name(static_cast<sdk::Result::Enum>(future_value)),
                  "Unknown(" + std::to_string(future_value) + ")");
    }

    TEST(SdkNames, OtherEnumerations)
    {
        EXPECT_EQ(vicon_receiver::direction_name(sdk::Direction::Forward), "Forward");
        EXPECT_EQ(vicon_receiver::direction_name(sdk::Direction::Up), "Up");
        EXPECT_EQ(vicon_receiver::server_orientation_name(sdk::ServerOrientation::ZUp), "ZUp");
        EXPECT_EQ(vicon_receiver::timecode_standard_name(sdk::TimecodeStandard::PAL), "PAL");
        EXPECT_EQ(vicon_receiver::timecode_standard_name(sdk::TimecodeStandard::None), "None");
        EXPECT_EQ(vicon_receiver::device_type_name(sdk::DeviceType::ForcePlate), "ForcePlate");
    }

    TEST(SdkNames, StreamModeNamesRoundTrip)
    {
        for (const sdk::StreamMode::Enum mode : {sdk::StreamMode::ServerPush,
                                                 sdk::StreamMode::ClientPull,
                                                 sdk::StreamMode::ClientPullPreFetch})
        {
            const std::string name = vicon_receiver::stream_mode_name(mode);
            EXPECT_EQ(vicon_receiver::stream_mode_from_name(name), mode) << name;
        }
        EXPECT_EQ(vicon_receiver::stream_mode_name(sdk::StreamMode::ServerPush), "ServerPush");
    }

    TEST(SdkNames, UnknownStreamModeNameIsRejected)
    {
        // Names are case-sensitive, as written in the parameters file
        EXPECT_THROW(vicon_receiver::stream_mode_from_name("serverpush"), std::invalid_argument);
        EXPECT_THROW(vicon_receiver::stream_mode_from_name(""), std::invalid_argument);
    }
} // namespace sdk_names_tests

// =================================================================================================
// TCP keepalive of the SDK connection
// =================================================================================================
namespace tcp_keepalive_tests
{
    using vicon_receiver::enable_tcp_keepalive;
    using vicon_receiver::established_socket_inodes;
    using vicon_receiver::find_tcp_sockets_to_port;
    using vicon_receiver::server_port;

    namespace
    {
        /**
     * @brief A listening socket on 127.0.0.1 and one client connected to it.
     */
        struct LocalConnection
        {
            int listener = -1;
            int client = -1;
            int accepted = -1;
            std::uint16_t port = 0;

            LocalConnection()
            {
                listener = socket(AF_INET, SOCK_STREAM, 0);
                sockaddr_in address{};
                address.sin_family = AF_INET;
                address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                address.sin_port = 0;
                bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address));
                listen(listener, 1);
                socklen_t length = sizeof(address);
                getsockname(listener, reinterpret_cast<sockaddr *>(&address), &length);
                port = ntohs(address.sin_port);

                client = socket(AF_INET, SOCK_STREAM, 0);
                connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address));
                accepted = accept(listener, nullptr, nullptr);
            }

            ~LocalConnection()
            {
                for (const int socket_fd : {accepted, client, listener})
                {
                    if (socket_fd >= 0)
                    {
                        close(socket_fd);
                    }
                }
            }
        };

        /// Integer value of a socket option.
        int get_option(int socket_fd, int level, int option)
        {
            int value = 0;
            socklen_t length = sizeof(value);
            getsockopt(socket_fd, level, option, &value, &length);
            return value;
        }
    } // namespace

    // ---------------------------------------------------------------------------------------------
    // Server port
    // ---------------------------------------------------------------------------------------------

    TEST(TcpKeepalive, ServerPortDefaultsToDataStreamPort)
    {
        EXPECT_EQ(server_port("10.250.223.40"), 801);
        EXPECT_EQ(server_port("vicon-pc"), 801);
        EXPECT_EQ(server_port("127.0.0.1:9000"), 9000);
        // IPv6 address without port
        EXPECT_EQ(server_port("fe80::1"), 801);
    }

    TEST(TcpKeepalive, InvalidServerPortIsRejected)
    {
        EXPECT_THROW(server_port("host:"), std::invalid_argument);
        EXPECT_THROW(server_port("host:80a"), std::invalid_argument);
        EXPECT_THROW(server_port("host:70000"), std::invalid_argument);
    }

    // ---------------------------------------------------------------------------------------------
    // /proc/net/tcp
    // ---------------------------------------------------------------------------------------------

    TEST(TcpKeepalive, EstablishedConnectionsToThePortAreFound)
    {
        // Port 801 = 0x0321; state 01 = established, 0A = listening, 06 = time wait
        std::istringstream table(
            "  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  "
            "timeout inode\n"
            "   0: 030AFA0A:B42C 28DFFA0A:0321 01 00000000:00000000 00:00000000 00000000  1000     "
            "   "
            "0 111 1 0000000000000000 20 4 30 10 -1\n"
            "   1: 0100007F:0321 00000000:0000 0A 00000000:00000000 00:00000000 00000000  1000     "
            "   "
            "0 222 1 0000000000000000 100 0 0 10 0\n"
            "   2: 030AFA0A:B42D 28DFFA0A:0321 06 00000000:00000000 03:00000F9E 00000000     0     "
            "   "
            "0 0 3 0000000000000000\n"
            "   3: 030AFA0A:C000 28DFFA0A:0050 01 00000000:00000000 00:00000000 00000000  1000     "
            "   "
            "0 333 1 0000000000000000 20 4 30 10 -1\n");
        EXPECT_EQ(established_socket_inodes(table, 801), std::set<std::uint64_t>{111});
    }

    // ---------------------------------------------------------------------------------------------
    // Real local connection
    // ---------------------------------------------------------------------------------------------

    TEST(TcpKeepalive, ClientSocketOfThisProcessIsFound)
    {
        LocalConnection connection;
        ASSERT_GE(connection.accepted, 0);

        // The client side has the listener port as remote port; the accepted side does not
        const std::vector<int> sockets = find_tcp_sockets_to_port(connection.port);
        EXPECT_NE(std::find(sockets.begin(), sockets.end(), connection.client), sockets.end());
        EXPECT_EQ(std::find(sockets.begin(), sockets.end(), connection.accepted), sockets.end());
    }

    TEST(TcpKeepalive, KeepaliveOptionsAreSet)
    {
        LocalConnection connection;
        ASSERT_GE(connection.client, 0);

        enable_tcp_keepalive(connection.client, 5);
        EXPECT_EQ(get_option(connection.client, SOL_SOCKET, SO_KEEPALIVE), 1);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPIDLE), 1);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPINTVL), 1);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPCNT), 4);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_USER_TIMEOUT), 5000);
    }

    TEST(TcpKeepalive, LongTimeoutsStayWithinTheProbeLimit)
    {
        // Linux rejects more than 127 probes: longer timeouts space the probes instead
        LocalConnection connection;
        ASSERT_GE(connection.client, 0);

        enable_tcp_keepalive(connection.client, 128);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPINTVL), 1);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPCNT), 127);

        // Largest accepted connection_timeout_s
        enable_tcp_keepalive(connection.client, 3600);
        const int interval_s = get_option(connection.client, IPPROTO_TCP, TCP_KEEPINTVL);
        const int probe_count = get_option(connection.client, IPPROTO_TCP, TCP_KEEPCNT);
        EXPECT_EQ(interval_s, 29);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_KEEPIDLE), interval_s);
        EXPECT_LE(probe_count, 127);
        // First probe after one interval, then probe_count intervals: about the timeout
        EXPECT_NEAR((probe_count + 1) * interval_s, 3600, interval_s);
        EXPECT_EQ(get_option(connection.client, IPPROTO_TCP, TCP_USER_TIMEOUT), 3600000);
    }

    TEST(TcpKeepalive, TooShortTimeoutIsRejected)
    {
        LocalConnection connection;
        EXPECT_THROW(enable_tcp_keepalive(connection.client, 1), std::invalid_argument);
    }

    TEST(TcpKeepalive, ClosedSocketCannotBeConfigured)
    {
        EXPECT_THROW(enable_tcp_keepalive(-1, 5), std::system_error);
    }

    TEST(TcpKeepalive, DeadPeerIsDetectedWithinTheTimeout)
    {
        // Drops every packet on lo: only inside the private user + network namespace that
        // tests/integration/test_connection_loss.py creates (no root needed there)
        const char *in_namespace = std::getenv("VICON_TEST_NETWORK_NAMESPACE");
        if (in_namespace == nullptr || std::string(in_namespace) != "1")
        {
            GTEST_SKIP() << "needs a private network namespace: "
                            "tests/integration/test_connection_loss.py runs it";
        }

        LocalConnection connection;
        ASSERT_GE(connection.accepted, 0);
        constexpr int kTimeoutS = 3;
        enable_tcp_keepalive(connection.client, kTimeoutS);

        // From now on the peer never answers, as with a Tracker switched off or a cable pulled
        ASSERT_EQ(std::system("tc qdisc add dev lo root netem loss 100%"), 0);
        const auto start = std::chrono::steady_clock::now();
        pollfd poll_fd{connection.client, POLLIN, 0};
        const int ready = poll(&poll_fd, 1, 20000);
        const double elapsed_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        ASSERT_EQ(ready, 1) << "dead connection not reported within 20 s";

        int error = 0;
        socklen_t length = sizeof(error);
        getsockopt(connection.client, SOL_SOCKET, SO_ERROR, &error, &length);
        EXPECT_EQ(error, ETIMEDOUT) << std::strerror(error);
        EXPECT_GE(elapsed_s, kTimeoutS - 1.0);
        EXPECT_LE(elapsed_s, kTimeoutS + 4.0);
    }
} // namespace tcp_keepalive_tests

// =================================================================================================
// Running statistics
// =================================================================================================
namespace running_min_mean_max_tests
{
    using vicon_receiver::RunningMinMeanMax;

    TEST(RunningMinMeanMax, EmptyStatisticsAreNaN)
    {
        const RunningMinMeanMax statistics;
        EXPECT_EQ(statistics.count(), 0u);
        EXPECT_TRUE(std::isnan(statistics.minimum()));
        EXPECT_TRUE(std::isnan(statistics.mean()));
        EXPECT_TRUE(std::isnan(statistics.maximum()));
    }

    TEST(RunningMinMeanMax, MinimumMeanMaximum)
    {
        RunningMinMeanMax statistics;
        for (const double sample : {3.0, 1.0, 2.0})
        {
            statistics.add(sample);
        }
        EXPECT_EQ(statistics.count(), 3u);
        EXPECT_DOUBLE_EQ(statistics.minimum(), 1.0);
        EXPECT_DOUBLE_EQ(statistics.mean(), 2.0);
        EXPECT_DOUBLE_EQ(statistics.maximum(), 3.0);

        statistics.reset();
        EXPECT_EQ(statistics.count(), 0u);
    }
} // namespace running_min_mean_max_tests
