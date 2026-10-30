/**
 * @file main.cpp
 * @author OPT4SMART group (original), PeterC (petercalifano.gs@gmail.com) (modifications)
 * @brief Entry point of the vicon_client executable: create the node, connect to the Vicon
 *        DataStream server and stream frames until shutdown.
 * @version 1.1.0
 * @date 2026-10-30
 */

#include "vicon_receiver/communicator.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace
{
    /**
     * @brief Report an invalid or incomplete parameters file and shut ROS 2 down.
     *
     * @param reason  Description of the problem (typically the exception message).
     * @return Process exit code to return from main().
     */
    int report_parameter_error(const std::string &reason)
    {
        const rclcpp::Logger logger = rclcpp::get_logger(Communicator::kNodeName);
        RCLCPP_FATAL(logger, "Invalid node configuration: %s", reason.c_str());
        RCLCPP_FATAL(logger,
                     "Start the node with '--ros-args --params-file config/vicon_client.yaml' "
                     "(or use scripts/launch.sh) and set every parameter with the right type.");
        rclcpp::shutdown();
        return 1;
    }
} // namespace

// Main function
int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Parameter problems surface as exceptions while constructing the node: report them
    // clearly instead of aborting with an uncaught exception.
    std::shared_ptr<Communicator> node;
    try
    {
        node = std::make_shared<Communicator>();
    }
    catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &error)
    {
        // Parameter missing from the parameters file (or no parameters file at all)
        return report_parameter_error(error.what());
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException &error)
    {
        // Parameter present but of the wrong type (e.g. 0 instead of 0.0 for a double)
        return report_parameter_error(error.what());
    }
    catch (const std::invalid_argument &error)
    {
        // Parameter value rejected by the node's own checks (e.g. wrong vector size)
        return report_parameter_error(error.what());
    }

    // Connect to the Vicon server
    node->connect();

    // Continuously retrieve frames while ROS 2 is running
    while (rclcpp::ok())
    {
        node->get_frame();
    }

    // Disconnect from the Vicon server and shut down ROS 2
    node->disconnect();
    rclcpp::shutdown();
    return 0;
}
