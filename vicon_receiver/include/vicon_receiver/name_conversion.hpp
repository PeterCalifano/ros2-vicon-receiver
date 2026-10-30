/**
 * @file name_conversion.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Conversion of Vicon subject/segment names into valid ROS 2 name tokens, used for the
 *        topic names and the TF child frame ids published by the node.
 * @version 0.1
 * @date 2026-10-30
 */

#ifndef VICON_RECEIVER_NAME_CONVERSION_HPP
#define VICON_RECEIVER_NAME_CONVERSION_HPP

#include <string>

namespace vicon_receiver
{
    /**
     * @brief Convert a Vicon name into a valid ROS 2 name token (one component of a topic name).
     *
     * Vicon Tracker accepts names that ROS does not, e.g. "g1-unitree-torso-cam". The rules are:
     *   - every character other than [A-Za-z0-9_] becomes '_' (this includes '/', so a Vicon name
     *     can never add namespace levels);
     *   - the token must start with a letter: a leading digit gets the prefix "vicon_" and a
     *     leading '_' the prefix "vicon" (ROS hides topics whose tokens start with '_');
     *   - an empty name becomes "vicon".
     *
     * Different Vicon names can map to the same token ("a-b" and "a_b"); the caller must detect
     * such collisions.
     *
     * @param vicon_name  Subject or segment name as reported by the Vicon DataStream SDK.
     * @return A token valid in ROS 2 topic names and TF frame ids.
     */
    std::string to_ros_name_token(const std::string &vicon_name);

    /**
     * @brief TF child frame id of a segment: "<subject>_<segment>", both parts converted with
     *        to_ros_name_token().
     */
    std::string make_segment_frame_id(const std::string &subject_name,
                                      const std::string &segment_name);

    /**
     * @brief Topic name of a segment pose: "<topic_namespace>/<subject>/<segment>", subject and
     *        segment converted with to_ros_name_token(). The namespace is used as given.
     */
    std::string make_segment_topic_name(const std::string &topic_namespace,
                                        const std::string &subject_name,
                                        const std::string &segment_name);
} // namespace vicon_receiver

#endif // VICON_RECEIVER_NAME_CONVERSION_HPP
