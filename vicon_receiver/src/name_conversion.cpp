/**
 * @file name_conversion.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Conversion of Vicon subject/segment names into valid ROS 2 name tokens.
 * @version 0.1
 * @date 2026-10-30
 */

#include "vicon_receiver/name_conversion.hpp"

#include <cctype>

namespace vicon_receiver
{
    namespace
    {
        /// Prefix that makes a token start with a letter (see to_ros_name_token()).
        const char *const kTokenPrefix = "vicon";

        /// Whether a character is allowed in a ROS 2 name token.
        bool is_allowed_character(const char character)
        {
            const unsigned char value = static_cast<unsigned char>(character);
            return std::isalnum(value) != 0 || character == '_';
        }
    } // namespace

    std::string to_ros_name_token(const std::string &vicon_name)
    {
        // Replace every character that ROS does not accept
        std::string token = vicon_name;
        for (char &character : token)
        {
            if (!is_allowed_character(character))
            {
                character = '_';
            }
        }

        if (token.empty())
        {
            return kTokenPrefix;
        }

        // Tokens must not start with a digit (invalid) or '_' (hidden topic): prefix a word.
        // std::isdigit needs an unsigned char value to be well defined.
        const unsigned char first = static_cast<unsigned char>(token.front());
        if (std::isdigit(first) != 0)
        {
            return std::string(kTokenPrefix) + "_" + token;
        }
        if (token.front() == '_')
        {
            return std::string(kTokenPrefix) + token;
        }
        return token;
    }

    std::string make_segment_frame_id(const std::string &subject_name,
                                      const std::string &segment_name)
    {
        return to_ros_name_token(subject_name) + "_" + to_ros_name_token(segment_name);
    }

    std::string make_segment_topic_name(const std::string &topic_namespace,
                                        const std::string &subject_name,
                                        const std::string &segment_name)
    {
        return topic_namespace + "/" + to_ros_name_token(subject_name) + "/" +
               to_ros_name_token(segment_name);
    }
} // namespace vicon_receiver
