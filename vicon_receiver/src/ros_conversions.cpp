/**
 * @file ros_conversions.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Conversions from Vicon names and SDK results to ROS names, messages, transforms and times.
 * @version 0.1
 * @date 2026-11-07
 */

#include "vicon_receiver/ros_conversions.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>

namespace vicon_receiver
{
    namespace sdk = ViconDataStreamSDK::CPP;

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

        /// Accepted deviation of the quaternion norm from 1 for a valid pose.
        constexpr double kQuaternionNormTolerance = 1e-3;

        /// Nanoseconds per second.
        constexpr double kNanosecondsPerSecond = 1e9;
    } // namespace

    // --------------------------------------------------------------------------------------------
    // Vicon names -> ROS names
    // --------------------------------------------------------------------------------------------

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

    bool NameOwnership::claim(const std::string &name, const std::string &owner)
    {
        const auto inserted = owners_.emplace(name, owner);
        return inserted.second || inserted.first->second == owner;
    }

    std::string NameOwnership::owner_of(const std::string &name) const
    {
        const auto owner_it = owners_.find(name);
        return owner_it == owners_.end() ? std::string() : owner_it->second;
    }

    // --------------------------------------------------------------------------------------------
    // SDK results -> messages, transforms and times
    // --------------------------------------------------------------------------------------------

    vicon_receiver_msgs::msg::SegmentState
    make_segment_state(const std::string &segment_name,
                       const std::string &frame_id,
                       const sdk::Output_GetSegmentGlobalTranslation &translation,
                       const sdk::Output_GetSegmentGlobalRotationQuaternion &rotation)
    {
        vicon_receiver_msgs::msg::SegmentState segment;
        segment.name = segment_name;
        segment.frame_id = frame_id;

        const bool translation_ok = translation.Result == sdk::Result::Success;
        if (translation_ok)
        {
            segment.position.x = translation.Translation[0] / kMillimetresPerMetre;
            segment.position.y = translation.Translation[1] / kMillimetresPerMetre;
            segment.position.z = translation.Translation[2] / kMillimetresPerMetre;
        }
        segment.translation_occluded = !translation_ok || translation.Occluded;

        // Vicon quaternion order is x, y, z, w, like ROS: copied as streamed
        const bool rotation_ok = rotation.Result == sdk::Result::Success;
        if (rotation_ok)
        {
            segment.orientation.x = rotation.Rotation[0];
            segment.orientation.y = rotation.Rotation[1];
            segment.orientation.z = rotation.Rotation[2];
            segment.orientation.w = rotation.Rotation[3];
        }
        else
        {
            // No measurement: same all-zero quaternion that Vicon sends for occluded segments
            segment.orientation.w = 0.0;
        }
        segment.rotation_occluded = !rotation_ok || rotation.Occluded;

        return segment;
    }

    vicon_receiver_msgs::msg::MarkerState
    make_marker_state(const std::string &marker_name,
                      const std::string &parent_segment,
                      const sdk::Output_GetMarkerGlobalTranslation &translation,
                      const std::vector<std::uint32_t> &camera_ids)
    {
        vicon_receiver_msgs::msg::MarkerState marker;
        marker.name = marker_name;
        marker.parent_segment = parent_segment;

        const bool translation_ok = translation.Result == sdk::Result::Success;
        if (translation_ok)
        {
            marker.position.x = translation.Translation[0] / kMillimetresPerMetre;
            marker.position.y = translation.Translation[1] / kMillimetresPerMetre;
            marker.position.z = translation.Translation[2] / kMillimetresPerMetre;
        }
        marker.occluded = !translation_ok || translation.Occluded;
        marker.camera_ids = camera_ids;
        return marker;
    }

    bool is_valid_pose(const vicon_receiver_msgs::msg::SegmentState &segment)
    {
        if (segment.translation_occluded || segment.rotation_occluded)
        {
            return false;
        }

        const auto &q = segment.orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return std::abs(norm - 1.0) <= kQuaternionNormTolerance;
    }

    geometry_msgs::msg::TransformStamped
    make_segment_transform(const vicon_receiver_msgs::msg::SegmentState &segment,
                           const std::string &parent_frame,
                           const builtin_interfaces::msg::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = segment.frame_id;

        transform.transform.translation.x = segment.position.x;
        transform.transform.translation.y = segment.position.y;
        transform.transform.translation.z = segment.position.z;
        transform.transform.rotation = segment.orientation;
        return transform;
    }

    builtin_interfaces::msg::Time to_time_msg(double seconds)
    {
        if (!std::isfinite(seconds) || seconds < 0.0)
        {
            throw std::invalid_argument("to_time_msg: invalid time " + std::to_string(seconds));
        }

        // Split first, so that the nanoseconds are not computed from a huge double product
        const double whole_seconds = std::floor(seconds);
        double nanoseconds = std::round((seconds - whole_seconds) * kNanosecondsPerSecond);

        builtin_interfaces::msg::Time time;
        time.sec = static_cast<std::int32_t>(whole_seconds);
        if (nanoseconds >= kNanosecondsPerSecond)
        {
            // Rounding reached the next second
            time.sec += 1;
            nanoseconds -= kNanosecondsPerSecond;
        }
        time.nanosec = static_cast<std::uint32_t>(nanoseconds);
        return time;
    }

    double to_seconds(const builtin_interfaces::msg::Time &time)
    {
        return static_cast<double>(time.sec) +
               static_cast<double>(time.nanosec) / kNanosecondsPerSecond;
    }
} // namespace vicon_receiver
