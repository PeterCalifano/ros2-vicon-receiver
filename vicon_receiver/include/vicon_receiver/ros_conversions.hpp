/**
 * @file ros_conversions.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief Conversions from Vicon names and SDK results to ROS names, messages, transforms and times.
 * @version 0.1
 * @date 2026-11-07
 */

#ifndef VICON_RECEIVER_ROS_CONVERSIONS_HPP
#define VICON_RECEIVER_ROS_CONVERSIONS_HPP

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vicon-datastream-sdk/DataStreamClient.h"
#include "vicon_receiver_msgs/msg/marker_state.hpp"
#include "vicon_receiver_msgs/msg/segment_state.hpp"

namespace vicon_receiver
{
    // --------------------------------------------------------------------------------------------
    // Vicon names -> ROS names
    // --------------------------------------------------------------------------------------------

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

    /**
     * @brief First-come ownership of the ROS names derived from Vicon names (topics, TF frames).
     *
     * Two segments whose names convert to the same token (e.g. "robot-a" and "robot_a") must not
     * share a topic or a TF frame: the first segment that claims a name keeps it.
     */
    class NameOwnership
    {
      public:
        /**
         * @brief Claim @p name for @p owner.
         * @return true if the name was free (now owned by @p owner) or already owned by it,
         *         false if another owner has it.
         */
        bool claim(const std::string &name, const std::string &owner);

        /// Owner of @p name, empty if nobody claimed it.
        std::string owner_of(const std::string &name) const;

      private:
        std::map<std::string, std::string> owners_;
    };

    // --------------------------------------------------------------------------------------------
    // SDK results -> messages, transforms and times
    // --------------------------------------------------------------------------------------------

    /// Vicon streams positions in millimetres; ROS uses metres.
    constexpr double kMillimetresPerMetre = 1000.0;

    /**
     * @brief Segment state from the SDK global pose of a segment.
     *
     * Positions are converted from millimetres to metres; the quaternion (x, y, z, w) and the
     * occlusion flags are copied as streamed. A call that did not succeed is reported as occluded
     * (its values are not a measurement).
     *
     * @param segment_name  Segment name as streamed.
     * @param frame_id      TF child frame of the segment (make_segment_frame_id()).
     * @param translation   Result of GetSegmentGlobalTranslation.
     * @param rotation      Result of GetSegmentGlobalRotationQuaternion.
     */
    vicon_receiver_msgs::msg::SegmentState make_segment_state(
        const std::string &segment_name,
        const std::string &frame_id,
        const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation &translation,
        const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion &rotation);

    /**
     * @brief Marker state from the SDK global position of a marker.
     *
     * The position is converted to metres. A call that did not succeed is reported as occluded.
     *
     * @param marker_name     Marker name as streamed.
     * @param parent_segment  Segment the marker belongs to.
     * @param translation     Result of GetMarkerGlobalTranslation.
     * @param camera_ids      Cameras whose rays reconstructed the marker.
     */
    vicon_receiver_msgs::msg::MarkerState
    make_marker_state(const std::string &marker_name,
                      const std::string &parent_segment,
                      const ViconDataStreamSDK::CPP::Output_GetMarkerGlobalTranslation &translation,
                      const std::vector<std::uint32_t> &camera_ids);

    /**
     * @brief Whether a segment pose can be published as a valid pose (PoseStamped, TF).
     *
     * False when the translation or the rotation is occluded, and when the quaternion is not of
     * unit norm (Vicon sends [0, 0, 0, 0] for occluded segments).
     */
    bool is_valid_pose(const vicon_receiver_msgs::msg::SegmentState &segment);

    /**
     * @brief Transform from the parent frame to the segment frame, for TF.
     *
     * @param segment       Segment state (pose in the parent frame, metres).
     * @param parent_frame  Frame the pose is expressed in (the Vicon frame).
     * @param stamp         Stamp of the transform (estimated acquisition time).
     */
    geometry_msgs::msg::TransformStamped
    make_segment_transform(const vicon_receiver_msgs::msg::SegmentState &segment,
                           const std::string &parent_frame,
                           const builtin_interfaces::msg::Time &stamp);

    /**
     * @brief Pose of a segment expressed in the world frame.
     *
     * @param segment         Segment with its pose in the Vicon frame [m].
     * @param vicon_in_world  Static transform of the Vicon frame in the world frame; its
     *                        header.frame_id becomes the frame of the result.
     * @param stamp           Stamp of the pose (estimated acquisition time).
     */
    geometry_msgs::msg::PoseStamped
    make_segment_pose_in_world(const vicon_receiver_msgs::msg::SegmentState &segment,
                               const geometry_msgs::msg::TransformStamped &vicon_in_world,
                               const builtin_interfaces::msg::Time &stamp);

    /**
     * @brief ROS time message from seconds since the epoch.
     *
     * @throws std::invalid_argument if the time is negative or not finite.
     */
    builtin_interfaces::msg::Time to_time_msg(double seconds);

    /// Seconds since the epoch from a ROS time message.
    double to_seconds(const builtin_interfaces::msg::Time &time);
} // namespace vicon_receiver

#endif // VICON_RECEIVER_ROS_CONVERSIONS_HPP
