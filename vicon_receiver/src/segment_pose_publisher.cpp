#include "vicon_receiver/segment_pose_publisher.hpp"

SegmentPosePublisher::SegmentPosePublisher(std::string topic_name, rclcpp::Node *node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
    is_ready = true;
}

void SegmentPosePublisher::publish(geometry_msgs::msg::PoseStamped pose_msg)
{
    position_publisher_->publish(pose_msg);
}
