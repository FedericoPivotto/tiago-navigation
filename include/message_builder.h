#ifndef MESSAGE_BUILDER_H
#define MESSAGE_BUILDER_H

/* Standard libraries */
#include <ros/ros.h>

/* Message builder namespace*/
namespace mb {
    std_msgs::Header get_header(uint32_t seq, ros::Time stamp, std::string frame_id);
    geometry_msgs::Point get_point(float x = 0, float y = 0, float z = 0);
    geometry_msgs::Quaternion get_quaternion(float x = 0, float y = 0, float z = 0, float w = 1);
    geometry_msgs::Quaternion get_quaternion_from_rpy(float x = 0, float y = 0, float z = 0);
    geometry_msgs::Pose get_pose(geometry_msgs::Point position = get_point(), geometry_msgs::Quaternion orientation = get_quaternion());
    geometry_msgs::PoseStamped get_pose_stamped(std_msgs::Header header, geometry_msgs::Pose pose = get_pose());
    geometry_msgs::PoseStamped get_frame_pose_stamped_wrt_frame(std::string source_frame_id, std::string target_frame_id);
    geometry_msgs::PoseStamped get_pose_stamped_wrt_frame(geometry_msgs::PoseStamped pose_stamped, std::string target_frame_id);
}

#endif // MESSAGE_BUILDER_H