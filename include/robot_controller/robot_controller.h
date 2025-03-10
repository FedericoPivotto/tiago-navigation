#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

/* Standard libraries */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <play_motion_msgs/PlayMotionAction.h>

/* Robot controller namespace */
namespace rc {
    /* Aliases for using move base client */
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> PlayMotionClient;

    move_base_msgs::MoveBaseGoal get_move_base_goal(geometry_msgs::PoseStamped pose_stamped);
    play_motion_msgs::PlayMotionGoal get_play_motion_goal(std::string motion_name);

    geometry_msgs::PoseStamped get_pose_stamped_rotation(std_msgs::Header header, float angle);
    tf2::Quaternion get_tf_quaternion_from_quaternion(geometry_msgs::Quaternion quaternion);
}

#endif // ROBOT_CONTROLLER_H