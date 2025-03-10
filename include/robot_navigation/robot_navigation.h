#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

/* Standard libraries */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <play_motion_msgs/PlayMotionAction.h>

/* User-defined libraries */
#include <message_builder.h>
#include <ir2425_group_15/RobotNavigationAction.h>

/* Robot navigation namespace */
namespace rn {
    std::vector<geometry_msgs::PoseStamped> parse_waypoints_yaml(std::string package, std::string path, std::string filename, std::string ext);
    nav_msgs::Path get_waypoint_path(std::string package, std::string path, std::string filename, std::string ext);
}

#endif // ROBOT_NAVIGATION_H