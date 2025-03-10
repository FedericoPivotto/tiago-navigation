/* User-defined libraries */
#include <robot_navigation_action.h>

int main(int argc, char **argv) {
    // Node "robot_navigation_node" with arguments
    ros::init(argc, argv, "robot_navigation_node");

    // Create the action server "robot_navigation_action"
    rn::RobotNavigationAction robot_navigation_action("robot_navigation_action");

    // Spin the callback function
    ros::spin();

    return 0;
}