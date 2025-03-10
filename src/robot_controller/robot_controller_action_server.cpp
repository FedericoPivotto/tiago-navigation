/* User-defined libraries */
#include <robot_controller_action.h>

int main(int argc, char **argv) {
    // Node "robot_controller_node" with arguments
    ros::init(argc, argv, "robot_controller_node");

    // Create the action server "robot_controller_action"
    rc::RobotControllerAction robot_controller_action("robot_controller_action");

    // Spin the callback function
    ros::spin();

    return 0;
}