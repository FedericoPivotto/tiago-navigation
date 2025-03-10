/* User-defined libraries */
#include <task_status_action.h>

int main(int argc, char **argv) {
    // Node "task_status_node" with arguments
    ros::init(argc, argv, "task_status_node");

    // Create the action server "task_status_action"
    ts::TaskStatusAction task_status_action("task_status_action");

    // Spin the AprilTags detect callback function
    ros::spin();

    return 0;
}