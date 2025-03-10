/* User-defined libraries */
#include <apriltags_detection_action.h>

int main(int argc, char **argv) {
    // Node "apriltags_detection_node" with arguments
    ros::init(argc, argv, "apriltags_detection_node");

    // Create the action server "apriltags_detection_action"
    ad::AprilTagsDetectionAction apriltags_detection_action("apriltags_detection_action");

    // Spin the AprilTags detect callback function
    ros::spin();

    return 0;
}