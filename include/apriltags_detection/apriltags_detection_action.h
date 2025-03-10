#ifndef APRILTAGS_DETECTION_ACTION_H
#define APRILTAGS_DETECTION_ACTION_H

/* User-defined libraries */
#include <apriltags_detection.h>

/* AprilTags detection namespace */
namespace ad {

    /* AprilTagsDetection action class handling the AprilTags detection */
    class AprilTagsDetectionAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::AprilTagsDetectionAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::AprilTagsDetectionFeedback feedback_;
            ir2425_group_15::AprilTagsDetectionResult result_;

            // Additional attributes
            ros::Subscriber tag_subscriber_;
            AprilTagsDetection apriltags_;

        public:
            // Task action constructor
            AprilTagsDetectionAction(std::string name);

            // Callback function to read the topic with the AprilTags detected
            void apriltags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

            // Callback function describing the action for the AprilTags detection
            void detect_apriltags_callback(const ir2425_group_15::AprilTagsDetectionGoalConstPtr& goal);

            // Auxiliary functions
            void update_apriltags_detection_feedback();
            void update_apriltags_detection_result();
    };
}

#endif // APRILTAGS_DETECTION_ACTION_H