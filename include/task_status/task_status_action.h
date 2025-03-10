#ifndef TASK_STATUS_ACTION_H
#define TASK_STATUS_ACTION_H

/* Standard libraries */
#include <actionlib/server/simple_action_server.h>

/* User-defined libraries */
#include <task_status.h>
#include <apriltags_detection_action.h>
#include <robot_navigation_action.h>

/* Task status namespace*/
namespace ts {

    /* TaskStatus action class handling the task with robot navigation and AprtilTags detection  */
    class TaskStatusAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::TaskStatusAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::TaskStatusFeedback feedback_;
            ir2425_group_15::TaskStatusResult result_;

            // Additional attributes
            actionlib::SimpleActionClient<ir2425_group_15::RobotNavigationAction> robot_navigation_client_;
            actionlib::SimpleActionClient<ir2425_group_15::AprilTagsDetectionAction> apriltags_detection_client_;

        public:
            // TaskStatus action constructor
            TaskStatusAction(std::string name);

            // Callback function describing the action for the robot navigation
            void task_status_callback(const ir2425_group_15::TaskStatusGoalConstPtr &goal);

            // Callback functions for the robot navigation
            void robot_navigation_active_callback();
            void robot_navigation_done_callback(const actionlib::SimpleClientGoalState& robot_navigation_state, const ir2425_group_15::RobotNavigationResultConstPtr& robot_navigation_result);
            void robot_navigation_status_feedback_callback(const ir2425_group_15::RobotNavigationFeedbackConstPtr& robot_navigation_feedback);

            // Callback functions for the AprilTags detection
            void apriltags_detection_done_callback(const actionlib::SimpleClientGoalState& apriltags_detection_state, const ir2425_group_15::AprilTagsDetectionResultConstPtr& apriltags_detection_result);
            void apriltags_detection_status_feedback_callback(const ir2425_group_15::AprilTagsDetectionFeedbackConstPtr& apriltags_detection_feedback);

            // Auxiliary functions
            bool is_robot_navigation_succeeded();
            bool is_apriltags_detection_succeeded();
    };    
}

#endif // TASK_STATUS_ACTION_H