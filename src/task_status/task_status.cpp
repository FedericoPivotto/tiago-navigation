/* User-defined libraries */
#include <task_status.h>
#include <apriltags_detection.h>

/* Feedback callback function for task status */
void ts::task_status_feedback_callback(const ir2425_group_15::TaskStatusFeedbackConstPtr& task_status_feedback) {
    // Format the task status feedback for AprilTags detection
    ad::AprilTagsDetection apriltags_feedback(
        task_status_feedback->apriltags_detection_feedback.srv_target_apriltag_ids,
        task_status_feedback->apriltags_detection_feedback.target_apriltag_ids, task_status_feedback->apriltags_detection_feedback.target_apriltag_poses, 
        task_status_feedback->apriltags_detection_feedback.nontarget_apriltag_ids, task_status_feedback->apriltags_detection_feedback.nontarget_apriltag_poses
    );

    // Print the task status feedback
    if(task_status_feedback) {
        // Print the task status feedback for robot navigation
        std::string robot_navigation_feedback_print = "[task_status feedback: robot_navigation]:";
        ROS_INFO_STREAM(
            robot_navigation_feedback_print << std::endl
            << "Robot state: " << task_status_feedback->robot_navigation_feedback.robot_controller_result.robot_state << std::endl
        );

        // Print the task status feedback for AprilTags detection
        std::string apriltags_detection_feedback_print = "[task_status feedback: apriltags_detection]:";
        ROS_INFO_STREAM(
            apriltags_detection_feedback_print << std::endl
            << "Target AprilTags IDs to detect: " + std::to_string(task_status_feedback->apriltags_detection_feedback.n_srv_target_apriltag_ids) << std::endl
            << "Target AprilTags IDs detected: " + std::to_string(task_status_feedback->apriltags_detection_feedback.n_target_apriltag_ids) << std::endl
            << "Non-target AprilTags IDs detected: " + std::to_string(task_status_feedback->apriltags_detection_feedback.n_nontarget_apriltag_ids) << std::endl
            << std::endl
            << apriltags_feedback
        );
    } else {
        std::string result_print = "[task_status feedback: error]:";
        ROS_ERROR_STREAM(
            result_print << std::endl 
            << "Error: task_status action failed";
        );
    }
    
    return;
}

/* Done callback function for task status */
void ts::task_status_done_callback(const actionlib::SimpleClientGoalState& task_status_state, const ir2425_group_15::TaskStatusResultConstPtr& task_status_result) {
    // Format the task status result
    ad::AprilTagsDetection apriltags_result(
        task_status_result->apriltags_detection_result.srv_target_apriltag_ids,
        task_status_result->apriltags_detection_result.target_apriltag_ids, task_status_result->apriltags_detection_result.target_apriltag_poses, 
        task_status_result->apriltags_detection_result.nontarget_apriltag_ids, task_status_result->apriltags_detection_result.nontarget_apriltag_poses
    );

    // Print the task status result
    if(task_status_result) {
        // Print the task status result for robot navigation
        std::string robot_navigation_result_print = "[task_status result: robot_navigation]:";
        ROS_INFO_STREAM(
            robot_navigation_result_print << std::endl
            << "Robot state: " << task_status_result->robot_navigation_result.robot_state << std::endl
        );

        // Print the task status result for AprilTags detection
        std::string apriltags_detection_result_print = "[task_status result: apritags_detection]:";
        ROS_INFO_STREAM(
            apriltags_detection_result_print << std::endl
            << "Target AprilTags IDs to detect: " + std::to_string(task_status_result->apriltags_detection_result.n_srv_target_apriltag_ids) << std::endl
            << "Target AprilTags IDs detected: " + std::to_string(task_status_result->apriltags_detection_result.n_target_apriltag_ids) << std::endl
            << "Non-target AprilTags IDs detected: " + std::to_string(task_status_result->apriltags_detection_result.n_nontarget_apriltag_ids) << std::endl
            << std::endl
            << apriltags_result
        );

        // Print task result
        bool result_status_message = task_status_result->apriltags_detection_result.n_srv_target_apriltag_ids == task_status_result->apriltags_detection_result.n_target_apriltag_ids;
        std::string result_state = result_status_message ? "COMPLETED" : "FAILED";
        ROS_INFO_STREAM("[task_status result]" << std::endl << "TASK " << result_state);
    } else {
        std::string result_print = "[task_status result: error]:";
        ROS_ERROR_STREAM(
            result_print << std::endl 
            << "Error: task_status action failed";
        );
    }
}