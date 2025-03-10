/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <task_status_action.h>

/* TaskStatus action constructor */
ts::TaskStatusAction::TaskStatusAction(std::string name) : as_(nh_, name, boost::bind(&TaskStatusAction::task_status_callback, this, _1), false), action_name_(name), robot_navigation_client_("robot_navigation_action", true), apriltags_detection_client_("apriltags_detection_action", true) {
    // Start the task status action
    as_.start();

    // Wait robot navigation and AprilTags detection servers
    robot_navigation_client_.waitForServer();
    apriltags_detection_client_.waitForServer();
}

/* Callback function describing the action for the robot navigation */
void ts::TaskStatusAction::task_status_callback(const ir2425_group_15::TaskStatusGoalConstPtr &goal) {
    // Action initial state
    bool success = true;
    
    // Initialize the robot navigation and AprilTags detection server
    robot_navigation_client_.sendGoal(goal->robot_navigation_goal, boost::bind(&TaskStatusAction::robot_navigation_done_callback, this, _1, _2), boost::bind(&TaskStatusAction::robot_navigation_active_callback, this), boost::bind(&TaskStatusAction::robot_navigation_status_feedback_callback, this, _1));
    apriltags_detection_client_.sendGoal(goal->apriltags_detection_goal, boost::bind(&TaskStatusAction::apriltags_detection_done_callback, this, _1, _2), NULL, boost::bind(&TaskStatusAction::apriltags_detection_status_feedback_callback, this, _1));

    // Feedback per second
    ros::Duration duration(2);

    while(!is_apriltags_detection_succeeded() && !is_robot_navigation_succeeded()) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Cancel action clients goal
            robot_navigation_client_.cancelGoal();
            apriltags_detection_client_.cancelGoal();

            // Update the action state
            success = false;
            break;
        }

        // Wait before sending the feedback
        duration.sleep();

        // Publish the feedback
        as_.publishFeedback(feedback_);
    }

    // Cancel robot navigation goal if not reached
    if(! is_robot_navigation_succeeded()) {
        robot_navigation_client_.cancelGoal();

        result_.robot_navigation_result.is_completed = false;
        result_.robot_navigation_result.robot_state = "Navigation preempted";
    }
    
    // Cancel AprilTags detection goal if not reached
    if(! is_apriltags_detection_succeeded()) {
        apriltags_detection_client_.cancelGoal();

        result_.apriltags_detection_result.n_target_apriltag_ids = feedback_.apriltags_detection_feedback.n_target_apriltag_ids;
        result_.apriltags_detection_result.target_apriltag_ids = feedback_.apriltags_detection_feedback.target_apriltag_ids;
        result_.apriltags_detection_result.target_apriltag_poses = feedback_.apriltags_detection_feedback.target_apriltag_poses;

        result_.apriltags_detection_result.n_nontarget_apriltag_ids = feedback_.apriltags_detection_feedback.n_nontarget_apriltag_ids;
        result_.apriltags_detection_result.nontarget_apriltag_ids = feedback_.apriltags_detection_feedback.nontarget_apriltag_ids;
        result_.apriltags_detection_result.nontarget_apriltag_poses = feedback_.apriltags_detection_feedback.nontarget_apriltag_poses;
    }

    // Publish the task status result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Done callback function for robot navigation */
void ts::TaskStatusAction::robot_navigation_done_callback(const actionlib::SimpleClientGoalState& robot_navigation_state, const ir2425_group_15::RobotNavigationResultConstPtr& robot_navigation_result) {
    result_.robot_navigation_result = *robot_navigation_result;
}

/* Done callback function for AprilTags detection */
void ts::TaskStatusAction::apriltags_detection_done_callback(const actionlib::SimpleClientGoalState& apriltags_detection_state, const ir2425_group_15::AprilTagsDetectionResultConstPtr& apriltags_detection_result) {    
    result_.apriltags_detection_result = *apriltags_detection_result;
    result_.is_completed = apriltags_detection_result->is_completed;
}

/* Active callback function for robot navigation */
void ts::TaskStatusAction::robot_navigation_active_callback() {
    feedback_.robot_navigation_feedback.robot_controller_result.robot_state = "Navigation started";
}

/* Feedback callback function printing the robot navigation status */
void ts::TaskStatusAction::robot_navigation_status_feedback_callback(const ir2425_group_15::RobotNavigationFeedbackConstPtr& robot_navigation_feedback) {
    feedback_.robot_navigation_feedback = *robot_navigation_feedback;
}

/* Feedback callback function printing the AprilTags detection status */
void ts::TaskStatusAction::apriltags_detection_status_feedback_callback(const ir2425_group_15::AprilTagsDetectionFeedbackConstPtr& apriltags_detection_feedback) {
    feedback_.apriltags_detection_feedback = *apriltags_detection_feedback;
}

/* Function to check if robot navigation is succeeded */
bool ts::TaskStatusAction::is_robot_navigation_succeeded() {
    return robot_navigation_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

/* Function to check if AprilTags detection is succeeded */
bool ts::TaskStatusAction::is_apriltags_detection_succeeded() {
    return apriltags_detection_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}