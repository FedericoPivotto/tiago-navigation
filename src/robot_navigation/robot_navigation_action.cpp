/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <robot_navigation_action.h>

// RobotNavigation action constructor
rn::RobotNavigationAction::RobotNavigationAction(std::string name) : as_(nh_, name, boost::bind(&RobotNavigationAction::navigate_robot_callback, this, _1), false), action_name_(name), robot_controller_client_("robot_controller_action", true) {
    // Start the robot navigation action
    as_.start();

    // Wait robot controller server
    robot_controller_client_.waitForServer();
}

/* Callback function describing the action for the robot navigation */
void rn::RobotNavigationAction::navigate_robot_callback(const ir2425_group_15::RobotNavigationGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Initial navigation
    ir2425_group_15::RobotControllerGoal initial_goal;
    initial_goal.motion = "initial";
    // Update robot navigation feedback with the robot control result
    robot_controller_client_.sendGoal(initial_goal, boost::bind(&RobotNavigationAction::robot_controller_done_callback, this, _1, _2), NULL, NULL);
    robot_controller_client_.waitForResult();
    // Publish the feedback
    as_.publishFeedback(feedback_);

    // Corridor navigation
    ir2425_group_15::RobotControllerGoal corridor_goal;
    corridor_goal.motion = "corridor";
    corridor_goal.waypoint = goal->waypoint_path.poses[0];
    // Update robot navigation feedback with the robot control result
    robot_controller_client_.sendGoal(corridor_goal, boost::bind(&RobotNavigationAction::robot_controller_done_callback, this, _1, _2), NULL, NULL);
    robot_controller_client_.waitForResult();
    // Publish the feedback
    as_.publishFeedback(feedback_);
    
    // Robot navigation steps
    for(size_t i = 1; i < goal->n_waypoints; ++i) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Set action failure
            success = false;
            break;
        }

        // Final navigation
        ir2425_group_15::RobotControllerGoal final_goal;
        final_goal.motion = "final";
        final_goal.waypoint = goal->waypoint_path.poses[i];
        // Update robot navigation feedback with the robot control result
        robot_controller_client_.sendGoal(final_goal, boost::bind(&RobotNavigationAction::robot_controller_done_callback, this, _1, _2), NULL, NULL);
        robot_controller_client_.waitForResult();
        // Publish the feedback
        as_.publishFeedback(feedback_);
    }

    // Prepare the robot navigation result
    result_.is_completed = success;
    result_.robot_state = success ? "Navigation success" : "Navigation preempted";

    // Publish the robot navigation result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Done callback function for robot navigation */
void rn::RobotNavigationAction::robot_controller_done_callback(const actionlib::SimpleClientGoalState& robot_controller_state, const ir2425_group_15::RobotControllerResultConstPtr& robot_controller_result) {
    feedback_.robot_controller_result = *robot_controller_result;
}