/* Standard libraries */
#include <geometry_msgs/Twist.h>

/* User-defined libraries */
#include <robot_controller_action.h>
#include <message_builder.h>

// RobotController action constructor
rc::RobotControllerAction::RobotControllerAction(std::string name) : as_(nh_, name, boost::bind(&rc::RobotControllerAction::control_robot_callback, this, _1), false), action_name_(name), move_base_client_("move_base", true), play_motion_client_("play_motion", true), is_corridor_(false) {
    // Start the robot controller action
    as_.start();

    // Subscribe to the topic "scan"
    laser_subscriber_ = nh_.subscribe("scan", 1, &rc::RobotControllerAction::corridor_check_callback, this);
    // Publish in the topic "mobile_base_controller/cmd_vel"
    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);

    // Wait robot navigation and AprilTags detection servers
    move_base_client_.waitForServer();
    play_motion_client_.waitForServer();
}

/* Callback function describing the action for the robot controller */
void rc::RobotControllerAction::control_robot_callback(const ir2425_group_15::RobotControllerGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Execute selected motion
    if(goal->motion == "initial")
        this->initial_navigation();
    else if(goal->motion == "corridor")
        this->corridor_navigation(goal->waypoint);
    else if(goal->motion == "final")
        this->final_navigation(goal->waypoint);
    else {
        success = false;
        result_.robot_state = "Navigation subroutine not implemented";
    }

    // Publish the robot control result
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Callback function to read laser scan topic and check corridor */
void rc::RobotControllerAction::corridor_check_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    // Setup thresholds for checking corridor
    float distance_check_threshold = 0.75;
    float corridor_side_threshold = 10;

    // Setup counter of close laser samples with respect to the robot
    int right_close_samples = 0;
    int left_close_samples = 0;

    // Number of samples to scan either on left or right side of the robot
    int side_samples_to_scan = 25;

    // Index in range list of msg representing right side and left side of the robot
    int right_side_index = (-(M_PI / 2.0) - msg->angle_min) / (msg->angle_increment);
    int left_side_index = ((M_PI / 2.0) - msg->angle_min) / (msg->angle_increment);

    // Scan current msg range list to right side
    for(int i = (-side_samples_to_scan / 2); i < (side_samples_to_scan / 2 + 1); i++) {
        if(msg->ranges[right_side_index + i] <= distance_check_threshold)
            right_close_samples += 1;
    }

    // Scan current msg range list to left side
    for(int i = (-side_samples_to_scan / 2); i < (side_samples_to_scan / 2 + 1); i++) {
        if(msg->ranges[left_side_index + i] <= distance_check_threshold)
            left_close_samples += 1;
    }

    // Check if robot is in corridor
    if((right_close_samples >= corridor_side_threshold) && (left_close_samples >= corridor_side_threshold)) {
        left_distance_ = msg->ranges[left_side_index];
        right_distance_ = msg->ranges[right_side_index];

        is_corridor_ = true;
    }
    else {
        is_corridor_ = false;
    }

    return;
}

/* Function moving the robot to the next goal in the initial room */
void rc::RobotControllerAction::initial_navigation() {
    // Perform complete rotation
    for(size_t i = 1; i <= 2; ++i) {
        // Send the play motion goal
        play_motion_msgs::PlayMotionGoal play_motion_goal = rc::get_play_motion_goal("inspect_surroundings");
        play_motion_client_.sendGoalAndWait(play_motion_goal);
        
        // Move the robot with constant velocity until in corridor
        geometry_msgs::Twist robot_vel;
        double rotation_frequency = 0.2;
        robot_vel.angular.z = M_PI * rotation_frequency;
        ros::Rate rate(10);
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
            velocity_publisher_.publish(robot_vel);
            rate.sleep();
        }
        
        // Publish robot velocity equal to zero
        robot_vel.angular.z = 0;
        velocity_publisher_.publish(robot_vel);
    }

    // Prepare the robot control result
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_link", "map");
    result_.robot_state = "Subroutine initial navigation completed";

    return;
}

/* Function moving the robot to the next goal in the corridor room */
void rc::RobotControllerAction::corridor_navigation(geometry_msgs::PoseStamped waypoint) {
    // Send the move base goal
    move_base_msgs::MoveBaseGoal goal = rc::get_move_base_goal(waypoint);
    move_base_client_.sendGoalAndWait(goal);
    
    // Set the check rate of corridor
    ros::Rate rate(10);

    // Move the robot with constant velocity until in corridor
    geometry_msgs::Twist robot_vel;
    robot_vel.linear.x = 0.4;

    bool is_initial = true;
    double initial_left_distance_, initial_right_distance_;
    while(is_corridor_) {
        // Initial corridor distances
        if(is_initial) {
            initial_left_distance_ = left_distance_;
            initial_right_distance_ = right_distance_;

            is_initial = false; 
        }

        // Wall collision avoidance
        double diff = (left_distance_ - initial_left_distance_) - (right_distance_ - initial_right_distance_);
        double factor = 0;
        if(diff > 0)
            factor = 1;
        else if (diff < 0)
            factor = -1;
        double rotational_velocity = 0.1;
        robot_vel.angular.z = factor * rotational_velocity;

        // Publish velocity
        velocity_publisher_.publish(robot_vel);
        rate.sleep();
    }

    // Publish robot velocity equal to zero
    robot_vel.linear.x = 0;
    velocity_publisher_.publish(robot_vel);

    // Send the play motion goal
    play_motion_msgs::PlayMotionGoal play_motion_goal = rc::get_play_motion_goal("inspect_surroundings");
    play_motion_client_.sendGoalAndWait(play_motion_goal);

    // Prepare the robot control result
    result_.waypoint = mb::get_frame_pose_stamped_wrt_frame("base_link", "map");
    bool success = play_motion_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.robot_state = success ? "Subroutine corridor navigation completed" : "Subroutine corridor navigation preempted";

    return;
}

/* Function moving the robot to the next goal in the final room */
void rc::RobotControllerAction::final_navigation(geometry_msgs::PoseStamped waypoint) {
    // Setup the play motion client
    play_motion_msgs::PlayMotionGoal play_motion_goal = rc::get_play_motion_goal("inspect_surroundings");
    
    // Send the move base goal with 120 secconds timeout
    move_base_msgs::MoveBaseGoal move_base_goal = rc::get_move_base_goal(waypoint);
    move_base_client_.sendGoalAndWait(move_base_goal, ros::Duration(120));
    
    // Send the move base goal for left rotation
    geometry_msgs::PoseStamped left_rotation_waypoint = rc::get_pose_stamped_rotation(mb::get_header(1, ros::Time::now(), "base_link"), M_PI/2);
    move_base_msgs::MoveBaseGoal left_rotation_goal = rc::get_move_base_goal(left_rotation_waypoint);
    move_base_client_.sendGoalAndWait(left_rotation_goal);

    // Send the play motion goal
    play_motion_client_.sendGoalAndWait(play_motion_goal);

    // Send the move base goal for right rotation
    geometry_msgs::Twist robot_vel;
    double rotation_frequency = 0.2;
    robot_vel.angular.z = M_PI * rotation_frequency;
    ros::Rate rate(10);
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        velocity_publisher_.publish(robot_vel);
        rate.sleep();
    }

    // Send the play motion goal
    play_motion_client_.sendGoalAndWait(play_motion_goal);

    // Send the move base goal to recover the initial orientation
    geometry_msgs::PoseStamped initial_rotation_waypoint = rc::get_pose_stamped_rotation(mb::get_header(3, ros::Time::now(), "base_link"), M_PI/2);
    move_base_msgs::MoveBaseGoal initial_rotation_goal = rc::get_move_base_goal(initial_rotation_waypoint);
    move_base_client_.sendGoalAndWait(initial_rotation_goal);

    // Prepare the robot control result
    result_.waypoint = waypoint;
    bool success = move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    result_.robot_state = success ? "Subroutine final navigation step completed" : "Subroutine final navigation step preempted";

    return;
}