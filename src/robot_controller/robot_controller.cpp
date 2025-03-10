/* User-defined libraries */
#include <robot_controller.h>
#include <message_builder.h>

/* Function to get the move_base goal for the given pose stamped */
move_base_msgs::MoveBaseGoal rc::get_move_base_goal(geometry_msgs::PoseStamped pose_stamped) {
    // Set the goal pose
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose_stamped;

    return goal;
}

/* Function to get the play_motion goal for the given motion */
play_motion_msgs::PlayMotionGoal rc::get_play_motion_goal(std::string motion_name) {
    // Set the goal motion
    play_motion_msgs::PlayMotionGoal goal;
    goal.motion_name = motion_name;
    goal.skip_planning = false;
    goal.priority = 0;

    return goal;
}

/* Function to get the move_base goal for a given rotation angle in radiants */
geometry_msgs::PoseStamped rc::get_pose_stamped_rotation(std_msgs::Header header, float angle) {
    // Set the robot goal pose stamped
    tf2::Quaternion tf_rotation = rc::get_tf_quaternion_from_quaternion(mb::get_quaternion_from_rpy(0, 0, angle));
    geometry_msgs::PoseStamped pose_stamped = mb::get_pose_stamped(header, mb::get_pose(mb::get_point(), tf2::toMsg(tf_rotation)));

    return pose_stamped;
}

/* Function to get tf2::Quaternion from geometry_msgs/Quaternion */
tf2::Quaternion rc::get_tf_quaternion_from_quaternion(geometry_msgs::Quaternion quaternion) {
    return tf2::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}