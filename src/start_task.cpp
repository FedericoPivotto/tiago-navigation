/* Standard libraries */
#include <ros/package.h>
#include <tiago_iaslab_simulation/Objs.h>

/* User-defined libraries */
#include <task_status_action.h>

int main(int argc, char **argv) {
    // Node "start_task_node" with arguments
    ros::init(argc, argv, "start_task_node");

    // Send an AprilTags request to the server "ids_generator_node" through the service "apriltag_ids_srv"
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("apriltag_ids_srv");

    // Service for AprilTag IDs
    tiago_iaslab_simulation::Objs srv;
    
    // Send service request 
    srv.request.ready = true;
    if(! client.call(srv)) {
        ROS_INFO_STREAM("Failed to call");
        return -1;
    }

    // Print the target AprilTag IDs
    std::string apriltag_ids_target;
    for(const auto& apriltag_id_target: srv.response.ids)
        apriltag_ids_target = apriltag_ids_target + std::to_string(apriltag_id_target) + ", ";
    ROS_INFO_STREAM("[start_task]:\nTarget AprilTag IDs: " << apriltag_ids_target << "\b\b ");

    // Initialize task status action client
    ts::TaskStatusClient action_client("task_status_action", true); 
    action_client.waitForServer();

    // Prepare the task status goal with the target AprilTag IDs
    ir2425_group_15::TaskStatusGoal goal;
    std::vector<int> server_apriltag_ids = srv.response.ids;
    goal.apriltags_detection_goal.srv_target_apriltag_ids = server_apriltag_ids;
    goal.apriltags_detection_goal.n_srv_target_apriltag_ids = server_apriltag_ids.size();
    // Prepare the task status goal with the waypoint path
    nav_msgs::Path waypoint_path = rn::get_waypoint_path(ros::package::getPath("ir2425_group_15"), "config/", "waypoints", "yaml");
    goal.robot_navigation_goal.waypoint_path = waypoint_path;
    goal.robot_navigation_goal.n_waypoints = waypoint_path.poses.size();

    // Send the task status goal and wait for result
    action_client.sendGoal(goal, &ts::task_status_done_callback, NULL, &ts::task_status_feedback_callback);
    
    // Spin the task status callback function
    ros::spin();

    return 0;
}