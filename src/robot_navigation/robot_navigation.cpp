/* Standard libraries */
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>

/* User-defined libraries */
#include <robot_navigation.h>

std::vector<geometry_msgs::PoseStamped> rn::parse_waypoints_yaml(std::string package, std::string path, std::string filename, std::string ext) {
    // Waypoint vector
    std::vector<geometry_msgs::PoseStamped> waypoints;

    // Load YAML file
    YAML::Node config_yaml = YAML::LoadFile(package + "/" + path + filename + "." + ext);

    // Waypoints array
    auto waypoints_yaml = config_yaml["waypoints"];

    // Waypoint element with header and point
    for(auto waypoint_yaml : waypoints_yaml) {
        // Header
        YAML::Node header = waypoint_yaml["header"];
        std_msgs::Header waypoint_header = mb::get_header(header["seq"].as<uint32_t>(), ros::Time::now(), header["frame_id"].as<std::string>());
        
        // Position
        YAML::Node point = waypoint_yaml["point"];
        geometry_msgs::Point position = mb::get_point(point["x"].as<float>(), point["y"].as<float>(), point["z"].as<float>());        
        // Orientation
        YAML::Node quaternion = waypoint_yaml["quaternion"];
        geometry_msgs::Quaternion orientation = mb::get_quaternion(quaternion["x"].as<float>(), quaternion["y"].as<float>(), quaternion["z"].as<float>(), quaternion["w"].as<float>());

        // Add pose to waypoint vector
        geometry_msgs::Pose waypoint_pose = mb::get_pose(position, orientation);
        waypoints.push_back(mb::get_pose_stamped(waypoint_header, waypoint_pose));
    }

    return waypoints;
}

/* Function to get the waypoint path */
nav_msgs::Path rn::get_waypoint_path(std::string package, std::string path, std::string filename, std::string ext) {
    // Waypoint path
    nav_msgs::Path waypoint_path;

    // Path frame: std_msgs/Header
    waypoint_path.header = mb::get_header(1, ros::Time::now(), "map");
    // Waypoint path: geometry_msgs/PoseStamped[]
    waypoint_path.poses = rn::parse_waypoints_yaml(package, path, filename, ext);
    
    return waypoint_path;
}