/* Standard libraries */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* User-defined libraries */
#include <message_builder.h>

/* Function to arbitrarily set std_msgs/Header */
std_msgs::Header mb::get_header(uint32_t seq, ros::Time stamp, std::string frame_id) {
    // Header
    std_msgs::Header header;
    
    // std_msgs/Header
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_id;

    return header;
}

/* Function to arbitrarily set geometry_msgs/Point */
geometry_msgs::Point mb::get_point(float x, float y, float z) {
    // Point
    geometry_msgs::Point point;
    
    // geometry_msgs/Point
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

/* Function to arbitrarily set geometry_msgs/Quaternion */
geometry_msgs::Quaternion mb::get_quaternion(float x, float y, float z, float w) {
    // Orientation
    geometry_msgs::Quaternion orientation;
    
    // geometry_msgs/Quaternion
    orientation.x = x;
    orientation.y = y;
    orientation.z = z;
    orientation.w = w;

    return orientation;
}

/* Function to get geometry_msgs/Quaternion from RPY angles */
geometry_msgs::Quaternion mb::get_quaternion_from_rpy(float x, float y, float z) {
    // Orientation
    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(x, y, z);
    geometry_msgs::Quaternion orientation = tf2::toMsg(tf_orientation);
    
    return orientation;
}

/* Function to arbitrarily set geometry_msgs/Pose */
geometry_msgs::Pose mb::get_pose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation) {
    // Pose
    geometry_msgs::Pose pose;
    
    // geometry_msgs/Pose
    pose.position = position;
    pose.orientation = orientation;
    
    return pose;
}

/* Function to arbitrarily set geometry_msgs/PoseStamped */
geometry_msgs::PoseStamped mb::get_pose_stamped(std_msgs::Header header, geometry_msgs::Pose pose) {
    // Pose
    geometry_msgs::PoseStamped pose_stamped;
    
    // geometry_msgs/PoseStamped
    pose_stamped.header = header;
    pose_stamped.pose = pose;
    
    return pose_stamped;
}

/* Function to arbitrarily get geometry_msgs/PoseStamped of the origin of the given frame */
geometry_msgs::PoseStamped mb::get_frame_pose_stamped_wrt_frame(std::string source_frame_id, std::string target_frame_id) {
    std_msgs::Header header = mb::get_header(1, ros::Time::now(), target_frame_id);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform(target_frame_id, source_frame_id, ros::Time(0)))
        ros::Duration(0.5).sleep();

    geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));

    geometry_msgs::Point position = mb::get_point(transformed.transform.translation.x, transformed.transform.translation.y, transformed.transform.translation.z);
    geometry_msgs::Quaternion orientation = mb::get_quaternion(transformed.transform.rotation.x, transformed.transform.rotation.y, transformed.transform.rotation.z, transformed.transform.rotation.w);
    geometry_msgs::Pose pose = mb::get_pose(position, orientation);

    return mb::get_pose_stamped(header, pose);
}

/* Function to arbitrarily get geometry_msgs/PoseStamped of the pose stamped given with respect to the given frame */
geometry_msgs::PoseStamped mb::get_pose_stamped_wrt_frame(geometry_msgs::PoseStamped pose_stamped, std::string target_frame_id) {
    std::string source_frame_id = pose_stamped.header.frame_id;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(!tfBuffer.canTransform(target_frame_id, source_frame_id, ros::Time(0)))
        ros::Duration(0.5).sleep();
    geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(1.0));
    
    // Transform from source frame to target frame
    geometry_msgs::PoseStamped pose_stamped_out;
    tf2::doTransform(pose_stamped, pose_stamped_out, transformed);

    return pose_stamped_out;
}