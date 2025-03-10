/* Standard libraries */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* User-defined libraries */
#include <apriltags_detection_action.h>
#include <message_builder.h>

// Task action constructor
ad::AprilTagsDetectionAction::AprilTagsDetectionAction(std::string name) : as_(nh_, name, boost::bind(&ad::AprilTagsDetectionAction::detect_apriltags_callback, this, _1), false), action_name_(name), apriltags_(){
    // Start the AprilTags detection action
    as_.start();

    // Subscribe to the topic "tag_detections"
    tag_subscriber_ = nh_.subscribe("tag_detections", 1, &ad::AprilTagsDetectionAction::apriltags_callback, this);
}

/* Callback function to read the topic with the AprilTags detected */
void ad::AprilTagsDetectionAction::apriltags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
    std::string source_frame = msg->header.frame_id, target_frame = "map";

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while(! tf_buffer.canTransform(target_frame, source_frame, ros::Time(0)))
        ros::Duration(0.5).sleep();

    // Transform available
    geometry_msgs::TransformStamped transformed = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::PoseStamped pose_in, pose_out;

    for(size_t i = 0; i < msg->detections.size(); ++i) {
        // Read detected AprilTag
        int apriltag_id = msg->detections.at(i).id[0];
        pose_in = mb::get_pose_stamped(
            msg->detections.at(i).pose.header,
            mb::get_pose(
                mb::get_point(
                    msg->detections.at(i).pose.pose.pose.position.x, 
                    msg->detections.at(i).pose.pose.pose.position.y, 
                    msg->detections.at(i).pose.pose.pose.position.z
                ),
                mb::get_quaternion(
                    msg->detections.at(i).pose.pose.pose.orientation.x, 
                    msg->detections.at(i).pose.pose.pose.orientation.y, 
                    msg->detections.at(i).pose.pose.pose.orientation.z, 
                    msg->detections.at(i).pose.pose.pose.orientation.w
                )
            )
        );
    
        // Transform from camera frame to map frame
        tf2::doTransform(pose_in, pose_out, transformed);

        // Update the AprilTags detected
        apriltags_.insert_apriltag(apriltag_id, pose_out);
    }
}

/* Callback function describing the action for the AprilTags detection */
void ad::AprilTagsDetectionAction::detect_apriltags_callback(const ir2425_group_15::AprilTagsDetectionGoalConstPtr& goal) {
    // Action initial state
    bool success = true;

    // Initialize AprilTags detection feedback
    feedback_.n_srv_target_apriltag_ids = goal->n_srv_target_apriltag_ids;
    feedback_.srv_target_apriltag_ids = goal->srv_target_apriltag_ids;

    // Initialize AprilTags detection result
    result_.n_srv_target_apriltag_ids = goal->n_srv_target_apriltag_ids;
    result_.srv_target_apriltag_ids = goal->srv_target_apriltag_ids;

    // Initialize AprilTags storage
    apriltags_.set_target_apriltag_ids(goal->srv_target_apriltag_ids);

    // Detected AprilTags reading
    while(! apriltags_.is_apriltags_detection_completed()) {
        // Check if preempt is requested by the client
        if(as_.isPreemptRequested()) {
            // Set action failure
            success = false;
            break;
        }

        // Publish the AprilTags detection feedback
        this->update_apriltags_detection_feedback();
        as_.publishFeedback(feedback_);
    }

    // Publish the AprilTags detection result
    this->update_apriltags_detection_result();
    success ? as_.setSucceeded(result_) : as_.setPreempted(result_);

    return;
}

/* Function to update the AprilTags detection feedback */
void ad::AprilTagsDetectionAction::update_apriltags_detection_feedback() {
    feedback_.n_target_apriltag_ids = apriltags_.count_target_apriltags_detected();
    feedback_.target_apriltag_ids = apriltags_.get_target_apriltag_ids_detected();
    feedback_.target_apriltag_poses = apriltags_.get_target_apriltag_poses_detected();

    feedback_.n_nontarget_apriltag_ids = apriltags_.count_nontarget_apriltags_detected();
    feedback_.nontarget_apriltag_ids = apriltags_.get_nontarget_apriltag_ids_detected();
    feedback_.nontarget_apriltag_poses = apriltags_.get_nontarget_apriltag_poses_detected();
}

/* Function to update the AprilTags detection result */
void ad::AprilTagsDetectionAction::update_apriltags_detection_result() {
    result_.is_completed = apriltags_.is_apriltags_detection_completed();

    result_.n_target_apriltag_ids = apriltags_.count_target_apriltags_detected();
    result_.target_apriltag_ids = apriltags_.get_target_apriltag_ids_detected();
    result_.target_apriltag_poses = apriltags_.get_target_apriltag_poses_detected();

    result_.n_nontarget_apriltag_ids = apriltags_.count_nontarget_apriltags_detected();
    result_.nontarget_apriltag_ids = apriltags_.get_nontarget_apriltag_ids_detected();
    result_.nontarget_apriltag_poses = apriltags_.get_nontarget_apriltag_poses_detected();
}