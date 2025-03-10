/* User-defined libraries */
#include <apriltags_detection.h>
#include <message_builder.h>

/* AprilTagsDetection constructor */
ad::AprilTagsDetection::AprilTagsDetection() {
}

/* AprilTagsDetection constructor for reading purpose */
ad::AprilTagsDetection::AprilTagsDetection(const std::vector<int> target_apriltag_ids, const std::vector<int> target_apriltag_ids_detected, const geometry_msgs::PoseArray target_apriltag_poses_detected, const std::vector<int> nontarget_apriltag_ids_detected, const geometry_msgs::PoseArray nontarget_apriltag_poses_detected) : target_apriltag_ids_(target_apriltag_ids) {
    // Insert target AprilTags detected
    for(size_t i = 0; i < target_apriltag_ids_detected.size(); ++i) {
        std_msgs::Header header = mb::get_header(i, ros::Time::now(), "map");
        insert_apriltag(target_apriltag_ids_detected[i], mb::get_pose_stamped(header, target_apriltag_poses_detected.poses[i]));
    }

    // Insert non-target AprilTags detected
    for(size_t i = 0; i < nontarget_apriltag_ids_detected.size(); ++i) {
        std_msgs::Header header = mb::get_header(i, ros::Time::now(), "map");
        insert_apriltag(nontarget_apriltag_ids_detected[i], mb::get_pose_stamped(header, nontarget_apriltag_poses_detected.poses[i]));
    }
}

/* Function to set the target AprilTag IDs */
void ad::AprilTagsDetection::set_target_apriltag_ids(const std::vector<int> target_apriltag_ids) {
    target_apriltag_ids_ = target_apriltag_ids;
}

/* Function to get the number of target Apriltags to find */
int ad::AprilTagsDetection::count_target_apriltags() const {
    return target_apriltag_ids_.size();
}

/* Function to get the number of target Apriltags found */
int ad::AprilTagsDetection::count_target_apriltags_detected() const {
    return target_apriltags_detected_.size();
}

/* Function to get the number of non-target Apriltags found */
int ad::AprilTagsDetection::count_nontarget_apriltags_detected() const {
    return nontarget_apriltags_detected_.size();
}

/* Function that returns the dict for the result with target AprilTags found */
const std::map<int, geometry_msgs::PoseStamped> ad::AprilTagsDetection::get_target_apriltags_detected() const {
    return target_apriltags_detected_;
}

/* Function that returns the IDs of the dict for the result with target AprilTags found */
const std::vector<int> ad::AprilTagsDetection::get_target_apriltag_ids_detected() const {
    std::vector<int> apriltag_ids;
    for (const std::pair<int, geometry_msgs::PoseStamped>& target_apriltag : target_apriltags_detected_)
        apriltag_ids.push_back(target_apriltag.first);

    return apriltag_ids;
}

/* Function that returns the poses of the dict for the result with target AprilTags found */
const geometry_msgs::PoseArray ad::AprilTagsDetection::get_target_apriltag_poses_detected() const {
    geometry_msgs::PoseArray apriltag_poses;
    apriltag_poses.header = mb::get_header(1, ros::Time::now(), "map");
    for (const std::pair<int, geometry_msgs::PoseStamped>& target_apriltag : target_apriltags_detected_)
        apriltag_poses.poses.push_back(target_apriltag.second.pose);

    return apriltag_poses;
}

/* Function that returns the non-target Apriltags, useful for the feedback */
const std::map<int, geometry_msgs::PoseStamped> ad::AprilTagsDetection::get_nontarget_apriltags_detected() const {
    return nontarget_apriltags_detected_;
}

/* Function that returns the IDs of the dict for the result with non-target AprilTags found */
const std::vector<int> ad::AprilTagsDetection::get_nontarget_apriltag_ids_detected() const {
    std::vector<int> apriltag_ids;
    for (const std::pair<int, geometry_msgs::PoseStamped>& nontarget_apriltag : nontarget_apriltags_detected_)
        apriltag_ids.push_back(nontarget_apriltag.first);

    return apriltag_ids;
}

/* Function that returns the poses of the dict for the result with non-target AprilTags found */
const geometry_msgs::PoseArray ad::AprilTagsDetection::get_nontarget_apriltag_poses_detected() const {
    geometry_msgs::PoseArray apriltag_poses;
    apriltag_poses.header = mb::get_header(1, ros::Time::now(), "map");
    for (const std::pair<int, geometry_msgs::PoseStamped>& nontarget_apriltag : nontarget_apriltags_detected_)
        apriltag_poses.poses.push_back(nontarget_apriltag.second.pose);

    return apriltag_poses;
}

/* Function that returns the IDs of the target AprilTags to find */
const std::vector<int> ad::AprilTagsDetection::get_target_apriltag_ids() const {
    return target_apriltag_ids_;
}

/* Function to print the AprilTags detected */
std::ostream& ad::operator<<(std::ostream& os, const ad::AprilTagsDetection& apriltags) {
    if(apriltags.count_target_apriltags() > 0) 
        os << "Target AprilTag IDs: ";
    for (const int apriltag_id : apriltags.target_apriltag_ids_)
        os << apriltag_id << ", ";
    os << "\b\b " << std::endl;
    
    if(apriltags.count_target_apriltags_detected() > 0)
        os << "\nTarget AprilTags detected:" << std::endl;
    for (const auto& [apriltag_id, apriltag_pose] : apriltags.target_apriltags_detected_)
        os << " - ID " << apriltag_id << ": " << "x = " << apriltag_pose.pose.position.x  << ", y = " << apriltag_pose.pose.position.y << std::endl;
 
    if(apriltags.count_nontarget_apriltags_detected() > 0)
        os << "\nNon-target AprilTags detected:" << std::endl;
    for (const auto& [apriltag_id, apriltag_pose] : apriltags.nontarget_apriltags_detected_)
        os << " - ID " << apriltag_id << ": " << "x = " << apriltag_pose.pose.position.x  << ", y = " << apriltag_pose.pose.position.y << std::endl;
 
    return os;
}

/* Function to insert an AprilTag*/
void ad::AprilTagsDetection::insert_apriltag(int apriltag_id, geometry_msgs::PoseStamped apriltag_pose_stamped) {
    bool is_target;
    for (const int target_apriltag_id : target_apriltag_ids_) {
        if(is_target = target_apriltag_id == apriltag_id) break;
    }

    std::pair<int, geometry_msgs::PoseStamped> apriltag(apriltag_id, apriltag_pose_stamped);
    is_target ? target_apriltags_detected_.insert(apriltag) : nontarget_apriltags_detected_.insert(apriltag);
}

/* Function to check if AprilTags detection is completed */
bool ad::AprilTagsDetection::is_apriltags_detection_completed() const {
    return count_target_apriltags() == count_target_apriltags_detected();
}