#ifndef APRILTAGS_DETECTION_H
#define APRILTAGS_DETECTION_H

/* Standard libraries */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>

/* User-defined libraries */
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ir2425_group_15/AprilTagsDetectionAction.h>

/* AprilTags detection namespace */
namespace ad {

    /* AprilTagsDetection storage class saving the AprilTags detection */
    class AprilTagsDetection {
    
        protected:            
            std::vector<int> target_apriltag_ids_;
            std::map<int, geometry_msgs::PoseStamped> target_apriltags_detected_;
            std::map<int, geometry_msgs::PoseStamped> nontarget_apriltags_detected_;

        public:
            AprilTagsDetection();
            AprilTagsDetection(const std::vector<int> target_apriltag_ids, const std::vector<int> target_apriltag_ids_detected, const geometry_msgs::PoseArray target_apriltag_poses_detected, const std::vector<int> nontarget_apriltag_ids_detected, const geometry_msgs::PoseArray nontarget_apriltag_poses_detected);

            // Function to set the target AprilTag IDs to detect
            void set_target_apriltag_ids(const std::vector<int> target_apriltag_ids);

            // Operator overload declaration
            friend std::ostream& operator<<(std::ostream& os, const ad::AprilTagsDetection& apriltags);
            
            // Function to get the number of apriltags found
            int count_target_apriltags() const;
            int count_target_apriltags_detected() const;
            int count_nontarget_apriltags_detected() const;

            // Function to get the IDs of the target AprilTags
            const std::vector<int> get_target_apriltag_ids() const;

            // Function that returns the dict for the result with target AprilTags found
            const std::map<int, geometry_msgs::PoseStamped> get_target_apriltags_detected() const;
            const std::vector<int> get_target_apriltag_ids_detected() const;
            const geometry_msgs::PoseArray get_target_apriltag_poses_detected() const;

            // Function that returns the non-target Apriltags, useful for the feedback
            const std::map<int, geometry_msgs::PoseStamped> get_nontarget_apriltags_detected() const;
            const std::vector<int> get_nontarget_apriltag_ids_detected() const;
            const geometry_msgs::PoseArray get_nontarget_apriltag_poses_detected() const;

            // Function to insert the given detected AprilTag
            void insert_apriltag(int apriltag_id, geometry_msgs::PoseStamped apriltag_pose_stamped);
            
            // Function to check if AprilTags detection is completed
            bool is_apriltags_detection_completed() const;
    };

    // Operator overload declaration outside the class
    std::ostream& operator<<(std::ostream& os, const AprilTagsDetection& apriltags);
}

#endif // APRILTAGS_DETECTION_H