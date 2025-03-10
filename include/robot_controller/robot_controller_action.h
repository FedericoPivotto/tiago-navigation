#ifndef ROBOT_CONTROLLER_ACTION_H
#define ROBOT_CONTROLLER_ACTION_H

/* Standard libraries */
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>

/* User-defined libraries */
#include <robot_controller.h>
#include <ir2425_group_15/RobotControllerAction.h>

/* Robot controller namespace */
namespace rc {
    /* RobotController action class handling the robot controller */
    class RobotControllerAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::RobotControllerAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::RobotControllerFeedback feedback_;
            ir2425_group_15::RobotControllerResult result_;

            // Action clients
            rc::PlayMotionClient play_motion_client_;
            rc::MoveBaseClient move_base_client_;

            // Additional attributes for corridor
            ros::Subscriber laser_subscriber_;
            ros::Publisher velocity_publisher_;
            bool is_corridor_;
            double left_distance_, right_distance_;

        public:
            // RobotController action constructor
            RobotControllerAction(std::string name);

            // Callback function describing the action for the robot controller
            void control_robot_callback(const ir2425_group_15::RobotControllerGoalConstPtr& goal);

            // Callback function to read laser scan topic and check corridor
            void corridor_check_callback(const sensor_msgs::LaserScanConstPtr& msg);

            // Motion "initial"
            void initial_navigation();
            // Motion "corridor"
            void corridor_navigation(geometry_msgs::PoseStamped waypoint);
            // Motion "final"
            void final_navigation(geometry_msgs::PoseStamped waypoint);
    };
}

#endif // ROBOT_CONTROLLER_ACTION_H