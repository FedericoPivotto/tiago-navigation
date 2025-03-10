#ifndef ROBOT_NAVIGATION_ACTION_H
#define ROBOT_NAVIGATION_ACTION_H

/* Standard libraries */
#include <actionlib/server/simple_action_server.h>

/* User-defined libraries */
#include <robot_navigation.h>
#include <robot_controller_action.h>

/* Robot navigation namespace */
namespace rn {

    /* RobotNavigation action class handling the robot navigation */
    class RobotNavigationAction {
        protected:
            // Node handler and action name
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<ir2425_group_15::RobotNavigationAction> as_;
            std::string action_name_;

            // Goal, feedback and result messages
            ir2425_group_15::RobotNavigationFeedback feedback_;
            ir2425_group_15::RobotNavigationResult result_;

            // Additional attributes
            actionlib::SimpleActionClient<ir2425_group_15::RobotControllerAction> robot_controller_client_;

        public:
            // RobotNavigation action constructor
            RobotNavigationAction(std::string name);

            // Callback function describing the action for the robot navigation
            void navigate_robot_callback(const ir2425_group_15::RobotNavigationGoalConstPtr& goal);

            // Callback functions for the robot controller
            void robot_controller_done_callback(const actionlib::SimpleClientGoalState& robot_controller_state, const ir2425_group_15::RobotControllerResultConstPtr& robot_controller_result);
    };
    
}

#endif // ROBOT_NAVIGATION_ACTION_H