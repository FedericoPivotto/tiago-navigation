#ifndef TASK_STATUS_H
#define TASK_STATUS_H

/* Standard libraries */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

/* User-defined libraries */
#include <ir2425_group_15/TaskStatusAction.h>

/* Robot navigation namespace*/
namespace ts {
    /* Alias for using move base client */
    typedef actionlib::SimpleActionClient<ir2425_group_15::TaskStatusAction> TaskStatusClient;

    // Feedback callback function for task status
    void task_status_feedback_callback(const ir2425_group_15::TaskStatusFeedbackConstPtr& task_feedback);
    // Done callback function for task status
    void task_status_done_callback(const actionlib::SimpleClientGoalState& task_status_state, const ir2425_group_15::TaskStatusResultConstPtr& task_status_result);
}

#endif // TASK_STATUS_H