
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include "panther_lights/SetLights.h"

void statusCallback(const actionlib_msgs::GoalStatusArray &msg)
{
    static int last_status;

    if (msg.status_list.size())
    {
        panther_lights::SetLights message;

        int status = msg.status_list[0].status;
        if (status != last_status)
        {
            message.request.animation = 9;
            message.request.custom_color = "";

            switch (status)
            {
            case 0: // PENDING
                message.request.animation = 6;
                break;
            case 1: // ACTIVE
                message.request.animation = 5;
                break;
            case 2: // PREEMPTED
                message.request.animation = 4;
                break;
            case 3: // SUCCEEDED
                message.request.animation = 9;
                message.request.custom_color = "0x950fba 0x950fba";
                break;
            case 4: // ABORTED
                message.request.animation = 9;
                break;
            default:
                return;
            }

            ros::service::waitForService("set_panther_lights");
            if (ros::service::call("set_panther_lights", message))
            {
                ROS_INFO("New move_base state!");
                last_status = status;
            }
        }
    }
    else
    {
        int status = -1;
        if (status != last_status)
        {
            panther_lights::SetLights message;
            message.request.animation = 9;
            message.request.custom_color = "0x49d925 0x49d925";
            ros::service::waitForService("set_panther_lights");
            if (ros::service::call("set_panther_lights", message))
            {
                ROS_INFO("New move_base state!");
                last_status = status;
            }
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lights_controller_simple");
    ros::NodeHandle nh;
    ros::Subscriber move_base_sub =
        nh.subscribe("move_base/status", 1, statusCallback);
    ROS_INFO("lights_controller_simple: subscription was made");
    ros::spin();

    return 0;
}