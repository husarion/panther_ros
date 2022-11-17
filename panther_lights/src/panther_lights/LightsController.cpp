#include "LightsController.hpp"

LightsController::LightsController(ros::NodeHandle &nh) : battery_reading(rading_limit, 40.0)
{
    nh_ = nh;
    initializeSubscribers();
};

void LightsController::printDebug()
{
    std::cout << "debug" << std::endl;
};

LightsController::~LightsController()
{
    ROS_INFO("Shutting down lights controller");
};

void LightsController::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    battery_sub = nh_.subscribe("/battery", 1, &LightsController::batteryCallback, this);
    move_base_sub = nh_.subscribe("move_base/status", 1, &LightsController::moveBaseCallback, this);
}

void LightsController::batteryCallback(const sensor_msgs::BatteryState &msg)
{
    float battery = msg.voltage;
    battery_reading[index] = battery;
    index++;
    if (index >= rading_limit)
    {
        index = 0;
    }
    mean_voltage = std::accumulate(battery_reading.begin(), battery_reading.end(), 0.0) / rading_limit;
    if (mean_voltage < min_voltage && battery_level_status <= 0)
    {
        ROS_WARN("Battery level is low, please charge your Panther");
        battery_level_status = 1;
    }
    else if ((mean_voltage > (min_voltage + 2.0) && battery_level_status >= 1))
    {
        {
            battery_level_status = 0;
        }
    }

    getStatus();
}

void LightsController::moveBaseCallback(const actionlib_msgs::GoalStatusArray &msg)
{
    if (msg.status_list.size() == 1)
    {
        int status = msg.status_list[0].status;
        if (status != move_base_status)
        {
            move_base_status = status;
        }
    }
    else if (msg.status_list.size() > 1)
    {
        int status = msg.status_list[1].status;
        if (status != move_base_status)
        {
            move_base_status = status;
        }
    }
    else //if status message is empty
    {
        int status = 5;
        if (status != move_base_status)
        {
            move_base_status = status;
        }
    }
    getStatus();
}

void LightsController::getStatus()
{
    int status_now;
    if (battery_level_status == 1)
    {
        status_now = -1; // error low battery
    }
    else if(battery_level_status != 1 && move_base_status == -2)
    {
        status_now = 10; // battery above low limit
    }
    else
    {
        status_now = move_base_status;
    }
    if (status != status_now)
    {
        status = status_now;
        setLights();
    }
}

void LightsController::setLights()
{

    panther_lights::SetLights lights_message;

    switch (status)
    {
    case -1: // LOW BATTERY
        lights_message.request.animation = 9;
        lights_message.request.custom_color = "0xff3300 0xff3300";
        break;
    case 0: // PENDING
        lights_message.request.animation = 6;
        lights_message.request.custom_color = "";
        break;
    case 1: // ACTIVE
        lights_message.request.animation = 5;
        lights_message.request.custom_color = "";
        break;
    case 2: // PREEMPTED
        lights_message.request.animation = 4;
        lights_message.request.custom_color = "";
        break;
    case 3: // SUCCEEDED
        lights_message.request.animation = 9;
        lights_message.request.custom_color = "0x950fba 0x950fba";
        break;
    case 4: // ABORTED
        lights_message.request.animation = 9;
        lights_message.request.custom_color = "";
        break;
    case 10: // CHARGED AND READY
        lights_message.request.animation = 10;
        lights_message.request.custom_color = "";
        break;
    default:
        lights_message.request.animation = 8;
        lights_message.request.custom_color = "";
        return;
    }

    ros::service::waitForService("set_panther_lights");
    if (ros::service::call("set_panther_lights", lights_message))
    {
        ; // set success
    }
    else
    {
        ROS_INFO("Error setting panther lights, are they turned on?");
    }
}