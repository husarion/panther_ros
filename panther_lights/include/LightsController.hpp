#pragma once
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <sensor_msgs/BatteryState.h>
#include "panther_lights/SetLights.h"
#include <vector>
#include <numeric>

class LightsController
{
public:
    LightsController(ros::NodeHandle &nh);
    ~LightsController();
    void printDebug();

protected:
    ros::NodeHandle nh_; //The node handle we'll be using
    ros::Subscriber battery_sub;
    ros::Subscriber move_base_sub;

    double mean_voltage;
    const int rading_limit = 50;
    const double min_voltage{35.0};
    std::vector<double> battery_reading;
    int index{0};
    uint8_t battery_level_status{0};
    int move_base_status{-2};
    int status;
    panther_lights::SetLights lights_message{};

    void initializeSubscribers();
    void setLights();
    void getStatus();
    void batteryCallback(const sensor_msgs::BatteryState &msg);
    void moveBaseCallback(const actionlib_msgs::GoalStatusArray &msg);
};
