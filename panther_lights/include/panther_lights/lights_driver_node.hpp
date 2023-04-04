#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include "panther_lights/apa102.hpp"

#include <memory>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace gpio
{
  class OutputPin
  {
  public:
    OutputPin(const int pin_number)
    {
      // Export pin
      std::ofstream export_file;
      export_file.open("/sys/class/gpio/export", std::ios::out);
      export_file << std::to_string(pin_number);
      export_file.close();

      // Set direction to output
      std::ofstream direction_file;
      direction_file.open("/sys/class/gpio/gpio" + std::to_string(pin_number) + "/direction", std::ios::out);
      direction_file << "out";
      direction_file.close();

      // Invert logic
      std::ofstream active_low_file;
      active_low_file.open("/sys/class/gpio/gpio" + std::to_string(pin_number) + "/active_low", std::ios::out);
      active_low_file << "1";
      active_low_file.close();

      pin_.open("/sys/class/gpio/gpio" + std::to_string(pin_number) + "/value", std::ios::out);
    }

    ~OutputPin()
    {
      pin_.close();
    }

    void set_val(const bool val)
    {
      pin_ << std::to_string(int(val));
    }
  private:
    std::ofstream pin_;
  };
}

namespace panther_lights_driver
{

  class LightsDriverNode
  {
  public:
    LightsDriverNode(
      std::shared_ptr<ros::NodeHandle> private_nh,
      std::shared_ptr<ros::NodeHandle> nh,
      std::shared_ptr<image_transport::ImageTransport> it);
    ~LightsDriverNode();

  private:
    const apa_102::APA102 front_panel_;
    const apa_102::APA102 rear_panel_;
    gpio::OutputPin power_pin_;
    std::string node_name_;
    double frame_timeout_;

    ros::Time front_panel_ts_;
    ros::Time rear_panel_ts_;

    int num_led_;
    void frame_cb(const sensor_msgs::ImageConstPtr& msg,
      const apa_102::APA102& panel, const ros::Time& last_time, const std::string panel_name) const;
    void set_brightness_cb();

    std::shared_ptr<ros::NodeHandle> ph_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber rear_light_sub_;
    image_transport::Subscriber front_light_sub_;
  };

} // panther_lights_driver_node

#endif // PANTHER_LIGHTS_DRIVER_NODE_HPP_