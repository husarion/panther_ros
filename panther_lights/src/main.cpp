#include <panther_lights/driver_node.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lights_driver_node");

  auto ph = std::make_shared<ros::NodeHandle>("~");
  auto nh = std::make_shared<ros::NodeHandle>();
  auto it = std::make_shared<image_transport::ImageTransport>(*nh);

  try {
    panther_lights::DriverNode driver_node(ph, nh, it);
    ros::spin();
  }

  catch (const std::exception & e) {
    std::cerr << "[" << ros::this_node::getName() << "]"
              << " Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[" << ros::this_node::getName() << "]"
            << " Shutting down" << std::endl;
  ros::shutdown();
  return 0;
}