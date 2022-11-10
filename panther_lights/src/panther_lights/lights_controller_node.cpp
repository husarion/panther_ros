#include "LightsController.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LightsControllerNode");
    ros::NodeHandle nh;
    LightsController lc(nh);
    ros::spin();
}