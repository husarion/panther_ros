// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "panther_gazebo/gz_light_converter_node.hpp"

#include "geometry_msgs/msg/vector3.hpp"

namespace panther_gazebo
{

GZLightConverter::GZLightConverter(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
    this->declare_parameter<std::string>("light_name", "light");
    this->get_parameter("light_name", light_name_);
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&GZLightConverter::timerCallback, this));
    
    light_pub_ = this->create_publisher<ros_gz_interfaces::msg::Light>(
        "_gz_lights", 1);

    light_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "lights/driver/channel_1_frame",
        10,
        std::bind(&GZLightConverter::imageCallback, this, std::placeholders::_1));

    last_button_ = 0;
}

void GZLightConverter::timerCallback()
{
    auto msgL = ros_gz_interfaces::msg::Light();
    msgL.name = light_name_;
    msgL.type = ros_gz_interfaces::msg::Light::SPOT;
    
    msgL.diffuse.r = randomFloat();
    msgL.diffuse.g = randomFloat();
    msgL.diffuse.b = randomFloat();
    msgL.diffuse.a = 1.0;
    
    msgL.cast_shadows = false;
    msgL.specular = msgL.diffuse;
    
    msgL.spot_inner_angle = 2.1;
    msgL.spot_outer_angle = 2.1;
    msgL.spot_falloff = 0.0;
    
    msgL.attenuation_constant = 1.0;
    msgL.attenuation_linear = 1.0;
    msgL.attenuation_quadratic = 0.0;
    
    msgL.intensity = 5.0;
    msgL.range = 100.0;
    
    light_pub_->publish(msgL);
}

void GZLightConverter::imageCallback(const sensor_msgs::msg::Image::SharedPtr /*msg*/)
{
    auto msgL = ros_gz_interfaces::msg::Light();
    msgL.name = light_name_;
    msgL.type = ros_gz_interfaces::msg::Light::SPOT;
    
    msgL.diffuse.r = randomFloat();
    msgL.diffuse.g = randomFloat();
    msgL.diffuse.b = randomFloat();
    msgL.diffuse.a = 1.0;
    
    msgL.cast_shadows = false;
    msgL.specular = msgL.diffuse;
    
    msgL.spot_inner_angle = 2.1;
    msgL.spot_outer_angle = 2.1;
    msgL.spot_falloff = 0.0;
    
    msgL.attenuation_constant = 1.0;
    msgL.attenuation_linear = 1.0;
    msgL.attenuation_quadratic = 0.0;
    
    msgL.intensity = 5.0;
    msgL.range = 100.0;
    geometry_msgs::msg::Vector3 direction;
    direction.x = 1;
    direction.y = 0;
    direction.z = 0;

    msgL.direction = direction;    
    light_pub_->publish(msgL);
}

float GZLightConverter::randomFloat()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen);
}

}  // namespace panther_gazebo
