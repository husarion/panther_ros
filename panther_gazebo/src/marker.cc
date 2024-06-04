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

#include <gz/common/Time.hh>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <iostream>

class MarkerManager
{
public:
  MarkerManager()
  {
    node.Subscribe("lights/driver/channel_1_frame", &MarkerManager::imageCallback, this);
  }

  void Run() { gz::transport::waitForShutdown(); }

private:
  void imageCallback(const gz::msgs::Image & _msg)
  {
    gz::msgs::Marker_V markerMsgs;

    int imageWidth = _msg.width();
    int imageHeight = _msg.height();

    const std::string & data = _msg.data();
    int numChannels = _msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 ? 3 : 4;

    for (int i = 0; i < imageWidth; ++i) {
      // Extract the pixel color
      int pixelIndex = i * numChannels;
      float r = static_cast<unsigned char>(data[pixelIndex]) / 255.0f;
      float g = static_cast<unsigned char>(data[pixelIndex + 1]) / 255.0f;
      float b = static_cast<unsigned char>(data[pixelIndex + 2]) / 255.0f;
      float a = numChannels == 4 ? static_cast<unsigned char>(data[pixelIndex + 3]) / 255.0f : 1.0f;

      auto markerMsg = markerMsgs.add_marker();
      markerMsg->set_ns("default");
      markerMsg->set_id(i);
      markerMsg->set_parent("panther");
      markerMsg->set_action(gz::msgs::Marker::ADD_MODIFY);
      markerMsg->set_type(gz::msgs::Marker::BOX);
      markerMsg->set_visibility(gz::msgs::Marker::GUI);

      // Set the color of the marker to match the pixel color
      SetColor(markerMsg, r, g, b, a);
      markerMsg->clear_point();

      // Set the position and size of the box
      float y = static_cast<float>(i) * 0.05;
      gz::msgs::Set(markerMsg->mutable_pose(), gz::math::Pose3d(1.0, y - 0.3, 0.05, 0, 0, 0));
      gz::msgs::Set(markerMsg->mutable_scale(), gz::math::Vector3d(0.03, 0.1, 0.1));
      markerMsg->mutable_lifetime()->set_nsec(100000);
    }

    gz::msgs::Boolean res;
    bool result;
    unsigned int timeout = 4900;

    bool executed = node.Request("/marker_array", markerMsgs, timeout, res, result);
    if (executed) {
      if (result)
        std::cout << "Response: [" << res.data() << "]" << std::endl;
      else
        std::cout << "Service call failed" << std::endl;
    } else
      std::cerr << "Service call timed out" << std::endl;
  }

  void SetColor(gz::msgs::Marker * marker, float r, float g, float b, float a)
  {
    marker->mutable_material()->mutable_ambient()->set_r(r);
    marker->mutable_material()->mutable_ambient()->set_g(g);
    marker->mutable_material()->mutable_ambient()->set_b(b);
    marker->mutable_material()->mutable_ambient()->set_a(a);
    marker->mutable_material()->mutable_diffuse()->set_r(r);
    marker->mutable_material()->mutable_diffuse()->set_g(g);
    marker->mutable_material()->mutable_diffuse()->set_b(b);
    marker->mutable_material()->mutable_diffuse()->set_a(a);
  }

  gz::transport::Node node;
};

int main(int argc, char ** argv)
{
  MarkerManager manager;
  manager.Run();
  return 0;
}
