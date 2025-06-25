// Copyright 2023 flochre
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

/*
 * Authors: flochre
 */

#ifndef TEST_CAMERA_SENSOR_BROADCASTER_HPP_
#define TEST_CAMERA_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "camera_sensor_broadcaster/camera_sensor_broadcaster.hpp"

class CameraSensorBroadcasterTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  // for the sake of the test
  // defining the parameter names same as in test/camera_sensor_broadcaster_params.yaml
  const std::string sensor_name_ = "camera_sensor";
  const std::string frame_id_ = "camera_sensor_frame";
  const std::string interface_name_ = "data";

  const uint32_t height_ = 480;
  const uint32_t width_ = 640;
  const std::string encoding_ = "bgr8";
  const uint8_t is_bigendian_ = 0;
  const uint32_t step = width_ * 3;
  
  std::vector<unsigned char> data = {};
  hardware_interface::StateInterface data_{sensor_name_, "data", &data};

  std::unique_ptr<camera_sensor_broadcaster::CameraSensorBroadcaster> camera_broadcaster_;

  controller_interface::return_type init_broadcaster(std::string broadcaster_name);
  controller_interface::CallbackReturn configure_broadcaster(
    std::vector<rclcpp::Parameter> & parameters);
  void subscribe_and_get_message(sensor_msgs::msg::Image & camera_msg);
};

#endif  // TEST_CAMERA_SENSOR_BROADCASTER_HPP_
