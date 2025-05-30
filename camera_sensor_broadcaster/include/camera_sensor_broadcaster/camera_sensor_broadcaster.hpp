// Copyright 2021 PAL Robotics SL.
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
 * Authors: Gilmar Correia
 */

#ifndef CAMERA_SENSOR_BROADCASTER__CAMERA_SENSOR_BROADCASTER_HPP_
#define CAMERA_SENSOR_BROADCASTER__CAMERA_SENSOR_BROADCASTER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/camera_sensor.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_sensor_broadcaster/camera_sensor_broadcaster_parameters.hpp"

namespace camera_sensor_broadcaster
{
class CameraSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<semantic_components::CameraSensor> camera_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace camera_sensor_broadcaster

#endif  // CAMERA_SENSOR_BROADCASTER__CAMERA_SENSOR_BROADCASTER_HPP_
