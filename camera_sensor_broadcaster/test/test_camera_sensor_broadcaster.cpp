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

#include <utility>

#include "test_camera_sensor_broadcaster.hpp"

#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

using testing::IsEmpty;
using testing::SizeIs;

void CameraSensorBroadcasterTest::SetUp()
{
  // initialize controller
  camera_broadcaster_ = std::make_unique<camera_sensor_broadcaster::CameraSensorBroadcaster>();
}

void CameraSensorBroadcasterTest::TearDown() { camera_broadcaster_.reset(nullptr); }

controller_interface::return_type CameraSensorBroadcasterTest::init_broadcaster(
  std::string broadcaster_name)
{
  controller_interface::return_type result = controller_interface::return_type::ERROR;
  result = camera_broadcaster_->init(
    broadcaster_name, "", 0, "", camera_broadcaster_->define_custom_node_options());

  if (controller_interface::return_type::OK == result)
  {
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(data_);

    camera_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
  }

  return result;
}

controller_interface::CallbackReturn CameraSensorBroadcasterTest::configure_broadcaster(
  std::vector<rclcpp::Parameter> & parameters)
{
  // Configure the broadcaster
  for (auto parameter : parameters)
  {
    camera_broadcaster_->get_node()->set_parameter(parameter);
  }

  return camera_broadcaster_->on_configure(rclcpp_lifecycle::State());
}

void CameraSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Image & camera_msg)
{
  // create a new subscriber
  sensor_msgs::msg::Image::SharedPtr received_msg;
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Image::SharedPtr msg) { received_msg = msg; };
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Image>(
    "/test_camera_sensor_broadcaster/raw", 10, subs_callback);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_subscription_node.get_node_base_interface());

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    camera_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    const auto timeout = std::chrono::milliseconds{5};
    const auto until = test_subscription_node.get_clock()->now() + timeout;
    while (!received_msg && test_subscription_node.get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    // check if message has been received
    if (received_msg.get())
    {
      break;
    }
  }
  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                            "controller/broadcaster update loop";
  ASSERT_TRUE(received_msg);

  // take message from subscription
  camera_msg = *received_msg;
}

TEST_F(CameraSensorBroadcasterTest, Initialize_CameraBroadcaster_Exception)
{
  ASSERT_THROW(init_broadcaster(""), std::exception);
}

TEST_F(CameraSensorBroadcasterTest, Initialize_CameraBroadcaster_Success)
{
  ASSERT_EQ(
    init_broadcaster("test_camera_sensor_broadcaster"), controller_interface::return_type::OK);
}

TEST_F(CameraSensorBroadcasterTest, Configure_CameraBroadcaster_Error_1)
{
  // First Test without frame_id ERROR Expected
  init_broadcaster("test_camera_sensor_broadcaster");

  std::vector<rclcpp::Parameter> parameters;
  // explicitly give an empty sensor name to generate an error
  parameters.emplace_back(rclcpp::Parameter("sensor_name", ""));
  ASSERT_EQ(configure_broadcaster(parameters), controller_interface::CallbackReturn::ERROR);
}

TEST_F(CameraSensorBroadcasterTest, Configure_CameraBroadcaster_Error_2)
{
  // Second Test without sensor_name ERROR Expected
  init_broadcaster("test_camera_sensor_broadcaster");

  std::vector<rclcpp::Parameter> parameters;
  // explicitly give an empty frame_id to generate an error
  parameters.emplace_back(rclcpp::Parameter("frame_id", ""));
  ASSERT_EQ(configure_broadcaster(parameters), controller_interface::CallbackReturn::ERROR);
}

TEST_F(CameraSensorBroadcasterTest, Configure_CameraBroadcaster_Success)
{
  // Third Test without sensor_name SUCCESS Expected
  init_broadcaster("test_camera_sensor_broadcaster");

  ASSERT_EQ(
    camera_broadcaster_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = camera_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  auto state_if_conf = camera_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
}

TEST_F(CameraSensorBroadcasterTest, ActivateDeactivate_CameraBroadcaster_Success)
{
  init_broadcaster("test_camera_sensor_broadcaster");

  camera_broadcaster_->on_configure(rclcpp_lifecycle::State());

  ASSERT_EQ(
    camera_broadcaster_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = camera_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = camera_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  ASSERT_EQ(
    camera_broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  cmd_if_conf = camera_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  state_if_conf = camera_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));  // did not change
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(CameraSensorBroadcasterTest, Update_CameraBroadcaster_Success)
{
  init_broadcaster("test_camera_sensor_broadcaster");

  camera_broadcaster_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    camera_broadcaster_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  auto result = camera_broadcaster_->update(
    camera_broadcaster_->get_node()->get_clock()->now(), rclcpp::Duration::from_seconds(0.01));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(CameraSensorBroadcasterTest, Publish_CameraBroadcaster_Success)
{
  init_broadcaster("test_camera_sensor_broadcaster");

  camera_broadcaster_->on_configure(rclcpp_lifecycle::State());
  camera_broadcaster_->on_activate(rclcpp_lifecycle::State());

  sensor_msgs::msg::Image camera_msg;
  subscribe_and_get_message(camera_msg);

  EXPECT_EQ(camera_msg.header.frame_id, frame_id_);
  EXPECT_EQ(camera_msg.height, height_);
  EXPECT_EQ(camera_msg.width, width_);
  EXPECT_EQ(camera_msg.encoding, encoding_);
  EXPECT_EQ(camera_msg.is_bigendian, is_bigendian_);
  EXPECT_EQ(camera_msg.step, step);
}

// TEST_F(RangeSensorBroadcasterTest, Publish_Bandaries_RangeBroadcaster_Success)
// {
//   init_broadcaster("test_range_sensor_broadcaster");

//   range_broadcaster_->on_configure(rclcpp_lifecycle::State());
//   range_broadcaster_->on_activate(rclcpp_lifecycle::State());

//   sensor_msgs::msg::Range range_msg;

//   sensor_range_ = 0.10f;
//   subscribe_and_get_message(range_msg);

//   EXPECT_EQ(range_msg.header.frame_id, frame_id_);
//   EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
//   EXPECT_EQ(range_msg.radiation_type, radiation_type_);
//   EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
//   EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
//   EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
// #if SENSOR_MSGS_VERSION_MAJOR >= 5
//   EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
// #endif

//   sensor_range_ = 4.0;
//   subscribe_and_get_message(range_msg);

//   EXPECT_EQ(range_msg.header.frame_id, frame_id_);
//   EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
//   EXPECT_EQ(range_msg.radiation_type, radiation_type_);
//   EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
//   EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
//   EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
// #if SENSOR_MSGS_VERSION_MAJOR >= 5
//   EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
// #endif
// }

// TEST_F(RangeSensorBroadcasterTest, Publish_OutOfBandaries_RangeBroadcaster_Success)
// {
//   init_broadcaster("test_range_sensor_broadcaster");

//   range_broadcaster_->on_configure(rclcpp_lifecycle::State());
//   range_broadcaster_->on_activate(rclcpp_lifecycle::State());

//   sensor_msgs::msg::Range range_msg;

//   sensor_range_ = 0.0;
//   subscribe_and_get_message(range_msg);

//   EXPECT_EQ(range_msg.header.frame_id, frame_id_);
//   // Even out of boundaries you will get the out_of_range range value
//   EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
//   EXPECT_EQ(range_msg.radiation_type, radiation_type_);
//   EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
//   EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
//   EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
// #if SENSOR_MSGS_VERSION_MAJOR >= 5
//   EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
// #endif

//   sensor_range_ = 6.0;
//   subscribe_and_get_message(range_msg);

//   EXPECT_EQ(range_msg.header.frame_id, frame_id_);
//   // Even out of boundaries you will get the out_of_range range value
//   EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
//   EXPECT_EQ(range_msg.radiation_type, radiation_type_);
//   EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
//   EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
//   EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
// #if SENSOR_MSGS_VERSION_MAJOR >= 5
//   EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
// #endif
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
