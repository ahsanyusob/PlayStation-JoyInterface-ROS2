// Copyright 2022 HarvestX Inc.
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


#include "p9n_node/teleop_ackermann_joy_node.hpp"

namespace p9n_node
{
TeleopAckermannJoyNode::TeleopAckermannJoyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("teleop_ackermann_joy_node", options)
{
  const std::string hw_name = this->declare_parameter<std::string>("hw_type", "DualShock3");
  const std::string out_topic = this->declare_parameter<std::string>("output_topic", "ackermann_drive");
  const double publish_hz = this->declare_parameter<double>("publish_hz", 10.0);
  
  // Initial limits
  this->max_speed_ = this->declare_parameter<double>("initial_max_speed", 1.0); // m/s
  this->max_steer_deg_ = this->declare_parameter<double>("initial_max_steer_deg", 30.0); // Degrees

  try {
    this->hw_type_ = p9n_interface::getHwType(hw_name);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please select hardware from %s",
      p9n_interface::getAllHwName().c_str());
    exit(EXIT_FAILURE);
    return;
  }

  this->p9n_if_ =
    std::make_unique<p9n_interface::PlayStationInterface>(this->hw_type_);

  using namespace std::placeholders;  // NOLINT
  this->joy_sub_ = this->create_subscription<Joy>(
    "joy", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&TeleopAckermannJoyNode::onJoy, this, _1));

  this->ack_pub_ = this->create_publisher<Ackermann>(
      out_topic, rclcpp::QoS(10).reliable().durability_volatile());

  using namespace std::chrono_literals; // NOLINT
  this->timer_watchdog_ = this->create_wall_timer(
    1s, std::bind(&TeleopAckermannJoyNode::onWatchdog, this));

  if (this->joy_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(this->get_logger(), "Joy node not launched");
  }
}

void TeleopAckermannJoyNode::onJoy(Joy::ConstSharedPtr joy_msg)
{
  this->timer_watchdog_->reset();
  this->p9n_if_->setJoyMsg(joy_msg);

  auto ack_msg = std::make_unique<Ackermann>();
  
  // 1. Get Raw R2 Axis
  // Assuming p9n_if_->pressedR2Analog() returns the raw axis value (1.0 to -1.0)
  double raw_r2 = this->p9n_if_->pressedR2Analog();

  // 2. Normalize to 0.0 ... 1.0
  // Formula: (1.0 - raw) / 2.0
  double throttle_normalized = (1.0 - raw_r2) / 2.0;

  // Small Deadzone (Ignore values less than 2%)
  if (throttle_normalized < 0.02) {
    throttle_normalized = 0.0;
  }

  // 3. Gear and Safety Buttons
  bool forward_gear   = this->p9n_if_->pressedCross();
  bool reverse_gear   = this->p9n_if_->pressedTriangle();
  bool emergency_stop = this->p9n_if_->pressedSquare();

  // 4. Determine Speed Logic
  if (emergency_stop) {
    ack_msg->speed = 0.0f;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "EMERGENCY STOP!");
  } 
  else if (forward_gear && throttle_normalized > 0.0) {
    ack_msg->speed = static_cast<float>(throttle_normalized * this->max_speed_);
  } 
  else if (reverse_gear && throttle_normalized > 0.0) {
    // Reverse: Negative speed
    ack_msg->speed = static_cast<float>(throttle_normalized * this->max_speed_ * -1.0);
  } 
  else {
    // No gear button held, or trigger not pressed
    ack_msg->speed = 0.0f;
  }

  // 5. Steering (Left Stick X)
  double steer_input = this->p9n_if_->tiltedStickLX();
  double steer_rad_limit = this->max_steer_deg_ * (M_PI / 180.0);
  ack_msg->steering_angle = static_cast<float>(steer_input * steer_rad_limit);

  // 6. Limit Adjustments
  // Increase/Decrease Max Speed (0.1 m/s steps)
  if (this->p9n_if_->pressedDPadUp()) {
    this->max_speed_ += 0.1;
    RCLCPP_INFO(this->get_logger(), "Limit Update: Max Speed increased to %.1f m/s", this->max_speed_);
  }
  if (this->p9n_if_->pressedDPadDown()) {
    this->max_speed_ = std::max(0.0, this->max_speed_ - 0.1);
    RCLCPP_INFO(this->get_logger(), "Limit Update: Max Speed decreased to %.1f m/s", this->max_speed_);
  }

  // Increase/Decrease Max Steering (1 degree steps)
  if (this->p9n_if_->pressedDPadRight()) {
    this->max_steer_deg_ += 1.0;
    RCLCPP_INFO(this->get_logger(), "Limit Update: Max Steer increased to %.1f deg", this->max_steer_deg_);
  }
  if (this->p9n_if_->pressedDPadLeft()) {
    this->max_steer_deg_ = std::max(0.0, this->max_steer_deg_ - 1.0);
    RCLCPP_INFO(this->get_logger(), "Limit Update: Max Steer decreased to %.1f deg", this->max_steer_deg_);
  }
  

  this->ack_pub_->publish(std::move(ack_msg));
}

void TeleopAckermannJoyNode::onWatchdog()
{
  RCLCPP_WARN(this->get_logger(), "Couldn't subscribe joy topic before timeout");

  // Publish zero velocity to stop vehicle
  auto ack_msg_ = std::make_unique<Ackermann>(rosidl_runtime_cpp::MessageInitialization::ZERO);
  this->ack_pub_->publish(std::move(ack_msg_));

  this->timer_watchdog_->cancel();
}
}  // namespace p9n_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(p9n_node::TeleopAckermannJoyNode)
