// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_ =
    declare_parameter<double>("steering_angle_to_servo_gain");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");
  servo_min_ = declare_parameter<double>("servo_min");
  servo_max_ = declare_parameter<double>("servo_max");

  if (!std::isfinite(servo_min_) || !std::isfinite(servo_max_) ||
    servo_min_ < 0.0 || servo_max_ > 1.0 || servo_min_ >= servo_max_)
  {
    throw std::invalid_argument(
            "servo_min and servo_max must satisfy 0.0 <= servo_min < servo_max <= 1.0");
  }

  RCLCPP_INFO(
    get_logger(), "Servo command safety range: [%.3f, %.3f]", servo_min_, servo_max_);
  
  // Motor startup parameters
  startup_duty_current_ = declare_parameter<double>("startup_duty_current", 40.0);
  startup_timeout_ms_ = declare_parameter<double>("startup_timeout_ms", 100.0);
  ackermann_watchdog_enabled_ = declare_parameter<bool>("ackermann_watchdog_enabled", true);
  ackermann_timeout_sec_ = declare_parameter<double>("ackermann_timeout_sec", 0.10);

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);
  // create publishers for current and duty modes
  current_pub_ = create_publisher<Float64>("commands/motor/current", 10);
  duty_pub_ = create_publisher<Float64>("commands/motor/duty_cycle", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  
  // subscribe to vesc state for motor speed monitoring
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&AckermannToVesc::vescStateCallback, this, _1));
  
  // Initialize startup state machine
  last_nonzero_speed_time_ = now();
  last_ackermann_command_time_ = now();
  last_current_command_ = 0.0;
  last_steering_command_ = 0.0;
  last_motor_speed_ = 0.0;
  has_received_ackermann_command_ = false;
  motor_has_detected_motion_ = false;
  motor_is_running_ = false;
  ackermann_watchdog_active_ = false;

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&AckermannToVesc::watchdogTimerCallback, this));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  last_ackermann_command_time_ = now();
  has_received_ackermann_command_ = true;
  ackermann_watchdog_active_ = false;

  // Convert steering angle to servo position and clip before publishing. The VESC driver
  // independently applies the same limits as a second hardware-safety layer.
  Float64 servo_msg;
  const double raw_servo =
    steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;
  servo_msg.data = std::max(servo_min_, std::min(raw_servo, servo_max_));

  if (servo_msg.data != raw_servo) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Servo command %.3f exceeds mechanical range [%.3f, %.3f]; clipped to %.3f.",
      raw_servo, servo_min_, servo_max_, servo_msg.data);
  }

  // Determine control mode based on jerk field (state flag)
  // jerk = 0.0: speed mode (regular mode)
  // jerk = 1.0: acceleration mode (with speed and acceleration)
  // jerk = 2.0: current mode (with startup state machine)
  // jerk = 3.0: duty cycle mode
  
  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);
    
    if (cmd->drive.jerk == 0.0) {
      // Speed mode: use speed field
      Float64 erpm_msg;
      erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;
      erpm_pub_->publish(erpm_msg);
      
    } else if (cmd->drive.jerk == 1.0) {
      // Acceleration mode: use speed + acceleration (feedforward)
      // Reset state machine when switching away from current mode
      // didn't finish yet
      RCLCPP_INFO(get_logger(), "Didn't finish yet");
      
    } else if (cmd->drive.jerk == 2.0) {
      // Current mode with startup state machine
      last_current_command_ = cmd->drive.acceleration;
      last_steering_command_ = cmd->drive.steering_angle;
      
      // Check if current command is zero, reset state machine
      if (std::abs(last_current_command_) < 0.1) {
        Float64 current_msg;
        current_msg.data = 0.0;
        current_pub_->publish(current_msg);
        return;  // Don't send any command
      }
      
      // State machine for motor startup
      if (!motor_is_running_) {

        Float64 current_msg;
        current_msg.data = (last_current_command_ > 0.0) ? startup_duty_current_ : -startup_duty_current_;
        current_pub_->publish(current_msg);
        RCLCPP_INFO(get_logger(), "Motor startup: sending current cycle %.2f", current_msg.data);
      } 
      else if (motor_is_running_) {
        // Send current command
        Float64 current_msg;
        current_msg.data = last_current_command_;
        current_pub_->publish(current_msg);
      }
      
    } else if (cmd->drive.jerk == 3.0) {
      // Duty cycle mode: use acceleration field as duty cycle value
      Float64 duty_msg;
      duty_msg.data = cmd->drive.acceleration;
      duty_pub_->publish(duty_msg);
    }
  }
}

void AckermannToVesc::publishSafeSpeedCommand()
{
  Float64 erpm_msg;
  erpm_msg.data = 0.0;
  erpm_pub_->publish(erpm_msg);
}

void AckermannToVesc::watchdogTimerCallback()
{
  if (!ackermann_watchdog_enabled_ || !has_received_ackermann_command_ ||
    ackermann_timeout_sec_ <= 0.0)
  {
    return;
  }

  const double elapsed_sec = (now() - last_ackermann_command_time_).seconds();
  if (elapsed_sec <= ackermann_timeout_sec_) {
    return;
  }

  publishSafeSpeedCommand();

  if (!ackermann_watchdog_active_) {
    RCLCPP_WARN(
      get_logger(),
      "No ackermann_cmd for %.3fs (> %.3fs); publishing zero speed command.",
      elapsed_sec,
      ackermann_timeout_sec_);
    ackermann_watchdog_active_ = true;
  }
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // Update motor speed from VESC telemetry
  last_motor_speed_ = state->state.speed;
  
  // Detect motor motion during startup phase
  if (std::abs(last_motor_speed_) > 1.0 && !motor_has_detected_motion_) {
    motor_has_detected_motion_ = true;
    last_nonzero_speed_time_ = now();  // Record the time when motion is detected
  }
  else if (std::abs(last_motor_speed_) < 1.0) {
    motor_has_detected_motion_ = false;
    motor_is_running_ = false;
  }
  
  // Check if we should transition to RUNNING
  if (motor_has_detected_motion_) {
    double elapsed_ms = (now() - last_nonzero_speed_time_).seconds() * 1000.0;
    if (elapsed_ms > startup_timeout_ms_) {
      // RCLCPP_INFO(get_logger(), "Motor started");
      motor_is_running_ = true;
    }
  }
  else {
    // RCLCPP_INFO(get_logger(), "Motor stopped");
    motor_is_running_ = false;
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
