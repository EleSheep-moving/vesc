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

#ifndef VESC_DRIVER__VESC_DRIVER_HPP_
#define VESC_DRIVER__VESC_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <experimental/optional>
#include <memory>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "aims_longitudinal_control/control.hpp"
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_command.hpp>
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <vesc_msgs/msg/vesc_imu.hpp>
#include <vesc_msgs/msg/vesc_imu_stamped.hpp>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{

using std_msgs::msg::Float64;
using vesc_msgs::msg::VescState;
using vesc_msgs::msg::VescStateStamped;
using vesc_msgs::msg::VescImuStamped;
using sensor_msgs::msg::Imu;

class VescDriver
  : public rclcpp::Node
{
public:
  explicit VescDriver(const rclcpp::NodeOptions & options);
  ~VescDriver() override;

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(
      rclcpp::Node * node_ptr,
      const std::string & str,
      const std::experimental::optional<double> & min_lower = std::experimental::optional<double>(),
      const std::experimental::optional<double> & max_upper =
      std::experimental::optional<double>());
    double clip(double value);
    rclcpp::Node * node_ptr;
    rclcpp::Logger logger;
    std::string name;
    std::experimental::optional<double> lower;
    std::experimental::optional<double> upper;
  };

  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit servo_limit_;
  bool command_watchdog_enabled_;
  double command_timeout_sec_;
  double command_max_age_sec_;
  double speed_gain_, speed_offset_, steering_gain_, steering_offset_;
  bool current_startup_enabled_;
  double startup_current_, startup_motion_sec_;
  std::mutex motion_mutex_;
  std::chrono::steady_clock::time_point motion_started_{}, last_telemetry_{};
  bool motion_detected_{false};

  std::string longitudinal_mode_, longitudinal_config_id_;
  aims_longitudinal_control::Config longitudinal_config_;
  std::unique_ptr<aims_longitudinal_control::WheelDerivative> wheel_derivative_;
  aims_longitudinal_control::Reference reference_{};
  vesc_msgs::msg::VescCommand reference_command_{};
  bool tracking_active_{false}, wheel_present_{false};
  double wheel_speed_{0.0}, wheel_stamp_{0.0}, reference_initial_age_{0.0}, last_tracking_ros_{0.0};
  int64_t last_reference_ns_{0};
  std::chrono::steady_clock::time_point wheel_received_{}, reference_received_{};
  std::atomic<double> actual_avg_iq_{std::numeric_limits<double>::quiet_NaN()};
  std::string tracking_reason_{"inactive"};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr tracking_pub_;
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  void wheelCallback(const nav_msgs::msg::Odometry & wheel);
  void trackingCallback();
  void stopTracking(const std::string & reason);
  double startupCurrent(double current);
  void publishTracking(const aims_longitudinal_control::ReferenceSample & sample,
    const aims_longitudinal_control::Components & components, double after_helper, double sent,
    const std::string & reason);

  // ROS services
  rclcpp::Publisher<VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<VescImuStamped>::SharedPtr imu_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_std_pub_;

  rclcpp::Publisher<Float64>::SharedPtr servo_sensor_pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING
  }
  driver_mode_t;

  // other variables
  std::atomic<driver_mode_t> driver_mode_;  ///< shared with the serial receive thread
  std::atomic<int> fw_version_major_;
  std::atomic<int> fw_version_minor_;
  std::chrono::steady_clock::time_point last_motor_command_time_;
  bool has_received_motor_command_;
  bool command_watchdog_active_;

  // ROS callbacks
  void commandCallback(
    const vesc_msgs::msg::VescCommand::SharedPtr command, const rclcpp::MessageInfo & info);
  void timerCallback();
  bool isCommandFresh(const vesc_msgs::msg::VescCommand & command,
    const rclcpp::MessageInfo & info) const;
  void markMotorCommandReceived();
  void checkCommandWatchdog();
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DRIVER_HPP_
