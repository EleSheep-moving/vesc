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

#include "vesc_driver/vesc_driver.hpp"
#include "vesc_msgs/command_validation.hpp"

#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace vesc_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using sensor_msgs::msg::Imu;

VescDriver::VescDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("vesc_driver", options),
  vesc_(
    std::string(),
    std::bind(&VescDriver::vescPacketCallback, this, _1),
    std::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0),
  current_limit_(this, "current"),
  brake_limit_(this, "brake", 0.0),
  speed_limit_(this, "speed"),
  servo_limit_(this, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING),
  fw_version_major_(-1),
  fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port = declare_parameter<std::string>("port", "");
  const auto baud_rate = declare_parameter<int>("baud_rate", 921600);
  const auto telemetry_rate = declare_parameter<double>("telemetry_rate_hz", 100.0);
  if (baud_rate <= 0 || baud_rate > std::numeric_limits<int>::max() ||
    !std::isfinite(telemetry_rate) || telemetry_rate < 1.0 || telemetry_rate > 1000.0)
  {
    throw std::invalid_argument("baud_rate must be positive; telemetry_rate_hz must be in [1, 1000]");
  }

  // attempt to connect to the serial port
  command_watchdog_enabled_ = declare_parameter<bool>("command_watchdog_enabled", true);
  command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.10);
  command_max_age_sec_ = declare_parameter<double>("command_max_age_sec", 0.10);
  if (!std::isfinite(command_max_age_sec_) || command_max_age_sec_ < 0.0) {
    throw std::invalid_argument("command_max_age_sec must be finite and nonnegative");
  }
  if (!std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("command_timeout_sec must be finite and positive");
  }
  // Retain the old parameter name but require a literal zero safe stop.
  if (declare_parameter<double>("watchdog_safe_erpm", 0.0) != 0.0) {
    throw std::invalid_argument("watchdog_safe_erpm must be zero");
  }
  speed_gain_ = declare_parameter<double>("speed_to_erpm_gain", 3465.0);
  speed_offset_ = declare_parameter<double>("speed_to_erpm_offset", 0.0);
  steering_gain_ = declare_parameter<double>("steering_angle_to_servo_gain", -0.5137);
  steering_offset_ = declare_parameter<double>("steering_angle_to_servo_offset", 0.506);
  current_startup_enabled_ = declare_parameter<bool>("current_startup_enabled", true);
  startup_current_ = declare_parameter<double>("startup_duty_current", 40.0);
  startup_motion_sec_ = declare_parameter<double>("startup_timeout_ms", 100.0) / 1000.0;
  if (!std::isfinite(speed_gain_) || speed_gain_ <= 0.0 ||
    !std::isfinite(speed_offset_) || !std::isfinite(steering_gain_) || steering_gain_ == 0.0 ||
    !std::isfinite(steering_offset_) || !std::isfinite(startup_current_) ||
    startup_current_ < 0.0 || !std::isfinite(startup_motion_sec_) || startup_motion_sec_ < 0.0)
  {
    throw std::invalid_argument("Invalid speed/steering conversion or current startup parameters");
  }
  last_motor_command_time_ = std::chrono::steady_clock::now();
  has_received_motor_command_ = false;
  command_watchdog_active_ = false;

  // create vesc state (telemetry) publisher
  state_pub_ = create_publisher<VescStateStamped>("sensors/core", rclcpp::QoS{10});
  imu_pub_ = create_publisher<VescImuStamped>("sensors/imu", rclcpp::QoS{10});
  imu_std_pub_ = create_publisher<Imu>("sensors/imu/raw", rclcpp::QoS{10});

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = create_publisher<Float64>(
    "sensors/servo_position_command", rclcpp::QoS{10});

  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  command_qos.lifespan(rclcpp::Duration::from_seconds(0.10));
  command_sub_ = create_subscription<vesc_msgs::msg::VescCommand>(
    "/vesc/command", command_qos, std::bind(&VescDriver::commandCallback, this, _1, _2));

  // Publishers and shared state exist before the receiver thread starts.
  vesc_.connect(port, static_cast<uint32_t>(baud_rate));
  // Poll core and IMU once per tick; commands are forwarded immediately on receipt.
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / telemetry_rate));
  timer_ = create_wall_timer(period, std::bind(&VescDriver::timerCallback, this));
  RCLCPP_INFO(
    get_logger(), "VESC port %s, baud %d, core/IMU polling %.1f Hz",
    port.c_str(), static_cast<int>(baud_rate), telemetry_rate);
}

VescDriver::~VescDriver()
{
  // Stop the receiver while its callbacks' members are still alive.
  vesc_.disconnect();
}

void VescDriver::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port.");
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      RCLCPP_INFO(
        get_logger(), "Connected to VESC with firmware version %d.%d",
        fw_version_major_.load(), fw_version_minor_.load());
      driver_mode_ = MODE_OPERATING;
    }
  } else if (driver_mode_ == MODE_OPERATING) {
    checkCommandWatchdog();
    // poll for vesc state (telemetry)
    vesc_.requestState();
    // poll for vesc imu
    vesc_.requestImuData();
  } else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const> & packet)
{
  if (packet->name() == "Values") {
    std::shared_ptr<VescPacketValues const> values =
      std::dynamic_pointer_cast<VescPacketValues const>(packet);

    auto state_msg = VescStateStamped();
    state_msg.header.stamp = now();

    state_msg.state.voltage_input = values->v_in();
    state_msg.state.current_motor = values->avg_motor_current();
    state_msg.state.current_input = values->avg_input_current();
    state_msg.state.avg_id = values->avg_id();
    state_msg.state.avg_iq = values->avg_iq();
    state_msg.state.duty_cycle = values->duty_cycle_now();
    state_msg.state.speed = values->rpm();

    state_msg.state.charge_drawn = values->amp_hours();
    state_msg.state.charge_regen = values->amp_hours_charged();
    state_msg.state.energy_drawn = values->watt_hours();
    state_msg.state.energy_regen = values->watt_hours_charged();
    state_msg.state.displacement = values->tachometer();
    state_msg.state.distance_traveled = values->tachometer_abs();
    state_msg.state.fault_code = values->fault_code();

    state_msg.state.pid_pos_now = values->pid_pos_now();
    state_msg.state.controller_id = values->controller_id();

    state_msg.state.ntc_temp_mos1 = values->temp_mos1();
    state_msg.state.ntc_temp_mos2 = values->temp_mos2();
    state_msg.state.ntc_temp_mos3 = values->temp_mos3();
    state_msg.state.avg_vd = values->avg_vd();
    state_msg.state.avg_vq = values->avg_vq();

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      const auto received = std::chrono::steady_clock::now();
      if (std::abs(values->rpm()) > 1.0) {
        if (!motion_detected_ ||
          std::chrono::duration<double>(received - last_telemetry_).count() > command_timeout_sec_)
        {
          motion_started_ = received;
        }
        motion_detected_ = true;
      } else {
        motion_detected_ = false;
      }
      last_telemetry_ = received;
    }
    state_pub_->publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
    RCLCPP_INFO(
      get_logger(),
      "-=%s=- hardware paired %d",
      fw_version->hwname().c_str(),
      fw_version->paired()
    );
  } else if (packet->name() == "ImuData") {
    std::shared_ptr<VescPacketImu const> imuData =
      std::dynamic_pointer_cast<VescPacketImu const>(packet);

    auto imu_msg = VescImuStamped();
    auto std_imu_msg = Imu();
    imu_msg.header.stamp = now();
    std_imu_msg.header.stamp = now();

    imu_msg.imu.ypr.x = imuData->roll();
    imu_msg.imu.ypr.y = imuData->pitch();
    imu_msg.imu.ypr.z = imuData->yaw();

    imu_msg.imu.linear_acceleration.x = imuData->acc_x();
    imu_msg.imu.linear_acceleration.y = imuData->acc_y();
    imu_msg.imu.linear_acceleration.z = imuData->acc_z();

    imu_msg.imu.angular_velocity.x = imuData->gyr_x();
    imu_msg.imu.angular_velocity.y = imuData->gyr_y();
    imu_msg.imu.angular_velocity.z = imuData->gyr_z();

    imu_msg.imu.compass.x = imuData->mag_x();
    imu_msg.imu.compass.y = imuData->mag_y();
    imu_msg.imu.compass.z = imuData->mag_z();

    imu_msg.imu.orientation.w = imuData->q_w();
    imu_msg.imu.orientation.x = imuData->q_x();
    imu_msg.imu.orientation.y = imuData->q_y();
    imu_msg.imu.orientation.z = imuData->q_z();

    std_imu_msg.linear_acceleration.x = imuData->acc_x();
    std_imu_msg.linear_acceleration.y = imuData->acc_y();
    std_imu_msg.linear_acceleration.z = imuData->acc_z();

    std_imu_msg.angular_velocity.x = imuData->gyr_x();
    std_imu_msg.angular_velocity.y = imuData->gyr_y();
    std_imu_msg.angular_velocity.z = imuData->gyr_z();

    std_imu_msg.orientation.w = imuData->q_w();
    std_imu_msg.orientation.x = imuData->q_x();
    std_imu_msg.orientation.y = imuData->q_y();
    std_imu_msg.orientation.z = imuData->q_z();


    imu_pub_->publish(imu_msg);
    imu_std_pub_->publish(std_imu_msg);
  }
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "%s packet received",
    packet->name().c_str()
  );
}

void VescDriver::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
}

bool VescDriver::isCommandFresh(
  const vesc_msgs::msg::VescCommand & c, const rclcpp::MessageInfo & info) const
{
  if (c.header.stamp.sec < 0 || c.header.stamp.nanosec >= 1000000000u) {return false;}
  const rclcpp::Time stamp(c.header.stamp, get_clock()->get_clock_type());
  const auto age = (now() - stamp).seconds();
  if (age < 0.0 || (command_max_age_sec_ > 0.0 && age > command_max_age_sec_)) {return false;}
  const auto source_stamp = info.get_rmw_message_info().source_timestamp;
  if (command_max_age_sec_ <= 0.0 || source_stamp <= 0) {return true;}
  // DDS publication time is system time, independent of ROS simulated time.
  const auto current_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const double dds_age = (current_stamp - source_stamp) * 1e-9;
  return dds_age >= 0.0 && dds_age <= command_max_age_sec_;
}

void VescDriver::markMotorCommandReceived()
{
  last_motor_command_time_ = std::chrono::steady_clock::now();
  has_received_motor_command_ = true;
  command_watchdog_active_ = false;
}

void VescDriver::checkCommandWatchdog()
{
  if (!command_watchdog_enabled_ || !has_received_motor_command_) {return;}
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_motor_command_time_).count();
  if (elapsed <= command_timeout_sec_) {return;}
  // Bypass calibrated offsets and configured minimums for every safe stop.
  vesc_.setSpeed(0.0);
  if (!command_watchdog_active_) {
    RCLCPP_WARN(get_logger(), "Command timeout %.3fs; sending zero ERPM, holding steering", elapsed);
    command_watchdog_active_ = true;
  }
}

void VescDriver::commandCallback(
  const vesc_msgs::msg::VescCommand::SharedPtr c, const rclcpp::MessageInfo & info)
{
  if (driver_mode_ != MODE_OPERATING || !isCommandFresh(*c, info)) {return;}
  if (!vesc_msgs::valid_command(*c) ||
    (c->control_mode == c->MODE_BRAKE_CURRENT && c->target_brake_current > *brake_limit_.upper))
  {
    vesc_.setSpeed(0.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Rejected invalid/disabled command; sending zero ERPM, holding steering");
    return;
  }
  const double servo = servo_limit_.clip(steering_gain_ * c->steering_angle + steering_offset_);
  std::unique_ptr<VescPacket> motor;
  switch (c->control_mode) {
    case vesc_msgs::msg::VescCommand::MODE_SPEED:
      motor.reset(new VescPacketSetRPM(c->target_speed_mps == 0.0 ? 0.0 :
        speed_limit_.clip(speed_gain_ * c->target_speed_mps + speed_offset_)));
      break;
    case vesc_msgs::msg::VescCommand::MODE_CURRENT: {
      double current = c->target_current;
      if (current_startup_enabled_ && std::abs(current) < 0.1) {
        current = 0.0;
      } else if (current_startup_enabled_) {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        const auto t = std::chrono::steady_clock::now();
        const bool running = motion_detected_ &&
          std::chrono::duration<double>(t - motion_started_).count() >= startup_motion_sec_ &&
          std::chrono::duration<double>(t - last_telemetry_).count() <= command_timeout_sec_;
        if (!running) {current = std::copysign(startup_current_, current);}
      }
      motor.reset(new VescPacketSetCurrent(current_limit_.clip(current)));
      break;
    }
    case vesc_msgs::msg::VescCommand::MODE_DUTY:
      motor.reset(new VescPacketSetDuty(duty_cycle_limit_.clip(c->target_duty)));
      break;
    case vesc_msgs::msg::VescCommand::MODE_BRAKE_CURRENT:
      motor.reset(new VescPacketSetCurrentBrake(brake_limit_.clip(c->target_brake_current)));
      break;
    default: return;  // Reserved acceleration mode fails shape validation above.
  }
  // Both packets are constructed only after the entire message has been validated.
  vesc_.sendMotorAndServo(*motor, VescPacketSetServoPos(servo));
  markMotorCommandReceived();
  Float64 echo;
  echo.data = servo;
  servo_sensor_pub_->publish(echo);
}

VescDriver::CommandLimit::CommandLimit(
  rclcpp::Node * node_ptr,
  const std::string & str,
  const std::experimental::optional<double> & min_lower,
  const std::experimental::optional<double> & max_upper)
: node_ptr(node_ptr),
  logger(node_ptr->get_logger()),
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  auto param_min =
    node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));

  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_min.get<double>() < *min_lower) {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_min.get<double>() > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      lower = param_min.get<double>();
    }
  } else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  auto param_max =
    node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));

  if (param_max.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_max.get<double>() < *min_lower) {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_max.get<double>() > *max_upper) {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      upper = param_max.get<double>();
    }
  } else if (max_upper) {
    upper = *max_upper;
  }

  if ((lower && !std::isfinite(*lower)) || (upper && !std::isfinite(*upper))) {
    throw std::invalid_argument("Command limits must be finite");
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    RCLCPP_WARN_STREAM(
      logger, "Parameter " << name << "_max (" << *upper <<
        ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";

  if (lower) {
    oss << *lower << " ";
  } else {
    oss << "(none) ";
  }

  if (upper) {
    oss << *upper;
  } else {
    oss << "(none)";
  }

  RCLCPP_DEBUG_STREAM(logger, oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);

  if (lower && value < lower) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) below minimum limit (%f), clipping.",
      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) above maximum limit (%f), clipping.",
      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)
