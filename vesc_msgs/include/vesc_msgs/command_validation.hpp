// Copyright 2026 AIMSRacer
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

#ifndef VESC_MSGS__COMMAND_VALIDATION_HPP_
#define VESC_MSGS__COMMAND_VALIDATION_HPP_
#include <cmath>
#include "vesc_msgs/msg/vesc_command.hpp"
namespace vesc_msgs
{
// Shape validation shared by the selector, driver, and command-history readers.
inline bool valid_command(const msg::VescCommand & c)
{
  if (c.header.frame_id != "base_link" || c.control_mode > c.MODE_SPEED_ACCELERATION ||
    !std::isfinite(c.steering_angle)) {return false;}
  const double targets[] = {c.target_speed_mps, c.target_current, c.target_duty,
    c.target_brake_current, c.target_acceleration_mps2};
  for (double target : targets) {
    if (!std::isfinite(target)) {
      return false;
    }
  }
  if (c.control_mode == c.MODE_SPEED_ACCELERATION) {
    if (c.target_current != 0.0 || c.target_duty != 0.0 || c.target_brake_current != 0.0 ||
      c.target_speed_mps < 0.0 || c.plan_id == 0 || c.longitudinal_config_id.empty() ||
      (c.target_speed_mps == 0.0 && c.target_acceleration_mps2 == 0.0) ||
      c.reference_stamp.sec < 0 || c.reference_stamp.nanosec >= 1000000000u ||
      c.reference_valid_until.sec < 0 || c.reference_valid_until.nanosec >= 1000000000u ||
      c.reference_duration.sec != 0 || c.reference_duration.nanosec == 0 ||
      c.reference_duration.nanosec > 20000000u) {return false;}
    const int64_t begin = static_cast<int64_t>(c.reference_stamp.sec) * 1000000000LL +
      c.reference_stamp.nanosec;
    const int64_t end = static_cast<int64_t>(c.reference_valid_until.sec) * 1000000000LL +
      c.reference_valid_until.nanosec;
    const double terminal_speed = c.target_speed_mps +
      c.target_acceleration_mps2 * c.reference_duration.nanosec * 1e-9;
    return begin > 0 && end > begin && end - begin <= 100000000LL && terminal_speed >= -1e-6;
  }
  for (unsigned i = 0; i < 5; ++i) {
    if (i != c.control_mode && targets[i] != 0.0) {return false;}
  }
  if (c.plan_id != 0 || c.reference_stamp.sec != 0 || c.reference_stamp.nanosec != 0 ||
    c.reference_duration.sec != 0 || c.reference_duration.nanosec != 0 ||
    c.reference_valid_until.sec != 0 || c.reference_valid_until.nanosec != 0 ||
    !c.longitudinal_config_id.empty()) {return false;}
  return std::abs(c.target_duty) <= 1.0 && c.target_brake_current >= 0.0;
}
}  // namespace vesc_msgs
#endif  // VESC_MSGS__COMMAND_VALIDATION_HPP_
