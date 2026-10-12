// Copyright 2026 AIMSRacer. Licensed under the Apache License, Version 2.0.
#ifndef VESC_MSGS__COMMAND_VALIDATION_HPP_
#define VESC_MSGS__COMMAND_VALIDATION_HPP_
#include <cmath>
#include "vesc_msgs/msg/vesc_command.hpp"
namespace vesc_msgs
{
// Shape validation is shared by actuator and observers. Acceleration is reserved.
inline bool valid_command(const msg::VescCommand & c)
{
  if (c.header.frame_id != "base_link" || c.control_mode > c.MODE_BRAKE_CURRENT ||
    !std::isfinite(c.steering_angle)) {return false;}
  const double targets[] = {c.target_speed_mps, c.target_current, c.target_duty,
    c.target_brake_current, c.target_acceleration_mps2};
  for (unsigned i = 0; i < 5; ++i) {
    if (!std::isfinite(targets[i]) || (i != c.control_mode && targets[i] != 0.0)) {
      return false;
    }
  }
  return std::abs(c.target_duty) <= 1.0 && c.target_brake_current >= 0.0;
}
}  // namespace vesc_msgs
#endif
