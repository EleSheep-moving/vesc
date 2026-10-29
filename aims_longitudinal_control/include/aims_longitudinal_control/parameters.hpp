// Copyright 2026 AIMSRacer. Licensed under the Apache License, Version 2.0.
#pragma once
#include "aims_longitudinal_control/control.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
namespace aims_longitudinal_control {
// A templated adapter keeps the numerical library independent of ROS headers.
template<class Node> Config declare_parameters(Node& node) {
  Config c;
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
#define AIMS_LONGITUDINAL_PARAMETER(field) c.field=node.template declare_parameter<double>("longitudinal." #field,c.field,descriptor)
  AIMS_LONGITUDINAL_PARAMETER(k_b);AIMS_LONGITUDINAL_PARAMETER(b_b);
  AIMS_LONGITUDINAL_PARAMETER(k_c);AIMS_LONGITUDINAL_PARAMETER(b_c);
  AIMS_LONGITUDINAL_PARAMETER(blend_low);AIMS_LONGITUDINAL_PARAMETER(blend_high);
  AIMS_LONGITUDINAL_PARAMETER(kv);AIMS_LONGITUDINAL_PARAMETER(ka);
  AIMS_LONGITUDINAL_PARAMETER(derivative_tau);AIMS_LONGITUDINAL_PARAMETER(source_gap);
  AIMS_LONGITUDINAL_PARAMETER(wheel_max_age);AIMS_LONGITUDINAL_PARAMETER(current_min);
  AIMS_LONGITUDINAL_PARAMETER(current_max);AIMS_LONGITUDINAL_PARAMETER(max_speed);
  AIMS_LONGITUDINAL_PARAMETER(accel_min);AIMS_LONGITUDINAL_PARAMETER(accel_max);
  AIMS_LONGITUDINAL_PARAMETER(steering_compensation_gain);
  AIMS_LONGITUDINAL_PARAMETER(steering_compensation_max_current);
  AIMS_LONGITUDINAL_PARAMETER(steering_compensation_max_angle);
#undef AIMS_LONGITUDINAL_PARAMETER
  c.steering_compensation_enabled=node.template declare_parameter<bool>(
    "longitudinal.steering_compensation_enabled",c.steering_compensation_enabled,descriptor);
  c.validate();return c;
}
} // namespace aims_longitudinal_control
