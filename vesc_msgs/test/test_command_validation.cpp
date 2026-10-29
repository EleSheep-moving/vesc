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

#include <cassert>
#include "vesc_msgs/command_validation.hpp"
int main()
{
  using C = vesc_msgs::msg::VescCommand;
  C c;
  c.header.frame_id = "base_link";
  c.control_mode = C::MODE_SPEED_ACCELERATION;
  c.target_speed_mps = 2.;
  c.target_acceleration_mps2 = .5;
  c.plan_id = 1;
  c.reference_stamp.sec = 10;
  c.reference_duration.nanosec = 20000000;
  c.reference_valid_until.sec = 10;
  c.reference_valid_until.nanosec = 100000000;
  c.longitudinal_config_id = "test-config";
  assert(vesc_msgs::valid_command(c));
  c.target_current = 1.;
  assert(!vesc_msgs::valid_command(c));
  c.target_current = 0.;
  c.reference_valid_until.nanosec = 100000001;
  assert(!vesc_msgs::valid_command(c));
  c.reference_valid_until.nanosec = 100000000;
  c.reference_duration.nanosec = 20000001;
  assert(!vesc_msgs::valid_command(c));
  c.reference_duration.nanosec = 20000000;
  c.plan_id = 0;
  assert(!vesc_msgs::valid_command(c));
  c.plan_id = 1;
  c.longitudinal_config_id = "";
  assert(!vesc_msgs::valid_command(c));
  C legacy;
  legacy.header.frame_id = "base_link";
  legacy.control_mode = C::MODE_SPEED;
  legacy.target_speed_mps = 2.;
  assert(vesc_msgs::valid_command(legacy));
  legacy.target_acceleration_mps2 = .1;
  assert(!vesc_msgs::valid_command(legacy));
  legacy.target_acceleration_mps2 = 0.;
  legacy.plan_id = 1;
  assert(!vesc_msgs::valid_command(legacy));
}
