# Vehicle command. Modes 0-3 admit one target; mode 4 carries a timed speed/acceleration pair.
# Publication stamp uses ROS time; frame_id must be base_link.
std_msgs/Header header
uint8 MODE_SPEED=0
uint8 MODE_CURRENT=1
uint8 MODE_DUTY=2
uint8 MODE_BRAKE_CURRENT=3
# Timed speed + acceleration reference, implemented by the ROS driver current loop.
uint8 MODE_ACCELERATION=4
uint8 MODE_SPEED_ACCELERATION=4
uint8 control_mode
float32 steering_angle # rad, left positive
float32 target_speed_mps # signed body longitudinal speed, forward positive
float32 target_current # signed motor current, A
float32 target_duty # [-1, 1]
float32 target_brake_current # nonnegative braking magnitude, A
float32 target_acceleration_mps2 # signed derivative of body longitudinal speed

# Mode 4 only. Immutable producer metadata; forwarding never renews this lease.
uint64 plan_id
builtin_interfaces/Time reference_stamp
builtin_interfaces/Duration reference_duration
builtin_interfaces/Time reference_valid_until
string longitudinal_config_id
