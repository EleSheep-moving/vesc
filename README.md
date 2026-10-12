# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## Vehicle command API

The driver subscribes directly to `/vesc/command` (`vesc_msgs/msg/VescCommand`).
Speed is m/s and steering is radians; conversion to electrical RPM/servo units is
owned by the driver. Only the selected mode's target may be nonzero, all fields
must be finite, and header.frame_id must be `base_link`. Stamp commands using the
publisher's current ROS clock.

Modes: SPEED=0, CURRENT=1, DUTY=2, BRAKE_CURRENT=3, ACCELERATION=4. Acceleration
is reserved and always rejected, including zero requests. Nonzero braking is
disabled by default (`brake_max=0`). The old Float64 actuation subscriptions and
`ackermann_to_vesc_node` are removed. `vesc_ackermann` retains wheel odometry and
its configurable longitudinal velocity variance (default 0.04).

Reliable KeepLast(1), 100 ms lifespan, header/DDS source age checks and a steady
100 ms command watchdog prevent stale commands from extending actuation. Invalid
commands and watchdogs send literal zero ERPM and retain the last servo position.
Motor/servo pairs share a blocking serial write mutex; telemetry still polls core
and IMU at 100 Hz with 921600 baud. Zero speed bypasses conversion offsets.

Current startup assistance is retained by default: signed 40 A until fresh
absolute ERPM > 1 is confirmed for 100 ms; targets below 0.1 A send zero. Current
limits still apply. Set `current_startup_enabled=false` to use direct current.
The message carries requested current; the startup serial target can differ.

Software tests use virtual PTYs only (`driver_link`, `interface_writes`,
`vehicle_extensions`). They never open a physical VESC device. Hardware tests
require the vehicle owner's explicit on-site instruction.

## Hardware setup (only under owner direction)

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`

## Modification
### Modify the odometry computation, as the original method causes a speed delay when decelerating from high velocities to a stop.
#### VescToOdom Callback v1 vs v2 — Odometry Propagation Differences

This note summarizes the key differences between `vescStateCallback()` (v1) and `vescStateCallbackV2()` (v2) in how odometry (`x_`, `y_`, `yaw_`) is propagated from VESC state + (optional) servo command. 🧭

---

#### 1) Core difference: update order (integration scheme)

Both callbacks implement the 2D kinematic model:

- x_dot = v * cos(yaw)
- y_dot = v * sin(yaw)
- yaw_dot = omega

Where:
- v = `current_speed`
- yaw = `yaw_`
- omega = `current_angular_velocity`
- h = `dt.seconds()`

##### v1: explicit Euler-like (position uses old yaw)

Code:
- x_dot = v * cos(yaw_k)
- y_dot = v * sin(yaw_k)
- x_{k+1} = x_k + x_dot * h
- y_{k+1} = y_k + y_dot * h
- yaw_{k+1} = yaw_k + omega_k * h  (if use_servo_cmd_)

Math form:
- x_{k+1} = x_k + h * v_k * cos(yaw_k)
- y_{k+1} = y_k + h * v_k * sin(yaw_k)
- yaw_{k+1} = yaw_k + h * omega_k

##### v2: semi-implicit / symplectic-like (position uses updated yaw)

Code:
- delta_x = v * h
- yaw_{k+1} = yaw_k + omega_k * h  (if use_servo_cmd_)
- x_{k+1} = x_k + delta_x * cos(yaw_{k+1})
- y_{k+1} = y_k + delta_x * sin(yaw_{k+1})

Math form:
- yaw_{k+1} = yaw_k + h * omega_k
- x_{k+1} = x_k + h * v_k * cos(yaw_{k+1})
- y_{k+1} = y_k + h * v_k * sin(yaw_{k+1})

Practical meaning:
- In turns (omega != 0), v2 typically reduces systematic curvature error because it advances heading first, then moves along the new heading.

---

#### 2) Steering angle mapping differs (⚠️ major behavioral change)

##### v1
- steering_angle = (servo - offset) / gain

##### v2
- steering_angle = 2 * (servo - offset) / gain

This scales the steering angle directly, and thus scales yaw rate:
- omega = v * tan(steering_angle) / wheelbase

For small angles, tan(delta) ≈ delta, so `*2` ≈ ~2× angular velocity.
This difference can dominate odom changes more than the integration order.

---

#### 3) Yaw normalization (wrap) exists only in v2

v2 clamps yaw_ to [-pi, pi]:
- if (yaw_ >  pi) yaw_ -= 2*pi
- if (yaw_ < -pi) yaw_ += 2*pi

v1 does not explicitly wrap yaw.

Practical meaning:
- v2 avoids yaw growing unbounded; can reduce downstream numeric issues (sin/cos stability, logs, filters).

---

#### 4) Position propagation form

##### v1
- x_ += (v * cos(yaw_)) * h
- y_ += (v * sin(yaw_)) * h

##### v2
- delta_x = v * h
- x_ += delta_x * cos(yaw_)
- y_ += delta_x * sin(yaw_)

These are algebraically equivalent IF yaw used is identical.
In practice yaw differs because v2 updates yaw before x/y.

---

#### 5) Time handling / initialization

Both versions:
- set `last_state_ = state` on first callback (dt is ~0 on first call)
- compute dt from message timestamps: dt = stamp - last_stamp
- publish odom.header.stamp = state->header.stamp

No major functional divergence here.

---

#### 6) Debug + minor differences

- v2 prints debug info via RCLCPP_INFO when DEBUG_FLAG is enabled.
- v2 contains commented alternatives (e.g., using last_yaw_ for x/y update).

---

#### 7) Summary table

| Aspect | v1 (`vescStateCallback`) | v2 (`vescStateCallbackV2`) |
|---|---|---|
| Integration order | Position -> Yaw | Yaw -> Position |
| Scheme intuition | Explicit Euler-like | Semi-implicit / symplectic-like |
| Steering mapping | (servo-offset)/gain | 2*(servo-offset)/gain ⚠️ |
| Yaw wrap [-pi, pi] | No | Yes |
| Position update | x_dot*h | delta_x*cos(yaw) |
| Debug prints | No | Yes (optional) |

---

#### 8) Notes for clean comparison

If you want to compare ONLY the integration scheme:
1) Make steering mapping identical (remove `*2` or fix gain calibration).
2) Make yaw wrapping consistent (add to v1 or remove from v2).
3) Test with constant v, constant steering, constant dt and

### Native message

See [VescCommand.msg](vesc_msgs/msg/VescCommand.msg) for the complete schema.
Mode-specific targets replace the historical Ackermann `jerk`/`acceleration` convention.

## 100 Hz vehicle profile

The driver polls `sensors/core` (ERPM, motor/input current, voltage and fault data) and `sensors/imu` / `sensors/imu/raw` at 100 Hz by default. `telemetry_rate_hz` is a startup parameter in the range 1–1000 Hz; the maximum supported rate depends on the link and controller. `baud_rate` defaults to 921600 with no hardware flow control. USB CDC transport does not gain UART bandwidth from this baud setting.

Motor and servo commands are forwarded immediately on each received ROS command. For 100 Hz command transmission, configure the upstream selector/publisher to 100 Hz, as AIMSRacer's vehicle launch does. This repeats the latest selected command; it does not increase the upstream planner's production of new commands. Core-driven wheel odometry follows the telemetry rate.

The driver command watchdog and wheel odometry variance extensions are retained
as described above. The old converter watchdog is replaced by central source TTL
and the driver watchdog. The native subscription checks both ROS header age and
available DDS system publication time. The receive thread processes incoming bytes
without the former 5 ms sleep. Motor/servo pairs keep storage alive and serialize
all bytes, including partial writes. Already transmitted frames cannot be recalled;
a synchronous write can still block on a stalled serial device.

With the workspace built and sourced, run `python3 -m pytest -q
vesc_driver/test/test_driver_link.py vesc_ackermann/test/test_vehicle_extensions.py`.
Tests cover telemetry rates, native modes, offsets and zero stop, startup assistance,
invalid targets, stalled DDS delivery, clock pause/rewind and wheel covariance.
`interface_writes` uses four callers and 2000 motor/servo pairs (4000 frames) to
check CRC integrity and adjacency without a physical device.
