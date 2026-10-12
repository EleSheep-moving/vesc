# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

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

### update duty, current control mode
```
Speed+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed
float32 acceleration 0
float32 jerk 0

speed+acc(forward)+steer （nav only）
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed
float32 acceleration 
float32 jerk 1

current+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed 0
float32 acceleration (current)
float32 jerk 2

duty+steer
float32 steering_angle
float32 steering_angle_velocity 0
float32 speed 0
float32 acceleration (duty)
float32 jerk 3
```

## 100 Hz vehicle profile

The driver polls `sensors/core` (ERPM, motor/input current, voltage and fault data) and `sensors/imu` / `sensors/imu/raw` at 100 Hz by default. `telemetry_rate_hz` is a startup parameter in the range 1–1000 Hz; the maximum supported rate depends on the link and controller. `baud_rate` defaults to 921600 with no hardware flow control. USB CDC transport does not gain UART bandwidth from this baud setting.

Motor and servo commands are forwarded immediately on each received ROS command. For 100 Hz command transmission, configure the upstream selector/publisher to 100 Hz, as AIMSRacer's vehicle launch does. This repeats the latest selected command; it does not increase the upstream planner's production of new commands. Core-driven wheel odometry follows the telemetry rate.

The receive thread processes incoming bytes immediately without the former 5 ms sleep. Transmits keep packet storage alive and serialize complete frames, including partial writes.

With the ROS workspace built and sourced, run `python3 -m pytest -q vesc_driver/test/test_driver_link.py`. This test uses a PTY VESC and an isolated ROS domain to check 50/100 Hz telemetry, 100 Hz motor/servo frames, ERPM/current decoding, fragmented replies and CRC integrity without opening vehicle devices.
