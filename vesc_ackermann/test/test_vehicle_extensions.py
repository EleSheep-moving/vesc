"""Check converter safety and wheel velocity covariance in an isolated ROS domain."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


@pytest.mark.parametrize('variance', [None, .09])
def test_vehicle_extensions(tmp_path, monkeypatch, variance):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(190 + os.getpid() % 30))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64
    from vesc_msgs.msg import VescStateStamped

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('vehicle_extension_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    ack = node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
    state = node.create_publisher(VescStateStamped, '/sensors/core', 10)
    servo_echo = node.create_publisher(Float64, '/sensors/servo_position_command', 10)
    speeds, servos, currents, odom = [], [], [], []
    node.create_subscription(Float64, '/commands/motor/speed',
                             lambda m: speeds.append((time.monotonic(), m.data)), 100)
    node.create_subscription(Float64, '/commands/servo/position',
                             lambda m: servos.append((time.monotonic(), m.data)), 100)
    node.create_subscription(Float64, '/commands/motor/current',
                             lambda m: currents.append((time.monotonic(), m.data)), 100)
    node.create_subscription(Odometry, '/odom',
                             lambda m: odom.append((time.monotonic(), m)), 100)
    prefix = Path(get_package_prefix('vesc_ackermann'))/'lib/vesc_ackermann'
    common = ['--ros-args', '-p', 'speed_to_erpm_gain:=3465.0', '-p',
              'speed_to_erpm_offset:=0.0', '-p', 'steering_angle_to_servo_gain:=-0.5137',
              '-p', 'steering_angle_to_servo_offset:=0.506', '-p', 'servo_min:=0.264',
              '-p', 'servo_max:=0.750']
    args = [[str(prefix/'ackermann_to_vesc_node'), *common],
            [str(prefix/'vesc_to_odom_node'), *common, '-p', 'wheelbase:=0.36',
             '-p', 'publish_tf:=false']]
    if variance is not None:
        args[1].extend(['-p', f'longitudinal_velocity_variance:={variance}'])
    logs = [(tmp_path/f'node{i}.log').open('w') for i in range(2)]
    processes = [subprocess.Popen(a, stdout=log, stderr=subprocess.STDOUT)
                 for a, log in zip(args, logs)]
    core = VescStateStamped()
    core.state.speed = 6930.0

    def spin_for(seconds, command=None):
        end = time.monotonic() + seconds
        next_publish = time.monotonic()
        while time.monotonic() < end:
            assert all(p.poll() is None for p in processes)
            if time.monotonic() >= next_publish:
                core.header.stamp = node.get_clock().now().to_msg()
                state.publish(core)
                servo_echo.publish(Float64(data=.506))
                if command is not None:
                    ack.publish(command)
                next_publish += .01
            executor.spin_once(timeout_sec=.001)

    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and (ack.get_subscription_count() == 0
                or state.get_subscription_count() < 2 or not odom):
            spin_for(.05)
        assert ack.get_subscription_count() > 0 and odom
        assert not speeds  # No watchdog stop before the first actual command.
        command = AckermannDriveStamped()
        command.drive.speed = 2.0
        command.drive.steering_angle = 100.0
        start = time.monotonic()
        spin_for(2, command)
        end = time.monotonic()
        for samples in (speeds, servos, odom):
            hz = sum(start+.2 <= t <= end-.2 for t, _ in samples)/(end-start-.4)
            assert 90 <= hz <= 110, hz
        assert all(v == pytest.approx(6930) for t, v in speeds if start+.2 <= t <= end-.2)
        assert all(v == pytest.approx(.264) for t, v in servos if start+.2 <= t <= end-.2)
        for _, message in odom:
            assert message.twist.covariance[0] == pytest.approx(.04 if variance is None else variance)
            assert message.twist.twist.linear.x == pytest.approx(2.0)
            assert message.header.frame_id == 'odom' and message.child_frame_id == 'base_link'
        last_active = speeds[-1][0]
        spin_for(.35)  # Core telemetry continues; Ackermann input stops.
        zeros = [t for t, v in speeds if t > last_active and v == 0]
        assert zeros and zeros[0]-last_active <= .10 + .01 + .05
        assert all(v == 0 for t, v in speeds if t >= zeros[0])
        # Fresh commands resume control and clip the opposite steering extreme.
        command.drive.speed = 1.0
        command.drive.steering_angle = -100.0
        begin = time.monotonic()
        spin_for(.25, command)
        assert any(t > begin and v == pytest.approx(3465) for t, v in speeds)
        assert any(t > begin and v == pytest.approx(.75) for t, v in servos)
        # Preserve the vehicle's configured startup-current behavior.
        core.state.speed = 0.0
        spin_for(.05, command)
        command.drive.jerk = 2.0
        command.drive.acceleration = 5.0
        begin = time.monotonic()
        spin_for(.05, command)
        assert any(t > begin and v == pytest.approx(40.0) for t, v in currents)
    finally:
        for p in processes:
            p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
