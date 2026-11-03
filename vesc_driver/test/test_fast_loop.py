# Copyright 2026 AIMSRacer.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise the fast current loop against a PTY only; never open vehicle hardware."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

from test_driver_link import VescEmulator


def test_fast_loop(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 30))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    from vesc_msgs.msg import VescCommand, VescStateStamped

    emulator = VescEmulator()
    emulator.rpm = 3465*3
    emulator.fast_iq = (1234, 3234)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('fast_loop_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    commands = node.create_publisher(VescCommand, '/vesc/command', 1)
    diagnostics, core = [], []
    node.create_subscription(DiagnosticArray, '/vesc/longitudinal_tracking', diagnostics.append, 10)
    node.create_subscription(VescStateStamped, '/sensors/core', core.append, 10)
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    args = [str(binary), '--ros-args', '-p', f'port:={emulator.port}']
    for name, value in {'longitudinal_mode': 'speed_acceleration', 'current_min': -80.,
                        'current_max': 80., 'speed_min': -40000., 'speed_max': 40000.,
                        'current_startup_enabled': 'false', 'fast_loop_enabled': 'true',
                        'telemetry_rate_hz': 1000., 'speed_to_erpm_gain': 3465.,
                        'imu_polling_enabled': 'false', 'fast_publish_divider': 2}.items():
        args += ['-p', f'{name}:={value}']
    process = None
    with (tmp_path/'driver.log').open('w') as log:
        try:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while time.monotonic() < deadline and not (commands.get_subscription_count() and diagnostics):
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
                executor.spin_once(timeout_sec=.01)
            assert diagnostics
            # No wheel odometry exists in this test: the serial reply is the only speed source.
            assert node.count_publishers('/rear_axle/wheel_odom') == 0
            assert node.count_subscribers('/rear_axle/wheel_odom') == 0
            config_id = {v.key: v.value for v in diagnostics[-1].status[0].values}['config_identity']
            ref = VescCommand()
            ref.header.frame_id = 'base_link'
            ref.control_mode = 4
            ref.steering_angle = .1
            ref.target_speed_mps = 3.2
            ref.target_acceleration_mps2 = .5
            ref.plan_id = 7
            ref.longitudinal_config_id = config_id
            ref.reference_duration.nanosec = 20000000

            def pump(seconds):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    now = node.get_clock().now()
                    ref.header.stamp = now.to_msg()
                    ref.reference_stamp = now.to_msg()
                    ref.reference_valid_until = (now+rclpy.duration.Duration(seconds=.1)).to_msg()
                    commands.publish(ref)
                    for _ in range(5):
                        executor.spin_once(timeout_sec=.002)
                    assert process.poll() is None, (tmp_path/'driver.log').read_text()

            pump(.3)
            start = time.monotonic()
            core.clear()
            pump(1.)
            span = time.monotonic()-start
            window = [p for t, p in emulator.packets if t > start]
            count = lambda kind: sum(p[0] == kind for p in window)/span
            assert count(50) > 630, count(50)      # small polls
            assert 70 < count(4) < 130, count(4)   # full state stays at 100 Hz
            assert count(6) > 700, count(6)        # one current per reply
            assert count(12) < 200, count(12)      # steering stays near 100 Hz
            assert count(65) == 0                  # VESC IMU polling is off
            assert 350 < len(core)/span < 650, len(core)/span
            # Iq alternates 12.34 and 32.34 A per read: consumers keep the mean, and full fields stay.
            # Every tenth read is a full packet at the mean, which shifts one sample by 10 A.
            assert sum(m.state.avg_iq for m in core)/len(core) == pytest.approx(22.34, abs=1.2)
            assert all(m.state.speed == emulator.rpm for m in core)
            assert all(m.state.current_motor == pytest.approx(12.34) for m in core)
            assert not emulator.errors
            frames = [{v.key: v.value for v in a.status[0].values} for a in diagnostics[-20:]]
            active = [v for v in frames if v['active'] == '1']
            assert active
            assert float(active[-1]['wheel_speed']) == pytest.approx(3.)
            assert float(active[-1]['wheel_source_age']) < .005
            emulator.mute_values = True
            lost = time.monotonic()
            pump(.25)
            after = [(t, p) for t, p in emulator.packets if t > lost]
            stops = [t for t, p in after if p[0] == 8 and int.from_bytes(p[1:], 'big', signed=True) == 0]
            assert stops, 'no stop after telemetry loss'
            assert stops[0]-lost < .13
            assert not [p for t, p in after if t > stops[0]+.001 and p[0] == 6]
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            emulator.close()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)


def test_fast_telemetry_in_speed_mode(tmp_path, monkeypatch):
    """Speed mode keeps its VESC speed loop but inherits the fast telemetry: small polls at 1 kHz, no NX current loop."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(210 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from vesc_msgs.msg import VescCommand, VescStateStamped
    emulator = VescEmulator()
    emulator.rpm = 3465*2
    emulator.fast_iq = (1234, 3234)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('speed_mode_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    commands = node.create_publisher(VescCommand, '/vesc/command', 1)
    core = []
    node.create_subscription(VescStateStamped, '/sensors/core', core.append, 10)
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    args = [str(binary), '--ros-args', '-p', f'port:={emulator.port}', '-p', 'fast_loop_enabled:=true', '-p', 'telemetry_rate_hz:=1000.0',
            '-p', 'fast_publish_divider:=2', '-p', 'imu_polling_enabled:=false', '-p', 'speed_min:=-40000.', '-p', 'speed_max:=40000.']
    process = None
    try:
        with (tmp_path/'driver.log').open('w') as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while time.monotonic() < deadline and not (commands.get_subscription_count() and core):
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
                executor.spin_once(timeout_sec=.01)
            ref = VescCommand(); ref.header.frame_id = 'base_link'; ref.control_mode = 0; ref.target_speed_mps = 2.
            start = time.monotonic(); core.clear()
            while time.monotonic()-start < 1.:
                ref.header.stamp = node.get_clock().now().to_msg(); commands.publish(ref)
                for _ in range(5):
                    executor.spin_once(timeout_sec=.002)
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
            span = time.monotonic()-start
            window = [p for t, p in emulator.packets if t > start]
            count = lambda kind: sum(p[0] == kind for p in window)/span
            assert count(50) > 630, count(50)              # small polls
            assert 70 < count(4) < 130, count(4)           # full state at 100 Hz
            assert count(65) == 0
            assert count(6) == 0                           # no NX current loop in speed mode
            assert count(8) > 50                           # speed commands still go to the VESC speed loop
            assert 350 < len(core)/span < 650, len(core)/span
            assert sum(m.state.avg_iq for m in core)/len(core) == pytest.approx(22.34, abs=1.2)
            assert node.count_subscribers('/rear_axle/wheel_odom') == 0
        assert 'Fast telemetry only' in (tmp_path/'driver.log').read_text()
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        emulator.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
