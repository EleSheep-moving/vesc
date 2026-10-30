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

"""Exercise paired tracking against a PTY only; never open vehicle hardware."""
import os
import math
import json
from pathlib import Path
import signal
import subprocess
import time

import pytest

from test_driver_link import VescEmulator


@pytest.mark.parametrize('startup', [False, True])
@pytest.mark.parametrize('steering_compensation', [False, True])
def test_longitudinal_tracking(tmp_path, monkeypatch, startup, steering_compensation):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 30))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from vesc_msgs.msg import VescCommand
    emulator = VescEmulator()
    emulator.rpm = 0  # Demand startup assistance when enabled.
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('paired_driver_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    commands = node.create_publisher(VescCommand, '/vesc/command', 1)
    wheels = node.create_publisher(Odometry, '/rear_axle/wheel_odom', 1)
    diagnostics = []
    node.create_subscription(DiagnosticArray, '/vesc/longitudinal_tracking', diagnostics.append, 10)
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    args = [str(binary), '--ros-args', '-p', f'port:={emulator.port}']
    for name, value in {'longitudinal_mode':'speed_acceleration', 'current_min':-80.,
                        'current_max':80., 'speed_min':-40000., 'speed_max':40000.,
                        'current_startup_enabled':str(startup).lower(),
                        'longitudinal.steering_compensation_enabled':str(steering_compensation).lower()}.items():
        args += ['-p', f'{name}:={value}']
    process = None
    wire_timing = {'timestamp_kind': 'monotonic at PTY frame parse, not driver receipt or TX'}
    with (tmp_path/'driver.log').open('w') as log:
        try:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while time.monotonic()<deadline and (commands.get_subscription_count()==0 or wheels.get_subscription_count()==0):
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
                executor.spin_once(timeout_sec=.01)
            assert wheels.get_subscription_count()>0
            deadline=time.monotonic()+3
            while not diagnostics and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            assert diagnostics
            config_id={v.key:v.value for v in diagnostics[-1].status[0].values}['config_identity']
            ref = VescCommand()
            ref.header.frame_id='base_link'
            ref.control_mode=4
            ref.steering_angle=.35
            ref.target_speed_mps=3.
            ref.target_acceleration_mps2=.5
            ref.plan_id=7
            ref.longitudinal_config_id=config_id
            ref.reference_duration.nanosec=20000000

            def pump(seconds, speed=2., new_ref=True, wheel=True, send=True, stamp_wheel=True):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    now=node.get_clock().now()
                    if wheel:
                        odom=Odometry(); odom.child_frame_id='base_link'; odom.header.frame_id='odom'
                        odom.header.stamp=now.to_msg() if stamp_wheel else old_stamp
                        odom.twist.twist.linear.x=speed; wheels.publish(odom)
                    if send:
                        ref.header.stamp=now.to_msg()
                        if new_ref:
                            ref.reference_stamp=now.to_msg()
                            ref.reference_valid_until=(now+rclpy.duration.Duration(seconds=.1)).to_msg()
                        commands.publish(ref)
                    for _ in range(5): executor.spin_once(timeout_sec=.002)
                    assert process.poll() is None, (tmp_path/'driver.log').read_text()

            def current_packets(after):
                return [int.from_bytes(p[1:], 'big', signed=True)/1000.
                        for t,p in emulator.packets if t>after and p[0]==6]
            def stopped(after):
                assert any(p[0]==8 and int.from_bytes(p[1:], 'big', signed=True)==0
                           for t,p in emulator.packets if t>after)
                assert not current_packets(after)

            start=time.monotonic(); pump(.25)
            currents=current_packets(start); assert currents and max(currents)<=30.
            frames = [{v.key:v.value for v in array.status[0].values} for array in diagnostics]
            active = [v for v in frames if v['active']=='1']
            assert active
            for values in active:
                expected = 3.9724336681547934*float(values['wheel_speed'])**2*math.tan(ref.steering_angle)**2 if steering_compensation else 0.
                assert float(values['steering_compensation_current']) == pytest.approx(min(expected,20.))
                assert values['steering_compensation_enabled'] == str(int(steering_compensation))
                total = sum(float(values[key]) for key in ('feedforward_current','steering_compensation_current','proportional_current','derivative_current'))
                assert float(values['unclamped_current']) == pytest.approx(total)
                if not startup:
                    assert float(values['sent_current']) == pytest.approx(max(-15.,min(30.,total)))
            if startup: assert max(currents)==pytest.approx(30.)
            else:
                # Below saturation, verify an actual wire packet equals the
                # four diagnostic components (VESC current encoding: 1 mA).
                diagnostics.clear()
                start=time.monotonic(); pump(.15, speed=3.)
                wire=current_packets(start)
                frames=[{v.key:v.value for v in a.status[0].values} for a in diagnostics]
                frames=[v for v in frames if v['active']=='1' and abs(float(v['wheel_speed'])-3.)<1e-6]
                assert frames and wire and any(10.<i<29. for i in wire)
                matched=False
                for values in frames:
                    extra=3.9724336681547934*9.*math.tan(ref.steering_angle)**2 if steering_compensation else 0.
                    assert float(values['steering_compensation_current']) == pytest.approx(extra)
                    total=sum(float(values[k]) for k in ('feedforward_current','steering_compensation_current','proportional_current','derivative_current'))
                    assert float(values['sent_current']) == pytest.approx(total)
                    matched |= any(abs(i-total)<.0011 for i in wire)
                assert matched
                start=time.monotonic(); pump(.15, speed=4.)
                assert max(current_packets(start)) < max(currents)
            # Re-stamping forwarded headers cannot extend the original reference lease.
            pump(.18,new_ref=False); start=time.monotonic(); pump(.08,new_ref=False)
            stopped(start)
            pump(.12); pump(.15,wheel=False); start=time.monotonic(); pump(.06,wheel=False)
            stopped(start)
            # Replayed wheel stamps cannot renew the source age.
            old_stamp=node.get_clock().now().to_msg()
            pump(.15,stamp_wheel=False); start=time.monotonic(); pump(.06,stamp_wheel=False)
            stopped(start)
            pump(.12)
            ref=VescCommand(); ref.header.frame_id='base_link'
            ref.control_mode=0; ref.target_speed_mps=0.; ref.target_acceleration_mps2=0.
            first_stop_packet=len(emulator.packets)
            wire_timing['stop_requested_monotonic']=time.monotonic()
            pump(.05,new_ref=False)
            packets=list(emulator.packets)
            # A prior current frame may already be in the PTY receive queue.
            # MODE_SPEED=0 writes an atomic zero-RPM/servo pair; tracking
            # writes current/servo, while expiry stops write zero RPM alone.
            # Use wire order, independent of this fixture's servo clipping.
            stop_pairs=[i for i in range(first_stop_packet,len(packets)-1)
                        if packets[i][1]==bytes([8,0,0,0,0]) and
                        len(packets[i+1][1])==3 and packets[i+1][1][0]==12]
            assert stop_pairs, packets[first_stop_packet:]
            stop_index=stop_pairs[0]
            wire_timing['stop_ack_packet_index']=stop_index
            wire_timing['stop_ack_rx_monotonic']=packets[stop_index][0]
            assert not [p for _,p in packets[stop_index+1:] if p[0]==6]
            pump(.05,send=False)
            # Keep the same first acknowledgement across both windows.
            assert not [p for _,p in emulator.packets[stop_index+1:] if p[0]==6]
            assert diagnostics
            values={v.key:v.value for v in diagnostics[-1].status[0].values}
            assert 'config_identity' in values and 'actual_avg_iq' in values
            assert not emulator.errors
        finally:
            if process is not None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            executor.shutdown(); node.destroy_node(); rclpy.shutdown(context=context); emulator.close()
            wire_timing['packets']=[{'index':i, 'rx_monotonic':t, 'payload_hex':p.hex()}
                                    for i,(t,p) in enumerate(emulator.packets)]
            (tmp_path/'wire-packets.json').write_text(json.dumps(wire_timing,indent=2)+'\n')


@pytest.mark.parametrize('mode,minimum,maximum,rejected', [
    ('speed_acceleration', -15., 10., True),
    ('speed_acceleration', -10., 30., True),
    ('speed_acceleration', -15., 30., False),
    ('speed', -10., 10., False),
])
def test_global_current_limits_contain_tracking_range(tmp_path, mode, minimum, maximum, rejected):
    """A matched model identity cannot hide narrower final hardware command limits."""
    from ament_index_python.packages import get_package_prefix
    emulator = VescEmulator()
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    args = [str(binary), '--ros-args', '-p', f'port:={emulator.port}',
            '-p', f'longitudinal_mode:={mode}', '-p', f'current_min:={minimum}',
            '-p', f'current_max:={maximum}']
    environment = dict(os.environ, ROS_DOMAIN_ID=str(130+os.getpid()%10),
                       ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path/'logs'))
    process = None
    with (tmp_path/'driver.log').open('w') as log:
        try:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env=environment)
            if rejected:
                assert process.wait(timeout=4) != 0
                assert 'contain longitudinal current range' in (tmp_path/'driver.log').read_text()
            else:
                time.sleep(.3)
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
            assert not [p for _, p in emulator.packets if p[0] == 6]
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            emulator.close()
