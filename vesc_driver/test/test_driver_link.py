"""Exercise the real ROS driver against a PTY VESC, without opening vehicle devices."""
import binascii
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import threading
import time

import pytest


def frame(payload):
    return bytes([2, len(payload)]) + payload + binascii.crc_hqx(payload, 0).to_bytes(2, 'big') + b'\x03'


class VescEmulator:
    def __init__(self):
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        self.rpm = 12345
        self.packets = []
        self.errors = []
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        while self.running:
            if not select.select([self.master], [], [], .01)[0]:
                continue
            buffer.extend(os.read(self.master, 4096))
            while len(buffer) >= 5:
                if buffer[0] != 2:
                    self.errors.append('invalid start')
                    del buffer[0]
                    continue
                n = buffer[1]
                if len(buffer) < n + 5:
                    break
                payload = bytes(buffer[2:2+n])
                crc = int.from_bytes(buffer[2+n:4+n], 'big')
                eof = buffer[4+n]
                del buffer[:n+5]
                if eof != 3 or crc != binascii.crc_hqx(payload, 0):
                    self.errors.append('invalid CRC/end')
                    continue
                self.packets.append((time.monotonic(), payload))
                if payload[0] == 0:
                    response = b'\x00\x06\x05PTY_VESC\x00' + bytes(24)
                elif payload[0] == 4:
                    response = bytearray(74)
                    response[0] = 4
                    response[23:27] = struct.pack('>i', self.rpm)
                    response[5:9] = struct.pack('>i', 1234)
                    response[27:29] = struct.pack('>h', 198)
                    response = bytes(response)
                elif payload[0] == 65:
                    response = bytes([65, 255, 255]) + struct.pack('>16f', *([1.0]*16))
                else:
                    continue
                # Split a valid response to exercise incremental frame assembly.
                packet = frame(response)
                os.write(self.master, packet[:3])
                os.write(self.master, packet[3:])

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


@pytest.mark.parametrize('rate,paused_clock,brake_max,offset,startup', [
    (100.0, False, 0.0, 0.0, False),
    (100.0, True, 80.0, 100.0, True),
    (50.0, False, 80.0, 100.0, False),
])
def test_driver_link(tmp_path, monkeypatch, rate, paused_clock, brake_max, offset, startup):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(140 + os.getpid() % 40))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float64
    from vesc_msgs.msg import VescCommand, VescStateStamped, VescImuStamped

    emulator = VescEmulator()
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('vesc_link_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    core, imu = [], []
    node.create_subscription(VescStateStamped, '/sensors/core',
                             lambda msg: core.append((time.monotonic(), msg)), 10)
    node.create_subscription(VescImuStamped, '/sensors/imu',
                             lambda msg: imu.append((time.monotonic(), msg)), 10)
    command_pub = node.create_publisher(VescCommand, '/vesc/command', 1)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    limits = {'speed_min': -40000.0, 'speed_max': 40000.0,
              'current_min': -80.0, 'current_max': 80.0,
              'duty_cycle_min': -1.0, 'duty_cycle_max': 1.0,
              'servo_min': .264, 'servo_max': .750,
              'speed_to_erpm_gain': 3465.0, 'speed_to_erpm_offset': offset,
              'current_startup_enabled': str(startup).lower(),
              'use_sim_time': str(paused_clock).lower()}
    if brake_max:
        limits['brake_max'] = brake_max
    limit_args = [arg for name, value in limits.items() for arg in ['-p', f'{name}:={value}']]
    clock_seconds = 100

    def msg(mode=0, value=0.0, steer=.1):
        c = VescCommand()
        c.header.frame_id = 'base_link'
        c.control_mode = mode
        c.steering_angle = steer
        if mode < 5:
            setattr(c, ['target_speed_mps', 'target_current', 'target_duty',
                        'target_brake_current', 'target_acceleration_mps2'][mode], value)
        return c

    def spin_for(seconds, command=None, restamp=True):
        deadline, next_publish = time.monotonic()+seconds, 0.0
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'driver.log').read_text()
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic()+.01
                if paused_clock:
                    clock = Clock()
                    clock.clock.sec = clock_seconds
                    clock_pub.publish(clock)
                if command is not None:
                    if restamp:
                        if paused_clock:
                            command.header.stamp.sec = clock_seconds
                            command.header.stamp.nanosec = 0
                        else:
                            command.header.stamp = node.get_clock().now().to_msg()
                    command_pub.publish(command)
            executor.spin_once(timeout_sec=.001)

    def packets_since(start, cmd=None):
        return [p for t, p in emulator.packets if t > start and (cmd is None or p[0] == cmd)]

    def decoded(p):
        return int.from_bytes(p[1:], 'big', signed=True)

    def watchdog(last):
        spin_for(.30)
        zeros = [(t, p) for t, p in emulator.packets if t > last and p[0] == 8 and decoded(p) == 0]
        assert zeros and zeros[0][0]-last <= .10 + 1/rate + .05
        assert all(decoded(p) == 0 for p in packets_since(zeros[0][0], 8))

    with (tmp_path/'driver.log').open('w') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '-p', f'port:={emulator.port}',
                                    '-p', f'telemetry_rate_hz:={rate}', *limit_args],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+8
            while time.monotonic() < deadline and (not core or not imu or
                    command_pub.get_subscription_count() == 0):
                spin_for(.02)
            assert core and imu and command_pub.get_subscription_count() > 0
            for old in ('motor/speed', 'motor/current', 'motor/duty_cycle', 'motor/brake',
                        'motor/position', 'servo/position'):
                probe = node.create_publisher(Float64, '/commands/'+old, 1)
                spin_for(.02)
                assert probe.get_subscription_count() == 0
                node.destroy_publisher(probe)
            assert not any(p[0] in (5, 6, 7, 8, 9, 12) for _, p in emulator.packets)
            start = time.monotonic()
            spin_for(2, msg(0, 2.0))
            end = time.monotonic()
            packets = [p for t, p in emulator.packets if start+.2 <= t <= end-.2]
            duration = end-start-.4
            for cmd, expected in [(4, rate), (65, rate), (8, 100), (12, 100)]:
                hz = sum(p[0] == cmd for p in packets)/duration
                assert expected*.88 <= hz <= expected*1.12, (cmd, hz)
            for samples in (core, imu):
                hz = sum(start+.2 <= t <= end-.2 for t, _ in samples)/duration
                assert rate*.88 <= hz <= rate*1.12, hz
            assert all(m.state.speed == emulator.rpm for _, m in core)
            assert all(m.state.current_motor == pytest.approx(12.34) for _, m in core)
            assert all(decoded(p) == 6930+offset for p in packets if p[0] == 8)
            assert all(decoded(p) == int((.506-.5137*.1)*1000) for p in packets if p[0] == 12)
            watchdog(max(t for t, p in emulator.packets if p[0] == 8))

            for mode, value, cmd, expected in [(0, 0., 8, 0), (0, -2., 8, -6930+offset),
                    (0, 100., 8, 40000), (1, 5., 6, 5000), (1, -5., 6, -5000),
                    (2, -.2, 5, -20000), (3, 0., 7, 0)]:
                start = time.monotonic()
                spin_for(.16, msg(mode, value, 100.0))
                active = packets_since(start+.04, cmd)
                assert active and all(decoded(p) == expected for p in active), (mode, active)
                assert all(decoded(p) == 264 for p in packets_since(start+.04, 12))
                watchdog(max(t for t, p in emulator.packets if t > start and p[0] == cmd))
            start = time.monotonic()
            spin_for(.1, msg(3, 2.))
            if brake_max:
                assert any(decoded(p) == 2000 for p in packets_since(start, 7))
            else:
                assert not packets_since(start, 7) and not packets_since(start, 12)
                assert all(decoded(p) == 0 for p in packets_since(start, 8))

            # Startup boost is preserved and can be explicitly disabled.
            emulator.rpm = 0
            spin_for(.12)
            start = time.monotonic()
            spin_for(.08, msg(1, 5.))
            assert all(decoded(p) == (40000 if startup else 5000) for p in packets_since(start, 6))
            emulator.rpm = 100
            start = time.monotonic()
            spin_for(.25, msg(1, 5.))
            assert all(decoded(p) == 5000 for p in packets_since(start+.15, 6))
            emulator.rpm = 0
            spin_for(.08, msg(1, 0.))
            assert decoded([p for _, p in emulator.packets if p[0] == 6][-1]) == 0
            watchdog(max(t for t, p in emulator.packets if p[0] == 6))

            # Invalid messages never send steering or refresh the watchdog.
            invalid = [msg(4), msg(4, 1.), msg(255), msg(0, float('nan')),
                       msg(0, 1., float('inf')), msg(3, -1.), msg(2, 1.1)]
            c = msg(0, 1.)
            c.target_current = 1.
            invalid.append(c)
            c = msg(1, 1.)
            c.target_acceleration_mps2 = float('nan')
            invalid.append(c)
            c = msg(0, 1.)
            c.header.frame_id = 'map'
            invalid.append(c)
            for c in invalid:
                start = time.monotonic()
                spin_for(.04, c)
                assert not packets_since(start, 12)
                assert not any(p[0] in (5, 6, 7, 9) for p in packets_since(start))
                assert all(decoded(p) == 0 for p in packets_since(start, 8))

            # Source-stamped expired commands are dropped, including steering.
            c = msg(0, 2.)
            c.header.stamp.sec = 1
            start = time.monotonic()
            spin_for(.15, c, restamp=False)
            assert not packets_since(start, 12)
            assert all(decoded(p) == 0 for p in packets_since(start, 8))
            if paused_clock:
                c.header.stamp.sec = 100
                clock_seconds = 90  # A ROS clock rewind cannot prolong last actuation.
                start = time.monotonic()
                spin_for(.15, c, restamp=False)
                assert not packets_since(start+.03, 12)
                assert all(decoded(p) == 0 for p in packets_since(start+.03, 8))
                clock_seconds = 100
                spin_for(.03)

            # DDS source age rejects messages stalled behind the executor, even with paused ROS time.
            process.send_signal(signal.SIGSTOP)
            spin_for(.05)
            spin_for(.03, msg(0, 3.))
            spin_for(.25)
            resumed = time.monotonic()
            process.send_signal(signal.SIGCONT)
            spin_for(.15)
            assert not packets_since(resumed, 12)
            assert all(decoded(p) == 0 for p in packets_since(resumed, 8))

            # Whole-command KeepLast(1) replaces both motor mode and steering.
            process.send_signal(signal.SIGSTOP)
            spin_for(.04)
            for target in range(20):
                spin_for(.003, msg(0, float(target)))
            spin_for(.20)
            spin_for(.02, msg(2, .25, -.1))
            resumed = time.monotonic()
            process.send_signal(signal.SIGCONT)
            spin_for(.08)
            assert packets_since(resumed, 5)
            assert all(decoded(p) == 25000 for p in packets_since(resumed, 5))
            assert all(decoded(p) == 0 for p in packets_since(resumed, 8))
            assert all(decoded(p) == int((.506+.5137*.1)*1000) for p in packets_since(resumed, 12))
            # Every active motor packet is immediately followed by its servo packet.
            all_packets = [p for _, p in emulator.packets]
            for i, p in enumerate(all_packets[:-1]):
                if p[0] in (5, 6, 7) or (p[0] == 8 and decoded(p) != 0):
                    assert all_packets[i+1][0] == 12
            assert not emulator.errors
        finally:
            process.send_signal(signal.SIGCONT)
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
            emulator.close()
