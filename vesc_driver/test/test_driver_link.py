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
                    response[23:27] = struct.pack('>i', 12345)
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


@pytest.mark.parametrize('rate', [50.0, 100.0])
def test_driver_link(tmp_path, monkeypatch, rate):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(140 + os.getpid() % 40))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix
    from std_msgs.msg import Float64
    from vesc_msgs.msg import VescStateStamped, VescImuStamped

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
    speed = node.create_publisher(Float64, '/commands/motor/speed', 10)
    servo = node.create_publisher(Float64, '/commands/servo/position', 10)
    binary = Path(get_package_prefix('vesc_driver'))/'lib/vesc_driver/vesc_driver_node'
    with (tmp_path/'driver.log').open('w') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '-p', f'port:={emulator.port}',
                                    '-p', f'telemetry_rate_hz:={rate}'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and (not core or not imu or
                    speed.get_subscription_count() == 0 or servo.get_subscription_count() == 0):
                assert process.poll() is None, (tmp_path/'driver.log').read_text()
                executor.spin_once(timeout_sec=.01)
            assert core and imu and speed.get_subscription_count() > 0
            start = time.monotonic()
            next_command = start
            while time.monotonic() < start + 3:
                if time.monotonic() >= next_command:
                    speed.publish(Float64(data=4321.0))
                    servo.publish(Float64(data=.5))
                    next_command += .01
                executor.spin_once(timeout_sec=.001)
            end = time.monotonic()
            packets = [p for t, p in emulator.packets if start+.2 <= t <= end-.2]
            duration = end - start - .4
            for cmd, expected in [(4, rate), (65, rate), (8, 100), (12, 100)]:
                measured = sum(p[0] == cmd for p in packets)/duration
                assert expected*.90 <= measured <= expected*1.10, (cmd, measured)
            for samples in [core, imu]:
                hz = sum(start+.2 <= t <= end-.2 for t, _ in samples)/duration
                assert rate*.90 <= hz <= rate*1.10, hz
            assert all(msg.state.speed == 12345 for _, msg in core)
            assert all(abs(msg.state.current_motor-12.34) < 1e-5 for _, msg in core)
            assert not emulator.errors
        finally:
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
