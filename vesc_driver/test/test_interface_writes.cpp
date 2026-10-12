#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <poll.h>
#include <pty.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "vesc_driver/vesc_interface.hpp"

namespace
{
uint16_t crc16(const std::vector<uint8_t> & bytes)
{
  uint16_t crc = 0;
  for (const auto byte : bytes) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

TEST(InterfaceWrites, ConcurrentTemporaryPacketsRemainComplete)
{
  int master, slave;
  char port[128];
  ASSERT_EQ(openpty(&master, &slave, port, nullptr, nullptr), 0);
  std::atomic<bool> running{true};
  std::atomic<int> invalid{0}, speed_count{0}, servo_count{0}, send_errors{0};
  std::thread controller([&]() {
    std::vector<uint8_t> bytes;
    bool awaiting_servo = false;
    while (running) {
      pollfd event{master, POLLIN, 0};
      if (poll(&event, 1, 10) <= 0) {
        continue;
      }
      uint8_t received[37];
      const auto count = read(master, received, sizeof(received));
      if (count <= 0) {
        continue;
      }
      bytes.insert(bytes.end(), received, received + count);
      while (bytes.size() >= 5) {
        if (bytes[0] != 2) {
          ++invalid;
          bytes.erase(bytes.begin());
          continue;
        }
        const auto n = bytes[1];
        if (bytes.size() < static_cast<size_t>(n + 5)) {
          break;
        }
        const std::vector<uint8_t> payload(bytes.begin() + 2, bytes.begin() + 2 + n);
        const auto crc = (bytes[n + 2] << 8) | bytes[n + 3];
        if (bytes[n + 4] != 3 || crc != crc16(payload)) {
          ++invalid;
        } else if (payload[0] == 8 && n == 5) {
          if (awaiting_servo) {++invalid;}
          awaiting_servo = true;
          ++speed_count;
        } else if (payload[0] == 12 && n == 3) {
          if (!awaiting_servo) {++invalid;}
          awaiting_servo = false;
          ++servo_count;
        } else if (payload[0] == 0) {
          // Answer the shutdown wake-up request so the receive thread can exit.
          std::vector<uint8_t> reply(40, 0);
          reply[1] = 6;
          reply[2] = 5;
          reply[3] = 'P';
          const auto checksum = crc16(reply);
          std::vector<uint8_t> frame{2, 40};
          frame.insert(frame.end(), reply.begin(), reply.end());
          frame.push_back(checksum >> 8);
          frame.push_back(checksum & 0xff);
          frame.push_back(3);
          if (write(master, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
            ++invalid;
          }
        } else {
          ++invalid;
        }
        bytes.erase(bytes.begin(), bytes.begin() + n + 5);
      }
    }
  });
  {
    vesc_driver::VescInterface interface(port, [](const auto &) {}, [](const auto &) {});
    std::vector<std::thread> writers;
    for (int worker = 0; worker < 4; ++worker) {
      writers.emplace_back([&, worker]() {
        try {
          for (int i = 0; i < 500; ++i) {
            interface.sendMotorAndServo(
              vesc_driver::VescPacketSetRPM(1000 + worker * 500 + i),
              vesc_driver::VescPacketSetServoPos(.5));
          }
        } catch (...) {
          ++send_errors;
        }
      });
    }
    for (auto & writer : writers) {
      writer.join();
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (speed_count + servo_count < 4000 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(send_errors, 0);
    EXPECT_EQ(speed_count, 2000);
    EXPECT_EQ(servo_count, 2000);
    EXPECT_EQ(invalid, 0);
  }
  running = false;
  controller.join();
  close(master);
  close(slave);
}
}  // namespace
