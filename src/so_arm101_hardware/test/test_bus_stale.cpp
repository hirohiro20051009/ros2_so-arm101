// Copyright 2026 so_arm101 authors. MIT License.
//
// バス層の「古い応答の取り違え」の試験（docs/bus_layer_fix_plan.md の 3・4 章）。
// 偽サーボ（pty。実機・実ポートは開かない）の「応答を遅らせる」機能で、競合を決定的に再現する。
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "fake_servo.hpp"
#include "so_arm101_hardware/serial_port.hpp"
#include "so_arm101_hardware/sts3215_bus.hpp"

using namespace so_arm101_hardware;  // NOLINT

namespace
{

const std::vector<uint8_t> kIds{1, 2, 3, 4, 5, 6};

struct Env
{
  fake::FakeServos servos{kIds, 6};
  SerialPort port;
  std::unique_ptr<Sts3215Bus> bus;
  Env()
  {
    std::string err;
    EXPECT_TRUE(port.open(servos.path(), 1000000, &err)) << err;
    bus = std::make_unique<Sts3215Bus>(port);
    for (uint8_t id : kIds) {servos.set_present(id, 1000 + id);}
  }
  /// id 1 の応答を 1 回壊し、id 2〜6 は 1.5 ms 遅れて 80 µs 間隔で届く（失敗の「後」に着く）。
  void arm_race(int delay_us = 1500)
  {
    servos.corrupt_read_replies(1, 56, 1);
    for (uint8_t id = 2; id <= 6; ++id) {servos.set_reply_delay_us(id, delay_us);}
    servos.set_frame_spacing_us(80);
  }
};

int decode(const std::vector<uint8_t> & d)
{
  return decode_sign_magnitude(static_cast<uint16_t>(d[0] | (d[1] << 8)), 15);
}

}  // namespace

// ---- 偽サーボの遅れの機能そのものの確認（ドライバ無関係）
TEST(FakeServoDelay, RepliesAreWrittenAtTheScheduledTimeAndLogged)
{
  fake::FakeServos s(kIds, 6);
  SerialPort port;
  std::string err;
  ASSERT_TRUE(port.open(s.path(), 1000000, &err)) << err;
  Sts3215Bus bus(port, std::chrono::milliseconds(50));
  s.set_reply_delay_us(2, 5000);
  std::vector<uint8_t> d;
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(bus.read(2, reg::kPresentPosition, &d), Status::Ok);
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(dt, std::chrono::microseconds(4500));
  const auto log = s.reply_log();
  ASSERT_EQ(log.size(), 1u);
  EXPECT_EQ(log[0].id, 2);
  EXPECT_EQ(log[0].addr, 56);
  EXPECT_FALSE(log[0].corrupt);
}

TEST(FakeServoDelay, FrameSpacingOrdersSyncReadRepliesAndKeepsTheOrder)
{
  fake::FakeServos s(kIds, 6);
  SerialPort port;
  std::string err;
  ASSERT_TRUE(port.open(s.path(), 1000000, &err)) << err;
  Sts3215Bus bus(port);
  s.set_frame_spacing_us(300);
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(bus.sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);
  const auto log = s.reply_log();
  ASSERT_EQ(log.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {EXPECT_EQ(log[i].id, kIds[i]);}
  EXPECT_GE(log[5].at - log[0].at, std::chrono::microseconds(1400));
}

TEST(FakeServoDelay, DelayNextRepliesAppliesOnlyToTheCountedReads)
{
  fake::FakeServos s(kIds, 6);
  SerialPort port;
  std::string err;
  ASSERT_TRUE(port.open(s.path(), 1000000, &err)) << err;
  Sts3215Bus bus(port, std::chrono::milliseconds(3));
  s.delay_next_replies(3, 56, 1, 20000);  // タイムアウト（3 ms）より遅れる
  std::vector<uint8_t> d;
  EXPECT_EQ(bus.read(3, reg::kPresentPosition, &d), Status::Timeout);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  port.flush_input();
  EXPECT_EQ(bus.read(3, reg::kPresentPosition, &d), Status::Ok);  // 2 回目は遅れない
}

