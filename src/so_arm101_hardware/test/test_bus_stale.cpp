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

// ---- 競合の再現（今のコードでは落ちる。修正 A・B の後に通る）
TEST(BusStale, StaleRepliesAfterAFailedSyncReadAreDiscarded)
{
  Env e;
  e.arm_race();
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed), Status::BadChecksum);
  EXPECT_EQ(failed, 1);
  // 失敗の後に値を変える。古い応答（変える前の値）を受け取れば、値が合わない
  for (uint8_t id : kIds) {e.servos.set_present(id, 2000 + id);}
  Status s = e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed);  // 即時の再試行
  ASSERT_EQ(s, Status::Ok) << e.bus->last_error();
  for (size_t i = 0; i < kIds.size(); ++i) {EXPECT_EQ(decode(out[i]), 2000 + kIds[i]) << "id " << int(kIds[i]);}
}

TEST(BusStale, SingleReadDoesNotAcceptAnOldFrameOfTheSameIdAfterAFailedSyncRead)
{
  Env e;
  e.arm_race();
  e.servos.set_u16(2, 16, 1000);  // Max_Torque_Limit（2 バイト）
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::BadChecksum);
  uint16_t v = 0;
  // id 2 の古い Present_Position の応答（2 バイト）が、単独の READ の応答として受け取られてはいけない
  const Status s = e.bus->read_u16(2, reg::kMaxTorqueLimit, &v);
  ASSERT_EQ(s, Status::Ok) << e.bus->last_error();
  EXPECT_EQ(v, 1000);
}

// ---- 限界の記録: 窓（quiet 2 ms）の外に着く遅れた応答は、修正（A・B）でも捨てられない。
// 単独の READ は「同じ ID・同じ長さ・正しいチェックサム」の古い応答を受け取る。値が違っても気づけない。
// この試験は修正の前後で同じ結果（古い値を受け取る）になることを固定する＝限界の記録。
// 構成: タイムアウト（10 ms）の直後（約 14 ms）に着く応答。タイムアウトの後の drain（最大 2 ms の静けさ待ち）より後。
TEST(BusStale, ImmediateRetryAfterTimeoutWithLateRepliesIsAKnownLimit)
{
  Env e;
  e.servos.delay_next_replies(1, 56, 1, 14000);
  std::vector<uint8_t> d;
  ASSERT_EQ(e.bus->read(1, reg::kPresentPosition, &d), Status::Timeout);
  e.servos.set_present(1, 2001);  // 実際の値は変わった
  ASSERT_EQ(e.bus->read(1, reg::kPresentPosition, &d), Status::Ok);  // 即時の再試行
  EXPECT_EQ(decode(d), 1001) << "known limit: a stale frame is accepted by a single READ";
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

// ---- drain_quiet_ms=0（案 A・B を止める）は従来どおり。競合が再現する（比較用に残す）
TEST(BusStale, DrainQuietZeroKeepsTheOldBehaviourAndTheRaceStillShowsUp)
{
  Env e;
  e.bus->set_drain_quiet(std::chrono::microseconds(0));
  e.arm_race();
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::BadChecksum);
  for (uint8_t id : kIds) {e.servos.set_present(id, 2000 + id);}
  EXPECT_NE(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

// ---- 残りの応答を読み切る（案 B）: 途中の ID で失敗しても、後ろの ID の応答は次の要求に入らない
TEST(BusStale, FailureInTheMiddleOfASyncReadAlsoDiscardsTheLaterIds)
{
  Env e;
  for (uint8_t id = 1; id <= 6; ++id) {e.servos.set_reply_delay_us(id, 1500);}
  e.servos.set_frame_spacing_us(80);
  e.servos.corrupt_read_replies(3, 56, 1);  // id 3 が壊れる。id 4〜6 の応答は失敗の後にも届く
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed), Status::BadChecksum);
  EXPECT_EQ(failed, 3);
  for (uint8_t id : kIds) {e.servos.set_present(id, 2000 + id);}
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed), Status::Ok) << e.bus->last_error();
  for (size_t i = 0; i < kIds.size(); ++i) {EXPECT_EQ(decode(out[i]), 2000 + kIds[i]);}
}

// ---- 応答しない ID（落ちたサーボ）がいても、後ろの応答は読み切り、次の読み取りは成功する
TEST(BusStale, ASilentIdInTheMiddleDoesNotPoisonTheNextRead)
{
  Env e;
  e.servos.drop_read_replies(2, 56, 1);
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed), Status::Timeout);
  EXPECT_EQ(failed, 2);
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out, &failed), Status::Ok) << e.bus->last_error();
  EXPECT_EQ(decode(out[0]), 1001);
}

// ---- 正常な経路は変わらない（読み取りの回数と所要時間。ドレインは失敗のときだけ）
TEST(BusStale, HealthyReadsAreNotSlowedByTheDrain)
{
  auto run = [](int quiet_us) {
      Env e;
      e.bus->set_drain_quiet(std::chrono::microseconds(quiet_us));
      std::vector<std::vector<uint8_t>> out;
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < 300; ++i) {
        EXPECT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);
      }
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    };
  const double off = run(0), on = run(2000);
  printf("[healthy] 300 sync_reads: drain_quiet=0 %.1f ms, drain_quiet=2ms %.1f ms\n", off, on);
  EXPECT_LT(on, off * 1.5 + 20.0);
}

// ---- 診断カウンタ（案 D）
TEST(BusCounters, AHealthyBusHasZeroDrainedBytesAndZeroMismatches)
{
  Env e;
  std::vector<std::vector<uint8_t>> out;
  for (int i = 0; i < 50; ++i) {ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);}
  const auto & c = e.bus->counters();
  EXPECT_EQ(c.drained_bytes, 0u);
  EXPECT_EQ(c.id_mismatch, 0u);
  EXPECT_EQ(c.answered_first, 0u);
  EXPECT_EQ(c.timeouts, 0u);
  EXPECT_EQ(c.bad_checksum, 0u);
  EXPECT_EQ(c.flushed_bytes, 0u);
  EXPECT_EQ(c.to_string(),
    "bus: flushed_requests=0 flushed_bytes=0 drained_bytes=0 id_mismatch=0 answered_first=0 timeouts=0 bad_checksum=0");
}

TEST(BusCounters, DrainedBytesEqualTheRemainingRepliesAfterAFailedSyncRead)
{
  Env e;
  e.arm_race();
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::BadChecksum);
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);
  const auto & c = e.bus->counters();
  EXPECT_EQ(c.bad_checksum, 1u);
  EXPECT_EQ(c.drained_bytes, 5u * 8u);  // id 2〜6 の 5 フレーム × 8 バイト（FF FF ID LEN ERR D0 D1 CHK）
  EXPECT_EQ(c.id_mismatch, 0u);
  EXPECT_EQ(c.answered_first, 0u);
}

TEST(BusCounters, WithoutTheFixTheRaceShowsUpInIdMismatchOrAnsweredFirst)
{
  Env e;
  e.bus->set_drain_quiet(std::chrono::microseconds(0));
  e.arm_race();
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::BadChecksum);
  EXPECT_NE(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Ok);
  const auto & c = e.bus->counters();
  EXPECT_GT(c.id_mismatch + c.answered_first, 0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

TEST(BusCounters, AnsweredFirstIsCountedAsNotAnIdMismatch)
{
  Env e;
  e.servos.drop_read_replies(1, 56, 1);  // id 1 は応答しない。id 2 が先に返る
  std::vector<std::vector<uint8_t>> out;
  ASSERT_EQ(e.bus->sync_read(kIds, reg::kPresentPosition, &out), Status::Timeout);
  const auto & c = e.bus->counters();
  EXPECT_EQ(c.answered_first, 1u);
  EXPECT_EQ(c.id_mismatch, 0u);
  EXPECT_EQ(c.drained_bytes, 4u * 8u);  // id 3〜6 の残り（id 2 は読み替えで使った）
}

TEST(BusCounters, StaleBytesBeforeARequestAreCountedAsFlushed)
{
  Env e;
  e.servos.set_reply_delay_us(1, 0);
  std::vector<uint8_t> d;
  // 応答のタイムアウト（2 ms）より遅れて着く応答は、次の要求の前の flush に入る
  Sts3215Bus fast(e.port, std::chrono::milliseconds(2));
  e.servos.delay_next_replies(1, 56, 1, 5000);
  ASSERT_EQ(fast.read(1, reg::kPresentPosition, &d), Status::Timeout);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(fast.read(1, reg::kPresentPosition, &d), Status::Ok);
  EXPECT_EQ(fast.counters().flushed_requests, 1u);
  EXPECT_EQ(fast.counters().flushed_bytes, 8u);
  EXPECT_EQ(fast.counters().timeouts, 1u);
}
