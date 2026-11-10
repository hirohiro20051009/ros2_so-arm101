// Copyright 2026 so_arm101 authors. MIT License.
//
// Sts3215WriteBus のバイト列・範囲検査・ポートの排他、およびドライバのソースに
// スレッド／タイマー／非同期が無いことの確認。偽サーボは pty（実機・実ポートは開かない）。
#include <gtest/gtest.h>

#include <atomic>
#include <fstream>
#include <regex>
#include <sstream>
#include <thread>

#include "fake_servo.hpp"
#include "so_arm101_hardware/serial_port.hpp"
#include "so_arm101_hardware/sts3215_write_bus.hpp"

using namespace so_arm101_hardware;  // NOLINT

TEST(WriteBytes, MatchScservoSdkLayout)
{
  // scservo_sdk: writeTxRx は FF FF id len 03 addr data.. chk、len = データ数 + 3
  // Torque_Enable(40)=1、id 2: len=4、chk = ~(2+4+3+40+1)&0xFF = ~50 = 0xCD
  const auto w = Sts3215WriteBus::build_write(2, 40, {1});
  EXPECT_EQ(w, (std::vector<uint8_t>{0xFF, 0xFF, 0x02, 0x04, 0x03, 0x28, 0x01, 0xCD}));
  // SYNC_WRITE Goal_Position(42, 2B)、id1=0x0123 id2=0x0456: FF FF FE len 83 2A 02 01 23 01 02 56 04 chk
  // len = (2+1)*2+4 = 10
  const auto s = Sts3215WriteBus::build_sync_write(42, 2, {1, 2}, {{0x23, 0x01}, {0x56, 0x04}});
  const std::vector<uint8_t> body{0xFE, 10, 0x83, 42, 2, 1, 0x23, 0x01, 2, 0x56, 0x04};
  unsigned sum = 0;
  for (auto b : body) {sum += b;}
  std::vector<uint8_t> expect{0xFF, 0xFF};
  expect.insert(expect.end(), body.begin(), body.end());
  expect.push_back(static_cast<uint8_t>(~sum & 0xFF));
  EXPECT_EQ(s, expect);
}

TEST(WriteBytes, SignMagnitudeEncode)
{
  uint16_t e = 0;
  EXPECT_TRUE(encode_sign_magnitude(2000, 15, &e));
  EXPECT_EQ(e, 2000);
  EXPECT_TRUE(encode_sign_magnitude(-5, 15, &e));
  EXPECT_EQ(e, 0x8005);
  EXPECT_EQ(decode_sign_magnitude(e, 15), -5);
  EXPECT_FALSE(encode_sign_magnitude(40000, 15, &e));
}

class WriteBusTest : public ::testing::Test
{
protected:
  fake::FakeServos servos{{1, 2, 3, 4, 5, 6}, 6};
  SerialPort port;
  void SetUp() override
  {
    std::string err;
    ASSERT_TRUE(port.open(servos.path(), 1000000, &err)) << err;
  }
};

TEST_F(WriteBusTest, WritesLandAndAreAcknowledged)
{
  Sts3215WriteBus bus(port, std::chrono::milliseconds(50));
  EXPECT_EQ(bus.write_lock(2, false), Status::Ok);
  EXPECT_EQ(bus.write_acceleration(2, 50), Status::Ok);
  EXPECT_EQ(bus.write_goal_position(2, 2000), Status::Ok);
  EXPECT_EQ(servos.u8(2, 55), 0);
  EXPECT_EQ(servos.u8(2, 41), 50);
  EXPECT_EQ(servos.u16(2, 42), 2000);
  EXPECT_EQ(bus.sync_write_goal_position({1, 3}, {1111, 2222}), Status::Ok);
  // SYNC_WRITE は応答が無いので、後続の読み取りで確認する
  std::vector<uint8_t> d;
  EXPECT_EQ(bus.read(3, reg::kGoalPosition, &d), Status::Ok);
  EXPECT_EQ(servos.u16(1, 42), 1111);
  EXPECT_EQ(servos.u16(3, 42), 2222);
}

TEST_F(WriteBusTest, EepromWriteIgnoredWhileLockedAndAcceptedWhenUnlocked)
{
  Sts3215WriteBus bus(port, std::chrono::milliseconds(50));
  EXPECT_EQ(bus.write_servo_setting(1, ServoSetting::P, 32), Status::Ok);
  EXPECT_EQ(servos.u8(1, 21), 16);  // Lock=1 なので無視された（読み戻しで検出できる）
  EXPECT_EQ(bus.write_lock(1, false), Status::Ok);
  EXPECT_EQ(bus.write_servo_setting(1, ServoSetting::P, 32), Status::Ok);
  EXPECT_EQ(servos.u8(1, 21), 32);
}

TEST_F(WriteBusTest, RejectsOutOfRangeValuesWithoutSending)
{
  Sts3215WriteBus bus(port, std::chrono::milliseconds(50));
  servos.clear_writes();
  EXPECT_EQ(bus.write_acceleration(1, 255), Status::BadFrame);
  EXPECT_EQ(bus.write_acceleration(1, -1), Status::BadFrame);
  EXPECT_EQ(bus.write_torque_limit(1, 1001), Status::BadFrame);
  EXPECT_EQ(bus.write_goal_position(1, 40000), Status::BadFrame);
  EXPECT_EQ(bus.sync_write_goal_position({1}, {1, 2}), Status::BadFrame);
  EXPECT_TRUE(servos.writes().empty());
}

TEST_F(WriteBusTest, OperatingModeCannotBeWritten)
{
  // allow-list に Operating_Mode(33) が無いこと（ServoSetting が指すレジスタを全部確認）
  for (auto s : {ServoSetting::P, ServoSetting::I, ServoSetting::D, ServoSetting::ReturnDelayTime,
      ServoSetting::MaxTorqueLimit, ServoSetting::ProtectionCurrent, ServoSetting::OverloadTorque})
  {
    EXPECT_NE(register_of(s).address, reg::kOperatingMode.address) << to_string(s);
    EXPECT_LE(register_of(s).address, 39) << to_string(s) << " must be an EEPROM register";
  }
}

TEST_F(WriteBusTest, ServoErrorOnWriteIsReported)
{
  Sts3215WriteBus bus(port, std::chrono::milliseconds(50));
  servos.set_err(4, 0x20);
  EXPECT_EQ(bus.write_lock(4, false), Status::ServoError);
  EXPECT_EQ(bus.last_servo_error(), 0x20);
  EXPECT_NE(bus.last_error().find("overload"), std::string::npos);
}

TEST_F(WriteBusTest, TwoThreadsUsingOnePortNeverCorruptEachOther)
{
  // 2 スレッドが同じポートを同時に叩く。往復が Transaction で排他なので、全部成功する
  // （排他が無ければ要求や応答のバイトが混ざって Checksum／IdMismatch／Timeout になる）。
  Sts3215WriteBus bus(port, std::chrono::milliseconds(200));
  std::atomic<int> bad{0};
  auto worker = [&](uint8_t id) {
      for (int i = 0; i < 300; ++i) {
        uint16_t v = 0;
        if (bus.read_u16(id, reg::kPresentPosition, &v) != Status::Ok) {++bad;}
        if (bus.write_goal_position(id, 1000 + i) != Status::Ok) {++bad;}
        std::vector<std::vector<uint8_t>> out;
        if (bus.sync_read({1, 2, 3, 4, 5, 6}, reg::kTorqueEnableAcceleration, &out) != Status::Ok) {++bad;}
      }
    };
  std::thread a(worker, 1), b(worker, 2);
  a.join();
  b.join();
  EXPECT_EQ(bad.load(), 0);
  EXPECT_EQ(servos.bad_requests(), 0u);  // 要求パケットのチェックサム不良もゼロ（バイトが混ざっていない）
  std::cout << "[info] port contention (waits) = " << port.contention_count() << std::endl;
}

TEST_F(WriteBusTest, SingleThreadedUseNeverContends)
{
  Sts3215WriteBus bus(port, std::chrono::milliseconds(50));
  for (int i = 0; i < 100; ++i) {
    uint16_t v = 0;
    EXPECT_EQ(bus.read_u16(1, reg::kPresentPosition, &v), Status::Ok);
  }
  EXPECT_EQ(port.contention_count(), 0u);
}

// ---- ドライバのソースにスレッド／タイマー／非同期が無い（1 Hz 監視を別スレッドにしない保証）----
TEST(DriverSource, HasNoThreadsTimersOrAsync)
{
  const std::regex forbidden(
    R"(std::thread|std::jthread|std::async|std::future|std::promise|pthread_|create_wall_timer|create_timer|)"
    R"(std::condition_variable|boost::asio|std::packaged_task)");
  for (const char * f : {
      "src/sts3215_system.cpp", "src/sts3215_bus.cpp", "src/sts3215_write_bus.cpp", "src/serial_port.cpp",
      "src/calibration.cpp", "src/servo_config.cpp", "include/so_arm101_hardware/sts3215_system.hpp",
      "include/so_arm101_hardware/sts3215_bus.hpp", "include/so_arm101_hardware/sts3215_write_bus.hpp",
      "include/so_arm101_hardware/serial_port.hpp"})
  {
    std::ifstream in(std::string(PACKAGE_SOURCE_DIR) + "/" + f);
    ASSERT_TRUE(in.good()) << f;
    std::string line;
    int n = 0;
    while (std::getline(in, line)) {
      ++n;
      const auto cm = line.find("//");
      const std::string code = cm == std::string::npos ? line : line.substr(0, cm);
      EXPECT_FALSE(std::regex_search(code, forbidden)) << f << ":" << n << ": " << line;
    }
  }
}
