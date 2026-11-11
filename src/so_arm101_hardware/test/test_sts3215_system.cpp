// Copyright 2026 so_arm101 authors. MIT License.
//
// Sts3215System（書き込みモード／読み取り専用）の試験。pty 上の偽サーボ（test/fake_servo.hpp）を使う。
// 実機・実ポートは開かない。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "fake_servo.hpp"
#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/sts3215_system.hpp"

using namespace so_arm101_hardware;  // NOLINT
using hardware_interface::CallbackReturn;

namespace
{
const char * kJoints[] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
const int kPresent[] = {2097, 2029, 1975, 2037, 2064, 2400};

struct Rig
{
  fake::FakeServos servos{{1, 2, 3, 4, 5, 6}, 6};
  Calibration cal{load_calibration(CALIBRATION_YAML)};
  std::unique_ptr<Sts3215System> sys;
  std::vector<hardware_interface::CommandInterface> cmds;
  std::vector<hardware_interface::StateInterface> states;
  double t{0.0};

  Rig()
  {
    for (int i = 0; i < 6; ++i) {servos.set_present(i + 1, kPresent[i]);}
    // 実機の EEPROM は yaml のキャリブレーションと一致している（段階2c-1a: 不一致なら書き込みモードは拒否する）。
    // 試験の homing_offset は正の値だけ（sign-magnitude でも同じ値）。
    for (const auto & j : cal.joints) {
      servos.set_eeprom(j.id, 31, j.eeprom.homing_offset, 2);
      servos.set_eeprom(j.id, 9, j.eeprom.min_limit, 2);
      servos.set_eeprom(j.id, 11, j.eeprom.max_limit, 2);
    }
  }

  bool init(std::map<std::string, std::string> extra = {}, bool read_only = false)
  {
    hardware_interface::HardwareInfo info;
    info.name = "test";
    info.type = "system";
    info.hardware_parameters = {
      {"serial_port", servos.path()}, {"calibration_file", CALIBRATION_YAML},
      {"read_only", read_only ? "true" : "false"}, {"timeout_ms", "30"}, {"monitor_period_s", "0.1"}};
    for (auto & kv : extra) {info.hardware_parameters[kv.first] = kv.second;}
    for (auto * j : kJoints) {
      hardware_interface::ComponentInfo ci;
      ci.name = j;
      ci.type = "joint";
      hardware_interface::InterfaceInfo pos;
      pos.name = "position";
      ci.command_interfaces.push_back(pos);
      ci.state_interfaces.push_back(pos);
      hardware_interface::InterfaceInfo vel;
      vel.name = "velocity";
      ci.state_interfaces.push_back(vel);
      info.joints.push_back(ci);
    }
    sys = std::make_unique<Sts3215System>();
    if (sys->on_init(info) != CallbackReturn::SUCCESS) {return false;}
    states = sys->export_state_interfaces();
    cmds = sys->export_command_interfaces();
    return true;
  }
  rclcpp_lifecycle::State st;
  CallbackReturn configure() {return sys->on_configure(st);}
  CallbackReturn activate() {return sys->on_activate(st);}
  CallbackReturn deactivate() {return sys->on_deactivate(st);}
  bool start()
  {
    return init() && configure() == CallbackReturn::SUCCESS && activate() == CallbackReturn::SUCCESS;
  }
  void cycle(int n = 1)
  {
    for (int i = 0; i < n; ++i) {
      t += 0.02;
      const rclcpp::Time now(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
      const auto period = rclcpp::Duration::from_seconds(0.02);
      sys->read(now, period);
      sys->write(now, period);
    }
    // SYNC_WRITE には応答が無いので、偽サーボが受け取り終えるのを少し待つ（試験の都合）
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
  double rad_of(int i, int raw) {return raw_to_rad(cal.joints[i], cal.angle_ticks, raw);}
  void command_raw(int i, int raw) {cmds[i].set_value(rad_of(i, raw));}
  double position(int i) {return states[2 * i].get_value();}
};

/// writes() から、(addr, data) に一致する最初／最後の位置を探す。
int first_index(const std::vector<fake::WriteRecord> & w, int addr, int data0, int id = -1)
{
  for (size_t i = 0; i < w.size(); ++i) {
    if (w[i].addr == addr && !w[i].data.empty() && w[i].data[0] == data0 && (id < 0 || w[i].id == id)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}
int last_index(const std::vector<fake::WriteRecord> & w, int addr, int id = -1)
{
  int r = -1;
  for (size_t i = 0; i < w.size(); ++i) {
    if (w[i].addr == addr && (id < 0 || w[i].id == id)) {r = static_cast<int>(i);}
  }
  return r;
}
}  // namespace

// ---------------------------------------------------------------- 起動の書き込み順
TEST(WriteMode, StartupWriteOrder)
{
  Rig r;
  ASSERT_TRUE(r.start());
  const auto w = r.servos.writes();
  ASSERT_FALSE(w.empty());
  // 最初の書き込みは Lock=0（アドレス 55）
  EXPECT_EQ(w[0].addr, 55);
  EXPECT_EQ(w[0].data[0], 0);
  const int lock0_last = [&] {
      int x = -1;
      for (size_t i = 0; i < w.size(); ++i) {
        if (w[i].addr == 55 && w[i].data[0] == 0) {x = static_cast<int>(i);}
      }
      return x;
    }();
  const int accel_first = first_index(w, 41, 50);
  const int accel_last = last_index(w, 41);
  const int goal_first = [&] {
      for (size_t i = 0; i < w.size(); ++i) {if (w[i].addr == 42) {return static_cast<int>(i);}}
      return -1;
    }();
  const int goal_last = last_index(w, 42);
  const int torque_on_first = first_index(w, 40, 1);
  const int lock1_first = first_index(w, 55, 1);
  ASSERT_GE(accel_first, 0);
  ASSERT_GE(goal_first, 0);
  ASSERT_GE(torque_on_first, 0);
  ASSERT_GE(lock1_first, 0);
  // Lock=0 → Acceleration → Goal_Position → Torque_Enable=1 → Lock=1
  EXPECT_LT(lock0_last, accel_first);
  EXPECT_LT(accel_last, goal_first);
  EXPECT_LT(goal_last, torque_on_first);
  EXPECT_LT(torque_on_first, lock1_first);
  // 全関節に Acceleration=50 が書かれた。0 は一度も書かれていない
  for (int id = 1; id <= 6; ++id) {
    EXPECT_GE(first_index(w, 41, 50, id), 0) << "id " << id;
    EXPECT_LT(first_index(w, 41, 0, id), 0) << "id " << id;
  }
  // Goal_Position = 現在位置（電源投入直後の Goal=0 のままトルクを ON にしない）
  for (int i = 0; i < 6; ++i) {EXPECT_EQ(r.servos.goal_position(i + 1), kPresent[i]) << kJoints[i];}
  // 最終状態
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 1);
    EXPECT_EQ(r.servos.u8(id, 55), 1);
    EXPECT_EQ(r.servos.u8(id, 41), 50);
  }
  // Torque_Limit は既に 1000（gripper 500）なので書かない
  EXPECT_LT(last_index(w, 48), 0);
  // Operating_Mode(33) と Goal_Velocity(46) には一度も書いていない
  EXPECT_LT(last_index(w, 33), 0);
  EXPECT_LT(last_index(w, 46), 0);
}

TEST(WriteMode, TorqueLimitParamWritesOnlyWhenDifferent)
{
  Rig r;
  ASSERT_TRUE(r.init({{"torque_limit", "500"}}));
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.activate(), CallbackReturn::SUCCESS);
  const auto w = r.servos.writes();
  // pan(1) と roll(5) だけ 500 に。重力を受ける lift(2)/elbow(3)/wrist_flex(4) は 1000 のまま（下げない）
  EXPECT_EQ(r.servos.u16(1, 48), 500);
  EXPECT_EQ(r.servos.u16(5, 48), 500);
  for (int id : {2, 3, 4}) {
    EXPECT_EQ(r.servos.u16(id, 48), 1000) << id;
    EXPECT_LT(last_index(w, 48, id), 0) << id;
  }
  EXPECT_EQ(r.servos.u16(6, 48), 500);   // gripper は元から 500
  EXPECT_LT(last_index(w, 48, 6), 0);    // gripper には書いていない
  EXPECT_GE(last_index(w, 48, 1), 0);
}

// ---------------------------------------------------------------- Acceleration=0 の拒否
TEST(WriteMode, RefusesTorqueWhenAccelerationStaysZero)
{
  Rig r;
  r.servos.ignore_writes_to(41);  // Acceleration が書けない（0 のまま）状況
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR);
  const auto w = r.servos.writes();
  EXPECT_LT(first_index(w, 40, 1), 0) << "Torque_Enable=1 must never be written";
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 0);
    EXPECT_EQ(r.servos.u8(id, 55), 1) << "Lock must be restored to 1 after the aborted start";
    EXPECT_EQ(r.servos.u8(id, 41), 0);
  }
}

TEST(WriteMode, ReadOnlyParamMustBeExactlyTrueOrFalse)
{
  for (const char * bad : {"yes", "ture", "1", "", "0"}) {
    Rig r;
    EXPECT_FALSE(r.init({{"read_only", bad}})) << "'" << bad << "' must not start (and must not mean write mode)";
  }
  Rig ok;
  EXPECT_TRUE(ok.init({{"read_only", "True"}}));
  Rig ok2;
  EXPECT_TRUE(ok2.init({{"read_only", "False"}}));
}

TEST(WriteMode, ParamAccelerationZeroIsRejected)
{
  Rig r;
  EXPECT_FALSE(r.init({{"acceleration", "0"}}));
}

TEST(WriteMode, RefusesWhenGoalPositionReadbackDiffers)
{
  Rig r;
  r.servos.ignore_writes_to(42);  // Goal_Position が書けない
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR);
  EXPECT_LT(first_index(r.servos.writes(), 40, 1), 0);
  for (int id = 1; id <= 6; ++id) {EXPECT_EQ(r.servos.u8(id, 55), 1);}
}

// 通信エラー（化け・無応答）は起動の読み戻しを再試行する。値の不一致は再試行しない。
TEST(WriteMode, CorruptedGoalReadbackIsRetriedAndStartupSucceeds)
{
  Rig r;
  r.servos.corrupt_read_replies(1, 42, 2);  // 1 台目の Goal_Position の読み戻しが 2 回化ける
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.sys->startup_read_retries(), 2u);
  EXPECT_EQ(r.servos.goal_position(1), kPresent[0]);
}

TEST(WriteMode, DroppedReadbackReplyIsRetriedAndStartupSucceeds)
{
  Rig r;
  r.servos.drop_read_replies(5, 42, 1);  // タイムアウト
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.sys->startup_read_retries(), 1u);
}

TEST(WriteMode, PersistentCorruptionNeverEnablesTorque)
{
  Rig r;
  r.servos.corrupt_read_replies(5, 42, 1000);  // 3 回とも化ける
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR);
  EXPECT_EQ(r.sys->startup_read_retries(), 2u);  // 3 回試して 2 回再試行
  EXPECT_LT(first_index(r.servos.writes(), 40, 1), 0);  // Torque_Enable=1 は書かれていない
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 0);
    EXPECT_EQ(r.servos.u8(id, 55), 1);  // Lock=1 に戻した
  }
}

TEST(WriteMode, ValueMismatchIsNotRetried)
{
  Rig r;
  r.servos.ignore_writes_to(42);  // 正しく受信できるが、値が期待と違う
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR);
  EXPECT_EQ(r.sys->startup_read_retries(), 0u);
}

TEST(WriteMode, AccelerationZeroIsNotRetried)
{
  Rig r;
  r.servos.ignore_writes_to(41);  // Acceleration が 0 のまま
  ASSERT_TRUE(r.init());
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR);
  EXPECT_EQ(r.sys->startup_read_retries(), 0u);
}

TEST(WriteMode, RefusesWhenGoalVelocityIsNotZero)
{
  Rig r;
  r.servos.set_u16(3, 46, 7);
  ASSERT_TRUE(r.init());
  EXPECT_EQ(r.configure(), CallbackReturn::ERROR);  // 診断の段階で中止
  // 診断を通ってしまった場合でも起動で止まる（診断後に値が変わった想定）
  Rig r2;
  ASSERT_TRUE(r2.init());
  ASSERT_EQ(r2.configure(), CallbackReturn::SUCCESS);
  r2.servos.set_u16(3, 46, 7);
  EXPECT_EQ(r2.activate(), CallbackReturn::ERROR);
  EXPECT_TRUE(r2.servos.writes().empty()) << "nothing may be written before the checks pass";
}

TEST(WriteMode, RefusesWhenOperatingModeIsNotPositionControl)
{
  Rig r;
  r.servos.set_u8(2, 33, 1);
  ASSERT_TRUE(r.init());
  EXPECT_EQ(r.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(r.servos.writes().empty());
  // apply_servo_config:=true でも Operating_Mode は書かない
  Rig r2;
  r2.servos.set_u8(2, 33, 1);
  ASSERT_TRUE(r2.init({{"apply_servo_config", "true"}}));
  EXPECT_EQ(r2.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(r2.servos.writes().empty());
}

// ---------------------------------------------------------------- 設定不一致 / apply_servo_config
TEST(WriteMode, ConfigMismatchAbortsUnlessApplyRequested)
{
  Rig r;
  r.servos.set_eeprom(2, 21, 32);  // P=32（LeRobot の 16 と違う）
  ASSERT_TRUE(r.init());
  EXPECT_EQ(r.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(r.servos.writes().empty());

  Rig a;
  a.servos.set_eeprom(2, 21, 32);
  a.servos.set_eeprom(6, 16, 1000, 2);  // gripper の Max_Torque_Limit
  ASSERT_TRUE(a.init({{"apply_servo_config", "true"}}));
  ASSERT_EQ(a.configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(a.activate(), CallbackReturn::SUCCESS);
  EXPECT_EQ(a.servos.u8(2, 21), 16);
  EXPECT_EQ(a.servos.u16(6, 16), 500);
  const auto w = a.servos.writes();
  // EEPROM は Lock=0 の後、トルク ON の前に書かれる
  EXPECT_GT(last_index(w, 21, 2), first_index(w, 55, 0));
  EXPECT_LT(last_index(w, 21, 2), first_index(w, 40, 1));
}

TEST(ReadOnlyMode, MismatchOnlyWarnsAndNothingIsWritten)
{
  Rig r;
  r.servos.set_eeprom(2, 21, 32);
  ASSERT_TRUE(r.init({}, true));
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.activate(), CallbackReturn::SUCCESS);
}

// ---------------------------------------------------------------- EEPROM と yaml の不一致（段階2c-1a）
// 書き込みモードは拒否する（トルクを ON にしない・何も書かない）。読み取り専用は WARN で継続する。
TEST(WriteMode, RefusesWhenHomingOffsetDiffersFromYaml)
{
  Rig r;
  r.servos.set_eeprom(3, 31, 123, 2);  // yaml は 122
  ASSERT_TRUE(r.init());
  EXPECT_EQ(r.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(r.servos.writes().empty());
}

TEST(WriteMode, RefusesWhenMinOrMaxLimitDiffersFromYaml)
{
  Rig a;
  a.servos.set_eeprom(1, 9, 744, 2);  // yaml は 743
  ASSERT_TRUE(a.init());
  EXPECT_EQ(a.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(a.servos.writes().empty());

  Rig b;
  b.servos.set_eeprom(6, 11, 3386, 2);  // yaml は 3385
  ASSERT_TRUE(b.init());
  EXPECT_EQ(b.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(b.servos.writes().empty());
}

TEST(WriteMode, CalibrationMismatchIsNotOverriddenByApplyServoConfig)
{
  Rig r;
  r.servos.set_eeprom(3, 31, 123, 2);
  ASSERT_TRUE(r.init({{"apply_servo_config", "true"}}));
  EXPECT_EQ(r.configure(), CallbackReturn::ERROR);
  EXPECT_TRUE(r.servos.writes().empty());
}

TEST(WriteMode, StartsWhenEepromMatchesYaml)
{
  Rig r;
  ASSERT_TRUE(r.start());
}

TEST(ReadOnlyMode, CalibrationMismatchOnlyWarnsAndNothingIsWritten)
{
  Rig r;
  r.servos.set_eeprom(3, 31, 123, 2);
  r.servos.set_eeprom(1, 9, 744, 2);
  r.servos.set_eeprom(6, 11, 3386, 2);
  ASSERT_TRUE(r.init({}, true));
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.activate(), CallbackReturn::SUCCESS);
  EXPECT_TRUE(r.servos.writes().empty());
}

// ---------------------------------------------------------------- 終了の手順（トルク OFF → Lock=1）
TEST(WriteMode, DeactivateTurnsTorqueOffThenRestoresLock)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.cycle(5);
  const size_t before = r.servos.writes().size();
  ASSERT_EQ(r.deactivate(), CallbackReturn::SUCCESS);
  const auto w = r.servos.writes();
  std::vector<fake::WriteRecord> tail(w.begin() + before, w.end());
  int last_off = -1, first_lock1 = -1;
  for (size_t i = 0; i < tail.size(); ++i) {
    if (tail[i].addr == 40 && tail[i].data[0] == 0) {last_off = static_cast<int>(i);}
    if (tail[i].addr == 55 && tail[i].data[0] == 1 && first_lock1 < 0) {first_lock1 = static_cast<int>(i);}
  }
  ASSERT_GE(last_off, 0);
  ASSERT_GE(first_lock1, 0);
  EXPECT_LT(last_off, first_lock1) << "all torque-OFF writes come before any Lock=1 write";
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 0);
    EXPECT_EQ(r.servos.u8(id, 55), 1);
  }
  // 2 度目（cleanup）は何も書かない
  const size_t after = r.servos.writes().size();
  EXPECT_EQ(r.sys->on_cleanup(r.st), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.servos.writes().size(), after);
}

TEST(WriteMode, DestructorAlsoTurnsTorqueOffAndRestoresLock)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.cycle(3);
  r.sys.reset();  // lifecycle を通らずに破棄
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 0);
    EXPECT_EQ(r.servos.u8(id, 55), 1);
  }
}

TEST(WriteMode, CleanupAfterFaultStillShutsDown)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.servos.power_cycle();       // サーボがリセットされた（トルク OFF）
  r.cycle(10);                  // 監視が検出する
  EXPECT_TRUE(r.sys->fault_latched());
  ASSERT_EQ(r.deactivate(), CallbackReturn::SUCCESS);
  for (int id = 1; id <= 6; ++id) {EXPECT_EQ(r.servos.u8(id, 55), 1);}
}

// ---------------------------------------------------------------- 送信とリセット検出
TEST(WriteMode, HoldsCurrentPositionAndSendsGoalEveryCycle)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.servos.clear_writes();
  r.cycle(10);
  const auto counts = r.servos.instruction_counts();
  EXPECT_GE(counts.at(0x83), 10u);
  for (int i = 0; i < 6; ++i) {EXPECT_EQ(r.servos.goal_position(i + 1), kPresent[i]) << kJoints[i];}
  EXPECT_FALSE(r.sys->fault_latched());
  EXPECT_EQ(r.sys->limited_total(), 0u);
}

TEST(WriteMode, MovesTowardCommandWithinOneStepLimit)
{
  Rig r;
  ASSERT_TRUE(r.start());
  // 0.40 rad の飛び（スケーリング 1.0 で JTC の更新が 1 回遅れた場合）は削られない
  const double tick = tick_in_rad(r.cal.joints[0], r.cal.angle_ticks);
  const int delta = static_cast<int>(0.40 / tick);
  r.command_raw(0, kPresent[0] + delta);
  r.cycle(1);
  EXPECT_EQ(r.servos.goal_position(1), kPresent[0] + delta);
  EXPECT_EQ(r.sys->limited_total(), 0u);
}

TEST(WriteMode, StepLimitCutsALargeJumpToTheLimit)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.command_raw(0, kPresent[0] + 1000);  // 1000 tick ≈ 1.53 rad の飛び
  r.cycle(1);
  const int step = static_cast<int>(std::floor(kDefaultMaxJointSpeed * kDefaultLimitWindow / (2 * M_PI / 4095)));
  EXPECT_EQ(r.servos.goal_position(1), kPresent[0] + step);
  EXPECT_EQ(r.sys->limited_total(), 1u);
  r.cycle(1);
  EXPECT_EQ(r.servos.goal_position(1), kPresent[0] + 2 * step);  // 次の周期でも進む（ランプ）
}

TEST(WriteMode, CommandsBeyondTheCalibratedRangeAreClamped)
{
  Rig r;
  ASSERT_TRUE(r.start());
  const int hi = r.cal.joints[1].raw_max;
  // 範囲を少し(許容内)超えた指令: 黙ってクランプし、異常とは数えない
  r.command_raw(1, hi + 10);
  for (int k = 0; k < 120; ++k) {r.cycle(1);}  // 120 周期 > anomaly_cycles
  EXPECT_FALSE(r.sys->fault_latched());
  EXPECT_LE(r.servos.goal_position(2), hi);
}

TEST(WriteMode, NanCommandsHoldThenFaultAfterManyCycles)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.cmds[0].set_value(std::nan(""));
  r.cycle(10);
  EXPECT_EQ(r.servos.goal_position(1), kPresent[0]);  // 直前の目標を保持
  EXPECT_FALSE(r.sys->fault_latched());
  r.cycle(60);
  EXPECT_TRUE(r.sys->fault_latched());
  // フォルト後は送信しない（トルクは保持＝OFF にしない）
  const auto n = r.servos.instruction_counts().at(0x83);
  r.cycle(10);
  EXPECT_EQ(r.servos.instruction_counts().at(0x83), n);
  EXPECT_EQ(r.servos.u8(1, 40), 1);
  // 正常な指令に戻ってもフォルトは自動では解除されない
  r.cmds[0].set_value(r.position(0));
  r.cycle(5);
  EXPECT_TRUE(r.sys->fault_latched());
}

TEST(WriteMode, ServoPowerResetIsDetectedWithinTheMonitorPeriodAndSendingStops)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.cycle(30);  // 0.6 s 健全
  EXPECT_FALSE(r.sys->fault_latched());
  const auto sent = r.servos.instruction_counts().at(0x83);
  r.servos.power_cycle();  // Acceleration=0・Lock=1・Torque_Enable=0 に戻る
  int detected_after = -1;
  for (int k = 1; k <= 20; ++k) {
    r.cycle(1);
    if (r.sys->fault_latched()) {detected_after = k; break;}
  }
  ASSERT_GT(detected_after, 0) << "reset must be detected";
  EXPECT_LE(detected_after * 0.02, 0.1 + 0.04 + 1e-9);  // monitor_period_s=0.1 + 余裕
  EXPECT_NE(r.sys->fault_reason().find("powered off or reset"), std::string::npos);
  const auto sent_at_detect = r.servos.instruction_counts().at(0x83);
  EXPECT_LE(sent_at_detect, sent + detected_after);
  r.cycle(20);
  EXPECT_EQ(r.servos.instruction_counts().at(0x83), sent_at_detect) << "no Goal_Position after the fault";
}

TEST(WriteMode, ServoErrorByteLatchesUntilProcessRestart)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.cycle(3);
  r.servos.set_err(2, 0x20);  // overload protection
  r.cycle(3);
  EXPECT_TRUE(r.sys->fault_latched());
  EXPECT_NE(r.sys->fault_reason().find("protection"), std::string::npos);
  r.servos.set_err(2, 0);
  ASSERT_EQ(r.deactivate(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::ERROR) << "deactivate -> activate must not clear a protection latch";
}

TEST(WriteMode, ARecoverableFaultCanBeClearedByDeactivateAndActivate)
{
  Rig r;
  ASSERT_TRUE(r.start());
  r.servos.power_cycle();
  r.cycle(10);
  ASSERT_TRUE(r.sys->fault_latched());
  ASSERT_EQ(r.deactivate(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.activate(), CallbackReturn::SUCCESS);  // 電源投入直後と同じ状態から安全に再起動できる
  EXPECT_FALSE(r.sys->fault_latched());
  for (int id = 1; id <= 6; ++id) {EXPECT_EQ(r.servos.u8(id, 41), 50);}
}

// ---------------------------------------------------------------- 1 Hz 監視とポートの同時使用
TEST(WriteMode, MonitorRunsInsideTheCycleAndNeverContendsForThePort)
{
  Rig r;
  ASSERT_TRUE(r.start());
  const auto before = r.servos.instruction_counts();
  r.cycle(500);  // 10 秒ぶん
  EXPECT_FALSE(r.sys->fault_latched());
  EXPECT_EQ(r.sys->port_contention(), 0u) << "the monitor must not use the port concurrently";
  EXPECT_EQ(r.servos.bad_requests(), 0u);
  // SYNC_READ は 1 周期 1 回の位置読み + 監視ぶん。監視（0.1 s ごと）が走った
  const auto after = r.servos.instruction_counts();
  EXPECT_GT(after.at(0x82) - before.at(0x82), 500u + 40u);
}

// ---------------------------------------------------------------- read_only:=true は書き込み 0 件
TEST(ReadOnlyMode, SendsNoWriteInstructionAtAll)
{
  Rig r;
  ASSERT_TRUE(r.init({}, true));
  ASSERT_EQ(r.configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.activate(), CallbackReturn::SUCCESS);
  r.command_raw(0, kPresent[0] + 300);  // 指令は破棄される
  r.cycle(200);
  EXPECT_NEAR(r.position(0), r.rad_of(0, kPresent[0]), 1e-9);  // 指令は無視され、位置は実測のまま
  ASSERT_EQ(r.deactivate(), CallbackReturn::SUCCESS);
  ASSERT_EQ(r.sys->on_cleanup(r.st), CallbackReturn::SUCCESS);
  EXPECT_TRUE(r.servos.writes().empty());
  for (auto & kv : r.servos.instruction_counts()) {
    EXPECT_TRUE(kv.first == 0x02 || kv.first == 0x82) << "unexpected instruction 0x" << std::hex << int(kv.first);
  }
  for (int id = 1; id <= 6; ++id) {
    EXPECT_EQ(r.servos.u8(id, 40), 0);  // トルクに触れていない
    EXPECT_EQ(r.servos.u8(id, 41), 0);  // Acceleration も電源投入時のまま
    EXPECT_EQ(r.servos.u8(id, 55), 1);
  }
}

// ---------------------------------------------------------------- 設計値の整合
TEST(DesignLimits, StepLimitCoversTheWorstCaseTrajectoryStep)
{
  const YAML::Node limits = YAML::LoadFile(JOINT_LIMITS_YAML)["joint_limits"];
  const YAML::Node ctrl = YAML::LoadFile(CONTROLLERS_YAML);
  const double update_rate = ctrl["arm_controller"]["ros__parameters"]["update_rate"].as<double>();
  ASSERT_GT(update_rate, 0.0);
  double max_v = 0.0;
  for (auto it = limits.begin(); it != limits.end(); ++it) {
    max_v = std::max(max_v, it->second["max_velocity"].as<double>());
  }
  // JTC の更新が 1 回遅れた最悪（2 更新ぶん）の 1.2 倍以上
  const double needed = 1.2 * max_v * 2.0 / update_rate;
  EXPECT_GE(kDefaultMaxJointSpeed * kDefaultLimitWindow + 1e-9, needed)
    << "max_joint_speed*limit_window_s must cover 1.2 * max_velocity * 2 / update_rate (" << needed << " rad)";
}

// ---- バス層の修正（docs/bus_layer_fix_plan.md）: 失敗時の待ち（最大 10 ms）が制御周期（約 20 ms）に与える影響の測定
// 1 サイクル = read + write。毎サイクルに故障を注入し、所要時間と「20 ms を超えたサイクルの数」を出す。
namespace
{
struct CycleStats
{
  double mean_ms{0}, max_ms{0};
  int over_20ms{0}, retried{0}, failed_cycles{0};
};

CycleStats measure_cycles(int quiet_ms, int scenario, int n = 100)
{
  Rig r;
  EXPECT_TRUE(r.init({{"timeout_ms", "10"}, {"drain_quiet_ms", std::to_string(quiet_ms)}}));
  EXPECT_EQ(r.configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(r.activate(), CallbackReturn::SUCCESS);
  if (scenario == 1) {  // 競合: id 1 が壊れ、id 2〜6 が失敗の後（1.5 ms 後）に届く。即時の再試行で成功する
    for (uint8_t id = 2; id <= 6; ++id) {r.servos.set_reply_delay_us(id, 1500);}
    r.servos.set_frame_spacing_us(80);
  }
  CycleStats st;
  double sum = 0;
  for (int i = 0; i < n; ++i) {
    if (scenario == 1) {r.servos.corrupt_read_replies(1, 56, 1);}
    if (scenario == 2) {r.servos.drop_read_replies(1, 56, 1);}   // 1 回目がタイムアウト、再試行で成功
    if (scenario == 3) {r.servos.drop_read_replies(1, 56, 2);}   // id 1 だけ 2 回応答しない（他は届く）
    const bool silent = (scenario == 4 && i % 2 == 1);          // 全サーボが黙る（本物のタイムアウト 2 回）
    if (silent) {r.servos.set_silent(true);}
    r.t += 0.02;
    const rclcpp::Time now(static_cast<int64_t>(r.t * 1e9), RCL_ROS_TIME);
    const auto period = rclcpp::Duration::from_seconds(0.02);
    const auto t0 = std::chrono::steady_clock::now();
    r.sys->read(now, period);
    r.sys->write(now, period);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (silent) {r.servos.set_silent(false);}
    sum += ms;
    st.max_ms = std::max(st.max_ms, ms);
    if (ms > 20.0) {++st.over_20ms;}
    std::this_thread::sleep_for(std::chrono::milliseconds(25));  // 次の周期まで（残りの応答・SYNC_WRITE を受け切る）
    r.servos.clear_reply_log();
  }
  st.mean_ms = sum / n;
  st.retried = static_cast<int>(r.sys->retried_total());
  st.failed_cycles = static_cast<int>(r.sys->failed_cycles_total());
  return st;
}
}  // namespace

TEST(BusFixCycleImpact, FailureWaitDoesNotBreakTheCycleBudgetBeyondTheExistingTimeouts)
{
  const char * names[] = {"normal", "race(corrupt+stale)", "id1 drop x1", "id1 drop x2", "all silent (odd)"};
  for (int sc = 0; sc < 5; ++sc) {
    const auto off = measure_cycles(0, sc);
    const auto on = measure_cycles(2, sc);
    printf(
      "[cycle] %-22s drain_quiet_ms=0: mean=%.2f max=%.2f over20=%d retried=%d failed=%d | "
      "drain_quiet_ms=2: mean=%.2f max=%.2f over20=%d retried=%d failed=%d\n",
      names[sc], off.mean_ms, off.max_ms, off.over_20ms, off.retried, off.failed_cycles,
      on.mean_ms, on.max_ms, on.over_20ms, on.retried, on.failed_cycles);
    if (sc == 0) {  // 正常な経路は変わらない
      EXPECT_EQ(on.over_20ms, 0);
      EXPECT_EQ(on.retried, 0);
      EXPECT_LT(on.mean_ms, off.mean_ms + 1.0);
    }
    if (sc == 1) {  // 競合: A・B ありなら、毎回の再試行が成功して周期を失わない
      EXPECT_EQ(on.failed_cycles, 0);
      EXPECT_EQ(on.over_20ms, 0);
      EXPECT_LT(on.max_ms, 10.0);
    }
    if (sc == 2) {EXPECT_EQ(on.failed_cycles, 0);}
    if (sc == 4) {  // 本物のタイムアウト: 待ちを足さない（従来と同じ所要時間）
      EXPECT_LT(on.max_ms, off.max_ms + 1.5);
      EXPECT_EQ(on.over_20ms, off.over_20ms);
    }
  }
}
