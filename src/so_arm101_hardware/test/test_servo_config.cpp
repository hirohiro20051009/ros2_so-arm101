// Copyright 2026 so_arm101 authors. MIT License.
// サーボ設定の比較と、不一致の扱い（read_only:=true は WARN のみ、false は中止）の試験。ハード不要。
#include <gtest/gtest.h>

#include "so_arm101_hardware/servo_config.hpp"

using namespace so_arm101_hardware;  // NOLINT

namespace
{
JointCalibration joint(JointKind k)
{
  JointCalibration j;
  j.kind = k;
  return j;
}

ServoSettings lerobot_arm()
{
  ServoSettings s;
  s.operating_mode = 0; s.p = 16; s.i = 0; s.d = 32; s.return_delay = 0;
  return s;
}

ServoSettings lerobot_gripper()
{
  ServoSettings s = lerobot_arm();
  s.max_torque_limit = 500; s.protection_current = 250; s.overload_torque = 25;
  return s;
}
}  // namespace

TEST(ServoConfig, ExpectedValuesAreLeRobotConfigure)
{
  // so101_follower.py configure(): mode 0, P 16, I 0, D 32; configure_motors(): Return_Delay_Time 0;
  // gripper: Max_Torque_Limit 500, Protection_Current 250, Overload_Torque 25
  const auto arm = expected_servo_config(joint(JointKind::Angle));
  EXPECT_EQ(arm.operating_mode, 0);
  EXPECT_EQ(arm.p, 16);
  EXPECT_EQ(arm.i, 0);
  EXPECT_EQ(arm.d, 32);
  EXPECT_EQ(arm.return_delay, 0);
  EXPECT_FALSE(arm.gripper_limits);
  const auto g = expected_servo_config(joint(JointKind::LinearPercent));
  EXPECT_TRUE(g.gripper_limits);
  EXPECT_EQ(g.max_torque_limit, 500);
  EXPECT_EQ(g.protection_current, 250);
  EXPECT_EQ(g.overload_torque, 25);
}

TEST(ServoConfig, MatchGivesNoDiffs)
{
  EXPECT_TRUE(compare_servo_config(expected_servo_config(joint(JointKind::Angle)), lerobot_arm()).empty());
  EXPECT_TRUE(compare_servo_config(expected_servo_config(joint(JointKind::LinearPercent)), lerobot_gripper()).empty());
}

TEST(ServoConfig, ArmIgnoresGripperOnlyRegisters)
{
  ServoSettings s = lerobot_arm();
  s.max_torque_limit = 1000;  // 腕の既定（LeRobot は腕には書かない）
  s.protection_current = 500;
  s.overload_torque = 80;
  EXPECT_TRUE(compare_servo_config(expected_servo_config(joint(JointKind::Angle)), s).empty());
}

TEST(ServoConfig, GripperChecksItsLimits)
{
  ServoSettings s = lerobot_gripper();
  s.max_torque_limit = 1000;  // 既定のまま
  const auto d = compare_servo_config(expected_servo_config(joint(JointKind::LinearPercent)), s);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].name, "Max_Torque_Limit");
  EXPECT_EQ(d[0].servo, 1000);
  EXPECT_EQ(d[0].expected, 500);
}

TEST(ServoConfig, DiffsAreReportedWithNames)
{
  ServoSettings s = lerobot_arm();
  s.p = 32;            // 工場出荷の P
  s.return_delay = 250;
  const auto d = compare_servo_config(expected_servo_config(joint(JointKind::Angle)), s);
  ASSERT_EQ(d.size(), 2u);
  const std::string text = to_string(d);
  EXPECT_NE(text.find("P_Coefficient(servo 32, expected 16)"), std::string::npos) << text;
  EXPECT_NE(text.find("Return_Delay_Time(servo 250, expected 0)"), std::string::npos) << text;
  EXPECT_FALSE(has_operating_mode_diff(d));
}

TEST(ServoConfig, OperatingModeDiffIsDetected)
{
  ServoSettings s = lerobot_arm();
  s.operating_mode = 1;  // 速度制御
  const auto d = compare_servo_config(expected_servo_config(joint(JointKind::Angle)), s);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_TRUE(has_operating_mode_diff(d));
}

TEST(ServoConfig, MismatchOnlyWarnsInReadOnlyAndAbortsWhenWriting)
{
  ServoSettings s = lerobot_arm();
  s.p = 32;
  const auto d = compare_servo_config(expected_servo_config(joint(JointKind::Angle)), s);
  EXPECT_EQ(config_action(d, /*read_only=*/true), ConfigAction::Warn);     // 2b-3a・リーダーアーム
  EXPECT_EQ(config_action(d, /*read_only=*/false), ConfigAction::Abort);   // 書き込みモードは起動しない
  // Operating_Mode が 0 以外でも同じ（read_only は WARN のみ、書き込みモードは中止）
  s = lerobot_arm();
  s.operating_mode = 1;
  const auto m = compare_servo_config(expected_servo_config(joint(JointKind::Angle)), s);
  EXPECT_EQ(config_action(m, true), ConfigAction::Warn);
  EXPECT_EQ(config_action(m, false), ConfigAction::Abort);
  // 一致ならどちらも Ok
  EXPECT_EQ(config_action({}, true), ConfigAction::Ok);
  EXPECT_EQ(config_action({}, false), ConfigAction::Ok);
}

// ---- 段階2b-3b: apply_servo_config ----
TEST(ConfigAction, ApplyOnlyInWriteModeAndNeverForOperatingMode)
{
  const std::vector<ConfigDiff> p_only{{"P_Coefficient", 32, 16}};
  const std::vector<ConfigDiff> with_mode{{"Operating_Mode", 1, 0}, {"P_Coefficient", 32, 16}};
  EXPECT_EQ(config_action(p_only, /*read_only=*/false, /*apply=*/true), ConfigAction::Apply);
  EXPECT_EQ(config_action(p_only, false, false), ConfigAction::Abort);          // 既定は拒否
  EXPECT_EQ(config_action(with_mode, false, true), ConfigAction::Abort);       // Operating_Mode は書かない
  EXPECT_EQ(config_action(p_only, true, true), ConfigAction::Warn);            // read_only は書かない
  EXPECT_EQ(config_action({}, false, true), ConfigAction::Ok);
}
