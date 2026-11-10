// Copyright 2026 so_arm101 authors. MIT License.
//
// サーボの設定（EEPROM）が LeRobot の値と一致しているかの判定（段階2b-3a。読み取りだけ）。
//
// 期待値は LeRobot 0.4.2 (Apache-2.0, Copyright 2024 The HuggingFace Inc. team) が書く値:
//   lerobot/robots/so101_follower/so101_follower.py  SO101Follower.configure()（149-171 行）
//   lerobot/motors/feetech/feetech.py                FeetechMotorsBus.configure_motors()（222-230 行）
// 出典と対応は docs/third_party.md。値は docs/inventory.md「段階2b-3 の計画 v3」の表。
//
// 不一致のときの扱い（承認済み）:
//   read_only:=true  → WARN のみで起動を続ける（2b-3a の確認、将来のリーダーアーム）
//   read_only:=false → 起動を中止する（2b-3b。apply_servo_config:=true のときだけ書く）
#pragma once

#include <string>
#include <vector>

#include "so_arm101_hardware/calibration.hpp"

namespace so_arm101_hardware
{

/// 診断で読んだ、比較対象の設定値。
struct ServoSettings
{
  int operating_mode{-1};
  int p{-1}, i{-1}, d{-1};
  int return_delay{-1};
  int max_torque_limit{-1};
  int protection_current{-1};
  int overload_torque{-1};
};

/// 期待する設定値。gripper_limits が true の関節だけ、後ろの 3 項目も比べる。
struct ExpectedServoConfig
{
  int operating_mode{0};  // 位置制御
  int p{16}, i{0}, d{32};
  int return_delay{0};
  bool gripper_limits{false};
  int max_torque_limit{500};
  int protection_current{250};
  int overload_torque{25};
};

/// 関節ごとの期待値。LinearPercent（gripper）だけ gripper_limits を true にする。
ExpectedServoConfig expected_servo_config(const JointCalibration & joint);

struct ConfigDiff
{
  std::string name;  // "Operating_Mode", "P_Coefficient", ...
  int servo{0};
  int expected{0};
};

/// 期待値との差を返す（一致なら空）。
std::vector<ConfigDiff> compare_servo_config(const ExpectedServoConfig & expected, const ServoSettings & actual);

/// 差のある項目を "P_Coefficient(servo 32, expected 16) ..." の形にする。
std::string to_string(const std::vector<ConfigDiff> & diffs);

enum class ConfigAction
{
  Ok,     // 一致
  Warn,   // 不一致だが続ける（read_only）
  Abort,  // 不一致なので起動を中止する（書き込みモード）
  Apply,  // 不一致だが apply_servo_config:=true なので、起動時に書き直す（書き込みモード。Operating_Mode 以外）
};

/// 不一致の扱い。書き込みモードでは、Operating_Mode を含むどの差でも Abort（トルクを ON にしない）。
/// apply_servo_config が true で、差に Operating_Mode が無ければ Apply（書き込みモードのみ）。
ConfigAction config_action(const std::vector<ConfigDiff> & diffs, bool read_only, bool apply_servo_config = false);

/// Operating_Mode の差が含まれるか。
bool has_operating_mode_diff(const std::vector<ConfigDiff> & diffs);

}  // namespace so_arm101_hardware
