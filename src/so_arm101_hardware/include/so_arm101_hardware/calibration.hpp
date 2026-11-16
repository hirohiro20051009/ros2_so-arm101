// Copyright 2026 so_arm101 authors. MIT License.
//
// 自前キャリブレーション yaml の読み込みと raw→rad 変換。
// yaml の形式と意味は docs/inventory.md「段階2b-1」。段階2c では同じ形式の中身を差し替える。
//
// 変換式は LeRobot 0.4.2 (Apache-2.0, Copyright 2024 The HuggingFace Inc. team) の
//   lerobot/motors/motors_bus.py  MotorsBus._normalize()
// （angle: MotorNormMode.DEGREES、gripper: MotorNormMode.RANGE_0_100）と、
// bridge の math.radians(値) を再現する。出典は docs/third_party.md。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace so_arm101_hardware
{

/// zero_offset_deg の絶対値の上限（度）。打ち間違いを止めるための値（elbow で要るのは約 6.3°）。
constexpr double kMaxZeroOffsetDeg = 10.0;

enum class JointKind
{
  Angle,          // 基準の実装（LeRobot 0.4.2）の DEGREES
  LinearPercent,  // 基準の実装の RANGE_0_100（gripper）
};

/// 起動時診断専用。変換には使わない。
struct EepromExpectation
{
  int homing_offset{0};
  int min_limit{0};
  int max_limit{0};
};

struct JointCalibration
{
  std::string name;
  uint8_t id{0};
  JointKind kind{JointKind::Angle};
  int direction{1};      // +1 / -1
  double center{0.0};    // Angle のみ
  // Angle のみ。角度の補正値（度）。真の角度 = 換算の出力 + zero_offset_deg（direction を掛けた後に足す）。
  // yaml の任意キー zero_offset_deg（既定 0）。LinearPercent では 0 以外を許さない。docs/joint_offset_plan.md
  double zero_offset_deg{0.0};
  int raw_min{0};
  int raw_max{0};
  EepromExpectation eeprom;
};

struct Calibration
{
  int schema_version{0};
  int baudrate{1000000};
  int protocol{0};
  int angle_ticks{4095};
  std::string source;  // ログ用の変換元の説明
  std::vector<JointCalibration> joints;

  const JointCalibration * find(const std::string & joint_name) const;
};

/// yaml を読む。形式が不正なら std::runtime_error（メッセージに原因）。
Calibration load_calibration(const std::string & path);

/// raw（符号デコード済みの Present_Position）→ rad。
/// Angle: (direction·(raw − center)·360 / angle_ticks + zero_offset_deg) 度 → rad（clamp なし）。
/// zero_offset_deg は LeRobot の式にない項（本リポジトリの拡張）。既定 0 で LeRobot の式と同じ値。
/// LinearPercent: clamp(raw, min, max) → 0..100% → rad = radians(%)。
/// 演算の順序は基準の実装（LeRobot 0.4.2）の _normalize と math.radians に合わせている（浮動小数まで一致させるため）。
double raw_to_rad(const JointCalibration & j, int angle_ticks, int raw);

/// rad → raw（Goal_Position 用）。
/// raw_to_rad の逆（Angle は度から zero_offset_deg を引いてから direction を掛ける）。基準の実装（LeRobot 0.4.2）の
/// MotorsBus._unnormalize() と同じ式（angle: val·ticks/360 + center、
/// gripper: 0..100 に clamp して min..max へ）だが、**最後の丸めだけ四捨五入**にしている
/// （基準の実装は int() で切り捨て。差は常に 0 か +1 tick。raw→rad→raw が恒等になり、
/// 保持指令が 1 tick ずれるのを避けるため）。Angle は clamp しない（限界の clamp は呼び出し側）。
int rad_to_raw(const JointCalibration & j, int angle_ticks, double rad);

/// rad_to_raw の途中値（丸める前の連続値）。基準の実装の切り捨てとの比較試験用。
double rad_to_raw_continuous(const JointCalibration & j, int angle_ticks, double rad);

/// 限界外の指令を「異常」と数えない許容（tick）。low は raw_min の下側、high は raw_max の上側。
/// zero_offset_deg で到達できる範囲が縮む側だけ、縮んだ分を広げる:
///   s = −direction·zero_offset_deg·angle_ticks/360（補正で rad_to_raw の出力が動く tick 数）
///   s > 0 なら high = base + ceil(s)、s < 0 なら low = base + ceil(−s)、s = 0 なら両方 base。
/// ceil は広げる向きの丸め。LinearPercent は常に base / base。docs/joint_offset_plan.md 4 章（案 B）
struct ClampTolerance
{
  int low{0};
  int high{0};
};
ClampTolerance clamp_tolerance_ticks(const JointCalibration & j, int angle_ticks, int base);

/// 1 tick に相当する rad（診断・比較用）。Angle: 2π/angle_ticks、LinearPercent: radians(100/(max−min))。
double tick_in_rad(const JointCalibration & j, int angle_ticks);

}  // namespace so_arm101_hardware
