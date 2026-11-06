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

enum class JointKind
{
  Angle,          // LeRobot の DEGREES
  LinearPercent,  // LeRobot の RANGE_0_100（gripper）
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
/// Angle: ((raw − center)·360 / angle_ticks) 度 → rad（clamp なし）。
/// LinearPercent: clamp(raw, min, max) → 0..100% → rad = radians(%)。
/// 演算の順序は LeRobot の _normalize と math.radians に合わせている（浮動小数まで一致させるため）。
double raw_to_rad(const JointCalibration & j, int angle_ticks, int raw);

/// 1 tick に相当する rad（診断・比較用）。Angle: 2π/angle_ticks、LinearPercent: radians(100/(max−min))。
double tick_in_rad(const JointCalibration & j, int angle_ticks);

}  // namespace so_arm101_hardware
