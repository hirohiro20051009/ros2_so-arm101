// Copyright 2026 so_arm101 authors. MIT License.
//
// キャリブレーション用の書き込みバス（段階2c）。Sts3215WriteBus を継承し、EEPROM の
// Homing_Offset と Min/Max_Position_Limit だけを書く関数を足す（allow-list。任意アドレスの書き込みは公開しない）。
// ros2_control のドライバ（libso_arm101_sts3215）にはこの書き込みは無い。このクラスは so_arm101_calibration だけが持つ。
//
// 値は「サーボに書く生の 16 bit」で渡す（Homing_Offset は sign-magnitude 符号化済み。復元ではバックアップの
// バイト列をそのまま書くため、符号化し直さない）。
#pragma once

#include <cstdint>

#include "so_arm101_hardware/sts3215_write_bus.hpp"

namespace so_arm101_calibration
{

enum class CalibReg
{
  HomingOffset = 0,
  MinPositionLimit = 1,
  MaxPositionLimit = 2,
};
constexpr int kNumCalibRegs = 3;

so_arm101_hardware::Register register_of(CalibReg r);
const char * to_string(CalibReg r);
/// 生の値として書ける最大値。Homing_Offset は 12 bit（bit11 が符号。0..0x0FFF）、limit は 0..4095。
int max_raw_of(CalibReg r);

class CalibWriteBus : public so_arm101_hardware::Sts3215WriteBus
{
public:
  using so_arm101_hardware::Sts3215WriteBus::Sts3215WriteBus;

  /// EEPROM の 1 レジスタへ書く（Lock=0・トルク OFF が前提。呼び出し側が確認する）。範囲外は書かずに BadFrame。
  so_arm101_hardware::Status write_calib_raw(uint8_t id, CalibReg r, uint16_t raw);
};

}  // namespace so_arm101_calibration
