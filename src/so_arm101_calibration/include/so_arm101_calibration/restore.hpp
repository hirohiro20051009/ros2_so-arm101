// Copyright 2026 so_arm101 authors. MIT License.
//
// restore: バックアップから Homing_Offset と Min/Max_Position_Limit を戻す。
// 既定は dry-run（write=false）。書くのは write=true かつ confirm() が true のときだけ。
// 書く前に: 全関節の機種番号 777・Torque_Enable=0 を確認する（満たさなければ何も書かない）。
#pragma once

#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"
#include "so_arm101_calibration/eeprom_ops.hpp"

namespace so_arm101_calibration
{

struct RestoreArgs
{
  Backup backup;
  std::string backup_path;           // 失敗時の表示用（restore のコマンド例）
  std::vector<JointId> all;          // yaml の全関節（前提の確認の対象）
  std::vector<uint8_t> selected;     // 復元する関節の ID（空なら全関節）
  Options opt;
  bool write{false};
};

/// 戻り値は終了コード（ExitCode）。
int run_restore(
  CalibWriteBus & bus, const RestoreArgs & args, const std::function<bool()> & confirm, std::ostream & out);

}  // namespace so_arm101_calibration
