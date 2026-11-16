// Copyright 2026 so_arm101 authors. MIT License.
//
// apply: 測定ファイルの値を EEPROM（Homing_Offset と Min/Max_Position_Limit）へ書く。
//  - 既定は dry-run。書くのは write=true かつ confirm() が true のときだけ。
//  - 書く前に: 全関節の機種番号 777・Torque_Enable=0、今の EEPROM がバックアップと一致していること、
//    測定ファイルが検査済み（valid）で、測定時の Homing_Offset が今の EEPROM と一致していること。
//  - 途中で失敗したら、その回で書いた全関節をバックアップの値に戻そうとする。戻せなかった項目は
//    関節ごと・レジスタごとに 旧/新/不明 を表示し、restore のコマンド例を出して非ゼロで終了する。
//  - yaml を更新するのは、全関節を 1 回の適用で書き、読み戻しを確認した後の 1 回だけ（一部の関節だけなら更新しない）。
#pragma once

#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"
#include "so_arm101_calibration/eeprom_ops.hpp"
#include "so_arm101_calibration/measure.hpp"

namespace so_arm101_calibration
{

/// 補正値 zero_offset_deg を持つ関節の center がこの tick 数以上動いたら、apply が警告を出す（約 0.9°）。
/// 補正値は古い center との組なので、再確認を促す。docs/joint_offset_plan.md 2.2・2.3
constexpr double kCenterShiftWarnTicks = 10.0;

struct ApplyArgs
{
  MeasureFile measure;
  std::string measure_path;
  Backup backup;
  std::string backup_path;
  std::vector<JointId> all;
  std::vector<uint8_t> selected;   // 空なら全関節
  Options opt;
  bool write{false};
  std::string yaml_path;           // 読んで更新する yaml（シンボリックリンクは解決して、実体を更新する）
  std::string fallback_yaml_path;  // yaml を更新できなかったときの退避先
};

int run_apply(
  CalibWriteBus & bus, const ApplyArgs & args, const std::function<bool()> & confirm, std::ostream & out);

/// 更新後の yaml のテキスト（従来の yaml と同じ形式。direction・bus・conversion と、キーがあった関節の
/// zero_offset_deg は元の yaml からそのまま引き継ぐ。キーが無かった関節にはキーを足さない）。
std::string render_calibration_yaml(
  const std::string & old_yaml_path, const MeasureFile & m, const std::string & backup_path,
  const std::string & measure_path, const std::string & applied_at);

/// yaml を実体の場所に、一時ファイル → 読み直して検査（全関節の zero_offset_deg が古い yaml と同じことを含む）
/// → rename で書く。
bool update_calibration_yaml(const std::string & yaml_path, const std::string & text, std::string * err);

}  // namespace so_arm101_calibration
