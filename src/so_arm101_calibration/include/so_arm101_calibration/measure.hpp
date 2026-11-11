// Copyright 2026 so_arm101 authors. MIT License.
//
// measure（測定）: 脱力 → 中央 → MIN/MAX を測って測定ファイルに保存する。**EEPROM には書かない。Lock にも触れない。**
// サーボに書くのは SRAM の Torque_Enable=0 だけ（トルクが入っているときだけ。腕が脱力する）。
//
// 式: Present_Position = Actual_Position − Homing_Offset（feetech.py _get_half_turn_homings の docstring）。
//   actual = (present + 現在の Homing_Offset) mod 4096。reset_calibration（0/0/4095 の中間書き込み）はしない。
//   新しい Homing_Offset = actual_center − 2047（LeRobot の set_half_turn_homings と同じ。int((4096−1)/2) = 2047）。
//   MIN/MAX は中央からの差 delta（±2048 に展開）で記録し、新座標 = 2047 + delta。wrist_roll のように 0/4095 を
//   またぐ関節も、中央から ±2047 以内なら連続した値になる。
#pragma once

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_ops.hpp"

namespace so_arm101_calibration
{

constexpr int kMeasureSchema = 1;
constexpr int kHalfTurn = 2047;     // int((4096 − 1) / 2)
constexpr int kMinSpanTicks = 200;  // 幅の下限（既定）。動かさなかった関節の検出

struct MeasuredJoint
{
  uint8_t id{0};
  std::string name;
  std::string kind;                 // yaml の kind（angle / linear_percent）
  int actual_center{0};             // 0..4095（中央姿勢の生のエンコーダ値）
  int measured_homing_offset{0};    // 測定した時点の EEPROM の Homing_Offset（符号付き）
  int new_homing_offset{0};         // actual_center − 2047
  int actual_min{0}, actual_max{0};  // 展開した actual（0..4095 の外もありうる）
  int new_min{0}, new_max{0};       // 新座標（0..4095 に収まるべき）
  double new_center{0.0};           // (new_min + new_max) / 2
  int samples{0};
  bool valid{false};
  std::string problems;             // 検査に落ちた理由（；区切り）
};

struct MeasureFile
{
  int schema_version{kMeasureSchema};
  std::string created, tool, port, backup_file;
  std::vector<MeasuredJoint> joints;
  const MeasuredJoint * find(uint8_t id) const;
};

/// 検査して valid / problems を設定する（|Homing_Offset| ≤ 2047、0 ≤ min < max ≤ 4095、幅 ≥ min_span）。
void check_measured_joint(MeasuredJoint * j, int min_span = kMinSpanTicks);

uint64_t measure_checksum(const MeasureFile & m);
std::string measure_to_json(const MeasureFile & m);
bool parse_measure(const std::string & text, MeasureFile * out, std::string * err);
bool load_measure_file(const std::string & path, MeasureFile * out, std::string * err);
bool save_measure_file(const std::string & path, const MeasureFile & m, std::string * err);

/// 生の Present_Position（デコード済み）と Homing_Offset から actual（0..4095）。
int actual_from_present(int present, int homing_offset);
/// 中央からの差を ±2048 に展開する。
int unwrap_delta(int actual, int actual_center);

/// 対話の部品。試験ではこれを差し替える。
struct MeasureIO
{
  std::function<bool(const std::string &)> wait_enter;  // 表示して Enter を待つ。false なら中止
  std::function<bool()> should_stop;                    // 記録中に呼ぶ。true で記録を終える
  std::function<void()> on_tick;                        // 記録の 1 周期ごと（試験で姿勢を動かす）
  std::function<void(int)> on_center_sample;            // 中央の k 回目の読み取りの前（試験で姿勢を動かす）
  int tick_ms{20};
  int center_samples{11};
  int center_tolerance_ticks{4};
};

/// 測定。戻り値は ExitCode: 0 = 全関節の検査が通った / 3 = 前提の失敗 / 4 = 中央が安定しない・検査の不合格
/// （4 のときも out に結果は入る。保存して原因を調べられる）。
int run_measure(
  CalibWriteBus & bus, const std::vector<JointId> & all, const std::vector<std::string> & kinds, const Options & opt,
  MeasureIO & io, std::ostream & out, MeasureFile * result);

/// 2 つの測定ファイルの再現性と、yaml との差を表示する（オフライン。シリアルポートを開かない）。
struct YamlJoint
{
  std::string kind;
  double center{0.0};
  int min{0}, max{0};
  int homing_offset{0};
};
void compare_measures(
  const MeasureFile & a, const MeasureFile & b, const std::vector<std::pair<std::string, YamlJoint>> & yaml,
  int angle_ticks, std::ostream & out);

}  // namespace so_arm101_calibration
