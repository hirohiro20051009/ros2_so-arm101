// Copyright 2026 so_arm101 authors. MIT License.
//
// EEPROM の読み書きの共通部品（restore と、段階2c-2 の apply が使う）。
//  - 再試行は通信エラー（Timeout / BadFrame / BadChecksum）のときだけ。値の不一致はすぐ失敗にする。
//  - EEPROM に書いた後は wait_ms だけ待ってから読み戻す。
#pragma once

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"

namespace so_arm101_calibration
{

/// 終了コード（計画 docs/stage2c_plan.md 2 章）。
enum ExitCode
{
  kExitOk = 0,           // 全成功
  kExitFailedAllOld = 1,  // 失敗したが、全関節が実行前の値で Lock=1 に戻っている
  kExitPartial = 2,      // 一部が実行前でも目標でもない／目標のまま残った、または Lock=1 を確認できなかった
  kExitPrecondition = 3,  // 前提の確認の失敗（何も書いていない）
  kExitInput = 4,        // 入力・検査の不合格、確認文字列の不一致（何も書いていない）
};

struct Options
{
  int wait_ms{50};   // EEPROM 書き込み後、読み戻すまでの待ち（仮置き。C0.5 で実測して決める）
  int retries{3};    // 通信エラーのときの再試行回数
  bool rewrite_same{false};  // 同じ値でも書く（復元経路の実機確認用）
};
constexpr int kMaxWaitMs = 1000;

struct JointId
{
  uint8_t id;
  std::string name;
};

/// 1 台の 3 レジスタの生の値（読めなければ nullopt）。
struct JointRegs
{
  std::optional<uint16_t> v[kNumCalibRegs];
};

so_arm101_hardware::Status read_calib_reg(
  so_arm101_hardware::Sts3215Bus & bus, uint8_t id, CalibReg r, int retries, uint16_t * out);
JointRegs read_joint_regs(so_arm101_hardware::Sts3215Bus & bus, uint8_t id, int retries);
/// Lock の現在値（読めなければ -1）。
int read_lock(so_arm101_hardware::Sts3215Bus & bus, uint8_t id, int retries);

struct StepResult
{
  bool ok{true};
  std::string msg;
};
/// 書く → wait_ms 待つ → 読み戻して一致を確認。書き込みの通信エラーは同じ値を再送、読み戻しの通信エラーは再読み。
/// 値の不一致、通信エラーの再試行切れ、サーボのエラー応答は失敗。
StepResult write_calib_verified(CalibWriteBus & bus, uint8_t id, CalibReg r, uint16_t raw, const Options & opt);
/// Lock を書いて読み戻す（待ちなし。SRAM）。
StepResult set_lock_verified(CalibWriteBus & bus, uint8_t id, bool locked, const Options & opt);

/// 全関節: 機種番号 777 と Torque_Enable=0 を確認する。満たさなければ false（理由を out へ）。
bool preflight(
  so_arm101_hardware::Sts3215Bus & bus, const std::vector<JointId> & all, int retries, std::ostream & out);

/// 1 レジスタの状態。target と一致なら "target"、before と一致なら "before"（両方なら "same"）、他は "unknown"。
std::string classify(const std::optional<uint16_t> & cur, uint16_t before, uint16_t target);
/// 表示用の値（符号付きの Homing_Offset も添える）。
std::string show_value(CalibReg r, const std::optional<uint16_t> & v);

}  // namespace so_arm101_calibration
