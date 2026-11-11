// Copyright 2026 so_arm101 authors. MIT License.
//
// EEPROM のバックアップ（アドレス 0〜39 の生バイト）の読み出し・保存・読み込み・検査。
// 読み出しだけでサーボには何も書かない（Sts3215Bus のみ使う）。
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_hardware/sts3215_bus.hpp"

namespace so_arm101_calibration
{

constexpr int kEepromBytes = 40;      // アドレス 0〜39 が EEPROM（tables.py の区分。実機で未確認の推定）
constexpr int kReadChunk = 8;         // 一度に読むバイト数（1 回の READ を短くする）
constexpr int kBackupSchema = 1;

struct JointEeprom
{
  uint8_t id{0};
  std::string name;
  uint16_t model{0};
  std::array<uint8_t, kEepromBytes> bytes{};

  uint16_t u16_at(int addr) const {return static_cast<uint16_t>(bytes[addr] | (bytes[addr + 1] << 8));}
  /// 生の値（Homing_Offset は sign-magnitude 符号化されたまま）。
  uint16_t raw(CalibReg r) const {return u16_at(register_of(r).address);}
  int homing_offset() const;  // デコード済み（符号付き）
};

struct Backup
{
  int schema_version{kBackupSchema};
  std::string created;   // ISO 8601（ローカル時刻）
  std::string port;
  std::string tool;
  std::vector<JointEeprom> joints;

  const JointEeprom * find(uint8_t id) const;
};

/// 全バイトの FNV-1a 64 bit（id と 40 バイトを順に）。保存・読み込みの破損検出用（暗号ではない）。
uint64_t bytes_checksum(const Backup & b);

std::string to_json(const Backup & b);
/// JSON（YAML のサブセットとして読む）を厳密に検査して読む。壊れている・欠けている・範囲外・チェックサム不一致なら false。
bool parse_backup(const std::string & text, Backup * out, std::string * err);
bool load_backup_file(const std::string & path, Backup * out, std::string * err);
/// 一時ファイルに書いて rename し、読み直して内容が一致することを確認する。
bool save_backup_file(const std::string & path, const Backup & b, std::string * err);

/// 復元に使えるか（機種番号 777、ID とバイト列の ID が一致、Homing_Offset/limit が書ける範囲）。使えなければ理由。
std::string validate_for_restore(const JointEeprom & j);

/// 1 台の EEPROM 0〜39 を読む。通信エラーのときだけ retries 回まで再試行。値は検査しない。
bool read_joint_eeprom(
  so_arm101_hardware::Sts3215Bus & bus, uint8_t id, const std::string & name, int retries, JointEeprom * out,
  std::string * err);

/// ISO 8601 のローカル時刻（ファイル名用は fmt_file=true: 20261111-153000）。
std::string now_string(bool fmt_file);

}  // namespace so_arm101_calibration
