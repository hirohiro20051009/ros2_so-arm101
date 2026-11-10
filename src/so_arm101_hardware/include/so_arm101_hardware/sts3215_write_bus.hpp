// Copyright 2026 so_arm101 authors. MIT License.
//
// Feetech STS3215 の書き込みバス（段階2b-3b）。読み取り専用の Sts3215Bus を継承し、WRITE(0x03) と
// SYNC_WRITE(0x83) を足す。read_only:=true のドライバはこのクラスを生成しない。
//
// 公開する書き込みは用途別の関数だけ。任意アドレス・任意値の汎用 write は公開しない
// （レジスタと値の範囲を関数ごとに固定する）。EEPROM に書くのは write_servo_setting() の allow-list
// （P/I/D、Return_Delay_Time、gripper の 3 項目）だけで、Operating_Mode は書けない。
// パケット形式は scservo_sdk（protocol_packet_handler.py の writeTxRx / syncWriteTxOnly）と同じ。出典は docs/third_party.md。
#pragma once

#include <vector>

#include "so_arm101_hardware/sts3215_bus.hpp"

namespace so_arm101_hardware
{

enum class WriteInstruction : uint8_t
{
  Write = 0x03,      // INST_WRITE（応答のステータスパケットあり）
  SyncWrite = 0x83,  // INST_SYNC_WRITE（応答なし）
};

/// 書き込みを許す EEPROM の項目（allow-list）。
enum class ServoSetting
{
  P,
  I,
  D,
  ReturnDelayTime,
  MaxTorqueLimit,
  ProtectionCurrent,
  OverloadTorque,
};
Register register_of(ServoSetting s);
const char * to_string(ServoSetting s);

constexpr int kMaxTorqueLimitValue = 1000;  // Torque_Limit の上限（Max_Torque_Limit の最大）
constexpr int kMaxAccelerationValue = 254;

class Sts3215WriteBus : public Sts3215Bus
{
public:
  using Sts3215Bus::Sts3215Bus;

  // --- 書き込み（応答を確認する。ERR バイトがあれば ServoError）---
  Status write_torque_enable(uint8_t id, bool on);
  Status write_lock(uint8_t id, bool locked);
  Status write_acceleration(uint8_t id, int value);       // 0..254（0 は書かせない: 呼び出し側が 1 以上を渡す）
  Status write_torque_limit(uint8_t id, int value);       // 0..1000
  Status write_goal_position(uint8_t id, int raw);        // sign-magnitude bit15
  Status write_servo_setting(uint8_t id, ServoSetting s, int value);  // EEPROM。Lock=0・トルク OFF が前提

  /// 全関節の Goal_Position を 1 回の SYNC_WRITE で送る（応答なし）。
  Status sync_write_goal_position(const std::vector<uint8_t> & ids, const std::vector<int> & raws);

  // --- テスト用に公開 ---
  static std::vector<uint8_t> build_write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);
  static std::vector<uint8_t> build_sync_write(
    uint8_t address, uint8_t data_len, const std::vector<uint8_t> & ids,
    const std::vector<std::vector<uint8_t>> & data);

private:
  Status write_bytes(uint8_t id, Register r, int value, int min_value, int max_value);
};

}  // namespace so_arm101_hardware
