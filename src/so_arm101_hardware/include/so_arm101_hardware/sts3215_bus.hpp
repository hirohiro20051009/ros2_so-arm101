// Copyright 2026 so_arm101 authors. MIT License.
//
// Feetech STS3215 (プロトコル 0) の読み取り専用バス。
//
// 書き込み系の命令（WRITE / REG_WRITE / ACTION / SYNC_WRITE）は意図的に実装していない。
// Instruction 型に存在しないので、このクラスからレジスタに書き込むことはコンパイル時にできない
// （トルクにも触れない）。書き込みは段階2b-3 で別途追加する。
//
// パケット形式・命令コード・レジスタ表は LeRobot 0.4.2 (Apache-2.0, Copyright 2024 The
// HuggingFace Inc. team) が使う Feetech の仕様に基づく。出典は docs/third_party.md。
//   lerobot/motors/feetech/tables.py        STS_SMS_SERIES_CONTROL_TABLE, STS_SMS_SERIES_ENCODINGS_TABLE
//   scservo_sdk/protocol_packet_handler.py  txPacket / rxPacket / syncReadTx
//   scservo_sdk/scservo_def.py              INST_*, BROADCAST_ID
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "so_arm101_hardware/serial_port.hpp"

namespace so_arm101_hardware
{

/// このバスが送れる命令。書き込み系は定義しない。
enum class Instruction : uint8_t
{
  Read = 0x02,      // INST_READ
  SyncRead = 0x82,  // INST_SYNC_READ
};

/// STS3215 のレジスタ（アドレス, バイト数）。tables.py:41-101。読み取りに使うものだけ。
struct Register
{
  uint8_t address;
  uint8_t length;
};
namespace reg
{
constexpr Register kModelNumber{3, 2};
constexpr Register kReturnDelayTime{7, 1};
constexpr Register kMinPositionLimit{9, 2};
constexpr Register kMaxPositionLimit{11, 2};
constexpr Register kHomingOffset{31, 2};     // 符号付き: sign-magnitude, bit11
constexpr Register kPresentPosition{56, 2};  // 符号付き: sign-magnitude, bit15

// 段階2b-3a: 起動時診断で「読む」だけのレジスタ（tables.py:41-101）。書き込みは 2b-3b で別クラスに追加する。
// アドレス 39 以下は EEPROM、40 以降は SRAM（tables.py の "# EPROM" / "# SRAM" の区分。推定。実機で未確認）。
constexpr Register kMaxTorqueLimit{16, 2};       // EEPROM（最大 1000）
constexpr Register kPCoefficient{21, 1};         // EEPROM
constexpr Register kDCoefficient{22, 1};         // EEPROM
constexpr Register kICoefficient{23, 1};         // EEPROM
constexpr Register kProtectionCurrent{28, 2};    // EEPROM（最大 500）
constexpr Register kOperatingMode{33, 1};        // EEPROM（0 = 位置制御）
constexpr Register kOverloadTorque{36, 1};       // EEPROM
constexpr Register kTorqueEnable{40, 1};         // SRAM
constexpr Register kAcceleration{41, 1};         // SRAM
constexpr Register kGoalVelocity{46, 2};         // SRAM, 符号付き: sign-magnitude, bit15
constexpr Register kTorqueLimit{48, 2};          // SRAM
constexpr Register kLock{55, 1};                 // SRAM
constexpr Register kPresentVoltage{62, 1};       // 0.1 V
constexpr Register kPresentTemperature{63, 1};   // ℃
constexpr Register kStatus{65, 1};
constexpr Register kMaximumAcceleration{85, 1};  // Factory 表（区分は不明）
}  // namespace reg
constexpr int kHomingOffsetSignBit = 11;
constexpr int kPositionSignBit = 15;
constexpr int kGoalVelocitySignBit = 15;
constexpr uint16_t kModelNumberSts3215 = 777;  // tables.py MODEL_NUMBER_TABLE["sts3215"]

enum class Status
{
  Ok,
  Timeout,      // 応答が時間内に揃わない
  BadChecksum,  // チェックサム不一致
  BadFrame,     // ヘッダ/長さが不正
  IdMismatch,   // 想定外の ID が返った
  ServoError,   // ステータスの ERR バイトが 0 でない
  PortError,    // 送信失敗/ポート未オープン
};
const char * to_string(Status s);

/// sign-magnitude を int に。bit が立っていれば負（encoding_utils.py decode_sign_magnitude）。
int decode_sign_magnitude(uint16_t encoded, int sign_bit);

class Sts3215Bus
{
public:
  /// timeout は 1 つのステータスパケットを待つ時間。
  explicit Sts3215Bus(SerialPort & port, std::chrono::microseconds timeout = std::chrono::milliseconds(10))
  : port_(port), timeout_(timeout) {}

  /// 1 台から読む。data に length バイト（リトルエンディアンの生バイト）。
  Status read(uint8_t id, Register r, std::vector<uint8_t> * data);

  /// 1 台から 2 バイトの符号なし値を読む。
  Status read_u16(uint8_t id, Register r, uint16_t * value);

  /// 複数台から同じレジスタを SYNC_READ で読む。out[i] は ids[i] の生バイト。
  /// 失敗した場合 failed_id にその ID（不明なら 0）を入れ、受信バッファを捨てる。
  Status sync_read(
    const std::vector<uint8_t> & ids, Register r, std::vector<std::vector<uint8_t>> * out,
    uint8_t * failed_id = nullptr);

  /// 直近の失敗の説明（ログ用）。
  const std::string & last_error() const {return last_error_;}

  // --- テスト用に公開 ---
  static uint8_t checksum(const uint8_t * body, size_t len);  // ~sum(ID..PARAM) & 0xFF
  static std::vector<uint8_t> build_request(
    uint8_t id, Instruction inst, const std::vector<uint8_t> & params);

private:
  Status receive_status(uint8_t expect_id, size_t data_len, std::vector<uint8_t> * data);
  Status fail(Status s, const std::string & msg);

  SerialPort & port_;
  std::chrono::microseconds timeout_;
  std::string last_error_;
  int last_rx_id_{-1};  // 直近に受信したパケットの ID（IdMismatch の分類用）
};

}  // namespace so_arm101_hardware
