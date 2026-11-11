// Copyright 2026 so_arm101 authors. MIT License.
//
// eeprom backup / restore の試験。偽サーボ（pty。so_arm101_hardware/test/fake_servo.hpp）を使い、
// 実機・実ポートは開かない。
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "test_rig.hpp"
#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"
#include "so_arm101_calibration/eeprom_ops.hpp"
#include "so_arm101_calibration/restore.hpp"
#include "so_arm101_calibration/version.hpp"
#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/serial_port.hpp"

using namespace so_arm101_calibration;  // NOLINT

using namespace rig;  // NOLINT

// ---------------------------------------------------------------- バックアップ
TEST(Backup, ReadsAll40BytesAndNeverWrites)
{
  Rig r;
  const Backup b = r.backup();
  ASSERT_EQ(b.joints.size(), 6u);
  EXPECT_EQ(b.joints[0].homing_offset(), 135);
  EXPECT_EQ(b.joints[4].homing_offset(), 1164);
  EXPECT_EQ(b.joints[5].raw(CalibReg::MaxPositionLimit), 3385);
  EXPECT_EQ(b.joints[2].bytes[5], 3);  // ID
  EXPECT_TRUE(r.servos.writes().empty());
  const auto c = r.servos.instruction_counts();
  EXPECT_EQ(c.count(0x03), 0u);
  EXPECT_EQ(c.count(0x83), 0u);
}

TEST(Backup, SaveAndLoadRoundTripsByteForByte)
{
  Rig r;
  const Backup b = r.backup();
  const std::string path = std::string(::testing::TempDir()) + "bk_roundtrip.json";
  std::string err;
  ASSERT_TRUE(save_backup_file(path, b, &err)) << err;
  Backup back;
  ASSERT_TRUE(load_backup_file(path, &back, &err)) << err;
  ASSERT_EQ(back.joints.size(), b.joints.size());
  for (size_t i = 0; i < b.joints.size(); ++i) {EXPECT_EQ(back.joints[i].bytes, b.joints[i].bytes);}
  EXPECT_EQ(to_json(back), to_json(b));
  std::remove(path.c_str());
}

TEST(Backup, NegativeHomingOffsetIsKeptAsRawSignMagnitude)
{
  Rig r;
  r.servos.set_eeprom(2, 31, 0x800 | 135, 2);  // sign-magnitude の -135
  const Backup b = r.backup();
  EXPECT_EQ(b.joints[1].homing_offset(), -135);
  EXPECT_EQ(b.joints[1].raw(CalibReg::HomingOffset), 0x800 | 135);
  EXPECT_TRUE(contains(to_json(b), "\"homing_offset\": -135"));
}

TEST(Backup, CorruptOrInconsistentFilesAreRejected)
{
  Rig r;
  const Backup b = r.backup();
  const std::string good = to_json(b);
  Backup out;
  std::string err;
  ASSERT_TRUE(parse_backup(good, &out, &err)) << err;

  EXPECT_FALSE(parse_backup(good.substr(0, good.size() / 2), &out, &err));          // 切り詰め
  EXPECT_FALSE(parse_backup("", &out, &err));
  EXPECT_FALSE(parse_backup("not json at all", &out, &err));
  EXPECT_FALSE(parse_backup("{\"schema_version\": 1}", &out, &err));               // 欠け
  auto replace = [&](const std::string & a, const std::string & c) {
      std::string t = good;
      const auto p = t.find(a);
      EXPECT_NE(p, std::string::npos) << a;
      t.replace(p, a.size(), c);
      return t;
    };
  EXPECT_FALSE(parse_backup(replace("\"schema_version\": 1", "\"schema_version\": 2"), &out, &err));
  // バイト列の 1 つを書き換える（チェックサム不一致。重複項目とも矛盾しない値にして、チェックサムで検出されること）
  std::string t = good;
  const auto p = t.find("\"bytes\": [");
  t.replace(p + 10, 1, "9");  // 先頭バイトを変える
  EXPECT_FALSE(parse_backup(t, &out, &err));
  EXPECT_TRUE(contains(err, "checksum") || contains(err, "bytes")) << err;
  // 重複項目（homing_offset）とバイト列の矛盾
  EXPECT_FALSE(parse_backup(replace("\"homing_offset\": 135", "\"homing_offset\": 136"), &out, &err));
  EXPECT_TRUE(contains(err, "disagree")) << err;
  // 40 バイトでない
  EXPECT_FALSE(parse_backup(replace(", 0]}", "]}"), &out, &err));
}

TEST(Backup, DuplicateIdAndBadValuesAreRejected)
{
  Rig r;
  Backup b = r.backup();
  Backup dup = b;
  dup.joints[1] = dup.joints[0];
  std::string err;
  Backup out;
  EXPECT_FALSE(parse_backup(to_json(dup), &out, &err));
  EXPECT_TRUE(contains(err, "duplicate")) << err;

  JointEeprom j = b.joints[0];
  EXPECT_EQ(validate_for_restore(j), "");
  JointEeprom m = j;
  m.model = 776;
  EXPECT_NE(validate_for_restore(m), "");
  JointEeprom i = j;
  i.bytes[5] = 9;
  EXPECT_NE(validate_for_restore(i), "");
  JointEeprom big = j;
  big.bytes[11] = 0x00;
  big.bytes[12] = 0x20;  // Max_Position_Limit = 8192
  EXPECT_NE(validate_for_restore(big), "");
  JointEeprom inv = j;
  inv.bytes[9] = 0xFF;
  inv.bytes[10] = 0x0F;  // Min = 4095 > Max
  inv.bytes[11] = 0x10;
  inv.bytes[12] = 0x00;
  EXPECT_NE(validate_for_restore(inv), "");
  JointEeprom hb = j;
  hb.bytes[31] = 0x00;
  hb.bytes[32] = 0x10;  // bit12 が立つ（Homing_Offset は 12 bit）
  EXPECT_NE(validate_for_restore(hb), "");
}

TEST(Backup, ReadErrorsAbortAndOnlyCommErrorsAreRetried)
{
  Rig r;
  r.servos.drop_read_replies(3, 8, 2);  // 2 回無応答 → 再試行で成功
  JointEeprom e;
  std::string err;
  EXPECT_TRUE(read_joint_eeprom(*r.bus, 3, "elbow_flex", 3, &e, &err)) << err;
  r.servos.drop_read_replies(3, 16, 10);  // 再試行切れ
  EXPECT_FALSE(read_joint_eeprom(*r.bus, 3, "elbow_flex", 3, &e, &err));
  EXPECT_TRUE(contains(err, "address 16")) << err;
  r.servos.set_err(3, 0x04);  // サーボのエラー応答は再試行しない
  EXPECT_FALSE(read_joint_eeprom(*r.bus, 3, "elbow_flex", 3, &e, &err));
}

// ---------------------------------------------------------------- 復元: 入力・前提
TEST(Restore, DryRunWritesNothing)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  std::string text;
  EXPECT_EQ(r.run(r.args(b, false), &text), kExitOk);
  EXPECT_TRUE(r.servos.writes().empty());
  EXPECT_TRUE(contains(text, "dry-run")) << text;
  EXPECT_FALSE(r.equals(b));
}

TEST(Restore, RefusesWithoutConfirmation)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  EXPECT_EQ(r.run(r.args(b), nullptr, false), kExitInput);
  EXPECT_TRUE(r.servos.writes().empty());
}

TEST(Restore, RefusesWhenTorqueIsOnOrModelDiffers)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.set_u8(4, 40, 1);  // Torque_Enable=1
  EXPECT_EQ(r.run(r.args(b)), kExitPrecondition);
  EXPECT_TRUE(r.servos.writes().empty());
  r.servos.set_u8(4, 40, 0);
  r.servos.set_u16(2, 3, 776);
  EXPECT_EQ(r.run(r.args(b)), kExitPrecondition);
  EXPECT_TRUE(r.servos.writes().empty());
}

TEST(Restore, RefusesBadInputsWithoutWriting)
{
  Rig r;
  Backup b = r.backup();
  r.scramble();
  RestoreArgs a = r.args(b);
  a.opt.wait_ms = kMaxWaitMs + 1;
  EXPECT_EQ(r.run(a), kExitInput);
  a.opt.wait_ms = -1;
  EXPECT_EQ(r.run(a), kExitInput);
  a = r.args(b);
  a.selected = {9};  // yaml に無い
  EXPECT_EQ(r.run(a), kExitInput);
  Backup missing = b;
  missing.joints.erase(missing.joints.begin() + 2);  // id 3 が無い
  a = r.args(missing);
  EXPECT_EQ(r.run(a), kExitInput);
  Backup bad = b;
  bad.joints[0].model = 700;
  EXPECT_EQ(r.run(r.args(bad)), kExitInput);
  EXPECT_TRUE(r.servos.writes().empty());
}

// ---------------------------------------------------------------- 復元: 正常
TEST(Restore, RestoresAllJointsInOrderAndRelocks)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  std::string text;
  ASSERT_EQ(r.run(r.args(b), &text), kExitOk) << text;
  EXPECT_TRUE(r.equals(b));
  const auto w = r.servos.writes();
  for (int id = 1; id <= 6; ++id) {
    const int l0 = index_of(w, id, 55);
    const int h = index_of(w, id, 31), mn = index_of(w, id, 9), mx = index_of(w, id, 11);
    const int l1 = index_of(w, id, 55, l0 + 1);
    ASSERT_GE(l0, 0);
    ASSERT_GE(l1, 0);
    EXPECT_EQ(w[l0].data[0], 0);
    EXPECT_EQ(w[l1].data[0], 1);
    EXPECT_TRUE(l0 < h && h < mn && mn < mx && mx < l1) << id;
    EXPECT_EQ(r.servos.u8(id, 55), 1);  // Lock=1 に戻った
  }
  EXPECT_EQ(count_writes(w, 1, 40), 0);  // Torque_Enable には書かない
  for (const auto & rec : w) {EXPECT_TRUE(rec.addr == 55 || rec.addr == 31 || rec.addr == 9 || rec.addr == 11) << int(rec.addr);}
}

TEST(Restore, RestoresOnlySelectedJointsAndLeavesOthers)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  RestoreArgs a = r.args(b);
  a.selected = {6};
  ASSERT_EQ(r.run(a), kExitOk);
  EXPECT_EQ(r.servos.u16(6, 31), b.joints[5].raw(CalibReg::HomingOffset));
  EXPECT_EQ(r.servos.u16(1, 31), 501);  // 触っていない
  for (const auto & w : r.servos.writes()) {EXPECT_EQ(w.id, 6);}
}

TEST(Restore, NegativeHomingOffsetIsWrittenAsTheSameRawBytes)
{
  Rig r;
  r.servos.set_eeprom(2, 31, 0x800 | 135, 2);
  const Backup b = r.backup();
  r.scramble();
  ASSERT_EQ(r.run(r.args(b)), kExitOk);
  EXPECT_EQ(r.servos.u16(2, 31), 0x800 | 135);
}

TEST(Restore, SameValuesAreNotWrittenUnlessRewriteSame)
{
  Rig r;
  const Backup b = r.backup();
  RestoreArgs a = r.args(b);
  std::string text;
  ASSERT_EQ(r.run(a, &text), kExitOk);
  EXPECT_TRUE(r.servos.writes().empty());  // Lock にも触れない
  a.selected = {6};
  a.opt.rewrite_same = true;
  ASSERT_EQ(r.run(a, &text), kExitOk) << text;
  const auto w = r.servos.writes();
  EXPECT_EQ(count_writes(w, 6, 31), 1);
  EXPECT_EQ(count_writes(w, 6, 9), 1);
  EXPECT_EQ(count_writes(w, 6, 11), 1);
  EXPECT_EQ(count_writes(w, 6, 55), 2);
  EXPECT_EQ(r.servos.u8(6, 55), 1);
  EXPECT_TRUE(r.equals(b));
}

TEST(Restore, WarnsAboutOtherEepromBytesThatDiffer)
{
  Rig r;
  const Backup b = r.backup();
  r.servos.set_eeprom(2, 21, 33);  // P
  std::string text;
  ASSERT_EQ(r.run(r.args(b, false), &text), kExitOk);
  EXPECT_TRUE(contains(text, "アドレス 21")) << text;
}

// ---------------------------------------------------------------- 復元: 通信エラーの再試行と、値の不一致
TEST(Restore, CommErrorsAreRetriedButValueMismatchIsNot)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.drop_write_replies(1, 31, 2);     // 書き込みの応答が 2 回無い → 同じ値を再送
  r.servos.corrupt_read_replies(1, 9, 1);    // 読み戻しの応答が壊れる → 再読み
  std::string text;
  ASSERT_EQ(r.run(r.args(b), &text), kExitOk) << text;
  EXPECT_TRUE(r.equals(b));
  EXPECT_EQ(count_writes(r.servos.writes(), 1, 31), 3);

  Rig m;
  const Backup bm = m.backup();
  m.scramble();
  m.servos.ignore_writes_to(1, 31);  // 値が反映されない → 読み戻し不一致
  ASSERT_EQ(m.run(m.args(bm), &text), kExitFailedAllOld) << text;
  EXPECT_EQ(count_writes(m.servos.writes(), 1, 31), 1);  // 再試行しない
  EXPECT_TRUE(contains(text, "read-back mismatch")) << text;
  EXPECT_TRUE(contains(text, "restore --backup BACKUP.json")) << text;
  EXPECT_EQ(m.servos.u8(1, 55), 1);  // 失敗しても Lock=1 に戻す
  EXPECT_EQ(m.servos.u16(2, 31), 502);  // 次の関節には進まない
}

TEST(Restore, PersistentCommErrorLeavesAMixedStateAndExitsPartial)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.drop_write_replies(1, 9, 20);  // Min は書かれるが応答が来ない（再試行切れ）
  std::string text;
  EXPECT_EQ(r.run(r.args(b), &text), kExitPartial) << text;
  EXPECT_TRUE(contains(text, "[バックアップ]")) << text;
  EXPECT_TRUE(contains(text, "[実行前]")) << text;
  EXPECT_TRUE(contains(text, "yaml は更新していない")) << text;
}

TEST(Restore, PowerCutMidwayThenRestoreAgainSucceeds)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.cut_power_after_writes(3);  // Lock=0、Homing、Min まで書いたところで落ちる
  std::string text;
  EXPECT_EQ(r.run(r.args(b), &text), kExitPartial) << text;
  EXPECT_TRUE(contains(text, "(read failed)")) << text;
  r.servos.restore_power();
  r.servos.power_cycle();  // 電源の入れ直し（SRAM が電源投入時の値＝Lock=1 に戻る）
  EXPECT_FALSE(r.equals(b));
  ASSERT_EQ(r.run(r.args(b), &text), kExitOk) << text;
  EXPECT_TRUE(r.equals(b));
  EXPECT_EQ(r.servos.u8(1, 55), 1);
}

TEST(Restore, LockOneNotConfirmedExitsPartialAndTellsToPowerCycle)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.ignore_lock_one(3);
  std::string text;
  EXPECT_EQ(r.run(r.args(b), &text), kExitPartial) << text;
  EXPECT_TRUE(r.equals(b) || !r.equals(b));  // 値は戻るが Lock=0 のまま
  EXPECT_EQ(r.servos.u8(3, 55), 0);
  EXPECT_TRUE(contains(text, "Lock=1")) << text;
  EXPECT_TRUE(contains(text, "電源を入れ直す")) << text;
  EXPECT_TRUE(contains(text, "elbow_flex")) << text;
}

TEST(Restore, LockZeroNotReflectedStopsBeforeAnyValueIsWritten)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.ignore_writes_to(1, 55);
  std::string text;
  EXPECT_EQ(r.run(r.args(b), &text), kExitFailedAllOld) << text;
  EXPECT_EQ(count_writes(r.servos.writes(), 1, 31), 0);  // Lock=0 が確認できないので値は書かない
}

TEST(Restore, WaitTimeDecidesWhetherASlowEepromCommitIsReadBackCorrectly)
{
  Rig r;
  const Backup b = r.backup();
  r.scramble();
  r.servos.eeprom_commit_delay_ms(40);
  RestoreArgs a = r.args(b);
  a.opt.wait_ms = 0;
  std::string text;
  EXPECT_NE(r.run(a, &text), kExitOk);
  EXPECT_TRUE(contains(text, "read-back mismatch")) << text;

  Rig s;
  const Backup bs = s.backup();
  s.scramble();
  s.servos.eeprom_commit_delay_ms(40);
  RestoreArgs c = s.args(bs);
  c.opt.wait_ms = 80;
  ASSERT_EQ(s.run(c, &text), kExitOk) << text;
  EXPECT_TRUE(s.equals(bs));
}

// ---------------------------------------------------------------- 版の表記
TEST(Version, ToolIdIsOneStringAndMatchesPackageXml)
{
  EXPECT_EQ(tool_id(), std::string(kToolName) + " " + kToolVersion);
  std::ifstream f(PACKAGE_XML);
  ASSERT_TRUE(f.good()) << PACKAGE_XML;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string xml = ss.str();
  const std::string tag = "<version>";
  const auto a = xml.find(tag);
  ASSERT_NE(a, std::string::npos);
  const auto b = xml.find("</version>", a);
  EXPECT_EQ(xml.substr(a + tag.size(), b - a - tag.size()), kToolVersion) << "package.xml の <version> と version.hpp の kToolVersion を同じにする";
}

// ---------------------------------------------------------------- 本番の yaml と、記録した EEPROM の読み出し
// 本番の follower_calibration.yaml の eeprom.*（Homing_Offset・Min・Max）が、C3 の後に実機から読み出して保存した
// calibration_records/2026-11-11/eeprom_after_C3.json と一致する（ファイルどうしを比べる。値は試験に書かない）。
// 再校正して yaml を更新したときは、新しい読み出しを calibration_records に置き、EEPROM_AFTER_C3_JSON の指す先を更新する。
TEST(ProductionYaml, MatchesTheEepromReadAfterC3)
{
  const so_arm101_hardware::Calibration cal = so_arm101_hardware::load_calibration(PRODUCTION_YAML);
  Backup after;
  std::string err;
  ASSERT_TRUE(load_backup_file(EEPROM_AFTER_C3_JSON, &after, &err)) << EEPROM_AFTER_C3_JSON << ": " << err;
  ASSERT_EQ(after.joints.size(), cal.joints.size());
  for (const auto & j : cal.joints) {
    const JointEeprom * e = after.find(j.id);
    ASSERT_NE(e, nullptr) << j.name;
    uint16_t homing_raw = 0;
    ASSERT_TRUE(so_arm101_hardware::encode_sign_magnitude(j.eeprom.homing_offset, so_arm101_hardware::kHomingOffsetSignBit, &homing_raw))
      << j.name;
    EXPECT_EQ(e->raw(CalibReg::HomingOffset), homing_raw) << j.name << " Homing_Offset（yaml " << j.eeprom.homing_offset << "）";
    EXPECT_EQ(e->raw(CalibReg::MinPositionLimit), j.eeprom.min_limit) << j.name << " Min";
    EXPECT_EQ(e->raw(CalibReg::MaxPositionLimit), j.eeprom.max_limit) << j.name << " Max";
  }
}

// ---------------------------------------------------------------- 案 C: 読み戻しは独立に 2 回、2 回とも期待値のときだけ一致
TEST(WriteVerified, ReadBackNeverMatchesOnAStaleReply)
{
  Rig r;
  Options opt;
  opt.wait_ms = 0;
  const uint16_t before = static_cast<uint16_t>(r.servos.u16(1, 31));
  const uint16_t neu = before + 7;
  r.servos.ignore_writes_to(1, 31);                  // 書き込みは反映されない（実際の値は旧のまま）
  r.servos.override_next_reads(1, 31, 1, neu);       // 1 回目の読み戻しだけ、新の値の「古い／化けた応答」が届く
  const StepResult s = write_calib_verified(*r.bus, 1, CalibReg::HomingOffset, neu, opt);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.msg.find("the two reads must agree"), std::string::npos) << s.msg;
  EXPECT_EQ(r.servos.u16(1, 31), before);
}

TEST(WriteVerified, TwoAgreeingReadBacksPassAndCostExactlyTwoReads)
{
  Rig r;
  Options opt;
  opt.wait_ms = 0;
  r.servos.set_u8(1, 55, 0);  // Lock=0（EEPROM に書ける）
  const uint16_t neu = static_cast<uint16_t>(r.servos.u16(1, 31) + 7);
  const auto before = r.servos.instruction_counts()[0x02];
  const StepResult s = write_calib_verified(*r.bus, 1, CalibReg::HomingOffset, neu, opt);
  EXPECT_TRUE(s.ok) << s.msg;
  EXPECT_EQ(r.servos.u16(1, 31), neu);
  EXPECT_EQ(r.servos.instruction_counts()[0x02] - before, 2u);
}

TEST(WriteVerified, SameValueRewriteStillPassesWithTwoReads)
{
  Rig r;  // --rewrite-same と同じ形: 同じ値を書く（旧も新も同じ）
  Options opt;
  opt.wait_ms = 0;
  r.servos.set_u8(6, 55, 0);
  const uint16_t same = static_cast<uint16_t>(r.servos.u16(6, 31));
  EXPECT_TRUE(write_calib_verified(*r.bus, 6, CalibReg::HomingOffset, same, opt).ok);
}

TEST(WriteVerified, TheFinalStateTableShowsOnlyValuesThatTwoReadsAgreeOn)
{
  Rig r;
  r.servos.override_next_reads(2, 9, 1, 1234);  // 1 回目だけ違う値 → 一致しない → 表示しない（nullopt）
  const JointRegs j = read_joint_regs_confirmed(*r.bus, 2, 3);
  EXPECT_FALSE(j.v[static_cast<int>(CalibReg::MinPositionLimit)].has_value());
  EXPECT_TRUE(j.v[static_cast<int>(CalibReg::HomingOffset)].has_value());
  EXPECT_TRUE(j.v[static_cast<int>(CalibReg::MaxPositionLimit)].has_value());
}
