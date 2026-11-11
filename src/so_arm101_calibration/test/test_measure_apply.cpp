// Copyright 2026 so_arm101 authors. MIT License.
//
// measure / apply / compare の試験。偽サーボ（pty）を使い、実機・実ポートは開かない。
#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "so_arm101_calibration/apply.hpp"
#include "so_arm101_calibration/measure.hpp"
#include "test_rig.hpp"

using namespace so_arm101_calibration;  // NOLINT
using namespace rig;                    // NOLINT
namespace fs = std::filesystem;

namespace
{
/// 偽サーボの Present_Position を、actual（物理位置）から設定する。present = actual − Homing_Offset（mod 4096）。
void set_actual(Rig & r, int id, int actual)
{
  const int h = so_arm101_hardware::decode_sign_magnitude(r.servos.u16(id, 31), 11);
  int p = (actual - h) % 4096;
  if (p < 0) {p += 4096;}
  r.servos.set_present(id, p);
}

const int kCenter[] = {2000, 2100, 2200, 2300, 2064, 2400};
const int kLo[] = {-1300, -1100, -1000, -1200, -1900, -700};
const int kHi[] = {1300, 1100, 1000, 1200, 1901, 700};

struct Measure
{
  Rig & r;
  std::vector<std::string> prompts;
  MeasureIO io;
  int tick{0};
  int moving_gripper{1};  // 0 にすると gripper を動かさない
  explicit Measure(Rig & rig_) : r(rig_)
  {
    io.tick_ms = 0;
    io.center_samples = 3;
    io.wait_enter = [this](const std::string & p) {prompts.push_back(p); return true;};
    io.on_center_sample = [this](int) {
        for (int i = 0; i < 6; ++i) {set_actual(r, i + 1, kCenter[i]);}
      };
    io.on_tick = [this]() {
        ++tick;
        for (int i = 0; i < 6; ++i) {
          const int id = i + 1;
          if (id == 6 && !moving_gripper) {set_actual(r, id, kCenter[i]); continue;}
          const int d = tick == 1 ? kLo[i] : (tick == 2 ? kHi[i] : 0);
          set_actual(r, id, (kCenter[i] + d + 4096) % 4096);
        }
      };
    io.should_stop = [this]() {return tick >= 4;};
  }
  int run(MeasureFile * out, std::string * text = nullptr, const Options & opt = Options())
  {
    std::vector<std::string> kinds(6, "angle");
    kinds[5] = "linear_percent";
    std::ostringstream o;
    const int rc = run_measure(*r.bus, r.all, kinds, opt, io, o, out);
    if (text) {*text = o.str();}
    return rc;
  }
};

/// 偽サーボの今の状態から、検査済みの測定ファイルを作る（apply の試験用）。
MeasureFile make_measure(Rig & r)
{
  MeasureFile m;
  m.created = "2026-11-11T00:00:00";
  m.tool = "test";
  m.port = "fake";
  m.backup_file = "BACKUP.json";
  for (int i = 0; i < 6; ++i) {
    MeasuredJoint j;
    j.id = i + 1;
    j.name = r.all[i].name;
    j.kind = i == 5 ? "linear_percent" : "angle";
    j.actual_center = kCenter[i];
    j.measured_homing_offset = so_arm101_hardware::decode_sign_magnitude(r.servos.u16(j.id, 31), 11);
    j.new_homing_offset = kCenter[i] - 2047;
    j.actual_min = kCenter[i] + kLo[i];
    j.actual_max = kCenter[i] + kHi[i];
    j.new_min = 2047 + kLo[i];
    j.new_max = 2047 + kHi[i];
    j.new_center = (j.new_min + j.new_max) / 2.0;
    j.samples = 100;
    check_measured_joint(&j);
    m.joints.push_back(j);
  }
  return m;
}

struct ApplyRig
{
  Rig r;
  Backup b{r.backup()};
  MeasureFile m{make_measure(r)};
  std::string dir{std::string(::testing::TempDir()) + "apply_" + std::to_string(reinterpret_cast<uintptr_t>(this))};
  std::string yaml;
  ApplyRig()
  {
    fs::create_directories(dir);
    yaml = dir + "/follower_calibration.yaml";
    fs::copy_file(CALIBRATION_YAML, yaml, fs::copy_options::overwrite_existing);
  }
  ~ApplyRig() {std::error_code ec; fs::remove_all(dir, ec);}
  ApplyArgs args(bool write = true)
  {
    ApplyArgs a;
    a.measure = m;
    a.measure_path = "MEASURE.json";
    a.backup = b;
    a.backup_path = "BACKUP.json";
    a.all = r.all;
    a.opt.wait_ms = 0;
    a.write = write;
    a.yaml_path = yaml;
    a.fallback_yaml_path = dir + "/fallback.yaml";
    return a;
  }
  int run(const ApplyArgs & a, std::string * text = nullptr, bool confirm = true)
  {
    std::ostringstream o;
    const int rc = run_apply(*r.bus, a, [confirm]() {return confirm;}, o);
    if (text) {*text = o.str();}
    return rc;
  }
  bool eeprom_is_new(const std::vector<int> & ids)
  {
    for (int id : ids) {
      const auto & j = m.joints[id - 1];
      uint16_t h = 0;
      so_arm101_hardware::encode_sign_magnitude(j.new_homing_offset, 11, &h);
      if (r.servos.u16(id, 31) != h || r.servos.u16(id, 9) != j.new_min || r.servos.u16(id, 11) != j.new_max) {return false;}
    }
    return true;
  }
  bool eeprom_is_backup() {return r.equals(b);}
  std::string yaml_text() {std::ifstream f(yaml); std::stringstream s; s << f.rdbuf(); return s.str();}
};
}  // namespace

// ---------------------------------------------------------------- measure
TEST(Measure, MeasuresCenterAndRangeWithoutWritingAnything)
{
  Rig r;
  Measure ms(r);
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  ASSERT_EQ(m.joints.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    const auto & j = m.joints[i];
    EXPECT_EQ(j.actual_center, kCenter[i]) << j.name;
    EXPECT_EQ(j.new_homing_offset, kCenter[i] - 2047) << j.name;
    EXPECT_EQ(j.new_min, 2047 + kLo[i]) << j.name;
    EXPECT_EQ(j.new_max, 2047 + kHi[i]) << j.name;
    EXPECT_EQ(j.actual_min, kCenter[i] + kLo[i]) << j.name;
    EXPECT_TRUE(j.valid) << j.problems;
    EXPECT_GT(j.samples, 0);
  }
  EXPECT_EQ(m.joints[4].measured_homing_offset, 1164);  // 測定時の Homing_Offset（wrist_roll は 0/4095 をまたぐ）
  EXPECT_EQ(m.joints[4].new_max - m.joints[4].new_min, 3801);
  // トルクは既に OFF: 何も書かない（EEPROM にも Lock にも触れない）
  EXPECT_TRUE(r.servos.writes().empty());
  EXPECT_EQ(ms.prompts.size(), 1u);  // 中央の Enter だけ（脱力の確認は無し）。記録の終了の Enter は should_stop
}

TEST(Measure, OnlyTorqueEnableZeroIsWrittenWhenTorqueIsOn)
{
  Rig r;
  r.servos.set_u8(2, 40, 1);
  r.servos.set_u8(5, 40, 1);
  Measure ms(r);
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  const auto w = r.servos.writes();
  ASSERT_EQ(w.size(), 2u);
  for (const auto & rec : w) {
    EXPECT_EQ(rec.addr, 40);
    EXPECT_EQ(rec.data[0], 0);
  }
  EXPECT_EQ(r.servos.u8(2, 40), 0);
  EXPECT_EQ(ms.prompts.size(), 2u);  // 脱力の確認が加わる（記録の終了の Enter は should_stop）
  EXPECT_EQ(count_writes(w, 1, 40), 0);  // トルクが入っていない関節には書かない
}

TEST(Measure, TorqueThatCannotBeTurnedOffStopsTheMeasurement)
{
  Rig r;
  r.servos.set_u8(2, 40, 1);
  r.servos.ignore_writes_to(2, 40);
  Measure ms(r);
  MeasureFile m;
  std::string text;
  EXPECT_EQ(ms.run(&m, &text), kExitPrecondition) << text;
  EXPECT_TRUE(m.joints.empty());
}

TEST(Measure, ARefusedPromptWritesNothing)
{
  Rig r;
  r.servos.set_u8(2, 40, 1);
  Measure ms(r);
  ms.io.wait_enter = [](const std::string &) {return false;};
  MeasureFile m;
  EXPECT_EQ(ms.run(&m), kExitInput);
  EXPECT_TRUE(r.servos.writes().empty());
}

TEST(Measure, MovingDuringTheCenterReadingIsRejected)
{
  Rig r;
  Measure ms(r);
  ms.io.on_center_sample = [&](int k) {
      for (int i = 0; i < 6; ++i) {set_actual(r, i + 1, kCenter[i] + (i == 3 ? k * 20 : 0));}
    };
  MeasureFile m;
  std::string text;
  EXPECT_EQ(ms.run(&m, &text), kExitInput) << text;
  EXPECT_NE(text.find("wrist_flex"), std::string::npos);
  EXPECT_TRUE(m.joints.empty());
}

TEST(Measure, AJointThatWasNotMovedFailsTheCheckButTheResultIsKept)
{
  Rig r;
  Measure ms(r);
  ms.moving_gripper = 0;
  MeasureFile m;
  std::string text;
  EXPECT_EQ(ms.run(&m, &text), kExitInput) << text;
  ASSERT_EQ(m.joints.size(), 6u);
  EXPECT_FALSE(m.joints[5].valid);
  EXPECT_NE(m.joints[5].problems.find("下限"), std::string::npos) << m.joints[5].problems;
  EXPECT_TRUE(m.joints[0].valid);
}

TEST(Measure, CenterAt4095GivesAnUnencodableHomingOffsetAndTravelBeyondHalfTurnIsInvalid)
{
  MeasuredJoint j;
  j.id = 1;
  j.name = "x";
  j.actual_center = 4095;
  j.new_homing_offset = 4095 - 2047;  // 2048: bit11 では表せない
  j.new_min = 1500;
  j.new_max = 2500;
  check_measured_joint(&j);
  EXPECT_FALSE(j.valid);
  EXPECT_NE(j.problems.find("bit11"), std::string::npos) << j.problems;
  MeasuredJoint k;
  k.actual_center = 2000;
  k.new_homing_offset = -47;
  k.new_min = -1;  // 中央から −2048 まで動いた
  k.new_max = 3000;
  check_measured_joint(&k);
  EXPECT_FALSE(k.valid);
  MeasuredJoint ok;
  ok.actual_center = 2000;
  ok.new_homing_offset = -47;
  ok.new_min = 500;
  ok.new_max = 3500;
  check_measured_joint(&ok);
  EXPECT_TRUE(ok.valid) << ok.problems;
  EXPECT_EQ(actual_from_present(4000, 1164), 1068);  // mod 4096
  EXPECT_EQ(unwrap_delta(100, 4000), 196);
  EXPECT_EQ(unwrap_delta(4000, 100), -196);
}

TEST(Measure, CommErrorsWhileRecordingAreTolerated)
{
  Rig r;
  Measure ms(r);
  r.servos.corrupt_read_replies(1, 56, 3);
  MeasureFile m;
  std::string text;
  EXPECT_EQ(ms.run(&m, &text), kExitOk) << text;
}

TEST(Measure, RefusesWhenAServoDoesNotAnswerOrIsNotAnSts3215)
{
  Rig r;
  r.servos.set_u16(3, 3, 776);
  Measure ms(r);
  MeasureFile m;
  EXPECT_EQ(ms.run(&m), kExitPrecondition);
  EXPECT_TRUE(r.servos.writes().empty());
}

// ---------------------------------------------------------------- 2 回 Enter・記録中の表示
TEST(Measure, TwoEntersCenterThenRecordImmediately)
{
  Rig r;
  Measure ms(r);
  std::vector<std::string> ev;
  ms.io.wait_enter = [&](const std::string & p) {ev.push_back("enter"); ms.prompts.push_back(p); return true;};
  auto center = ms.io.on_center_sample;
  ms.io.on_center_sample = [&, center](int k) {ev.push_back("center"); center(k);};
  ms.io.on_recording_start = [&]() {ev.push_back("flush");};
  auto tick = ms.io.on_tick;
  ms.io.on_tick = [&, tick]() {ev.push_back("tick"); tick();};
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  // Enter は中央の 1 回だけ（トルク OFF）。記録の開始に Enter は要らない
  EXPECT_EQ(std::count(ev.begin(), ev.end(), "enter"), 1);
  EXPECT_EQ(ev.front(), "enter");
  // 中央の測定 → 入力を捨てる（1 回）→ 記録
  const auto flush = std::find(ev.begin(), ev.end(), "flush");
  ASSERT_NE(flush, ev.end());
  EXPECT_EQ(std::count(ev.begin(), ev.end(), "flush"), 1);
  EXPECT_EQ(std::count(ev.begin(), flush, "center"), 3);
  EXPECT_EQ(std::count(ev.begin(), flush, "tick"), 0);
  EXPECT_GT(std::count(flush, ev.end(), "tick"), 0);
  EXPECT_EQ(std::count(flush, ev.end(), "center"), 0);
  // 表示の順序: 「中央を測定中」→「中央の測定 OK、記録中」→ 記録の統計
  const auto p1 = text.find("中央を測定中"), p2 = text.find("中央の測定 OK、記録中"), p3 = text.find("記録: 読み取り");
  ASSERT_NE(p1, std::string::npos);
  ASSERT_NE(p2, std::string::npos);
  ASSERT_NE(p3, std::string::npos);
  EXPECT_LT(p1, p2);
  EXPECT_LT(p2, p3);
}

TEST(Measure, TwoEntersWhenTorqueIsOnTheSupportEnterComesFirst)
{
  Rig r;
  r.servos.set_u8(2, 40, 1);
  Measure ms(r);
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  ASSERT_EQ(ms.prompts.size(), 2u);
  EXPECT_NE(ms.prompts[0].find("支えて"), std::string::npos);
  EXPECT_NE(ms.prompts[1].find("中央"), std::string::npos);
}

TEST(Measure, ARejectedCenterNeverStartsRecording)
{
  Rig r;
  Measure ms(r);
  int starts = 0;
  ms.io.on_recording_start = [&]() {++starts;};
  int ticks = 0;
  ms.io.on_tick = [&]() {++ticks;};
  ms.io.on_center_sample = [&](int k) {
      for (int i = 0; i < 6; ++i) {set_actual(r, i + 1, kCenter[i] + (i == 0 ? k * 30 : 0));}
    };
  MeasureFile m;
  std::string text;
  EXPECT_EQ(ms.run(&m, &text), kExitInput) << text;
  EXPECT_EQ(starts, 0);
  EXPECT_EQ(ticks, 0);
  EXPECT_EQ(text.find("記録中"), std::string::npos);
  EXPECT_TRUE(m.joints.empty());
}

TEST(LiveDisplay, GapIsReadInTheSameDirectionOnBothSides)
{
  EXPECT_EQ(format_gap(12), "あと 12");
  EXPECT_EQ(format_gap(0), "届いた");
  EXPECT_EQ(format_gap(-30), "届いた(+30)");
  LiveJoint j;
  j.name = "shoulder_pan";
  j.has_ref = true;
  j.ref_min = -1300;
  j.ref_max = 1300;
  // min 側は「記録した min − yaml の min」、max 側は「yaml の max − 記録した max」。どちらも「あと」が正
  j.min = -1288;  // 前回の端（−1300）まで 12 tick 足りない
  j.max = 1288;   // 前回の端（+1300）まで 12 tick 足りない
  std::string l = format_log_line(3, j);
  EXPECT_NE(l.find("min 側 あと 12"), std::string::npos) << l;
  EXPECT_NE(l.find("max 側 あと 12"), std::string::npos) << l;
  j.min = -1300;
  j.max = 1300;
  l = format_log_line(3, j);
  EXPECT_NE(l.find("min 側 届いた /"), std::string::npos) << l;
  EXPECT_NE(l.find("max 側 届いた"), std::string::npos) << l;
  j.min = -1330;  // 30 tick 超えて押した（今回の pan のように）
  j.max = 1305;
  j.min_recent = true;
  l = format_log_line(3, j);
  EXPECT_NE(l.find("min 側 届いた(+30)*"), std::string::npos) << l;
  EXPECT_NE(l.find("max 側 届いた(+5)"), std::string::npos) << l;
  EXPECT_EQ(l.find("max 側 届いた(+5)*"), std::string::npos) << l;  // max 側は直近に更新されていない
  // 端末の表と同じ表記
  const auto rows = format_live_table({j});
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_NE(rows[1].find("届いた(+30)*"), std::string::npos) << rows[1];
  EXPECT_NE(rows[1].find("届いた(+5)"), std::string::npos) << rows[1];
  EXPECT_EQ(rows[1].find("\033"), std::string::npos);  // ANSI を含まない
  // yaml の範囲が無いとき
  j.has_ref = false;
  EXPECT_NE(format_log_line(0, j).find("min 側 - "), std::string::npos);
}

TEST(LiveDisplay, YamlRangeCrossing4095IsUnwrappedMod4096)
{
  // wrist_roll の yaml の生の範囲: min_limit 164 + Homing_Offset 1164 = 1328、max_limit 3965 + 1164 = 5129（4095 をまたぐ）
  auto r1 = yaml_range_around(1328, 5129, 3228);   // 中央 3228 のとき: 中点 3228.5 → 差 0.5
  EXPECT_EQ(r1.first, -1900);
  EXPECT_EQ(r1.second, 1901);
  // 中央が 4096 ずれた別の表し方（= 同じ物理位置）でも同じ。mod 4096 で折り返す
  EXPECT_EQ(yaml_range_around(1328 + 4096, 5129 + 4096, 3228), r1);
  EXPECT_EQ(yaml_range_around(1328 - 4096, 5129 - 4096, 3228), r1);
  // 中央が 100（= 4196 と同じ位置）のとき: 中点 3228.5 は 100 から見て −967.5。幅 3801 は折り返さない
  auto r2 = yaml_range_around(1328, 5129, 100);
  EXPECT_EQ(r2.first, -2868);
  EXPECT_EQ(r2.second, 933);
  // 4095 をまたがない関節は、素直な差
  auto r3 = yaml_range_around(743 + 135, 3452 + 135, 2097);
  EXPECT_EQ(r3.first, 743 + 135 - 2097);
  EXPECT_EQ(r3.second, 3452 + 135 - 2097);
}

TEST(LiveDisplay, WristRollGapIsCorrectWhenTheYamlRangeCrossesTheWrap)
{
  Rig r;
  Measure ms(r);
  ms.io.live_every_ticks = 1;
  ms.io.log_every_ticks = 1;
  // roll の中央は 3228。yaml の生の範囲は 1328..5129（4095 をまたぐ）。他の関節は yaml の範囲 = 測る範囲
  ms.io.yaml_ref.clear();
  for (int i = 0; i < 6; ++i) {ms.io.yaml_ref.push_back({kCenter[i] + kLo[i], kCenter[i] + kHi[i]});}
  ms.io.yaml_ref[4] = {1328, 5129};
  ms.io.on_center_sample = [&](int) {
      for (int i = 0; i < 6; ++i) {set_actual(r, i + 1, i == 4 ? 3228 : kCenter[i]);}
    };
  ms.io.on_tick = [&]() {
      ++ms.tick;
      for (int i = 0; i < 6; ++i) {
        const int base = i == 4 ? 3228 : kCenter[i];
        int d = 0;
        if (ms.tick == 1) {d = i == 4 ? -1895 : kLo[i];}   // roll は前回の端（−1900）の 5 手前
        if (ms.tick == 2) {d = i == 4 ? 1905 : kHi[i];}    // roll は前回の端（+1901）を 4 超えた
        set_actual(r, i + 1, (base + d + 4096) % 4096);
      }
    };
  std::vector<std::vector<LiveJoint>> frames;
  ms.io.on_live = [&](const std::vector<LiveJoint> & v) {frames.push_back(v);};
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  ASSERT_FALSE(frames.empty());
  const LiveJoint & roll = frames.back()[4];
  EXPECT_EQ(roll.name, "wrist_roll");
  EXPECT_EQ(roll.min, -1895);
  EXPECT_EQ(roll.max, 1905);
  EXPECT_EQ(roll.ref_min, -1900);
  EXPECT_EQ(roll.ref_max, 1901);
  const auto line = text.find("wrist_roll");
  ASSERT_NE(line, std::string::npos);
  EXPECT_NE(text.find("min 側 あと 5"), std::string::npos) << text;
  EXPECT_NE(text.find("max 側 届いた(+4)"), std::string::npos) << text;
  // 他の関節は前回の端と同じ位置まで来ている
  EXPECT_NE(text.find("shoulder_pan"), std::string::npos);
  EXPECT_EQ(frames.back()[0].min - frames.back()[0].ref_min, 0);
}

TEST(LiveDisplay, FramesAndLogLinesAreEmittedAtTheirOwnCadence)
{
  Rig r;
  Measure ms(r);
  ms.io.live_every_ticks = 2;
  ms.io.log_every_ticks = 4;
  ms.io.recent_ticks = 2;
  ms.io.should_stop = [&]() {return ms.tick >= 8;};
  int frames = 0;
  std::vector<bool> recent;
  ms.io.on_live = [&](const std::vector<LiveJoint> & v) {++frames; recent.push_back(v[0].min_recent);};
  MeasureFile m;
  std::string text;
  ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
  EXPECT_EQ(frames, 4);  // 8 tick / 2
  EXPECT_EQ(ms.io.stats.live_frames, 4);
  // ログの行: 4 tick ごと（8 tick で 2 回）× 6 関節
  size_t joint_lines = 0;
  for (const char * n : {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}) {
    for (size_t pos = text.find(std::string(" s] ") + n); pos != std::string::npos; pos = text.find(std::string(" s] ") + n, pos + 1)) {
      ++joint_lines;
    }
  }
  EXPECT_EQ(joint_lines, 12u) << text;
  // min は tick 1 で更新（kLo）。recent_ticks=2 なので、tick 2 の枠では * が付き、tick 8 の枠では消えている
  ASSERT_EQ(recent.size(), 4u);
  EXPECT_TRUE(recent.front());
  EXPECT_FALSE(recent.back());
}

TEST(LiveDisplay, DisplayDoesNotChangeTheReadCadence)
{
  // 表示（整形・on_live・ログ行）を付けても、記録の読み取りの回数と間隔は付けないときと同じ。
  // 回数は決定的に等しい（1 周期に SYNC_READ を 1 回）。間隔は実時間なので、付けない場合との差を許容範囲で比べる。
  auto run_once = [](bool display, int * sync_reads, RecordStats * st) {
      Rig r;
      Measure ms(r);
      ms.io.tick_ms = 20;
      ms.io.center_samples = 3;
      ms.io.should_stop = [&]() {return ms.tick >= 60;};
      std::string sink;
      if (display) {
        ms.io.yaml_ref.clear();
        for (int i = 0; i < 6; ++i) {ms.io.yaml_ref.push_back({kCenter[i] + kLo[i], kCenter[i] + kHi[i]});}
        ms.io.on_live = [&](const std::vector<LiveJoint> & v) {for (const auto & row : format_live_table(v)) {sink += row + "\n";}};
        ms.io.live_every_ticks = 10;
        ms.io.log_every_ticks = 50;
      } else {
        ms.io.log_every_ticks = 1000000;
      }
      const auto before = r.servos.instruction_counts();
      MeasureFile m;
      std::string text;
      ASSERT_EQ(ms.run(&m, &text), kExitOk) << text;
      auto after = r.servos.instruction_counts();
      *sync_reads = static_cast<int>(after[0x82] - (before.count(0x82) ? before.at(0x82) : 0u));
      *st = ms.io.stats;
    };
  int reads_off = 0, reads_on = 0;
  RecordStats off, on;
  run_once(false, &reads_off, &off);
  run_once(true, &reads_on, &on);
  EXPECT_EQ(off.ticks, 60);
  EXPECT_EQ(on.ticks, 60);
  EXPECT_EQ(reads_off, 60 + 3);  // 記録 60 回 + 中央 3 回
  EXPECT_EQ(reads_on, reads_off);
  EXPECT_EQ(off.read_failures, 0);
  EXPECT_EQ(on.read_failures, 0);
  EXPECT_EQ(on.live_frames, 6);
  EXPECT_LT(on.mean_interval_ms, off.mean_interval_ms * 1.25 + 3.0)
    << "表示あり " << on.mean_interval_ms << " ms / なし " << off.mean_interval_ms << " ms";
  EXPECT_LT(on.live_max_ms, 5.0) << "表示 1 回の最大 " << on.live_max_ms << " ms";
}

TEST(MeasureFileIo, RoundTripAndCorruptionAreDetected)
{
  Rig r;
  Measure ms(r);
  MeasureFile m;
  ASSERT_EQ(ms.run(&m), kExitOk);
  const std::string path = std::string(::testing::TempDir()) + "measure_rt.json";
  std::string err;
  ASSERT_TRUE(save_measure_file(path, m, &err)) << err;
  MeasureFile back;
  ASSERT_TRUE(load_measure_file(path, &back, &err)) << err;
  EXPECT_EQ(measure_to_json(back), measure_to_json(m));
  std::remove(path.c_str());

  const std::string good = measure_to_json(m);
  MeasureFile out;
  EXPECT_FALSE(parse_measure(good.substr(0, good.size() / 2), &out, &err));
  EXPECT_FALSE(parse_measure("", &out, &err));
  auto replace = [&](const std::string & a, const std::string & b) {
      std::string t = good;
      const auto p = t.find(a);
      EXPECT_NE(p, std::string::npos) << a;
      t.replace(p, a.size(), b);
      return t;
    };
  EXPECT_FALSE(parse_measure(replace("\"new_min\": 747", "\"new_min\": 748"), &out, &err));  // 編集（new_center と矛盾）
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(parse_measure(replace("\"actual_min\": 700", "\"actual_min\": 701"), &out, &err));  // 他に冗長な項目が無い → チェックサム
  EXPECT_NE(err.find("checksum"), std::string::npos) << err;
  EXPECT_FALSE(parse_measure(replace("\"valid\": true", "\"valid\": false"), &out, &err));
  EXPECT_FALSE(parse_measure(replace("\"schema_version\": 1", "\"schema_version\": 3"), &out, &err));
}

namespace
{
std::vector<std::pair<std::string, YamlJoint>> yaml_joints(Rig & r)
{
  std::vector<std::pair<std::string, YamlJoint>> yj;
  for (const auto & j : r.cal.joints) {
    YamlJoint y;
    y.kind = j.kind == so_arm101_hardware::JointKind::Angle ? "angle" : "linear_percent";
    y.center = j.center;
    y.min = j.raw_min;
    y.max = j.raw_max;
    y.homing_offset = j.eeprom.homing_offset;
    yj.push_back({j.name, y});
  }
  return yj;
}
/// 範囲 [amin, amax]（actual）を、中央の姿勢 center で測った結果にする。
void set_range(MeasuredJoint * j, int center, int amin, int amax)
{
  j->actual_center = center;
  j->new_homing_offset = center - kHalfTurn;
  j->actual_min = amin;
  j->actual_max = amax;
  j->new_min = kHalfTurn + (amin - center);
  j->new_max = kHalfTurn + (amax - center);
  j->new_center = (j->new_min + j->new_max) / 2.0;
}
std::string compare_text(Rig & r, const MeasureFile & a, const MeasureFile & b)
{
  std::ostringstream o;
  compare_measures(a, b, yaml_joints(r), 4095, o);
  return o.str();
}
}  // namespace

TEST(Compare, ShowsReproducibilityAndTheDifferenceFromTheYaml)
{
  Rig r;
  Measure ms(r);
  MeasureFile a, b;
  ASSERT_EQ(ms.run(&a), kExitOk);
  Measure ms2(r);
  ASSERT_EQ(ms2.run(&b), kExitOk);
  b.joints[0].actual_min += 3;  // pan: 端の押し切りが 3 tick 違う → 範囲の中点（0 rad の位置）は 1.5 tick 動く
  b.joints[2].actual_center += 2;
  const std::string t = compare_text(r, a, b);
  EXPECT_NE(t.find("最大の差（d_amin / d_amax / d_zero。d_cent は含めない）: 3.0 tick"), std::string::npos) << t;
  EXPECT_NE(t.find("d_zero[rad]"), std::string::npos);
  EXPECT_NE(t.find("zero_shift"), std::string::npos);
  EXPECT_NE(t.find("d_cent[rad]"), std::string::npos);
  EXPECT_NE(t.find("gripper は center と 0 rad を使わない"), std::string::npos);
}

TEST(Compare, ADifferentCenterPostureIsNotReportedAsADifferenceInTheRange)
{
  Rig r;
  Measure ms(r);
  MeasureFile a, b;
  ASSERT_EQ(ms.run(&a), kExitOk);
  b = a;
  // 同じ物理範囲を、中央の姿勢だけ 70 tick ずらして測った（新座標の min/max と Homing_Offset は変わる）
  for (auto & j : b.joints) {
    set_range(&j, j.actual_center + 70, j.actual_min, j.actual_max);
  }
  const std::string t = compare_text(r, a, b);
  EXPECT_NE(t.find("最大の差（d_amin / d_amax / d_zero。d_cent は含めない）: 0.0 tick"), std::string::npos) << t;
  EXPECT_NE(t.find("      70"), std::string::npos) << t;  // d_cent は 70 と表示される（参考）
}

/// 再現性の表の、その関節の行を数値に分ける（d_amin, d_amax, d_span, d_zero, d_zero[rad], d_cent）。
std::vector<double> repro_row(const std::string & text, const std::string & joint)
{
  const auto start = text.find(joint);
  const auto end = text.find("\n== ", start);  // 再現性の表の行は、次の見出しより前
  std::istringstream in(text.substr(start, std::min(end, text.find('\n', start)) - start));
  std::string name;
  in >> name;
  std::vector<double> v;
  std::string tok;
  while (in >> tok && v.size() < 6) {
    if (tok == "-") {v.push_back(std::nan(""));} else {v.push_back(std::stod(tok));}
  }
  return v;
}

TEST(Compare, ZeroPositionIsTheMidpointOfTheRangeAndFoldsAcrossTheWrap)
{
  Rig r;
  Measure ms(r);
  MeasureFile a, b;
  ASSERT_EQ(ms.run(&a), kExitOk);
  b = a;
  // wrist_roll（id 5）: 範囲が 4095 をまたぐ（展開した actual は 4096 以上）。B は A より 4 tick 先
  set_range(&a.joints[4], 2200, 300, 4100);
  set_range(&b.joints[4], 2204, 304, 4104);
  auto v = repro_row(compare_text(r, a, b), "wrist_roll");
  ASSERT_EQ(v.size(), 6u);
  EXPECT_DOUBLE_EQ(v[0], 4.0);  // d_amin
  EXPECT_DOUBLE_EQ(v[1], 4.0);  // d_amax
  EXPECT_DOUBLE_EQ(v[3], 4.0);  // d_zero
  EXPECT_NEAR(v[4], 4 * 2 * M_PI / 4095, 1e-4);
  // 同じ物理位置を 4096 ずらして持っていても（B の max を折り返した値 8）、差は同じ 4 tick（mod 4096 で折り返す）
  b.joints[4].actual_max = 4104 - 4096;
  v = repro_row(compare_text(r, a, b), "wrist_roll");
  EXPECT_DOUBLE_EQ(v[1], 4.0);
  // 反対向き（B が 4 tick 手前）なら −4
  set_range(&b.joints[4], 2196, 296, 4096);
  v = repro_row(compare_text(r, a, b), "wrist_roll");
  EXPECT_DOUBLE_EQ(v[0], -4.0);
  EXPECT_DOUBLE_EQ(v[1], -4.0);
  EXPECT_DOUBLE_EQ(v[3], -4.0);
}

TEST(Compare, GripperShowsOnlyTheEndsAndNoZeroPosition)
{
  Rig r;
  Measure ms(r);
  MeasureFile a, b;
  ASSERT_EQ(ms.run(&a), kExitOk);
  b = a;
  b.joints[5].actual_max += 10;
  const std::string t = compare_text(r, a, b);
  const auto pos = t.find("gripper");
  ASSERT_NE(pos, std::string::npos);
  const std::string line = t.substr(pos, t.find('\n', pos) - pos);
  EXPECT_NE(line.find("-"), std::string::npos) << line;
  EXPECT_NE(line.find("10"), std::string::npos) << line;
  EXPECT_NE(t.find("最大の差（d_amin / d_amax / d_zero。d_cent は含めない）: 10.0 tick"), std::string::npos) << t;
}

// ---------------------------------------------------------------- apply
TEST(Apply, DryRunAndRefusedConfirmationWriteNothing)
{
  ApplyRig t;
  std::string text;
  EXPECT_EQ(t.run(t.args(false), &text), kExitOk);
  EXPECT_TRUE(t.r.servos.writes().empty());
  EXPECT_NE(text.find("dry-run"), std::string::npos) << text;
  EXPECT_EQ(t.run(t.args(true), &text, false), kExitInput);
  EXPECT_TRUE(t.r.servos.writes().empty());
  EXPECT_TRUE(t.eeprom_is_backup());
}

TEST(Apply, RefusesWhenTheEepromDiffersFromTheBackup)
{
  ApplyRig t;
  t.r.servos.set_eeprom(3, 9, 870, 2);  // バックアップの後で誰かが変えた
  std::string text;
  EXPECT_EQ(t.run(t.args(), &text), kExitPrecondition) << text;
  EXPECT_TRUE(t.r.servos.writes().empty());
  EXPECT_NE(text.find("バックアップと違う"), std::string::npos) << text;
}

TEST(Apply, RefusesWhenTheHomingOffsetAtMeasurementDiffersFromTheEeprom)
{
  ApplyRig t;
  t.m.joints[1].measured_homing_offset += 5;
  std::string text;
  EXPECT_EQ(t.run(t.args(), &text), kExitPrecondition) << text;
  EXPECT_TRUE(t.r.servos.writes().empty());
}

TEST(Apply, RefusesAnInvalidOrIncompleteMeasurementAndABackupThatCannotRestore)
{
  ApplyRig t;
  t.m.joints[2].new_min = 3000;  // min >= max → 検査に落ちる
  EXPECT_EQ(t.run(t.args()), kExitInput);
  ApplyRig u;
  u.m.joints.erase(u.m.joints.begin() + 4);
  EXPECT_EQ(u.run(u.args()), kExitInput);
  ApplyRig v;
  v.b.joints[0].model = 700;
  EXPECT_EQ(v.run(v.args()), kExitInput);
  ApplyRig w;
  w.b.joints.erase(w.b.joints.begin());
  EXPECT_EQ(w.run(w.args()), kExitInput);
  for (ApplyRig * x : {&t, &u, &v, &w}) {EXPECT_TRUE(x->r.servos.writes().empty());}
}

TEST(Apply, WritesAllJointsInOrderThenUpdatesTheYamlOnce)
{
  ApplyRig t;
  const std::string before = t.yaml_text();
  std::string text;
  ASSERT_EQ(t.run(t.args(), &text), kExitOk) << text;
  EXPECT_TRUE(t.eeprom_is_new({1, 2, 3, 4, 5, 6}));
  const auto w = t.r.servos.writes();
  EXPECT_EQ(count_writes(w, 1, 40), 0);  // トルクには書かない
  for (int id = 1; id <= 6; ++id) {
    const int l0 = index_of(w, id, 55), h = index_of(w, id, 31), mn = index_of(w, id, 9), mx = index_of(w, id, 11);
    const int l1 = index_of(w, id, 55, l0 + 1);
    EXPECT_TRUE(l0 >= 0 && l0 < h && h < mn && mn < mx && mx < l1) << id;
    EXPECT_EQ(t.r.servos.u8(id, 55), 1);
  }
  EXPECT_EQ(w.size(), 6u * 5u);  // 書き込みは 1 回ずつ（巻き戻しなし）
  // yaml: 1 回だけ更新され、読み込めて、値が新しい
  EXPECT_NE(t.yaml_text(), before);
  const auto cal = so_arm101_hardware::load_calibration(t.yaml);
  ASSERT_EQ(cal.joints.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    const auto & j = cal.joints[i];
    EXPECT_EQ(j.raw_min, t.m.joints[i].new_min);
    EXPECT_EQ(j.raw_max, t.m.joints[i].new_max);
    EXPECT_EQ(j.eeprom.homing_offset, t.m.joints[i].new_homing_offset);
    EXPECT_EQ(j.eeprom.min_limit, t.m.joints[i].new_min);
    if (i < 5) {EXPECT_DOUBLE_EQ(j.center, t.m.joints[i].new_center);}
  }
  EXPECT_EQ(cal.angle_ticks, 4095);
  const YAML::Node y = YAML::LoadFile(t.yaml);
  EXPECT_EQ(y["source"]["kind"].as<std::string>(), "ros_calibrated");
  EXPECT_EQ(y["source"]["backup_file"].as<std::string>(), "BACKUP.json");
  EXPECT_FALSE(fs::exists(t.yaml + ".new"));
  // 書き込みモードの診断と同じ比較（EEPROM と yaml が一致）になる
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(t.r.servos.u16(i + 1, 9), cal.joints[i].eeprom.min_limit);
    EXPECT_EQ(t.r.servos.u16(i + 1, 11), cal.joints[i].eeprom.max_limit);
  }
}

TEST(Apply, TheYamlThroughASymlinkIsUpdatedAtItsTargetAndTheLinkStays)
{
  ApplyRig t;
  const std::string link = t.dir + "/link.yaml";
  fs::create_symlink(t.yaml, link);
  ApplyArgs a = t.args();
  a.yaml_path = link;
  ASSERT_EQ(t.run(a), kExitOk);
  EXPECT_TRUE(fs::is_symlink(link));
  EXPECT_EQ(so_arm101_hardware::load_calibration(t.yaml).joints[0].raw_min, t.m.joints[0].new_min);
}

TEST(Apply, AGroupOfJointsIsWrittenWithoutTouchingTheYamlOrTheOthers)
{
  ApplyRig t;
  const std::string before = t.yaml_text();
  ApplyArgs a = t.args();
  a.selected = {6};
  std::string text;
  ASSERT_EQ(t.run(a, &text), kExitOk) << text;
  EXPECT_TRUE(t.eeprom_is_new({6}));
  EXPECT_EQ(t.r.servos.u16(1, 31), t.b.joints[0].raw(CalibReg::HomingOffset));
  EXPECT_EQ(t.yaml_text(), before);
  EXPECT_NE(text.find("yaml は更新していない"), std::string::npos) << text;
  for (const auto & w : t.r.servos.writes()) {EXPECT_EQ(w.id, 6);}
  // C2 のあと: その関節だけバックアップへ戻せる
  RestoreArgs ra;
  ra.backup = t.b;
  ra.backup_path = "BACKUP.json";
  ra.all = t.r.all;
  ra.selected = {6};
  ra.opt.wait_ms = 0;
  ra.write = true;
  std::ostringstream o;
  EXPECT_EQ(run_restore(*t.r.bus, ra, []() {return true;}, o), kExitOk) << o.str();
  EXPECT_TRUE(t.eeprom_is_backup());
}

TEST(Apply, AFailureHalfwayRollsBackEveryJointWrittenInThisRun)
{
  ApplyRig t;
  const std::string before = t.yaml_text();
  t.r.servos.ignore_writes_to(3, 11);  // elbow_flex の Max が反映されない → 読み戻し不一致
  std::string text;
  EXPECT_EQ(t.run(t.args(), &text), kExitFailedAllOld) << text;
  EXPECT_TRUE(t.eeprom_is_backup());
  EXPECT_EQ(t.yaml_text(), before);
  for (int id = 1; id <= 6; ++id) {EXPECT_EQ(t.r.servos.u8(id, 55), 1) << id;}
  const auto w = t.r.servos.writes();
  EXPECT_GE(index_of(w, 1, 31), 0);   // 関節 1・2 は新しい値が書かれた後、戻された
  EXPECT_EQ(count_writes(w, 1, 31), 2);
  EXPECT_EQ(count_writes(w, 2, 31), 2);
  EXPECT_EQ(count_writes(w, 4, 31), 0);  // 失敗した関節より後ろには進まない
  EXPECT_NE(text.find("read-back mismatch"), std::string::npos) << text;
  EXPECT_NE(text.find("[旧]"), std::string::npos) << text;
}

TEST(Apply, WhenTheRollbackAlsoFailsTheStateTableShowsOldNewUnknownAndRestoreWorksAfterAPowerCycle)
{
  ApplyRig t;
  t.r.servos.cut_power_after_writes(9);  // 関節 1 を書き終え、関節 2 の途中で落ちる
  std::string text;
  EXPECT_EQ(t.run(t.args(), &text), kExitPartial) << text;
  EXPECT_NE(text.find("so_arm101_calib restore --backup BACKUP.json --write"), std::string::npos) << text;
  EXPECT_NE(text.find("不明"), std::string::npos) << text;
  EXPECT_NE(text.find("yaml は更新していない"), std::string::npos) << text;
  EXPECT_EQ(so_arm101_hardware::load_calibration(t.yaml).joints[0].raw_min, 743);  // yaml は古いまま
  t.r.servos.restore_power();
  t.r.servos.power_cycle();
  EXPECT_FALSE(t.eeprom_is_backup());
  RestoreArgs ra;
  ra.backup = t.b;
  ra.backup_path = "BACKUP.json";
  ra.all = t.r.all;
  ra.opt.wait_ms = 0;
  ra.write = true;
  std::ostringstream o;
  ASSERT_EQ(run_restore(*t.r.bus, ra, []() {return true;}, o), kExitOk) << o.str();
  EXPECT_TRUE(t.eeprom_is_backup());
}

TEST(Apply, LockOneNotConfirmedExitsPartialAndAsksForAPowerCycle)
{
  ApplyRig t;
  t.r.servos.ignore_lock_one(2);
  std::string text;
  EXPECT_EQ(t.run(t.args(), &text), kExitPartial) << text;
  EXPECT_TRUE(t.eeprom_is_backup());  // 値は戻る
  EXPECT_EQ(t.r.servos.u8(2, 55), 0);
  EXPECT_NE(text.find("shoulder_lift"), std::string::npos);
  EXPECT_NE(text.find("電源を入れ直す"), std::string::npos) << text;
}

TEST(Apply, AFailureToUpdateTheYamlLeavesTheEepromNewAndExitsPartial)
{
  ApplyRig t;
  ApplyArgs a = t.args();
  a.yaml_path = t.dir + "/does_not_exist.yaml";
  std::string text;
  EXPECT_EQ(t.run(a, &text), kExitPartial) << text;
  EXPECT_TRUE(t.eeprom_is_new({1, 2, 3, 4, 5, 6}));
  EXPECT_NE(text.find("yaml を更新できなかった"), std::string::npos) << text;
}

TEST(Apply, TheFirstSessionOfCommErrorsIsRetriedButAMismatchIsNot)
{
  ApplyRig t;
  t.r.servos.drop_write_replies(1, 31, 2);
  t.r.servos.corrupt_read_replies(2, 9, 1);
  std::string text;
  ASSERT_EQ(t.run(t.args(), &text), kExitOk) << text;
  EXPECT_TRUE(t.eeprom_is_new({1, 2, 3, 4, 5, 6}));
}

TEST(Apply, TheRenderedYamlKeepsTheOldFormat)
{
  ApplyRig t;
  const std::string text = render_calibration_yaml(t.yaml, t.m, "B.json", "M.json", "2026-11-11T00:00:00");
  EXPECT_NE(text.find("shoulder_pan: {id: 1, kind: angle, direction: 1, center: "), std::string::npos) << text;
  EXPECT_NE(text.find("gripper: {id: 6, kind: linear_percent, direction: 1, min: "), std::string::npos) << text;
  EXPECT_EQ(text.find("gripper: {id: 6, kind: linear_percent, direction: 1, center"), std::string::npos);
  EXPECT_NE(text.find("conversion: {angle_ticks: 4095}"), std::string::npos);
}
