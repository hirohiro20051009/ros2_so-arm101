// Copyright 2026 so_arm101 authors. MIT License.
//
// raw → rad の変換が LeRobot 0.4.2 の変換と一致することを確認する（ハード不要）。
// 正解表 test/data/conversion_golden.csv は LeRobot 0.4.2 の
// MotorsBus._normalize()（Apache-2.0, Copyright 2024 The HuggingFace Inc. team）で生成したもの
// （生成スクリプトは削除済み。タグ stage2c-done の tools/gen_conversion_golden.py にある）。
// raw は 0..4095 の全値 + 範囲外 (-100, 4096, 5000)、全 6 関節。
//
// 一致の基準: |Δrad| <= 1e-12。どちらも IEEE754 double で数回の四則演算しかしない（演算順序も
// LeRobot に合わせている）ので誤差は 1e-15 程度で、1e-12 には 3 桁の余裕がある。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/sts3215_bus.hpp"

using namespace so_arm101_hardware;  // NOLINT

namespace
{
constexpr double kTol = 1e-12;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(Conversion, MatchesReferenceGoldenTable)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  std::ifstream f(GOLDEN_CSV);
  ASSERT_TRUE(f.is_open()) << GOLDEN_CSV;
  std::string line;
  std::map<std::string, size_t> rows;
  std::map<std::string, double> max_err;
  size_t total = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("joint,", 0) == 0) {continue;}
    std::stringstream ss(line);
    std::string joint, raw_s, val_s, rad_s;
    std::getline(ss, joint, ',');
    std::getline(ss, raw_s, ',');
    std::getline(ss, val_s, ',');
    std::getline(ss, rad_s, ',');
    const JointCalibration * j = cal.find(joint);
    ASSERT_NE(j, nullptr) << joint;
    const int raw = std::stoi(raw_s);
    const double expected = std::stod(rad_s);
    const double got = raw_to_rad(*j, cal.angle_ticks, raw);
    const double err = std::fabs(got - expected);
    if (err > max_err[joint]) {max_err[joint] = err;}
    EXPECT_LE(err, kTol) << joint << " raw=" << raw << " got=" << got << " expected=" << expected;
    ++rows[joint];
    ++total;
  }
  for (const auto & kv : rows) {
    // 0..4095 の 4096 + 範囲外 3
    EXPECT_EQ(kv.second, 4099u) << kv.first;
    std::printf("[conversion] %-14s rows=%zu max|err|=%.3g rad\n", kv.first.c_str(), kv.second, max_err[kv.first]);
  }
  EXPECT_EQ(rows.size(), 6u);
  EXPECT_EQ(total, 6u * 4099u);
}

TEST(Conversion, GripperIsClampedToRangeLikeReference)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  const JointCalibration * g = cal.find("gripper");
  ASSERT_NE(g, nullptr);
  EXPECT_DOUBLE_EQ(raw_to_rad(*g, cal.angle_ticks, g->raw_min), 0.0);
  EXPECT_DOUBLE_EQ(raw_to_rad(*g, cal.angle_ticks, g->raw_min - 500), 0.0);   // 範囲外（下）は 0
  EXPECT_DOUBLE_EQ(raw_to_rad(*g, cal.angle_ticks, -100), 0.0);
  EXPECT_NEAR(raw_to_rad(*g, cal.angle_ticks, g->raw_max), kPi / 180.0 * 100.0, 1e-15);
  EXPECT_NEAR(raw_to_rad(*g, cal.angle_ticks, g->raw_max + 700), kPi / 180.0 * 100.0, 1e-15);  // 上も clamp
  EXPECT_NEAR(raw_to_rad(*g, cal.angle_ticks, 5000), kPi / 180.0 * 100.0, 1e-15);
}

TEST(Conversion, AngleIsNotClampedLikeReference)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  const JointCalibration * p = cal.find("shoulder_pan");
  ASSERT_NE(p, nullptr);
  // center 2097.5 → raw 0 は −2097.5 tick。範囲外でも clamp しない（−184.3956… 度）
  EXPECT_NEAR(raw_to_rad(*p, cal.angle_ticks, 0), -2097.5 * 2.0 * kPi / 4095.0, 1e-12);
  EXPECT_LT(raw_to_rad(*p, cal.angle_ticks, p->raw_min - 100), raw_to_rad(*p, cal.angle_ticks, p->raw_min));
  // 1 tick = 2π/4095
  EXPECT_NEAR(raw_to_rad(*p, cal.angle_ticks, 2098) - raw_to_rad(*p, cal.angle_ticks, 2097),
    2.0 * kPi / 4095.0, 1e-12);
  EXPECT_NEAR(tick_in_rad(*p, cal.angle_ticks), 2.0 * kPi / 4095.0, 1e-15);
}

TEST(Conversion, GripperTickInRad)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  const JointCalibration * g = cal.find("gripper");
  ASSERT_NE(g, nullptr);
  // 100% / (3385 − 1900) tick を度とみなして rad にしたもの
  EXPECT_NEAR(tick_in_rad(*g, cal.angle_ticks), (100.0 / 1485.0) * kPi / 180.0, 1e-15);
}

// CALIBRATION_YAML は、試験専用の固定ファイル（test/data/follower_calibration_fixture_20260907.yaml。旧校正＝
// LeRobot の json から変換した値）。本番の yaml は C3 で更新されたので、この試験は本番の yaml を読まない
// （この試験は LeRobot の json そのものは読まない。json の値は、固定ファイルの値としてここに書いてある）。
TEST(Conversion, YamlMatchesFixtureSource)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  EXPECT_EQ(cal.schema_version, 1);
  EXPECT_EQ(cal.angle_ticks, 4095);
  EXPECT_EQ(cal.baudrate, 1000000);
  ASSERT_EQ(cal.joints.size(), 6u);
  const JointCalibration * wr = cal.find("wrist_roll");
  ASSERT_NE(wr, nullptr);
  EXPECT_EQ(int(wr->id), 5);
  EXPECT_DOUBLE_EQ(wr->center, 2064.5);
  EXPECT_EQ(wr->eeprom.homing_offset, 1164);
  EXPECT_EQ(wr->eeprom.min_limit, 164);
  EXPECT_NE(cal.source.find("2026-09-07"), std::string::npos) << cal.source;
}

TEST(Calibration, RejectsBrokenFiles)
{
  EXPECT_THROW(load_calibration("/nonexistent/cal.yaml"), std::runtime_error);
  const std::string path = ::testing::TempDir() + "bad_cal.yaml";
  auto write = [&](const std::string & body) {std::ofstream(path) << body;};
  write("schema_version: 2\n");
  EXPECT_THROW(load_calibration(path), std::runtime_error);
  // id の重複
  write(
    "schema_version: 1\nbus: {baudrate: 1000000, protocol: 0}\nconversion: {angle_ticks: 4095}\njoints:\n"
    "  a: {id: 1, kind: angle, direction: 1, center: 1, min: 0, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n"
    "  b: {id: 1, kind: angle, direction: 1, center: 1, min: 0, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n");
  EXPECT_THROW(load_calibration(path), std::runtime_error);
  // 未知の kind
  write(
    "schema_version: 1\nbus: {baudrate: 1000000, protocol: 0}\nconversion: {angle_ticks: 4095}\njoints:\n"
    "  a: {id: 1, kind: weird, direction: 1, center: 1, min: 0, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n");
  EXPECT_THROW(load_calibration(path), std::runtime_error);
  // min >= max
  write(
    "schema_version: 1\nbus: {baudrate: 1000000, protocol: 0}\nconversion: {angle_ticks: 4095}\njoints:\n"
    "  a: {id: 1, kind: angle, direction: 1, center: 1, min: 5, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n");
  EXPECT_THROW(load_calibration(path), std::runtime_error);
}

// ---- 本番の yaml（config/follower_calibration.yaml）の構造の検査。値は固定しない（再校正で変わる） ----
TEST(ProductionYaml, LoadsAndIsStructurallyConsistent)
{
  const Calibration cal = load_calibration(PRODUCTION_YAML);
  EXPECT_EQ(cal.schema_version, 1);
  EXPECT_EQ(cal.baudrate, 1000000);
  EXPECT_EQ(cal.angle_ticks, 4095);
  EXPECT_FALSE(cal.source.empty());
  ASSERT_EQ(cal.joints.size(), 6u);
  const char * const names[] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
  for (int i = 0; i < 6; ++i) {
    const JointCalibration * j = cal.find(names[i]);
    ASSERT_NE(j, nullptr) << names[i];
    EXPECT_EQ(int(j->id), i + 1) << names[i];
    EXPECT_EQ(j->direction, 1) << names[i];
    EXPECT_LT(j->raw_min, j->raw_max) << names[i];
    EXPECT_GE(j->raw_min, 0) << names[i];
    EXPECT_LE(j->raw_max, 4095) << names[i];
    // eeprom.* は、変換に使わない診断用で、min/max と同じ値（apply が書く形）
    EXPECT_EQ(j->eeprom.min_limit, j->raw_min) << names[i];
    EXPECT_EQ(j->eeprom.max_limit, j->raw_max) << names[i];
    EXPECT_LE(std::abs(j->eeprom.homing_offset), 2047) << names[i];  // Homing_Offset は bit11 の符号付き
    if (j->kind == JointKind::Angle) {
      EXPECT_GT(j->center, j->raw_min) << names[i];
      EXPECT_LT(j->center, j->raw_max) << names[i];
    }
  }
  EXPECT_EQ(cal.find("gripper")->kind, JointKind::LinearPercent);
  EXPECT_EQ(cal.find("shoulder_pan")->kind, JointKind::Angle);
}

// F6: 本番の補正値は elbow_flex だけ。0 か、決めた帯（-7.0〜-5.5）。値は固定しない（C4 で見直しうる。行を消せば 0 で通る）
TEST(ProductionYaml, OffsetsAreZeroExceptAnElbowInTheDecidedBand)
{
  const Calibration cal = load_calibration(PRODUCTION_YAML);
  for (const auto & j : cal.joints) {
    if (j.name == "elbow_flex") {
      EXPECT_TRUE(j.zero_offset_deg == 0.0 || (j.zero_offset_deg >= -7.0 && j.zero_offset_deg <= -5.5))
        << j.zero_offset_deg;
    } else {
      EXPECT_EQ(j.zero_offset_deg, 0.0) << j.name;
    }
  }
}

// ---- 起動ログの出所の表示（`calibration.source`）。本番の yaml には依存しない（一時ファイルを書く） ----
namespace
{
const char * const kJointsYaml =
  "bus: {baudrate: 1000000, protocol: 0}\nconversion: {angle_ticks: 4095}\njoints:\n"
  "  a: {id: 1, kind: angle, direction: 1, center: 1, min: 0, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n";

std::string source_of(const std::string & source_block)
{
  const std::string path = ::testing::TempDir() + "source_cal.yaml";
  {
    std::ofstream f(path);
    f << "schema_version: 1\n" << source_block << kJointsYaml;
  }
  const std::string src = load_calibration(path).source;
  std::remove(path.c_str());
  return src;
}
}  // namespace

TEST(CalibrationSource, OldFormatIsShownAsBefore)
{
  EXPECT_EQ(
    source_of("source:\n  kind: converted_from_lerobot_json\n  file: ~/x/so101_follower.json\n  file_date: 2026-09-07\n"
              "  converted_at: 2026-11-06\n  converted_by: tools/convert_lerobot_calibration.py\n"),
    "converted_from_lerobot_json, file=~/x/so101_follower.json, file_date=2026-09-07");
  // 旧形式で項目が欠けたときは、従来どおり ?
  EXPECT_EQ(source_of("source:\n  kind: converted_from_lerobot_json\n"), "converted_from_lerobot_json, file=?, file_date=?");
  EXPECT_EQ(source_of("source:\n  file: a.json\n"), "?, file=a.json, file_date=?");
}

TEST(CalibrationSource, NewFormatShowsTheFieldsThatApplyWrites)
{
  EXPECT_EQ(
    source_of("source:\n  kind: ros_calibrated\n  measure_file: calibration_records/2026-11-11/measure_C1c_1.json\n"
              "  backup_file: calibration_records/2026-11-11/eeprom_backup_20261111-163729.json\n"
              "  applied_at: 2026-11-11T19:43:04\n  applied_by: so_arm101_calib 0.2.0\n"),
    "ros_calibrated, measure_file=calibration_records/2026-11-11/measure_C1c_1.json, "
    "backup_file=calibration_records/2026-11-11/eeprom_backup_20261111-163729.json, "
    "applied_at=2026-11-11T19:43:04, applied_by=so_arm101_calib 0.2.0");
  // 項目が欠けたら、その項目だけ出さない（? は出さない）
  EXPECT_EQ(source_of("source:\n  kind: ros_calibrated\n  applied_at: 2026-11-11T19:43:04\n"), "ros_calibrated, applied_at=2026-11-11T19:43:04");
  EXPECT_EQ(source_of("source:\n  applied_by: x 1\n"), "?, applied_by=x 1");
}

TEST(CalibrationSource, NoSourceStaysEmpty)
{
  EXPECT_EQ(source_of(""), "");
}

TEST(SignMagnitude, DecodesLikeReference)
{
  // encoding_utils.py: decode_sign_magnitude(encoded, sign_bit)
  EXPECT_EQ(decode_sign_magnitude(0x0000, 15), 0);
  EXPECT_EQ(decode_sign_magnitude(0x0805, 11), -5);   // Homing_Offset (bit11)
  EXPECT_EQ(decode_sign_magnitude(0x07FF, 11), 2047);
  EXPECT_EQ(decode_sign_magnitude(0x8005, 15), -5);   // Present_Position (bit15)
  EXPECT_EQ(decode_sign_magnitude(0x0FFF, 15), 4095);
  EXPECT_EQ(decode_sign_magnitude(0x7FFF, 15), 32767);
}

// ---- 段階2b-3a: 逆変換 rad → raw（まだ書き込みには使わない） ----

TEST(InverseConversion, MatchesReferenceUnnormalizeWithinOneTick)
{
  // unnormalize_golden.csv は LeRobot 0.4.2 の MotorsBus._unnormalize()（int() で切り捨て）の結果。
  // こちらは最後の丸めだけ四捨五入なので、差は 0 か +1 tick（切り捨て ≤ 四捨五入）でなければならない。
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  std::ifstream f(UNNORMALIZE_GOLDEN_CSV);
  ASSERT_TRUE(f.is_open()) << UNNORMALIZE_GOLDEN_CSV;
  std::string line;
  std::map<std::string, size_t> rows, plus_one;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("joint,", 0) == 0) {continue;}
    std::stringstream ss(line);
    std::string joint, val_s, raw_s;
    std::getline(ss, joint, ',');
    std::getline(ss, val_s, ',');
    std::getline(ss, raw_s, ',');
    const JointCalibration * j = cal.find(joint);
    ASSERT_NE(j, nullptr) << joint;
    const double value = std::stod(val_s);         // 度（angle）または 0..100（gripper）
    const int reference_raw = std::stoi(raw_s);
    const double rad = value * (kPi / 180.0);     // bridge.py: math.radians(値)
    const int got = rad_to_raw(*j, cal.angle_ticks, rad);
    const int diff = got - reference_raw;
    const double x = rad_to_raw_continuous(*j, cal.angle_ticks, rad);
    if (x >= 0.0) {
      // 切り捨て ≤ 四捨五入: 差は 0 か +1。連続値は [reference_raw, reference_raw + 1)
      EXPECT_TRUE(diff == 0 || diff == 1) << joint << " value=" << value << " got=" << got << " reference=" << reference_raw;
      EXPECT_GE(x, reference_raw - 1e-6) << joint << " value=" << value;
      EXPECT_LT(x, reference_raw + 1.0 + 1e-6) << joint << " value=" << value;
    } else {
      // raw が負になる範囲外（実機では出ない。Goal_Position は呼び出し側で clamp する）。
      // int() は 0 へ向けて切り捨てるので向きが逆。差は 0 か −1
      EXPECT_TRUE(diff == 0 || diff == -1) << joint << " value=" << value << " got=" << got << " reference=" << reference_raw;
    }
    ++rows[joint];
    if (diff == 1) {++plus_one[joint];}
  }
  EXPECT_EQ(rows.size(), 6u);
  for (const auto & kv : rows) {
    std::printf("[inverse] %-14s rows=%zu (+1 tick vs reference truncation: %zu)\n", kv.first.c_str(), kv.second,
      plus_one[kv.first]);
  }
}

TEST(InverseConversion, RoundTripIsIdentityInsideTheRange)
{
  // raw → rad → raw が恒等（保持指令が 1 tick ずれない）。gripper は clamp 範囲 [min, max] の中だけ。
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  for (const auto & j : cal.joints) {
    int lo = 0, hi = 4095;
    if (j.kind == JointKind::LinearPercent) {lo = j.raw_min; hi = j.raw_max;}
    for (int raw = lo; raw <= hi; ++raw) {
      ASSERT_EQ(rad_to_raw(j, cal.angle_ticks, raw_to_rad(j, cal.angle_ticks, raw)), raw)
        << j.name << " raw=" << raw;
    }
  }
}

TEST(InverseConversion, GripperIsClampedAndAngleIsNot)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  const JointCalibration * g = cal.find("gripper");
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(rad_to_raw(*g, cal.angle_ticks, -1.0), g->raw_min);   // 0% 未満は min
  EXPECT_EQ(rad_to_raw(*g, cal.angle_ticks, 5.0), g->raw_max);    // 100% 超は max
  const JointCalibration * p = cal.find("shoulder_pan");
  ASSERT_NE(p, nullptr);
  // 角度は clamp しない（限界でのクランプは呼び出し側＝2b-3b の責任）
  EXPECT_LT(rad_to_raw(*p, cal.angle_ticks, -3.5), 0);
  EXPECT_GT(rad_to_raw(*p, cal.angle_ticks, 3.5), 4095);
}

TEST(InverseConversion, HardwareLimitsAreInsideTheLimitsOfTheYaml)
{
  // 参考: yaml の min/max（= EEPROM のリミット）を rad に直してから raw に戻すと、元の値になる。
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  for (const auto & j : cal.joints) {
    EXPECT_EQ(rad_to_raw(j, cal.angle_ticks, raw_to_rad(j, cal.angle_ticks, j.raw_min)), j.raw_min) << j.name;
    EXPECT_EQ(rad_to_raw(j, cal.angle_ticks, raw_to_rad(j, cal.angle_ticks, j.raw_max)), j.raw_max) << j.name;
  }
}

// ---- 角度の補正値 zero_offset_deg（J1。docs/joint_offset_plan.md）----
namespace
{
constexpr double kDegToRad = kPi / 180.0;

JointCalibration make_angle(int direction, double zero_offset_deg)
{
  JointCalibration j;
  j.name = "t";
  j.id = 1;
  j.kind = JointKind::Angle;
  j.direction = direction;
  j.center = 1969.5;
  j.zero_offset_deg = zero_offset_deg;
  j.raw_min = 849;
  j.raw_max = 3090;
  return j;
}

// zero_offset_deg を足す前の式（旧式）を、試験の中に直書きしたもの
double old_raw_to_rad(const JointCalibration & j, int ticks, int raw)
{
  double value = (static_cast<double>(raw) - j.center) * 360.0 / static_cast<double>(ticks);
  value = j.direction * value;
  return value * kDegToRad;
}
double old_rad_to_raw_continuous(const JointCalibration & j, int ticks, double rad)
{
  const double value = rad * (180.0 / kPi);
  return (j.direction * value) * static_cast<double>(ticks) / 360.0 + j.center;
}

// 1 関節だけの yaml を書いて読む。joint_extra は center の後ろに入れる任意の記述（例 "zero_offset_deg: 5, "）
Calibration load_one(const std::string & kind_part, const std::string & extra)
{
  const std::string path = ::testing::TempDir() + "zo_cal.yaml";
  {
    std::ofstream f(path);
    f << "schema_version: 1\nbus: {baudrate: 1000000, protocol: 0}\nconversion: {angle_ticks: 4095}\njoints:\n"
      << "  a: {id: 1, " << kind_part << extra << " min: 0, max: 2, eeprom: {homing_offset: 0, min_limit: 0, max_limit: 2}}\n";
  }
  Calibration c = load_calibration(path);
  std::remove(path.c_str());
  return c;
}
Calibration load_angle(const std::string & zo) {return load_one("kind: angle, direction: 1, center: 1, ", zo);}
}  // namespace

TEST(ZeroOffset, DefaultsToZeroWhenTheKeyIsAbsent)
{
  const Calibration cal = load_calibration(CALIBRATION_YAML);
  for (const auto & j : cal.joints) {EXPECT_EQ(j.zero_offset_deg, 0.0) << j.name;}
  EXPECT_EQ(load_angle("").joints[0].zero_offset_deg, 0.0);
}

TEST(ZeroOffset, IsReadFromTheYaml)
{
  EXPECT_DOUBLE_EQ(load_angle("zero_offset_deg: -6.3, ").joints[0].zero_offset_deg, -6.3);
  EXPECT_DOUBLE_EQ(load_angle("zero_offset_deg: 0.1, ").joints[0].zero_offset_deg, 0.1);
  EXPECT_DOUBLE_EQ(load_angle("zero_offset_deg: 0, ").joints[0].zero_offset_deg, 0.0);
}

TEST(ZeroOffset, RejectsNonFiniteAndNonNumbers)
{
  for (const char * v : {".nan", ".inf", "-.inf", "abc", "~", "[1, 2]", "{a: 1}", "\"\""}) {
    SCOPED_TRACE(v);
    EXPECT_THROW(load_angle(std::string("zero_offset_deg: ") + v + ", "), std::runtime_error);
  }
}

TEST(ZeroOffset, AcceptsUpToTheLimitAndRejectsBeyondIt)
{
  EXPECT_EQ(kMaxZeroOffsetDeg, 10.0);
  EXPECT_DOUBLE_EQ(load_angle("zero_offset_deg: 10.0, ").joints[0].zero_offset_deg, 10.0);
  EXPECT_DOUBLE_EQ(load_angle("zero_offset_deg: -10.0, ").joints[0].zero_offset_deg, -10.0);
  EXPECT_THROW(load_angle("zero_offset_deg: 10.0001, "), std::runtime_error);
  EXPECT_THROW(load_angle("zero_offset_deg: -10.0001, "), std::runtime_error);
  EXPECT_THROW(load_angle("zero_offset_deg: 1e9, "), std::runtime_error);
}

TEST(ZeroOffset, GripperMustBeZero)
{
  const std::string kind = "kind: linear_percent, direction: 1, ";
  EXPECT_EQ(load_one(kind, "").joints[0].zero_offset_deg, 0.0);
  EXPECT_EQ(load_one(kind, "zero_offset_deg: 0, ").joints[0].zero_offset_deg, 0.0);
  EXPECT_THROW(load_one(kind, "zero_offset_deg: 1, "), std::runtime_error);
  EXPECT_THROW(load_one(kind, "zero_offset_deg: -0.1, "), std::runtime_error);
  EXPECT_THROW(load_one(kind, "zero_offset_deg: .nan, "), std::runtime_error);
  EXPECT_THROW(load_one(kind, "zero_offset_deg: abc, "), std::runtime_error);
}

TEST(ZeroOffset, DefaultIsBitIdenticalToTheOldFormula)
{
  // 固定 yaml と本番 yaml の全 angle 関節（本番の補正値は 0 に戻して比べる）。raw は範囲外も含める
  const std::string files[] = {CALIBRATION_YAML, PRODUCTION_YAML};
  for (const auto & file : files) {
    Calibration cal = load_calibration(file);
    for (auto & j : cal.joints) {
      if (j.kind != JointKind::Angle) {continue;}
      j.zero_offset_deg = 0.0;
      SCOPED_TRACE(file + " " + j.name);
      for (int raw = -100; raw <= 5000; ++raw) {
        ASSERT_EQ(raw_to_rad(j, cal.angle_ticks, raw), old_raw_to_rad(j, cal.angle_ticks, raw)) << raw;
      }
      for (double rad = -4.0; rad <= 4.0; rad += 0.000371) {
        ASSERT_EQ(rad_to_raw_continuous(j, cal.angle_ticks, rad), old_rad_to_raw_continuous(j, cal.angle_ticks, rad))
          << rad;
        ASSERT_EQ(rad_to_raw(j, cal.angle_ticks, rad),
          static_cast<int>(std::lround(old_rad_to_raw_continuous(j, cal.angle_ticks, rad)))) << rad;
      }
    }
  }
}

TEST(ZeroOffset, ReadShiftsByPlusDeltaAndDirectionDoesNotFlipIt)
{
  for (int dir : {1, -1}) {
    for (double d : {-10.0, -6.3, 0.1, 6.3, 10.0}) {
      SCOPED_TRACE(std::to_string(dir) + " " + std::to_string(d));
      const JointCalibration z = make_angle(dir, 0.0), o = make_angle(dir, d);
      for (int raw : {0, 849, 1969, 1970, 3090, 4095}) {
        EXPECT_NEAR(raw_to_rad(o, 4095, raw) - raw_to_rad(z, 4095, raw), d * kDegToRad, 1e-12) << raw;
      }
    }
  }
}

TEST(ZeroOffset, WriteShiftsByMinusDeltaInTheSameCoordinate)
{
  for (int dir : {1, -1}) {
    for (double d : {-10.0, -6.3, 0.1, 6.3, 10.0}) {
      SCOPED_TRACE(std::to_string(dir) + " " + std::to_string(d));
      const JointCalibration z = make_angle(dir, 0.0), o = make_angle(dir, d);
      for (double rad : {-1.5, -0.3, 0.0, 0.7, 1.6}) {
        // 真の角度 rad を出すには、補正なしより dir·(−δ)·ticks/360 だけ raw が動く
        EXPECT_NEAR(
          rad_to_raw_continuous(o, 4095, rad) - rad_to_raw_continuous(z, 4095, rad), -dir * d * 4095.0 / 360.0, 1e-9)
          << rad;
      }
    }
  }
}

TEST(ZeroOffset, RoundTripIsIdentityForEveryRaw)
{
  for (int dir : {1, -1}) {
    for (double d : {-10.0, -6.3, 0.1, 6.3, 10.0}) {
      SCOPED_TRACE(std::to_string(dir) + " " + std::to_string(d));
      const JointCalibration j = make_angle(dir, d);
      for (int raw = 0; raw <= 4095; ++raw) {
        ASSERT_EQ(rad_to_raw(j, 4095, raw_to_rad(j, 4095, raw)), raw) << raw;
      }
    }
  }
}

TEST(ZeroOffset, RoundTripFromRadStaysWithinHalfATick)
{
  for (int dir : {1, -1}) {
    for (double d : {-10.0, -6.3, 0.1, 6.3, 10.0}) {
      SCOPED_TRACE(std::to_string(dir) + " " + std::to_string(d));
      const JointCalibration j = make_angle(dir, d);
      const double half_tick = 0.5 * tick_in_rad(j, 4095);
      for (double rad = -1.5; rad <= 1.5; rad += 0.0007) {
        ASSERT_LE(std::fabs(raw_to_rad(j, 4095, rad_to_raw(j, 4095, rad)) - rad), half_tick + 1e-9) << rad;
      }
    }
  }
}

TEST(ZeroOffset, GripperFormulaIgnoresTheField)
{
  // 手組みの gripper に補正値を入れても、換算は変わらない（読み込みが 0 以外を止める前提の二重の守り）
  JointCalibration g0;
  g0.kind = JointKind::LinearPercent;
  g0.direction = 1;
  g0.raw_min = 1454;
  g0.raw_max = 2944;
  JointCalibration g1 = g0;
  g1.zero_offset_deg = 5.0;
  for (int raw : {1000, 1454, 2000, 2944, 3500}) {EXPECT_EQ(raw_to_rad(g1, 4095, raw), raw_to_rad(g0, 4095, raw)) << raw;}
  for (double rad : {-1.0, 0.0, 0.8, 1.7, 5.0}) {
    EXPECT_EQ(rad_to_raw_continuous(g1, 4095, rad), rad_to_raw_continuous(g0, 4095, rad)) << rad;
  }
}

// ---------------------------------------------------------------- 異常判定の許容（J2。案 B）
namespace
{
JointCalibration angle_joint(int direction, double zero_offset_deg)
{
  JointCalibration j;
  j.kind = JointKind::Angle;
  j.direction = direction;
  j.center = 1969.5;
  j.zero_offset_deg = zero_offset_deg;
  j.raw_min = 849;
  j.raw_max = 3090;
  return j;
}
}  // namespace

TEST(ClampTolerance, IsExactlyTheBaseWhenTheOffsetIsZero)
{
  const std::string files[] = {CALIBRATION_YAML, PRODUCTION_YAML};
  for (const auto & file : files) {
    Calibration cal = load_calibration(file);
    for (auto & j : cal.joints) {
      j.zero_offset_deg = 0.0;  // 本番の補正値は 0 に戻して比べる
      const ClampTolerance t = clamp_tolerance_ticks(j, cal.angle_ticks, 50);
      EXPECT_EQ(t.low, 50) << file << " " << j.name;
      EXPECT_EQ(t.high, 50) << file << " " << j.name;
    }
  }
}

TEST(ClampTolerance, NegativeOffsetWithDirectionPlusWidensOnlyTheHighSide)
{
  // 6.3 * 4095 / 360 = 71.66 → ceil 72。50 + 72 = 122（floor だと 121）
  const ClampTolerance t = clamp_tolerance_ticks(angle_joint(+1, -6.3), 4095, 50);
  EXPECT_EQ(t.high, 122);
  EXPECT_EQ(t.low, 50);
}

TEST(ClampTolerance, SideFollowsTheSignOfDirectionTimesOffset)
{
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, +6.3), 4095, 50).low, 122);
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, +6.3), 4095, 50).high, 50);
  // direction −1: raw の向きが逆になるので、δ が負でも広がるのは下側
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(-1, -6.3), 4095, 50).low, 122);
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(-1, -6.3), 4095, 50).high, 50);
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(-1, +6.3), 4095, 50).high, 122);
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(-1, +6.3), 4095, 50).low, 50);
}

TEST(ClampTolerance, LargestOffsetGivesBasePlus114)
{
  // 10 * 4095 / 360 = 113.75 → 114。上限の幅は 164
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, -kMaxZeroOffsetDeg), 4095, 50).high, 164);
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, +kMaxZeroOffsetDeg), 4095, 50).low, 164);
}

TEST(ClampTolerance, TinyOffsetStillWidensByOneTickBecauseOfCeil)
{
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, -0.001), 4095, 50).high, 51);
}

TEST(ClampTolerance, UsesAngleTicksNotTheLiteral4095)
{
  // angle_ticks = 360 なら 1 tick = 1 度。4.5° → ceil(4.5) = 5 → 55（4095 を直書きすると別の値になる）
  EXPECT_EQ(clamp_tolerance_ticks(angle_joint(+1, -4.5), 360, 50).high, 55);
}

TEST(ClampTolerance, GripperIsAlwaysTheBase)
{
  JointCalibration g;
  g.kind = JointKind::LinearPercent;
  g.direction = 1;
  g.raw_min = 1454;
  g.raw_max = 2944;
  g.zero_offset_deg = 5.0;  // 読み込みが止める値。手組みで入れても広がらない
  const ClampTolerance t = clamp_tolerance_ticks(g, 4095, 50);
  EXPECT_EQ(t.low, 50);
  EXPECT_EQ(t.high, 50);
}
