// Copyright 2026 so_arm101 authors. MIT License.
//
// raw → rad の変換が LeRobot 0.4.2 の変換と一致することを確認する（ハード不要）。
// 正解表 test/data/conversion_golden.csv は tools/gen_conversion_golden.py が LeRobot 0.4.2 の
// MotorsBus._normalize()（Apache-2.0, Copyright 2024 The HuggingFace Inc. team）で生成したもの。
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

TEST(Conversion, MatchesLeRobotGoldenTable)
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

TEST(Conversion, GripperIsClampedToRangeLikeLeRobot)
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

TEST(Conversion, AngleIsNotClampedLikeLeRobot)
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

TEST(Conversion, YamlMatchesLeRobotJsonSource)
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

TEST(SignMagnitude, DecodesLikeLeRobot)
{
  // encoding_utils.py: decode_sign_magnitude(encoded, sign_bit)
  EXPECT_EQ(decode_sign_magnitude(0x0000, 15), 0);
  EXPECT_EQ(decode_sign_magnitude(0x0805, 11), -5);   // Homing_Offset (bit11)
  EXPECT_EQ(decode_sign_magnitude(0x07FF, 11), 2047);
  EXPECT_EQ(decode_sign_magnitude(0x8005, 15), -5);   // Present_Position (bit15)
  EXPECT_EQ(decode_sign_magnitude(0x0FFF, 15), 4095);
  EXPECT_EQ(decode_sign_magnitude(0x7FFF, 15), 32767);
}
