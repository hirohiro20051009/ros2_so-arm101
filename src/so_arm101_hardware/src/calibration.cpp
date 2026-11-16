// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace so_arm101_hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
// CPython の math.radians(x) は x * (pi / 180)
constexpr double kDegToRad = kPi / 180.0;

template<typename T>
T require(const YAML::Node & n, const char * key, const std::string & ctx)
{
  if (!n[key]) {throw std::runtime_error(ctx + ": missing key '" + key + "'");}
  try {
    return n[key].as<T>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(ctx + ": invalid value for '" + key + "': " + e.what());
  }
}
}  // namespace

const JointCalibration * Calibration::find(const std::string & joint_name) const
{
  for (const auto & j : joints) {
    if (j.name == joint_name) {return &j;}
  }
  return nullptr;
}

Calibration load_calibration(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read calibration '" + path + "': " + e.what());
  }
  const std::string ctx = "calibration '" + path + "'";
  Calibration c;
  c.schema_version = require<int>(root, "schema_version", ctx);
  if (c.schema_version != 1) {
    throw std::runtime_error(ctx + ": unsupported schema_version " + std::to_string(c.schema_version));
  }
  if (root["source"]) {
    // ログ用の出所の説明（動作には使わない）。
    //  旧形式（LeRobot の json からの変換。読み込みの互換のため残す）: kind, file, file_date（無い項目は ?）。表示は従来のまま。
    //  新形式（so_arm101_calib の apply が書く）: kind, measure_file, backup_file, applied_at, applied_by（ある項目だけ）。
    const auto s = root["source"];
    auto text = [&s](const char * key) {return s[key].as<std::string>();};
    const std::string kind = s["kind"] ? text("kind") : std::string("?");
    const char * const new_keys[] = {"measure_file", "backup_file", "applied_at", "applied_by"};
    bool has_new = false;
    for (const char * k : new_keys) {has_new = has_new || static_cast<bool>(s[k]);}
    if (has_new && !s["file"] && !s["file_date"]) {
      c.source = kind;
      for (const char * k : new_keys) {
        if (s[k]) {c.source += std::string(", ") + k + "=" + text(k);}
      }
    } else {
      c.source = kind + ", file=" + (s["file"] ? text("file") : std::string("?")) + ", file_date=" +
        (s["file_date"] ? text("file_date") : std::string("?"));
    }
  }
  const auto bus = root["bus"];
  if (!bus) {throw std::runtime_error(ctx + ": missing 'bus'");}
  c.baudrate = require<int>(bus, "baudrate", ctx + " bus");
  c.protocol = require<int>(bus, "protocol", ctx + " bus");
  if (c.protocol != 0) {throw std::runtime_error(ctx + ": only protocol 0 is supported");}
  const auto conv = root["conversion"];
  if (!conv) {throw std::runtime_error(ctx + ": missing 'conversion'");}
  c.angle_ticks = require<int>(conv, "angle_ticks", ctx + " conversion");
  if (c.angle_ticks <= 0) {throw std::runtime_error(ctx + ": angle_ticks must be positive");}

  const auto joints = root["joints"];
  if (!joints || !joints.IsMap() || joints.size() == 0) {
    throw std::runtime_error(ctx + ": missing 'joints'");
  }
  std::set<int> ids;
  for (auto it = joints.begin(); it != joints.end(); ++it) {
    JointCalibration j;
    j.name = it->first.as<std::string>();
    const auto n = it->second;
    const std::string jctx = ctx + " joint '" + j.name + "'";
    const int id = require<int>(n, "id", jctx);
    if (id < 1 || id > 252) {throw std::runtime_error(jctx + ": invalid id " + std::to_string(id));}
    if (!ids.insert(id).second) {throw std::runtime_error(jctx + ": duplicated id " + std::to_string(id));}
    j.id = static_cast<uint8_t>(id);
    const auto kind = require<std::string>(n, "kind", jctx);
    if (kind == "angle") {
      j.kind = JointKind::Angle;
      j.center = require<double>(n, "center", jctx);
      if (n["zero_offset_deg"]) {
        try {
          j.zero_offset_deg = n["zero_offset_deg"].as<double>();
        } catch (const YAML::Exception & e) {
          throw std::runtime_error(jctx + ": invalid value for 'zero_offset_deg': " + e.what());
        }
        if (!std::isfinite(j.zero_offset_deg) || std::fabs(j.zero_offset_deg) > kMaxZeroOffsetDeg) {
          throw std::runtime_error(
            jctx + ": zero_offset_deg must be finite and within +-" + std::to_string(kMaxZeroOffsetDeg) + " deg");
        }
      }
    } else if (kind == "linear_percent") {
      j.kind = JointKind::LinearPercent;
      if (n["zero_offset_deg"]) {
        double v = std::nan("");
        try {
          v = n["zero_offset_deg"].as<double>();
        } catch (const YAML::Exception & e) {
          throw std::runtime_error(jctx + ": invalid value for 'zero_offset_deg': " + e.what());
        }
        if (v != 0.0) {throw std::runtime_error(jctx + ": zero_offset_deg must be 0 for a linear_percent joint");}
      }
    } else {
      throw std::runtime_error(jctx + ": unknown kind '" + kind + "'");
    }
    j.direction = require<int>(n, "direction", jctx);
    if (j.direction != 1 && j.direction != -1) {throw std::runtime_error(jctx + ": direction must be 1 or -1");}
    j.raw_min = require<int>(n, "min", jctx);
    j.raw_max = require<int>(n, "max", jctx);
    if (j.raw_min >= j.raw_max) {throw std::runtime_error(jctx + ": min must be less than max");}
    const auto e = n["eeprom"];
    if (!e) {throw std::runtime_error(jctx + ": missing 'eeprom' (診断用)");}
    j.eeprom.homing_offset = require<int>(e, "homing_offset", jctx + " eeprom");
    j.eeprom.min_limit = require<int>(e, "min_limit", jctx + " eeprom");
    j.eeprom.max_limit = require<int>(e, "max_limit", jctx + " eeprom");
    c.joints.push_back(j);
  }
  return c;
}

double raw_to_rad(const JointCalibration & j, int angle_ticks, int raw)
{
  double value;  // 基準の実装（LeRobot 0.4.2）の _normalize の戻り値（angle: 度、gripper: 0..100）
  if (j.kind == JointKind::Angle) {
    // DEGREES: (val − mid) * 360 / max_res。clamp しない（motors_bus.py:790-793）
    value = (static_cast<double>(raw) - j.center) * 360.0 / static_cast<double>(angle_ticks);
    value = j.direction * value;
    value = value + j.zero_offset_deg;  // 度のまま（rad に直す前）。direction を掛けた後に足す
  } else {
    // RANGE_0_100: bounded = clamp(raw, min, max); (bounded − min) / (max − min) * 100
    // （motors_bus.py:783,787-789）。drive_mode に相当する direction=-1 は 100 − norm
    const double bounded = std::min<double>(j.raw_max, std::max<double>(j.raw_min, raw));
    const double norm = ((bounded - j.raw_min) / static_cast<double>(j.raw_max - j.raw_min)) * 100.0;
    value = (j.direction == -1) ? 100.0 - norm : norm;
  }
  return value * kDegToRad;  // bridge.py: math.radians(値)（gripper の 0..100 も度として扱う）
}

double rad_to_raw_continuous(const JointCalibration & j, int angle_ticks, double rad)
{
  const double value = rad * (180.0 / kPi);  // math.degrees(rad)（bridge.py radians_to_normalized）
  if (j.kind == JointKind::Angle) {
    // DEGREES: val * max_res / 360 + mid（motors_bus.py _unnormalize）。direction は自己逆（±1）
    // raw_to_rad の逆: 度から zero_offset_deg を引いてから direction を掛ける
    return (j.direction * (value - j.zero_offset_deg)) * static_cast<double>(angle_ticks) / 360.0 + j.center;
  }
  // RANGE_0_100: bounded = clamp(val, 0, 100); bounded / 100 * (max − min) + min
  const double v = (j.direction == -1) ? 100.0 - value : value;
  const double bounded = std::min(100.0, std::max(0.0, v));
  return (bounded / 100.0) * static_cast<double>(j.raw_max - j.raw_min) + j.raw_min;
}

int rad_to_raw(const JointCalibration & j, int angle_ticks, double rad)
{
  return static_cast<int>(std::lround(rad_to_raw_continuous(j, angle_ticks, rad)));
}

double tick_in_rad(const JointCalibration & j, int angle_ticks)
{
  if (j.kind == JointKind::Angle) {return 2.0 * kPi / static_cast<double>(angle_ticks);}
  return (100.0 / static_cast<double>(j.raw_max - j.raw_min)) * kDegToRad;
}

}  // namespace so_arm101_hardware
