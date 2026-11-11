// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/eeprom_backup.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>

namespace so_arm101_calibration
{
using so_arm101_hardware::Sts3215Bus;
using so_arm101_hardware::Status;

int JointEeprom::homing_offset() const
{
  return so_arm101_hardware::decode_sign_magnitude(raw(CalibReg::HomingOffset), so_arm101_hardware::kHomingOffsetSignBit);
}

const JointEeprom * Backup::find(uint8_t id) const
{
  for (const auto & j : joints) {
    if (j.id == id) {return &j;}
  }
  return nullptr;
}

uint64_t bytes_checksum(const Backup & b)
{
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](uint8_t v) {h ^= v; h *= 1099511628211ULL;};
  for (const auto & j : b.joints) {
    mix(j.id);
    for (auto v : j.bytes) {mix(v);}
  }
  return h;
}

std::string now_string(bool fmt_file)
{
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), fmt_file ? "%Y%m%d-%H%M%S" : "%Y-%m-%dT%H:%M:%S", &tm);
  return buf;
}

static std::string escape(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {o += '\\';}
    if (static_cast<unsigned char>(c) < 0x20) {continue;}
    o += c;
  }
  return o;
}

std::string to_json(const Backup & b)
{
  char hex[32];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(bytes_checksum(b)));
  std::ostringstream o;
  o << "{\n  \"schema_version\": " << b.schema_version << ",\n  \"created\": \"" << escape(b.created)
    << "\",\n  \"port\": \"" << escape(b.port) << "\",\n  \"tool\": \"" << escape(b.tool)
    << "\",\n  \"bytes_checksum\": \"" << hex << "\",\n  \"eeprom_range\": \"addresses 0..39\",\n  \"joints\": [\n";
  for (size_t i = 0; i < b.joints.size(); ++i) {
    const auto & j = b.joints[i];
    o << "    {\"id\": " << int(j.id) << ", \"name\": \"" << escape(j.name) << "\", \"model\": " << j.model
      << ", \"homing_offset\": " << j.homing_offset() << ", \"min_limit\": " << j.raw(CalibReg::MinPositionLimit)
      << ", \"max_limit\": " << j.raw(CalibReg::MaxPositionLimit) << ", \"bytes\": [";
    for (int k = 0; k < kEepromBytes; ++k) {o << (k ? ", " : "") << int(j.bytes[k]);}
    o << "]}" << (i + 1 < b.joints.size() ? "," : "") << "\n";
  }
  o << "  ]\n}\n";
  return o.str();
}

bool parse_backup(const std::string & text, Backup * out, std::string * err)
{
  auto fail = [&](const std::string & m) {
      if (err) {*err = m;}
      return false;
    };
  // 厳密に JSON として検査する簡易チェック: 先頭が '{'、末尾（空白を除く）が '}'（切り詰めの検出）
  size_t first = text.find_first_not_of(" \t\r\n"), last = text.find_last_not_of(" \t\r\n");
  if (first == std::string::npos || text[first] != '{' || text[last] != '}') {
    return fail("not a JSON object (truncated or corrupt)");
  }
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const std::exception & e) {
    return fail(std::string("cannot parse: ") + e.what());
  }
  try {
    if (!root.IsMap()) {return fail("top level is not a map");}
    Backup b;
    b.schema_version = root["schema_version"].as<int>();
    if (b.schema_version != kBackupSchema) {return fail("unsupported schema_version " + std::to_string(b.schema_version));}
    b.created = root["created"].as<std::string>();
    b.port = root["port"].as<std::string>();
    b.tool = root["tool"].as<std::string>();
    const std::string want = root["bytes_checksum"].as<std::string>();
    const YAML::Node js = root["joints"];
    if (!js.IsSequence() || js.size() == 0) {return fail("joints is missing or empty");}
    for (const auto & jn : js) {
      JointEeprom j;
      const int id = jn["id"].as<int>();
      if (id < 0 || id > 253) {return fail("id out of range: " + std::to_string(id));}
      j.id = static_cast<uint8_t>(id);
      j.name = jn["name"].as<std::string>();
      const int model = jn["model"].as<int>();
      if (model < 0 || model > 0xFFFF) {return fail("model out of range");}
      j.model = static_cast<uint16_t>(model);
      const YAML::Node by = jn["bytes"];
      if (!by.IsSequence() || by.size() != kEepromBytes) {
        return fail("joint id " + std::to_string(id) + ": bytes must have exactly 40 entries");
      }
      for (int k = 0; k < kEepromBytes; ++k) {
        const int v = by[k].as<int>();
        if (v < 0 || v > 255) {return fail("joint id " + std::to_string(id) + ": byte out of range");}
        j.bytes[k] = static_cast<uint8_t>(v);
      }
      if (b.find(j.id)) {return fail("duplicate id " + std::to_string(id));}
      // 読みやすさのための重複項目が、バイト列と矛盾していないこと
      if (jn["homing_offset"].as<int>() != j.homing_offset() ||
        jn["min_limit"].as<int>() != j.raw(CalibReg::MinPositionLimit) ||
        jn["max_limit"].as<int>() != j.raw(CalibReg::MaxPositionLimit))
      {
        return fail("joint id " + std::to_string(id) + ": homing_offset/min_limit/max_limit disagree with bytes");
      }
      b.joints.push_back(j);
    }
    char hex[32];
    std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(bytes_checksum(b)));
    if (want != hex) {return fail("bytes_checksum mismatch (file was edited or damaged)");}
    *out = b;
  } catch (const std::exception & e) {
    return fail(std::string("missing or invalid field: ") + e.what());
  }
  return true;
}

bool load_backup_file(const std::string & path, Backup * out, std::string * err)
{
  std::ifstream f(path);
  if (!f) {
    if (err) {*err = "cannot open " + path;}
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_backup(ss.str(), out, err);
}

bool save_backup_file(const std::string & path, const Backup & b, std::string * err)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) {
      if (err) {*err = "cannot create " + tmp;}
      return false;
    }
    f << to_json(b);
    f.flush();
    if (!f) {
      if (err) {*err = "cannot write " + tmp;}
      return false;
    }
  }
  Backup back;
  std::string e2;
  if (!load_backup_file(tmp, &back, &e2)) {
    if (err) {*err = "saved file does not read back: " + e2;}
    return false;
  }
  if (to_json(back) != to_json(b)) {
    if (err) {*err = "saved file differs from the data read from the servos";}
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    if (err) {*err = "cannot rename " + tmp + " to " + path;}
    return false;
  }
  return true;
}

std::string validate_for_restore(const JointEeprom & j)
{
  if (j.model != so_arm101_hardware::kModelNumberSts3215) {
    return "model number " + std::to_string(j.model) + " (expected 777)";
  }
  if (j.bytes[5] != j.id) {return "ID in the EEPROM bytes (" + std::to_string(j.bytes[5]) + ") differs from the joint id";}
  for (int i = 0; i < kNumCalibRegs; ++i) {
    const CalibReg r = static_cast<CalibReg>(i);
    if (j.raw(r) > max_raw_of(r)) {
      return std::string(to_string(r)) + " raw value " + std::to_string(j.raw(r)) + " cannot be written";
    }
  }
  if (j.raw(CalibReg::MinPositionLimit) > j.raw(CalibReg::MaxPositionLimit)) {return "Min_Position_Limit > Max_Position_Limit";}
  return "";
}

bool read_joint_eeprom(
  Sts3215Bus & bus, uint8_t id, const std::string & name, int retries, JointEeprom * out, std::string * err)
{
  JointEeprom j;
  j.id = id;
  j.name = name;
  for (int a = 0; a < kEepromBytes; a += kReadChunk) {
    std::vector<uint8_t> d;
    Status s = Status::Ok;
    for (int t = 0; t <= retries; ++t) {
      s = bus.read(id, so_arm101_hardware::Register{static_cast<uint8_t>(a), kReadChunk}, &d);
      if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}
    }
    if (s != Status::Ok || d.size() != kReadChunk) {
      if (err) {
        *err = name + " (id " + std::to_string(id) + "): cannot read EEPROM address " + std::to_string(a) + ": " +
          so_arm101_hardware::to_string(s) + " " + bus.last_error();
      }
      return false;
    }
    for (int k = 0; k < kReadChunk; ++k) {j.bytes[a + k] = d[k];}
  }
  j.model = j.u16_at(3);
  *out = j;
  return true;
}

}  // namespace so_arm101_calibration
