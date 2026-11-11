// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/measure.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <thread>

#include "so_arm101_calibration/eeprom_backup.hpp"

namespace so_arm101_calibration
{
using so_arm101_hardware::Status;

const MeasuredJoint * MeasureFile::find(uint8_t id) const
{
  for (const auto & j : joints) {
    if (j.id == id) {return &j;}
  }
  return nullptr;
}

int actual_from_present(int present, int homing_offset)
{
  int a = (present + homing_offset) % 4096;
  if (a < 0) {a += 4096;}
  return a;
}

int unwrap_delta(int actual, int actual_center)
{
  int d = (actual - actual_center) % 4096;
  if (d >= 2048) {d -= 4096;}
  if (d < -2048) {d += 4096;}
  return d;
}

void check_measured_joint(MeasuredJoint * j, int min_span)
{
  std::vector<std::string> p;
  if (j->actual_center < 0 || j->actual_center > 4095) {p.push_back("actual_center が 0..4095 の外");}
  if (j->new_homing_offset != j->actual_center - kHalfTurn) {p.push_back("new_homing_offset が actual_center−2047 と違う");}
  if (j->new_homing_offset > 2047 || j->new_homing_offset < -2047) {
    p.push_back("Homing_Offset " + std::to_string(j->new_homing_offset) + " が bit11 の範囲（±2047）の外");
  }
  if (j->new_min < 0 || j->new_max > 4095) {
    p.push_back("新座標の範囲 " + std::to_string(j->new_min) + ".." + std::to_string(j->new_max) + " が 0..4095 の外（中央から ±2047 を超えて動いた）");
  }
  if (j->new_min >= j->new_max) {p.push_back("min >= max");}
  if (j->new_max - j->new_min < min_span) {
    p.push_back("幅 " + std::to_string(j->new_max - j->new_min) + " tick が下限 " + std::to_string(min_span) + " 未満（動かしていない？）");
  }
  j->valid = p.empty();
  j->problems.clear();
  for (size_t i = 0; i < p.size(); ++i) {j->problems += (i ? "; " : "") + p[i];}
}

uint64_t measure_checksum(const MeasureFile & m)
{
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](int64_t v) {
      for (int i = 0; i < 8; ++i) {
        h ^= static_cast<uint8_t>((static_cast<uint64_t>(v) >> (8 * i)) & 0xFF);
        h *= 1099511628211ULL;
      }
    };
  for (const auto & j : m.joints) {
    mix(j.id); mix(j.actual_center); mix(j.measured_homing_offset); mix(j.new_homing_offset);
    mix(j.actual_min); mix(j.actual_max); mix(j.new_min); mix(j.new_max); mix(j.samples);
  }
  return h;
}

static std::string esc(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {o += '\\';}
    if (static_cast<unsigned char>(c) < 0x20) {continue;}
    o += c;
  }
  return o;
}

std::string measure_to_json(const MeasureFile & m)
{
  char hex[32];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(measure_checksum(m)));
  std::ostringstream o;
  o << "{\n  \"schema_version\": " << m.schema_version << ",\n  \"created\": \"" << esc(m.created)
    << "\",\n  \"tool\": \"" << esc(m.tool) << "\",\n  \"port\": \"" << esc(m.port) << "\",\n  \"backup_file\": \""
    << esc(m.backup_file) << "\",\n  \"checksum\": \"" << hex << "\",\n  \"joints\": [\n";
  for (size_t i = 0; i < m.joints.size(); ++i) {
    const auto & j = m.joints[i];
    o << "    {\"id\": " << int(j.id) << ", \"name\": \"" << esc(j.name) << "\", \"kind\": \"" << esc(j.kind)
      << "\", \"actual_center\": " << j.actual_center << ", \"measured_homing_offset\": " << j.measured_homing_offset
      << ", \"new_homing_offset\": " << j.new_homing_offset << ", \"actual_min\": " << j.actual_min
      << ", \"actual_max\": " << j.actual_max << ", \"new_min\": " << j.new_min << ", \"new_max\": " << j.new_max
      << ", \"new_center\": " << std::setprecision(10) << j.new_center << ", \"samples\": " << j.samples
      << ", \"valid\": " << (j.valid ? "true" : "false") << ", \"problems\": \"" << esc(j.problems) << "\"}"
      << (i + 1 < m.joints.size() ? "," : "") << "\n";
  }
  o << "  ]\n}\n";
  return o.str();
}

bool parse_measure(const std::string & text, MeasureFile * out, std::string * err)
{
  auto fail = [&](const std::string & m) {
      if (err) {*err = m;}
      return false;
    };
  const size_t first = text.find_first_not_of(" \t\r\n"), last = text.find_last_not_of(" \t\r\n");
  if (first == std::string::npos || text[first] != '{' || text[last] != '}') {
    return fail("not a JSON object (truncated or corrupt)");
  }
  try {
    const YAML::Node root = YAML::Load(text);
    if (!root.IsMap()) {return fail("top level is not a map");}
    MeasureFile m;
    m.schema_version = root["schema_version"].as<int>();
    if (m.schema_version != kMeasureSchema) {return fail("unsupported schema_version");}
    m.created = root["created"].as<std::string>();
    m.tool = root["tool"].as<std::string>();
    m.port = root["port"].as<std::string>();
    m.backup_file = root["backup_file"].as<std::string>();
    const std::string want = root["checksum"].as<std::string>();
    const YAML::Node js = root["joints"];
    if (!js.IsSequence() || js.size() == 0) {return fail("joints is missing or empty");}
    for (const auto & n : js) {
      MeasuredJoint j;
      const int id = n["id"].as<int>();
      if (id < 0 || id > 253) {return fail("id out of range");}
      j.id = static_cast<uint8_t>(id);
      j.name = n["name"].as<std::string>();
      j.kind = n["kind"].as<std::string>();
      j.actual_center = n["actual_center"].as<int>();
      j.measured_homing_offset = n["measured_homing_offset"].as<int>();
      j.new_homing_offset = n["new_homing_offset"].as<int>();
      j.actual_min = n["actual_min"].as<int>();
      j.actual_max = n["actual_max"].as<int>();
      j.new_min = n["new_min"].as<int>();
      j.new_max = n["new_max"].as<int>();
      j.new_center = n["new_center"].as<double>();
      j.samples = n["samples"].as<int>();
      j.valid = n["valid"].as<bool>();
      j.problems = n["problems"].as<std::string>();
      if (m.find(j.id)) {return fail("duplicate id " + std::to_string(id));}
      if (std::abs(j.new_center - (j.new_min + j.new_max) / 2.0) > 1e-6) {return fail("new_center disagrees with new_min/new_max");}
      m.joints.push_back(j);
    }
    char hex[32];
    std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(measure_checksum(m)));
    if (want != hex) {return fail("checksum mismatch (file was edited or damaged)");}
    // 検査し直して、保存されている valid と一致することを確認（編集の検出）
    for (auto & j : m.joints) {
      MeasuredJoint c = j;
      check_measured_joint(&c);
      if (c.valid != j.valid) {return fail("joint " + j.name + ": valid flag disagrees with the values");}
    }
    *out = m;
  } catch (const std::exception & e) {
    return fail(std::string("cannot parse or missing field: ") + e.what());
  }
  return true;
}

bool load_measure_file(const std::string & path, MeasureFile * out, std::string * err)
{
  std::ifstream f(path);
  if (!f) {
    if (err) {*err = "cannot open " + path;}
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_measure(ss.str(), out, err);
}

bool save_measure_file(const std::string & path, const MeasureFile & m, std::string * err)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) {
      if (err) {*err = "cannot create " + tmp;}
      return false;
    }
    f << measure_to_json(m);
    f.flush();
    if (!f) {
      if (err) {*err = "cannot write " + tmp;}
      return false;
    }
  }
  MeasureFile back;
  std::string e2;
  if (!load_measure_file(tmp, &back, &e2)) {
    if (err) {*err = "saved file does not read back: " + e2;}
    return false;
  }
  if (measure_to_json(back) != measure_to_json(m)) {
    if (err) {*err = "saved file differs from the measurement";}
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    if (err) {*err = "cannot rename " + tmp + " to " + path;}
    return false;
  }
  return true;
}

namespace
{
std::string describe(so_arm101_hardware::Sts3215Bus & bus, Status s)
{
  return std::string(so_arm101_hardware::to_string(s)) + ": " + bus.last_error();
}

/// 全関節の Present_Position を SYNC_READ（通信エラーは再試行）。デコード済みの値。
bool read_present(
  so_arm101_hardware::Sts3215Bus & bus, const std::vector<uint8_t> & ids, int retries, std::vector<int> * out,
  std::string * err)
{
  Status s = Status::Ok;
  std::vector<std::vector<uint8_t>> raw;
  for (int t = 0; t <= retries; ++t) {
    raw.clear();
    uint8_t failed = 0;
    s = bus.sync_read(ids, so_arm101_hardware::reg::kPresentPosition, &raw, &failed);
    if (s == Status::Ok || !so_arm101_hardware::Sts3215Bus::is_comm_error(s)) {break;}
  }
  if (s != Status::Ok) {
    if (err) {*err = describe(bus, s);}
    return false;
  }
  out->clear();
  for (const auto & r : raw) {
    if (r.size() != 2) {
      if (err) {*err = "short Present_Position";}
      return false;
    }
    out->push_back(so_arm101_hardware::decode_sign_magnitude(
        static_cast<uint16_t>(r[0] | (r[1] << 8)), so_arm101_hardware::kPositionSignBit));
  }
  return true;
}

void sleep_ms(int ms)
{
  if (ms > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(ms));}
}
}  // namespace

int run_measure(
  CalibWriteBus & bus, const std::vector<JointId> & all, const std::vector<std::string> & kinds, const Options & opt,
  MeasureIO & io, std::ostream & out, MeasureFile * result)
{
  const size_t n = all.size();
  std::vector<uint8_t> ids;
  for (const auto & j : all) {ids.push_back(j.id);}
  // 1. 前提: 機種番号 777、現在の Homing_Offset、Torque_Enable
  std::vector<int> homing(n), torque(n, 0);
  bool torque_on = false;
  out << "== 前提の確認（全関節）\n";
  for (size_t i = 0; i < n; ++i) {
    uint16_t model = 0;
    Status s = Status::Ok;
    for (int t = 0; t <= opt.retries; ++t) {
      s = bus.read_u16(all[i].id, so_arm101_hardware::reg::kModelNumber, &model);
      if (s == Status::Ok || !so_arm101_hardware::Sts3215Bus::is_comm_error(s)) {break;}
    }
    if (s != Status::Ok || model != so_arm101_hardware::kModelNumberSts3215) {
      out << "NG  " << all[i].name << ": " << (s != Status::Ok ? describe(bus, s) : "model number " + std::to_string(model)) << "\n";
      return kExitPrecondition;
    }
    uint16_t h = 0;
    s = read_calib_reg(bus, all[i].id, CalibReg::HomingOffset, opt.retries, &h);
    std::vector<uint8_t> t;
    Status s2 = Status::Ok;
    for (int k = 0; k <= opt.retries; ++k) {
      s2 = bus.read(all[i].id, so_arm101_hardware::reg::kTorqueEnable, &t);
      if (s2 == Status::Ok || !so_arm101_hardware::Sts3215Bus::is_comm_error(s2)) {break;}
    }
    if (s != Status::Ok || s2 != Status::Ok || t.empty()) {
      out << "NG  " << all[i].name << ": cannot read Homing_Offset/Torque_Enable\n";
      return kExitPrecondition;
    }
    homing[i] = so_arm101_hardware::decode_sign_magnitude(h, so_arm101_hardware::kHomingOffsetSignBit);
    torque[i] = t[0];
    torque_on = torque_on || t[0] != 0;
    out << "OK  " << all[i].name << " (id " << int(all[i].id) << "): model 777, Homing_Offset=" << homing[i]
        << ", Torque_Enable=" << int(t[0]) << "\n";
  }
  // 2. 脱力（トルクが入っているときだけ。書くのは Torque_Enable=0 だけ）
  if (torque_on) {
    if (!io.wait_enter("トルクが入っている関節がある。Torque_Enable=0 を書く（腕が脱力して落ちる）。腕を手で支えて Enter")) {
      out << "中止（何も書いていない）\n";
      return kExitInput;
    }
    for (size_t i = 0; i < n; ++i) {
      if (!torque[i]) {continue;}
      Status s = Status::Ok;
      for (int t = 0; t <= opt.retries; ++t) {
        s = bus.write_torque_enable(all[i].id, false);
        if (s == Status::Ok || !so_arm101_hardware::Sts3215Bus::is_comm_error(s)) {break;}
      }
      std::vector<uint8_t> d;
      if (s == Status::Ok) {s = bus.read(all[i].id, so_arm101_hardware::reg::kTorqueEnable, &d);}
      if (s != Status::Ok || d.empty() || d[0] != 0) {
        out << "ERROR: " << all[i].name << " のトルクを OFF にできない（" << (s != Status::Ok ? describe(bus, s) : "読み戻しが 0 でない") << "）\n";
        return kExitPrecondition;
      }
      out << "Torque_Enable=0: " << all[i].name << "（読み戻し確認）\n";
    }
  } else {
    out << "全関節のトルクは OFF（何も書かない）\n";
  }
  // 3. 中央の姿勢
  if (!io.wait_enter("腕を「中央」の姿勢（各関節の可動範囲のほぼ中央。LeRobot の中央姿勢と同じ）にして、動かさずに Enter")) {
    out << "中止（何も書いていない）\n";
    return kExitInput;
  }
  std::vector<std::vector<int>> deltas(n);
  std::vector<int> first_actual(n, -1);
  std::vector<std::vector<int>> actual_samples(n);
  std::string err;
  for (int k = 0; k < io.center_samples; ++k) {
    if (io.on_center_sample) {io.on_center_sample(k);}
    std::vector<int> pos;
    if (!read_present(bus, ids, opt.retries, &pos, &err)) {
      out << "ERROR: Present_Position を読めない: " << err << "\n";
      return kExitPrecondition;
    }
    for (size_t i = 0; i < n; ++i) {
      const int a = actual_from_present(pos[i], homing[i]);
      if (first_actual[i] < 0) {first_actual[i] = a;}
      actual_samples[i].push_back(first_actual[i] + unwrap_delta(a, first_actual[i]));
    }
    sleep_ms(io.tick_ms);
  }
  std::vector<int> center(n);
  for (size_t i = 0; i < n; ++i) {
    auto v = actual_samples[i];
    std::sort(v.begin(), v.end());
    if (v.back() - v.front() > io.center_tolerance_ticks) {
      out << "ERROR: " << all[i].name << " が中央の測定中に動いた（" << v.front() << "〜" << v.back()
          << " tick。許容 " << io.center_tolerance_ticks << "）。動かさずにやり直す\n";
      return kExitInput;
    }
    int c = v[v.size() / 2];
    c %= 4096;
    if (c < 0) {c += 4096;}
    center[i] = c;
  }
  // 4. MIN/MAX の記録
  if (!io.wait_enter("全関節を端から端まで順に動かす。終わったら Enter（記録を終える）。Enter を押すと記録を始める")) {
    out << "中止（何も書いていない）\n";
    return kExitInput;
  }
  std::vector<int> dmin(n, 0), dmax(n, 0), count(n, 0);
  int consecutive_fail = 0;
  long ticks = 0;
  while (true) {
    if (io.on_tick) {io.on_tick();}
    std::vector<int> pos;
    if (read_present(bus, ids, opt.retries, &pos, &err)) {
      consecutive_fail = 0;
      for (size_t i = 0; i < n; ++i) {
        const int d = unwrap_delta(actual_from_present(pos[i], homing[i]), center[i]);
        dmin[i] = std::min(dmin[i], d);
        dmax[i] = std::max(dmax[i], d);
        ++count[i];
      }
    } else if (++consecutive_fail >= 20) {
      out << "ERROR: 記録中に Present_Position を連続して読めない: " << err << "\n";
      return kExitPrecondition;
    }
    ++ticks;
    if (io.should_stop && io.should_stop()) {break;}
    sleep_ms(io.tick_ms);
  }
  // 5. 結果
  MeasureFile m;
  m.created = now_string(false);
  m.tool = "so_arm101_calib 0.1.0";
  bool all_valid = true;
  out << "\n== 測定結果（新座標。Homing_Offset = actual_center − 2047）\n";
  out << std::left << std::setw(15) << "joint" << std::right << std::setw(8) << "center" << std::setw(9) << "measured"
      << std::setw(8) << "new_h" << std::setw(7) << "min" << std::setw(7) << "max" << std::setw(7) << "span"
      << std::setw(9) << "samples" << "  検査\n";
  for (size_t i = 0; i < n; ++i) {
    MeasuredJoint j;
    j.id = all[i].id;
    j.name = all[i].name;
    j.kind = i < kinds.size() ? kinds[i] : "angle";
    j.actual_center = center[i];
    j.measured_homing_offset = homing[i];
    j.new_homing_offset = center[i] - kHalfTurn;
    j.actual_min = center[i] + dmin[i];
    j.actual_max = center[i] + dmax[i];
    j.new_min = kHalfTurn + dmin[i];
    j.new_max = kHalfTurn + dmax[i];
    j.new_center = (j.new_min + j.new_max) / 2.0;
    j.samples = count[i];
    check_measured_joint(&j);
    all_valid = all_valid && j.valid;
    out << std::left << std::setw(15) << j.name << std::right << std::setw(8) << j.actual_center << std::setw(9)
        << j.measured_homing_offset << std::setw(8) << j.new_homing_offset << std::setw(7) << j.new_min
        << std::setw(7) << j.new_max << std::setw(7) << (j.new_max - j.new_min) << std::setw(9) << j.samples << "  "
        << (j.valid ? "OK" : "NG: " + j.problems) << "\n";
    m.joints.push_back(j);
  }
  *result = m;
  if (!all_valid) {
    out << "\n検査に通らない関節がある。測定ファイルには保存するが、apply はこのファイルを受け付けない（やり直す）\n";
    return kExitInput;
  }
  return kExitOk;
}

void compare_measures(
  const MeasureFile & a, const MeasureFile & b, const std::vector<std::pair<std::string, YamlJoint>> & yaml,
  int angle_ticks, std::ostream & out)
{
  const double rad_per_tick = 2.0 * M_PI / angle_ticks;  // 360/angle_ticks 度 = 2π/angle_ticks rad
  out << "== 再現性（A と B の差。tick）\n";
  out << std::left << std::setw(15) << "joint" << std::right << std::setw(8) << "d_cent" << std::setw(8) << "d_min"
      << std::setw(8) << "d_max" << std::setw(11) << "d_center" << std::setw(10) << "d_homing" << "\n";
  int worst = 0;
  for (const auto & ja : a.joints) {
    const MeasuredJoint * jb = b.find(ja.id);
    if (!jb) {
      out << ja.name << ": B に無い\n";
      continue;
    }
    const int dc = unwrap_delta(jb->actual_center, ja.actual_center);
    const int dmin = jb->new_min - ja.new_min, dmax = jb->new_max - ja.new_max;
    const double dcen = jb->new_center - ja.new_center;
    const int dh = jb->new_homing_offset - ja.new_homing_offset;
    worst = std::max({worst, std::abs(dc), std::abs(dmin), std::abs(dmax)});
    out << std::left << std::setw(15) << ja.name << std::right << std::setw(8) << dc << std::setw(8) << dmin
        << std::setw(8) << dmax << std::setw(11) << std::fixed << std::setprecision(1) << dcen << std::setw(10) << dh << "\n";
  }
  out << "最大の差（actual_center / min / max）: " << worst << " tick\n";
  for (int which = 0; which < 2; ++which) {
    const MeasureFile & m = which == 0 ? a : b;
    out << "\n== 今の yaml との差（" << (which == 0 ? "A" : "B") << " − yaml。tick と rad。rad は angle_ticks=" << angle_ticks << "）\n";
    out << std::left << std::setw(15) << "joint" << std::right << std::setw(8) << "d_min" << std::setw(8) << "d_max"
        << std::setw(11) << "d_center" << std::setw(13) << "d_cent[rad]" << std::setw(10) << "d_homing"
        << std::setw(12) << "zero_shift" << std::setw(13) << "zero[rad]" << "\n";
    for (const auto & j : m.joints) {
      const YamlJoint * y = nullptr;
      for (const auto & p : yaml) {
        if (p.first == j.name) {y = &p.second;}
      }
      if (!y) {
        out << j.name << ": yaml に無い\n";
        continue;
      }
      const double dcen = j.new_center - y->center;
      const int dh = j.new_homing_offset - y->homing_offset;
      // 0 rad の物理位置（actual 座標）= center + Homing_Offset。その移動量
      int shift = static_cast<int>(std::lround((j.new_center + j.new_homing_offset) - (y->center + y->homing_offset)));
      shift = unwrap_delta(shift + 4096, 0);
      const bool angle = j.kind == "angle";
      out << std::left << std::setw(15) << j.name << std::right << std::setw(8) << (j.new_min - y->min) << std::setw(8)
          << (j.new_max - y->max) << std::setw(11) << std::fixed << std::setprecision(1) << dcen;
      if (angle) {
        out << std::setw(13) << std::setprecision(4) << dcen * rad_per_tick << std::setw(10) << dh << std::setw(12) << shift
            << std::setw(13) << std::setprecision(4) << shift * rad_per_tick << "\n";
      } else {
        out << std::setw(13) << "-" << std::setw(10) << dh << std::setw(12) << "-" << std::setw(13) << "-" << "（gripper は center を使わない）\n";
      }
    }
  }
  out << "\nzero_shift = 0 rad の物理位置（actual = center + Homing_Offset）の移動量。0 に近いほど、今の URDF・出発点の姿勢の数値が変わらない\n";
}

}  // namespace so_arm101_calibration
