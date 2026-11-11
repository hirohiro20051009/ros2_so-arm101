// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/apply.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <sstream>

#include "so_arm101_hardware/calibration.hpp"

namespace so_arm101_calibration
{
using so_arm101_hardware::Status;

namespace
{
const char * kLabel[] = {"Homing_Offset", "Min_Position_Limit", "Max_Position_Limit"};

std::string name_of(const std::vector<JointId> & all, uint8_t id)
{
  for (const auto & j : all) {
    if (j.id == id) {return j.name;}
  }
  return "id" + std::to_string(id);
}

std::string ja(const std::string & c)
{
  if (c == "target") {return "新";}
  if (c == "before") {return "旧";}
  if (c == "same") {return "旧=新";}
  return "不明";
}

bool encode_homing(int v, uint16_t * raw)
{
  return so_arm101_hardware::encode_sign_magnitude(v, so_arm101_hardware::kHomingOffsetSignBit, raw);
}

std::string num(double v)
{
  std::ostringstream o;
  if (std::floor(v) == v) {
    o << static_cast<long long>(v);
  } else {
    o << v;
  }
  return o.str();
}
}  // namespace

std::string render_calibration_yaml(
  const std::string & old_yaml_path, const MeasureFile & m, const std::string & backup_path,
  const std::string & measure_path, const std::string & applied_at)
{
  const YAML::Node old = YAML::LoadFile(old_yaml_path);
  std::ostringstream o;
  o << "# so_arm101 サーボキャリブレーション（フォロワー）。スキーマと意味は docs/inventory.md「段階2b-1」。\n"
    << "# 段階2c の apply（so_arm101_calib）が生成。eeprom.* は変換に使わず、起動時診断用（書き込みモードは EEPROM と\n"
    << "# 違うと起動を拒否する）。\n"
    << "schema_version: " << old["schema_version"].as<int>() << "\n"
    << "source:\n  kind: ros_calibrated\n  measure_file: " << measure_path << "\n  backup_file: " << backup_path
    << "\n  applied_at: " << applied_at << "\n  applied_by: so_arm101_calib 0.1.0\n"
    << "bus: {baudrate: " << old["bus"]["baudrate"].as<int>() << ", protocol: " << old["bus"]["protocol"].as<int>() << "}\n"
    << "# LeRobot 等価（分母 4095）。物理的には 4096 の可能性があり、変える場合は別途決める。\n"
    << "conversion: {angle_ticks: " << old["conversion"]["angle_ticks"].as<int>() << "}\n"
    << "joints:\n";
  for (const auto & j : m.joints) {
    const YAML::Node oj = old["joints"][j.name];
    if (!oj.IsDefined()) {throw std::runtime_error("joint " + j.name + " is not in " + old_yaml_path);}
    uint16_t hraw = 0;
    if (!encode_homing(j.new_homing_offset, &hraw)) {throw std::runtime_error("homing offset not encodable");}
    o << "  " << j.name << ": {id: " << int(j.id) << ", kind: " << oj["kind"].as<std::string>()
      << ", direction: " << oj["direction"].as<int>();
    if (oj["kind"].as<std::string>() == "angle") {o << ", center: " << num(j.new_center);}
    o << ", min: " << j.new_min << ", max: " << j.new_max << ", eeprom: {homing_offset: " << j.new_homing_offset
      << ", min_limit: " << j.new_min << ", max_limit: " << j.new_max << "}}\n";
  }
  return o.str();
}

bool update_calibration_yaml(const std::string & yaml_path, const std::string & text, std::string * err)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path real = fs::canonical(yaml_path, ec);  // シンボリックリンクを解決（colcon の symlink-install）
  if (ec) {
    if (err) {*err = "cannot resolve " + yaml_path + ": " + ec.message();}
    return false;
  }
  const std::string tmp = real.string() + ".new";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) {
      if (err) {*err = "cannot create " + tmp;}
      return false;
    }
    f << text;
    f.flush();
    if (!f) {
      if (err) {*err = "cannot write " + tmp;}
      return false;
    }
  }
  try {
    const auto c = so_arm101_hardware::load_calibration(tmp);  // 読み直して検査
    if (c.joints.empty()) {throw std::runtime_error("no joints");}
  } catch (const std::exception & e) {
    if (err) {*err = std::string("new yaml does not load: ") + e.what();}
    std::remove(tmp.c_str());
    return false;
  }
  fs::rename(tmp, real, ec);
  if (ec) {
    if (err) {*err = "cannot rename " + tmp + ": " + ec.message();}
    return false;
  }
  return true;
}

int run_apply(
  CalibWriteBus & bus, const ApplyArgs & a, const std::function<bool()> & confirm, std::ostream & out)
{
  std::vector<uint8_t> sel = a.selected;
  if (sel.empty()) {
    for (const auto & j : a.all) {sel.push_back(j.id);}
  }
  const bool all_joints = sel.size() == a.all.size();
  // 1. 入力の検査（何も書かない）
  if (const std::string why = measure_not_applicable_reason(a.measure); !why.empty()) {
    out << "ERROR: " << why << "。何も書かない\n";
    return kExitInput;
  }
  if (a.opt.wait_ms < 0 || a.opt.wait_ms > kMaxWaitMs || a.opt.retries < 0 || a.opt.retries > 10) {
    out << "ERROR: --eeprom-wait-ms は 0〜" << kMaxWaitMs << "、retries は 0〜10\n";
    return kExitInput;
  }
  struct Plan
  {
    uint8_t id;
    uint16_t neu[kNumCalibRegs];
    uint16_t old_[kNumCalibRegs];
  };
  std::vector<Plan> plans;
  for (const auto & j : a.all) {
    const auto * b = a.backup.find(j.id);
    if (!b) {
      out << "ERROR: バックアップに " << j.name << " (id " << int(j.id) << ") が無い。書かない\n";
      return kExitInput;
    }
    const std::string why = validate_for_restore(*b);
    if (!why.empty()) {
      out << "ERROR: バックアップの " << j.name << " は復元に使えない: " << why << "。書かない（失敗時に戻せない）\n";
      return kExitInput;
    }
  }
  for (uint8_t id : sel) {
    const auto * m = a.measure.find(id);
    const auto * b = a.backup.find(id);
    if (!m || std::none_of(a.all.begin(), a.all.end(), [&](const JointId & j) {return j.id == id;})) {
      out << "ERROR: 測定ファイルまたは yaml に id " << int(id) << " が無い。書かない\n";
      return kExitInput;
    }
    MeasuredJoint c = *m;
    check_measured_joint(&c);
    if (!c.valid || !m->valid) {
      out << "ERROR: " << m->name << " の測定が検査に通っていない: " << c.problems << "。書かない（測定をやり直す）\n";
      return kExitInput;
    }
    Plan p{id, {0, 0, 0}, {0, 0, 0}};
    if (!encode_homing(m->new_homing_offset, &p.neu[0])) {
      out << "ERROR: " << m->name << " の Homing_Offset " << m->new_homing_offset << " を符号付き 12 bit で表せない。書かない\n";
      return kExitInput;
    }
    p.neu[1] = static_cast<uint16_t>(m->new_min);
    p.neu[2] = static_cast<uint16_t>(m->new_max);
    for (int i = 0; i < kNumCalibRegs; ++i) {p.old_[i] = b->raw(static_cast<CalibReg>(i));}
    plans.push_back(p);
  }
  // 2. 前提（全関節）
  out << "== 前提の確認（全関節）\n";
  if (!preflight(bus, a.all, a.opt.retries, out)) {
    out << "前提を満たさないので、何も書かない\n";
    return kExitPrecondition;
  }
  // 3. 今の EEPROM がバックアップと一致していること（全関節）。測定時の Homing_Offset とも一致していること
  out << "\n== 今の EEPROM とバックアップの照合（全関節。違えば書かない）\n";
  bool match = true;
  std::vector<JointRegs> current;
  for (const auto & j : a.all) {
    const JointRegs r = read_joint_regs(bus, j.id, a.opt.retries);
    const auto * b = a.backup.find(j.id);
    for (int i = 0; i < kNumCalibRegs; ++i) {
      if (!r.v[i]) {
        out << "NG  " << j.name << " の " << kLabel[i] << " を読めない\n";
        return kExitPrecondition;
      }
      if (*r.v[i] != b->raw(static_cast<CalibReg>(i))) {
        out << "NG  " << j.name << " " << kLabel[i] << ": 現在 " << show_value(static_cast<CalibReg>(i), r.v[i])
            << " / バックアップ " << show_value(static_cast<CalibReg>(i), b->raw(static_cast<CalibReg>(i))) << "\n";
        match = false;
      }
    }
    current.push_back(r);
  }
  if (!match) {
    out << "今の EEPROM がバックアップと違う。新しいバックアップを取り直す（backup）か、restore で戻してから。何も書かない\n";
    return kExitPrecondition;
  }
  out << "OK  全関節がバックアップと一致\n";
  for (uint8_t id : sel) {
    const auto * m = a.measure.find(id);
    const int cur = so_arm101_hardware::decode_sign_magnitude(
      *current[std::find_if(a.all.begin(), a.all.end(), [&](const JointId & j) {return j.id == id;}) - a.all.begin()].v[0],
      so_arm101_hardware::kHomingOffsetSignBit);
    if (cur != m->measured_homing_offset) {
      out << "NG  " << m->name << ": 測定時の Homing_Offset " << m->measured_homing_offset << " が今の EEPROM（" << cur
          << "）と違う。actual の計算の前提が崩れている。測定をやり直す。何も書かない\n";
      return kExitPrecondition;
    }
  }
  // 4. 計画
  out << "\n== 書き込む値（生の値。Homing_Offset は括弧内が符号付き）\n";
  for (const auto & p : plans) {
    for (int i = 0; i < kNumCalibRegs; ++i) {
      const CalibReg r = static_cast<CalibReg>(i);
      out << "  " << name_of(a.all, p.id) << " (id " << int(p.id) << ") " << kLabel[i] << ": "
          << show_value(r, p.old_[i]) << " -> " << show_value(r, p.neu[i]) << "\n";
    }
  }
  out << (all_joints ? "全関節を 1 回で書く。成功したら yaml を 1 回だけ更新する: " + a.yaml_path :
    "一部の関節だけを書く。yaml は更新しない") << "\n";
  if (!a.write) {
    out << "\ndry-run: 何も書いていない。書くには --write を付け、確認文字列 " << kConfirmText << " を入力する\n";
    return kExitOk;
  }
  out << "\n書き込む対象: ";
  for (const auto & p : plans) {out << name_of(a.all, p.id) << " ";}
  out << "（Lock=0 → 3 レジスタ → Lock=1。各項目を " << a.opt.wait_ms << " ms 待って読み戻す。失敗したら全部バックアップへ戻す）\n";
  if (!confirm()) {
    out << "確認文字列が一致しない。何も書かない\n";
    return kExitInput;
  }
  // 5. 書き込み
  bool failed = false;
  std::vector<std::string> problems;
  std::vector<uint8_t> lock_unconfirmed, touched;
  for (const auto & p : plans) {
    const std::optional<uint16_t> vals[kNumCalibRegs] = {p.neu[0], p.neu[1], p.neu[2]};
    const JointWriteResult r = write_joint_sequence(bus, p.id, vals, a.opt, &out, name_of(a.all, p.id));
    if (r.touched) {touched.push_back(p.id);}
    for (const auto & s : r.problems) {problems.push_back(s);}
    if (!r.ok) {
      failed = true;
      break;
    }
  }
  // 6. 失敗したら、書いた全関節（失敗した関節を含む）をバックアップの値へ戻す
  if (failed) {
    out << "\nERROR: 書き込みに失敗した。この回で触れた全関節をバックアップの値に戻す\n";
    for (uint8_t id : touched) {
      const auto & p = *std::find_if(plans.begin(), plans.end(), [&](const Plan & q) {return q.id == id;});
      const std::optional<uint16_t> vals[kNumCalibRegs] = {p.old_[0], p.old_[1], p.old_[2]};
      const JointWriteResult r = write_joint_sequence(bus, id, vals, a.opt, &out, name_of(a.all, id));
      for (const auto & s : r.problems) {problems.push_back("(rollback) " + s);}
      if (!r.lock_ok) {lock_unconfirmed.push_back(id);}
    }
  }
  // 7. 最終の状態（旧 / 新 / 不明）
  out << "\n== 書き込み後の状態（旧 = バックアップの値、新 = 測定ファイルの値）\n";
  bool all_target = true, all_before = true;
  for (const auto & p : plans) {
    const JointRegs now = read_joint_regs(bus, p.id, a.opt.retries);
    for (int i = 0; i < kNumCalibRegs; ++i) {
      const std::string c = classify(now.v[i], p.old_[i], p.neu[i]);
      all_target = all_target && (c == "target" || c == "same");
      all_before = all_before && (c == "before" || c == "same");
      out << "  " << name_of(a.all, p.id) << " " << kLabel[i] << ": " << show_value(static_cast<CalibReg>(i), now.v[i]) << "  ["
          << ja(c) << "]\n";
    }
    const int lk = read_lock(bus, p.id, a.opt.retries);
    if (lk != 1 && std::find(lock_unconfirmed.begin(), lock_unconfirmed.end(), p.id) == lock_unconfirmed.end()) {
      lock_unconfirmed.push_back(p.id);
    }
    out << "  " << name_of(a.all, p.id) << " Lock: " << (lk < 0 ? std::string("(read failed)") : std::to_string(lk)) << "\n";
  }
  auto names = [&](const std::vector<uint8_t> & ids) {
      std::string s;
      for (size_t i = 0; i < ids.size(); ++i) {s += (i ? "," : "") + name_of(a.all, ids[i]);}
      return s;
    };
  if (!failed && all_target && lock_unconfirmed.empty()) {
    out << "\nOK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した\n";
    if (!all_joints) {
      out << "yaml は更新していない（一部の関節だけ）。元に戻すなら: so_arm101_calib restore --backup " << a.backup_path
          << " --joints " << names(sel) << " --write\n";
      return kExitOk;
    }
    // yaml を 1 回だけ更新
    const std::string applied_at = now_string(false);
    std::string err;
    try {
      const std::string text = render_calibration_yaml(a.yaml_path, a.measure, a.backup_path, a.measure_path, applied_at);
      if (update_calibration_yaml(a.yaml_path, text, &err)) {
        out << "yaml を更新した: " << a.yaml_path << "（git の差分を確認して commit する）\n";
        return kExitOk;
      }
      std::ofstream f(a.fallback_yaml_path, std::ios::trunc);
      f << text;
    } catch (const std::exception & e) {
      err = e.what();
    }
    out << "ERROR: EEPROM は新しい値になったが、yaml を更新できなかった: " << err << "\n"
        << "  yaml は古いまま（この状態で書き込みモードの launch は起動を拒否する）。退避先: " << a.fallback_yaml_path
        << "（あれば）。yaml を直して置き換えるか、restore で戻す: so_arm101_calib restore --backup " << a.backup_path
        << " --write\n";
    return kExitPartial;
  }
  out << "\nERROR: 適用が完了していない\n";
  for (const auto & p : problems) {out << "  - " << p << "\n";}
  if (!lock_unconfirmed.empty()) {
    out << "  ! Lock=1 を確認できない関節: " << names(lock_unconfirmed)
        << "（EEPROM が書き換え可能なまま）。サーボの電源を入れ直す（Lock は電源投入時に 1 になる）\n";
  }
  out << "  戻すには（ros2_control を止めたまま）:\n    so_arm101_calib restore --backup " << a.backup_path
      << " --write\n  yaml は更新していない。この状態で書き込みモード（read_only:=false）の launch を起動しない"
         "（EEPROM と yaml が違えば起動を拒否する）\n";
  return (all_before && lock_unconfirmed.empty()) ? kExitFailedAllOld : kExitPartial;
}

}  // namespace so_arm101_calibration
