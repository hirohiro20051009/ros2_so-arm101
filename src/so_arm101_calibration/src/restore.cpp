// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/restore.hpp"

#include <algorithm>
#include <ostream>

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
  if (c == "target") {return "バックアップ";}
  if (c == "before") {return "実行前";}
  if (c == "same") {return "バックアップ=実行前";}
  return "不明";
}

std::string join_ids(const std::vector<uint8_t> & ids, const std::vector<JointId> & all)
{
  std::string s;
  for (size_t i = 0; i < ids.size(); ++i) {s += (i ? "," : "") + name_of(all, ids[i]);}
  return s;
}
}  // namespace

static int run_restore_impl(
  CalibWriteBus & bus, const RestoreArgs & a, const std::function<bool()> & confirm, std::ostream & out)
{
  std::vector<uint8_t> sel = a.selected;
  if (sel.empty()) {
    for (const auto & j : a.all) {sel.push_back(j.id);}
  }
  // 1. 入力の検査（何も書かない）
  if (a.opt.wait_ms < 0 || a.opt.wait_ms > kMaxWaitMs || a.opt.retries < 0 || a.opt.retries > 10) {
    out << "ERROR: --eeprom-wait-ms は 0〜" << kMaxWaitMs << "、retries は 0〜10\n";
    return kExitInput;
  }
  for (uint8_t id : sel) {
    const auto * b = a.backup.find(id);
    if (!b) {
      out << "ERROR: バックアップに id " << int(id) << " (" << name_of(a.all, id) << ") が無い。書かない\n";
      return kExitInput;
    }
    const std::string why = validate_for_restore(*b);
    if (!why.empty()) {
      out << "ERROR: バックアップの " << name_of(a.all, id) << " は復元に使えない: " << why << "。書かない\n";
      return kExitInput;
    }
    if (std::none_of(a.all.begin(), a.all.end(), [&](const JointId & j) {return j.id == id;})) {
      out << "ERROR: id " << int(id) << " は yaml の関節に無い。書かない\n";
      return kExitInput;
    }
  }
  // 2. 前提（全関節）
  out << "== 前提の確認（全関節）\n";
  if (!preflight(bus, a.all, a.opt.retries, out)) {
    out << "前提を満たさないので、何も書かない\n";
    return kExitPrecondition;
  }
  // 3. 現在の値と計画
  struct Entry
  {
    uint8_t id;
    JointRegs before;
  };
  std::vector<Entry> entries;
  out << "\n== 現在の値とバックアップ（生の値。Homing_Offset は括弧内が符号付き）\n";
  bool any_write = false;
  for (uint8_t id : sel) {
    Entry e{id, read_joint_regs(bus, id, a.opt.retries)};
    const auto * b = a.backup.find(id);
    JointEeprom cur;
    std::string err;
    for (int i = 0; i < kNumCalibRegs; ++i) {
      const CalibReg r = static_cast<CalibReg>(i);
      if (!e.before.v[i]) {
        out << "ERROR: " << name_of(a.all, id) << " の " << kLabel[i] << " を読めない。何も書かない\n";
        return kExitPrecondition;
      }
      const bool same = *e.before.v[i] == b->raw(r);
      any_write = any_write || !same || a.opt.rewrite_same;
      out << "  " << name_of(a.all, id) << " (id " << int(id) << ") " << kLabel[i] << ": 現在 "
          << show_value(r, e.before.v[i]) << " / バックアップ " << show_value(r, b->raw(r)) << " -> "
          << (same ? (a.opt.rewrite_same ? "同じ（--rewrite-same で書き直す）" : "同じ（書かない）") : "書く") << "\n";
    }
    if (read_joint_eeprom(bus, id, name_of(a.all, id), a.opt.retries, &cur, &err)) {
      for (int k = 0; k < kEepromBytes; ++k) {
        const int addr = k;
        const bool in_restored = (addr >= 9 && addr <= 12) || addr == 31 || addr == 32;
        if (!in_restored && cur.bytes[k] != b->bytes[k]) {
          out << "  注意: " << name_of(a.all, id) << " の EEPROM アドレス " << addr << " がバックアップと違う（現在 "
              << int(cur.bytes[k]) << "、バックアップ " << int(b->bytes[k]) << "）。この道具の復元の対象外\n";
        }
      }
    }
    entries.push_back(e);
  }
  if (!a.write) {
    out << "\ndry-run: 何も書いていない。書くには --write を付け、確認文字列 " << kConfirmText << " を入力する\n";
    return kExitOk;
  }
  if (!any_write) {
    out << "\n書くものはない（全部同じ）。何も書かない\n";
    return kExitOk;
  }
  out << "\n書き込む対象: " << join_ids(sel, a.all) << "（Lock=0 → 3 レジスタ → Lock=1。各項目を " << a.opt.wait_ms
      << " ms 待って読み戻す）\n";
  if (!confirm()) {
    out << "確認文字列が一致しない。何も書かない\n";
    return kExitInput;
  }
  // 4. 書き込み
  bool failed = false;
  std::vector<std::string> problems;
  std::vector<uint8_t> lock_unconfirmed;
  for (const auto & e : entries) {
    if (failed) {break;}
    const auto * b = a.backup.find(e.id);
    const std::string nm = name_of(a.all, e.id);
    bool touched = false;
    bool joint_ok = true;
    // Lock=0 が必要な書き込みがあるか
    bool needs = false;
    for (int i = 0; i < kNumCalibRegs; ++i) {
      needs = needs || (*e.before.v[i] != b->raw(static_cast<CalibReg>(i))) || a.opt.rewrite_same;
    }
    if (!needs) {continue;}
    out << "-- " << nm << ": Lock=0\n";
    StepResult s = set_lock_verified(bus, e.id, false, a.opt);
    touched = true;
    if (!s.ok) {
      problems.push_back(s.msg);
      joint_ok = false;
    }
    for (int i = 0; i < kNumCalibRegs && joint_ok; ++i) {
      const CalibReg r = static_cast<CalibReg>(i);
      if (*e.before.v[i] == b->raw(r) && !a.opt.rewrite_same) {continue;}
      out << "   " << kLabel[i] << " <- " << b->raw(r) << "\n";
      s = write_calib_verified(bus, e.id, r, b->raw(r), a.opt);
      if (!s.ok) {
        problems.push_back(s.msg);
        joint_ok = false;
      }
    }
    if (touched) {
      s = set_lock_verified(bus, e.id, true, a.opt);
      if (!s.ok) {
        problems.push_back(s.msg);
        lock_unconfirmed.push_back(e.id);
        joint_ok = false;
      }
    }
    if (!joint_ok) {failed = true;}
  }
  // 5. 最終の状態
  out << "\n== 書き込み後の状態\n";
  bool all_target = true, all_before = true;
  for (const auto & e : entries) {
    const auto * b = a.backup.find(e.id);
    const JointRegs now = read_joint_regs(bus, e.id, a.opt.retries);
    for (int i = 0; i < kNumCalibRegs; ++i) {
      const CalibReg r = static_cast<CalibReg>(i);
      const std::string c = classify(now.v[i], *e.before.v[i], b->raw(r));
      all_target = all_target && (c == "target" || c == "same");
      all_before = all_before && (c == "before" || c == "same");
      out << "  " << name_of(a.all, e.id) << " " << kLabel[i] << ": " << show_value(r, now.v[i]) << "  [" << ja(c) << "]\n";
    }
    const int lk = read_lock(bus, e.id, a.opt.retries);
    if (lk != 1 && std::find(lock_unconfirmed.begin(), lock_unconfirmed.end(), e.id) == lock_unconfirmed.end()) {
      // Lock を触らなかった関節は読むだけ（電源投入時は 1）。1 でなければ知らせる
      if (lk != 1) {lock_unconfirmed.push_back(e.id);}
    }
    out << "  " << name_of(a.all, e.id) << " Lock: " << (lk < 0 ? std::string("(read failed)") : std::to_string(lk)) << "\n";
  }
  if (!failed && all_target && lock_unconfirmed.empty()) {
    out << "\nOK: バックアップの値に戻り、Lock=1 を確認した\n";
    return kExitOk;
  }
  out << "\nERROR: 復元が完了していない\n";
  for (const auto & p : problems) {out << "  - " << p << "\n";}
  if (!lock_unconfirmed.empty()) {
    out << "  ! Lock=1 を確認できない関節: " << join_ids(lock_unconfirmed, a.all)
        << "（EEPROM が書き換え可能なまま）。サーボの電源を入れ直す（Lock は電源投入時に 1 になる）\n";
  }
  out << "  やり直すには（ros2_control を止めたまま）:\n    so_arm101_calib restore --backup " << a.backup_path
      << " --write\n  yaml は更新していない。この状態で書き込みモード（read_only:=false）の launch を起動しない\n";
  return (all_before && lock_unconfirmed.empty()) ? kExitFailedAllOld : kExitPartial;
}

/// 失敗（終了コードが ExitOk 以外）のとき、バス層のカウンタを 1 行出す（古い応答の取り違え・化けの切り分け用）。
int run_restore(
  CalibWriteBus & bus, const RestoreArgs & args, const std::function<bool()> & confirm, std::ostream & out)
{
  const int code = run_restore_impl(bus, args, confirm, out);
  if (code != kExitOk) {out << bus.counters().to_string() << "\n";}
  return code;
}

}  // namespace so_arm101_calibration
